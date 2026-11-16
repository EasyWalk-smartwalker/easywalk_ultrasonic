# Serial Publisher -- reads the Arduino of the 6 HC-SR04 and publishes the ranges.
#
# Required firmware: arduino/walker_sensors.ino v2.0, which sends the raw time
# of flight of every ping (see the header of the sketch).
#
# Published topics, for each sensor <name> in {l30, l15, ctr, r15, r30, bot}:
#   walker/range/<name>     sensor_msgs/Range, meters, REP 117 convention:
#                             +inf = no echo within max_range
#                             -inf = object closer than min_range (too close)
#                             NaN  = invalid reading
#   walker/sonar/raw        std_msgs/String, raw serial line (to recompute
#                           everything offline from the rosbags).
#
# Distance: d = c(T) * tof / 2, with c(T) = 331.3 + 0.606 T [m/s], followed by
# the per-sensor linear correction d_cal = gain * d + offset_m.
# Timestamp: Arduino time (micros) at the middle of the flight, converted to
# ROS time by estimating the offset between the two clocks.

import math
import threading
import time
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Range
from std_msgs.msg import String
import serial

SENSORS = ['L30', 'L15', 'CTR', 'R15', 'R30', 'BOT']   # index = firmware sensor id
UINT32 = 1 << 32                                        # micros() wraps at 2^32 us


class ClockSync:
    """Estimates the offset between the Arduino micros() and the ROS clock.

    For every received line: sample = ROS reception time - Arduino time at the
    end of the measurement. The transmission delay is always >= 0, so the
    minimum over a recent window is the estimate least affected by latency
    (minimum filter). Also handles the 32-bit wrap of micros() (~71 minutes).
    """

    def __init__(self, window=200):
        self._window = deque(maxlen=window)   # recent offset samples [ns]
        self._last_raw = None
        self._wraps = 0

    def unwrap(self, raw_us):
        """Raw micros() value -> monotonic microseconds (adds the wraps)."""
        if self._last_raw is not None and raw_us < self._last_raw - UINT32 // 2:
            self._wraps += 1
        self._last_raw = raw_us
        return raw_us + self._wraps * UINT32

    def update(self, ros_ns, arduino_us):
        """Adds one offset sample (ROS reception time vs. Arduino time)."""
        self._window.append(ros_ns - arduino_us * 1000)

    def to_ros_ns(self, arduino_us):
        """Arduino time [us] -> ROS time [ns], or None before the first sample."""
        if not self._window:
            return None
        return arduino_us * 1000 + min(self._window)


class SerialPublisher(Node):

    def __init__(self):
        super().__init__('serial_publisher')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        # Temperature of the session: sets the speed of sound.
        self.declare_parameter('temperature_c', 20.0)
        # Frame of each Range message: <NAME><suffix>, e.g. CTR_hcsr04_link.
        self.declare_parameter('frame_suffix', '_hcsr04_link')
        # Configuration sent to the firmware at startup (see the sketch header)
        self.declare_parameter('configure_firmware', True)
        self.declare_parameter('fw_mode', 'ring')          # 'ring' or 'single'
        self.declare_parameter('fw_single_sensor', 2)      # index, if fw_mode=single
        self.declare_parameter('fw_order', [0, 3, 2, 1, 4, 5])
        self.declare_parameter('fw_inter_ping_delay_ms', 0)
        self.declare_parameter('fw_cycle_delay_ms', 20)
        self.declare_parameter('fw_timeout_us', 15000)
        self.declare_parameter('fw_min_sensor_period_ms', 60)
        # Per-sensor calibration (neutral defaults, see
        # config/sonar_calibration.yaml and the protocol, sections S3-S4)
        self.calib = {}
        for name in SENSORS:
            self.declare_parameter(f'{name}.gain', 1.0)
            self.declare_parameter(f'{name}.offset_m', 0.0)
            self.declare_parameter(f'{name}.fov_rad', 0.26)
            self.declare_parameter(f'{name}.min_range_m', 0.03)
            self.declare_parameter(f'{name}.max_range_m', 2.5)
            self.calib[name] = {
                k: self.get_parameter(f'{name}.{k}').value
                for k in ('gain', 'offset_m', 'fov_rad', 'min_range_m', 'max_range_m')
            }

        self.echo_timeout_us = int(self.get_parameter('fw_timeout_us').value)
        temperature = self.get_parameter('temperature_c').value
        self.speed_of_sound = 331.3 + 0.606 * temperature   # [m/s]
        self.frame_suffix = self.get_parameter('frame_suffix').value

        self.range_pubs = {
            n: self.create_publisher(Range, f'walker/range/{n.lower()}', 10) for n in SENSORS}
        self.raw_pub = self.create_publisher(String, 'walker/sonar/raw', 50)

        self.sync = ClockSync()
        self.last_cycle = None   # last cycle counter seen (to detect lost cycles)
        self.lost_cycles = 0

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
        except serial.SerialException as e:
            self.get_logger().error(f'Could not open serial port {port}: {e}')
            raise
        self.get_logger().info(
            f'Connected to Arduino on {port} @ {baud} baud, '
            f'c = {self.speed_of_sound:.1f} m/s (T = {temperature:.1f} C)')

        if self.get_parameter('configure_firmware').value:
            self._configure_firmware()

        # The serial port is read in a dedicated thread, so that a slow or
        # blocked read never stalls the ROS executor.
        self._running = True
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        self.create_timer(5.0, self._report_health)

    # ----------------------------------------------------------- firmware

    def _configure_firmware(self):
        # The Arduino Mega resets when the serial port is opened: wait for its
        # banner (max 3 s) before sending the commands.
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            line = self.ser.readline().decode('ascii', errors='replace').strip()
            if line.startswith('# easywalk_hcsr04'):
                break
        order = ','.join(str(i) for i in self.get_parameter('fw_order').value)
        cmds = [
            f'O {order}',                                            # firing order
            f'D {self.get_parameter("fw_inter_ping_delay_ms").value}',   # pause between pings
            f'P {self.get_parameter("fw_cycle_delay_ms").value}',        # pause after a cycle
            f'T {self.get_parameter("fw_timeout_us").value}',            # echo timeout
            f'Q {self.get_parameter("fw_min_sensor_period_ms").value}',  # min period per sensor
        ]
        if self.get_parameter('fw_mode').value == 'single':
            cmds.append(f'M S {self.get_parameter("fw_single_sensor").value}')
        else:
            cmds.append('M R')
        for cmd in cmds:
            self.ser.write((cmd + '\n').encode('ascii'))
            time.sleep(0.05)
        self.get_logger().info(f'Firmware configured: {"; ".join(cmds)}')

    # ------------------------------------------------------------ reading

    def _read_loop(self):
        while self._running and rclpy.ok():
            try:
                raw = self.ser.readline()
            except serial.SerialException as e:
                # Nothing is published: downstream nodes see the sonars as
                # stale (e.g. sonar_to_sectors marks their sectors STALE).
                self.get_logger().error(f'Serial error: {e}', throttle_duration_sec=5.0)
                time.sleep(0.5)
                continue
            if not raw:
                continue
            recv_ns = self.get_clock().now().nanoseconds   # for the clock sync
            line = raw.decode('ascii', errors='replace').strip()
            if not line:
                continue
            # Every line is republished as is, to keep the raw data in the bags.
            self.raw_pub.publish(String(data=line))
            try:
                if line.startswith('R,'):
                    self._handle_ping(line, recv_ns)
                elif line.startswith('C,'):
                    self._handle_cycle(line)
                elif line.startswith('#'):
                    self.get_logger().debug(f'fw: {line}')
                else:
                    self.get_logger().warn(
                        f'Unknown line "{line}" (firmware v2.0 required)',
                        throttle_duration_sec=5.0)
            except (ValueError, IndexError) as e:
                self.get_logger().warn(f'Malformed line "{line}": {e}',
                                       throttle_duration_sec=2.0)

    def _handle_ping(self, line, recv_ns):
        """R,<cycle>,<sensor>,<t_trig_us>,<tof_us>,<status> -> one Range message."""
        _, cycle, idx, t_trig, tof, status = line.split(',')
        idx, tof, status = int(idx), int(tof), int(status)
        name = SENSORS[idx]
        t_trig = self.sync.unwrap(int(t_trig))
        # The line is printed at the end of the measurement: t_trig + tof with
        # an echo, t_trig + timeout without (pulseIn waits for the whole
        # timeout). This is the instant closest to the reception, used for the
        # clock offset.
        t_end = t_trig + (tof if status == 0 and tof > 0 else self.echo_timeout_us)
        self.sync.update(recv_ns, t_end)
        # Stamp: middle of the flight, i.e. when the sound hit the obstacle.
        stamp_ns = self.sync.to_ros_ns(t_trig + tof // 2)

        cal = self.calib[name]
        if status == 1 or tof == 0:
            value = math.inf                      # no echo
        else:
            d = self.speed_of_sound * tof * 1e-6 / 2.0   # round trip -> one way
            d = cal['gain'] * d + cal['offset_m']        # per-sensor calibration
            if d < cal['min_range_m']:
                value = -math.inf                 # too close
            elif d > cal['max_range_m']:
                value = math.inf                  # beyond the range
            else:
                value = d
        self._publish(name, value, stamp_ns)

    def _handle_cycle(self, line):
        """C,<cycle>,<t_us>: counts lost cycles from gaps in the cycle counter."""
        cycle = int(line.split(',')[1])
        if self.last_cycle is not None:
            gap = (cycle - self.last_cycle) % UINT32 - 1
            if gap > 0:
                self.lost_cycles += gap
        self.last_cycle = cycle

    # ------------------------------------------------------- publishing

    def _publish(self, name, value, stamp_ns):
        cal = self.calib[name]
        stamp = (Time(nanoseconds=stamp_ns) if stamp_ns is not None
                 else self.get_clock().now()).to_msg()

        msg = Range()
        msg.header.stamp = stamp
        msg.header.frame_id = f'{name}{self.frame_suffix}'
        msg.radiation_type = Range.ULTRASOUND
        # Cone and limits from the calibration file: downstream nodes (e.g.
        # sonar_to_sectors) read them from the message.
        msg.field_of_view = float(cal['fov_rad'])
        msg.min_range = float(cal['min_range_m'])
        msg.max_range = float(cal['max_range_m'])
        msg.range = float(value)
        self.range_pubs[name].publish(msg)

    def _report_health(self):
        """Every 5 s: warns if serial lines (whole cycles) were lost."""
        if self.lost_cycles:
            self.get_logger().warn(f'Lost {self.lost_cycles} sonar cycles in the last 5 s')
            self.lost_cycles = 0

    def destroy_node(self):
        self._running = False
        try:
            if self._reader.is_alive():
                self._reader.join(timeout=1.0)
        except KeyboardInterrupt:
            pass   # second Ctrl+C: the thread is a daemon, it ends with the process
        self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
