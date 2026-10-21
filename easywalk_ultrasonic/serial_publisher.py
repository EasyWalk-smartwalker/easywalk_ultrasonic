# Serial Publisher -- legge l'Arduino dei 6 HC-SR04 e pubblica le misure.
#
# Firmware richiesto: arduino/walker_sensors.ino v2.0, che invia il tempo di
# volo grezzo di ogni ping (vedi l'intestazione dello sketch).
#
# Topic pubblicati, per ogni sensore <name> in {l30, l15, ctr, r15, r30, bot}:
#   walker/range/<name>     sensor_msgs/Range, metri, convenzione REP 117:
#                             +inf = nessuna eco entro max_range
#                             -inf = oggetto sotto min_range (troppo vicino)
#                             NaN  = lettura non valida
#   walker/sonar/raw        std_msgs/String, riga seriale grezza (per
#                           ricalcolare tutto offline dai rosbag).
#
# Distanza: d = c(T) * tof / 2, con c(T) = 331.3 + 0.606 T [m/s], poi la
# correzione lineare per sensore d_cal = gain * d + offset_m.
# Timestamp: tempo Arduino (micros) all'istante medio del volo, convertito
# nel tempo ROS stimando l'offset tra i due orologi.

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

SENSORS = ['L30', 'L15', 'CTR', 'R15', 'R30', 'BOT']   # indice = id firmware
UINT32 = 1 << 32


class ClockSync:
    """Stima l'offset tra micros() di Arduino e il clock ROS.

    Per ogni riga ricevuta: offset_campione = t_ricezione_ros - t_fine_misura_arduino.
    Il ritardo di trasmissione e' sempre >= 0, quindi il minimo su una finestra
    recente e' la stima meno affetta da latenza (filtro di minimo).
    Gestisce anche il wrap a 32 bit di micros() (~71 minuti)."""

    def __init__(self, window=200):
        self._window = deque(maxlen=window)
        self._last_raw = None
        self._wraps = 0

    def unwrap(self, raw_us):
        if self._last_raw is not None and raw_us < self._last_raw - UINT32 // 2:
            self._wraps += 1
        self._last_raw = raw_us
        return raw_us + self._wraps * UINT32

    def update(self, ros_ns, arduino_us):
        self._window.append(ros_ns - arduino_us * 1000)

    def to_ros_ns(self, arduino_us):
        if not self._window:
            return None
        return arduino_us * 1000 + min(self._window)


class SerialPublisher(Node):

    def __init__(self):
        super().__init__('serial_publisher')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('temperature_c', 20.0)
        self.declare_parameter('frame_suffix', '_hcsr04_link')
        # Configurazione inviata al firmware all'avvio
        self.declare_parameter('configure_firmware', True)
        self.declare_parameter('fw_mode', 'ring')          # 'ring' oppure 'single'
        self.declare_parameter('fw_single_sensor', 2)      # indice, se fw_mode=single
        self.declare_parameter('fw_order', [0, 3, 2, 1, 4, 5])
        self.declare_parameter('fw_inter_ping_delay_ms', 0)
        self.declare_parameter('fw_cycle_delay_ms', 20)
        self.declare_parameter('fw_timeout_us', 15000)
        self.declare_parameter('fw_min_sensor_period_ms', 60)
        # Calibrazione per sensore (valori neutri di default, vedi
        # config/sonar_calibration.yaml e il protocollo, Sez. S3-S4)
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
        self.speed_of_sound = 331.3 + 0.606 * temperature
        self.frame_suffix = self.get_parameter('frame_suffix').value

        self.range_pubs = {
            n: self.create_publisher(Range, f'walker/range/{n.lower()}', 10) for n in SENSORS}
        self.raw_pub = self.create_publisher(String, 'walker/sonar/raw', 50)

        self.sync = ClockSync()
        self.last_cycle = None
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

        self._running = True
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        self.create_timer(5.0, self._report_health)

    # ----------------------------------------------------------- firmware

    def _configure_firmware(self):
        # L'Arduino Mega si resetta all'apertura della seriale: attende il
        # banner (max 3 s) prima di inviare i comandi.
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            line = self.ser.readline().decode('ascii', errors='replace').strip()
            if line.startswith('# easywalk_hcsr04'):
                break
        order = ','.join(str(i) for i in self.get_parameter('fw_order').value)
        cmds = [
            f'O {order}',
            f'D {self.get_parameter("fw_inter_ping_delay_ms").value}',
            f'P {self.get_parameter("fw_cycle_delay_ms").value}',
            f'T {self.get_parameter("fw_timeout_us").value}',
            f'Q {self.get_parameter("fw_min_sensor_period_ms").value}',
        ]
        if self.get_parameter('fw_mode').value == 'single':
            cmds.append(f'M S {self.get_parameter("fw_single_sensor").value}')
        else:
            cmds.append('M R')
        for cmd in cmds:
            self.ser.write((cmd + '\n').encode('ascii'))
            time.sleep(0.05)
        self.get_logger().info(f'Firmware configured: {"; ".join(cmds)}')

    # ------------------------------------------------------------ lettura

    def _read_loop(self):
        while self._running and rclpy.ok():
            try:
                raw = self.ser.readline()
            except serial.SerialException as e:
                # Nessun dato pubblicato: i nodi a valle vedono i sonar come
                # stantii (ultrasonic_safety_node lo tratta come pericolo).
                self.get_logger().error(f'Serial error: {e}', throttle_duration_sec=5.0)
                time.sleep(0.5)
                continue
            if not raw:
                continue
            recv_ns = self.get_clock().now().nanoseconds
            line = raw.decode('ascii', errors='replace').strip()
            if not line:
                continue
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
        _, cycle, idx, t_trig, tof, status = line.split(',')
        idx, tof, status = int(idx), int(tof), int(status)
        name = SENSORS[idx]
        t_trig = self.sync.unwrap(int(t_trig))
        # La riga viene stampata a fine misura: t_trig + tof se c'e' l'eco,
        # t_trig + timeout se non c'e' (pulseIn attende tutto il timeout).
        # Questo e' l'istante piu' vicino alla ricezione, usato per l'offset.
        t_end = t_trig + (tof if status == 0 and tof > 0 else self.echo_timeout_us)
        self.sync.update(recv_ns, t_end)
        stamp_ns = self.sync.to_ros_ns(t_trig + tof // 2)

        cal = self.calib[name]
        if status == 1 or tof == 0:
            value = math.inf                      # nessuna eco
        else:
            d = self.speed_of_sound * tof * 1e-6 / 2.0
            d = cal['gain'] * d + cal['offset_m']
            if d < cal['min_range_m']:
                value = -math.inf                 # troppo vicino
            elif d > cal['max_range_m']:
                value = math.inf                  # oltre la portata
            else:
                value = d
        self._publish(name, value, stamp_ns)

    def _handle_cycle(self, line):
        cycle = int(line.split(',')[1])
        if self.last_cycle is not None:
            gap = (cycle - self.last_cycle) % UINT32 - 1
            if gap > 0:
                self.lost_cycles += gap
        self.last_cycle = cycle

    # ------------------------------------------------------- pubblicazione

    def _publish(self, name, value, stamp_ns):
        cal = self.calib[name]
        stamp = (Time(nanoseconds=stamp_ns) if stamp_ns is not None
                 else self.get_clock().now()).to_msg()

        msg = Range()
        msg.header.stamp = stamp
        msg.header.frame_id = f'{name}{self.frame_suffix}'
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = float(cal['fov_rad'])
        msg.min_range = float(cal['min_range_m'])
        msg.max_range = float(cal['max_range_m'])
        msg.range = float(value)
        self.range_pubs[name].publish(msg)

    def _report_health(self):
        if self.lost_cycles:
            self.get_logger().warn(f'Lost {self.lost_cycles} sonar cycles in the last 5 s')
            self.lost_cycles = 0

    def destroy_node(self):
        self._running = False
        try:
            if self._reader.is_alive():
                self._reader.join(timeout=1.0)
        except KeyboardInterrupt:
            pass   # secondo Ctrl+C: il thread e' daemon, termina col processo
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
