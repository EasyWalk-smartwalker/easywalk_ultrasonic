# Serial Publisher — legge Arduino, pubblica distanze come Float32 e Range

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Range
import serial

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE   = 57600
SENSORS     = ['L30', 'L15', 'CTR', 'R15', 'R30', 'BOT']

class SerialPublisher(Node):

    def __init__(self):
        super().__init__('serial_publisher')

        self.float_pubs = {
            name: self.create_publisher(Float32, f'walker/distance/{name.lower()}', 10)
            for name in SENSORS
        }
        self.range_pubs = {
            name: self.create_publisher(Range, f'walker/range/{name.lower()}', 10)
            for name in SENSORS
        }

        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            self.get_logger().info(f'Connected to Arduino on {SERIAL_PORT}')
        except Exception as e:
            self.get_logger().error(f'Could not open serial port: {e}')
            raise

        self.create_timer(0.02, self.read_serial)

    def read_serial(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if not line:
                return

            parts = line.split(',')
            for part in parts:
                if ':' not in part:
                    continue
                name, value = part.split(':')
                name = name.strip()
                dist = float(value)

                if name in self.float_pubs:
                    # Float32
                    msg = Float32()
                    msg.data = dist
                    self.float_pubs[name].publish(msg)

                    # Range
                    range_msg = Range()
                    range_msg.header.stamp = self.get_clock().now().to_msg()
                    range_msg.header.frame_id = f'{name}_hcsr04_link'
                    range_msg.radiation_type = Range.ULTRASOUND
                    range_msg.field_of_view = 0.26  # ~15 gradi in radianti
                    range_msg.min_range = 0.02
                    range_msg.max_range = 3.0
                    range_msg.range = dist / 100.0  # cm → metri
                    self.range_pubs[name].publish(range_msg)

        except Exception as e:
            self.get_logger().warn(f'Serial read error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
