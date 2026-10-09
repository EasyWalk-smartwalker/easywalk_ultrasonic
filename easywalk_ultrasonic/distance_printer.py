import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

SENSORS = ['L30', 'L15', 'CTR', 'R15', 'R30', 'BOT']

class DistancePrinter(Node):

    def __init__(self):
        super().__init__('distance_printer')

        self.distances = {name: -1.0 for name in SENSORS}

        for name in SENSORS:
            self.create_subscription(
                Float32,
                f'walker/distance/{name.lower()}',
                lambda msg, n=name: self.cb(msg, n),
                10
            )

        self.create_timer(0.1, self.print_distances)

    def cb(self, msg, name):
        self.distances[name] = msg.data

    def print_distances(self):
        print(
            f"L30:{self.distances['L30']:6.1f}  "
            f"L15:{self.distances['L15']:6.1f}  "
            f"CTR:{self.distances['CTR']:6.1f}  "
            f"R15:{self.distances['R15']:6.1f}  "
            f"R30:{self.distances['R30']:6.1f}  "
            f"BOT:{self.distances['BOT']:6.1f}",
            end='\r'
        )

def main(args=None):
    rclpy.init(args=args)
    node = DistancePrinter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
