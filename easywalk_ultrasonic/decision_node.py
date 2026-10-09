# Decision Node — legge distanze, decide azione, pubblica su walker/action

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

class DecisionNode(Node):

    def __init__(self):
        super().__init__('decision_node')

        # Parametri dal yaml
        self.declare_parameter('stop_distance',    50.0)
        self.declare_parameter('warn_distance',    80.0)
        self.declare_parameter('side_distance',    60.0)
        self.declare_parameter('drop_threshold',    8.0)
        self.declare_parameter('bottom_valid_max', 40.0)
        self.declare_parameter('stop_hysteresis',   2.0)
        self.declare_parameter('warn_hysteresis',   2.0)
        self.declare_parameter('steer_margin',     15.0)

        self.distances = {
            'L30': -1.0,
            'L15': -1.0,
            'CTR': -1.0,
            'R15': -1.0,
            'R30': -1.0,
            'BOT': -1.0,
        }

        self.prev_state  = 'CLEAR'
        self.prev_center = -1.0

        for name in self.distances:
            self.create_subscription(
                Float32,
                f'walker/distance/{name.lower()}',
                lambda msg, n=name: self.sensor_callback(msg, n),
                10
            )

        self.action_pub = self.create_publisher(String, 'walker/action', 10)
        self.create_timer(0.05, self.decide)
        self.get_logger().info('Decision node ready.')

    def sensor_callback(self, msg, name):
        self.distances[name] = msg.data

    def safe_avg(self, a, b):
        if a < 0 and b < 0: return -1.0
        if a < 0: return b
        if b < 0: return a
        return (a + b) / 2.0

    def decide(self):
        # Legge parametri live dal yaml
        STOP_DIST        = self.get_parameter('stop_distance').value
        WARN_DIST        = self.get_parameter('warn_distance').value
        SIDE_DIST        = self.get_parameter('side_distance').value
        DROP_THRESHOLD   = self.get_parameter('drop_threshold').value
        BOTTOM_VALID_MAX = self.get_parameter('bottom_valid_max').value
        STOP_HYST        = self.get_parameter('stop_hysteresis').value
        WARN_HYST        = self.get_parameter('warn_hysteresis').value
        STEER_MARGIN     = self.get_parameter('steer_margin').value

        L30 = self.distances['L30']
        L15 = self.distances['L15']
        C   = self.distances['CTR']
        R15 = self.distances['R15']
        R30 = self.distances['R30']
        B   = self.distances['BOT']

        if 0 < B < BOTTOM_VALID_MAX and B > DROP_THRESHOLD:
            self.prev_state = 'DROP STOP'
            self.publish('DROP STOP')
            return

        if C < 0:
            self.publish(self.prev_state)
            return

        left_score  = self.safe_avg(L30, L15)
        right_score = self.safe_avg(R15, R30)

        in_critical = any(x in self.prev_state for x in ['STOP', 'STEER', 'DROP'])
        if in_critical and C < STOP_DIST + STOP_HYST:
            if   left_score  > right_score + STEER_MARGIN: action = 'STEER LEFT'
            elif right_score > left_score  + STEER_MARGIN: action = 'STEER RIGHT'
            else:                                           action = 'STOP'
            self.prev_state = action
            self.publish(action)
            return

        if self.prev_state.startswith('WARN') and C < WARN_DIST + WARN_HYST:
            self.publish(self.prev_state)
            return

        if C < STOP_DIST:
            if   left_score  > right_score + STEER_MARGIN: action = 'STEER LEFT'
            elif right_score > left_score  + STEER_MARGIN: action = 'STEER RIGHT'
            else:                                           action = 'STOP'
            self.prev_state = action
            self.publish(action)
            return

        if C < WARN_DIST:
            action = 'WARN LEFT' if left_score >= right_score else 'WARN RIGHT'
            self.prev_state = action
            self.publish(action)
            return

        if 0 < left_score < SIDE_DIST:
            self.prev_state = 'MOVE RIGHT'
            self.publish('MOVE RIGHT')
            return
        if 0 < right_score < SIDE_DIST:
            self.prev_state = 'MOVE LEFT'
            self.publish('MOVE LEFT')
            return

        self.prev_state = 'CLEAR'
        self.publish('CLEAR')

    def publish(self, action):
        msg = String()
        msg.data = action
        self.action_pub.publish(msg)
        if action != getattr(self, '_last_logged', None):
            self.get_logger().info(f'L30:{self.distances["L30"]:.1f}  '
                                   f'L15:{self.distances["L15"]:.1f}  '
                                   f'CTR:{self.distances["CTR"]:.1f}  '
                                   f'R15:{self.distances["R15"]:.1f}  '
                                   f'R30:{self.distances["R30"]:.1f}  '
                                   f'BOT:{self.distances["BOT"]:.1f}  '
                                   f'=> {action}')
            self._last_logged = action

def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
