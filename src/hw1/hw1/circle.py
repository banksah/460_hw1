import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircleNode(Node):
    def __init__(self):
        super().__init__('circle')
        self.pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        self.linear_speed = 1.0    # forward speed (m/s)
        self.angular_speed = 1.0   # rotation (rad/s)
        # radius = linear_speed / angular_speed

        # publish at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = float(self.linear_speed)
        msg.angular.z = float(self.angular_speed)
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CircleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
