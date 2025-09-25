import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class RectangleNode(Node):
    def __init__(self):
        super().__init__('rectangle_node')
        self.pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        # rectangle sides (meters) and speed
        self.linear_speed = 1.0
        self.angular_speed = 1.57  # ~90 degrees in rad/s
        self.side_length = 2.0

        self.step = 0
        self.state = "move"
        self.timer = self.create_timer(0.1, self.draw_rectangle)
        self.move_start_time = None

    def draw_rectangle(self):
        msg = Twist()

        if self.step >= 4:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.pub.publish(msg)
            return

        if self.state == "move":
            msg.linear.x = self.linear_speed
            msg.angular.z = 0.0
            self.pub.publish(msg)
            self.move_start_time = self.get_clock().now()
            self.state = "turn_check"

        elif self.state == "turn_check":
            elapsed = (self.get_clock().now() - self.move_start_time).nanoseconds / 1e9
            if elapsed >= self.side_length / self.linear_speed:
                self.state = "turn"

        elif self.state == "turn":
            msg.linear.x = 0.0
            msg.angular.z = self.angular_speed
            self.pub.publish(msg)
            time.sleep(1)  # rotate ~90 degrees
            self.step += 1
            self.state = "move"

def main(args=None):
    rclpy.init(args=args)
    node = RectangleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
