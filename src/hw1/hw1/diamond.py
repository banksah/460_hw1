#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class DiamondNode(Node):
    def __init__(self):
        super().__init__('diamond_node')
        self.pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        self.speed = 1.0
        self.side_length = 2.0
        self.turn_angle = math.pi / 2  # 90 degrees
        self.initial_turn = math.pi / 4  # 45 degrees
        self.step = 0
        self.state = "initial_turn"
        self.timer = self.create_timer(0.1, self.run)
        self.start_time = None

    def run(self):
        msg = Twist()

        if self.step >= 4:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.pub.publish(msg)
            return

        if self.state == "initial_turn":
            msg.angular.z = self.initial_turn
            msg.linear.x = 0.0
            self.pub.publish(msg)
            time.sleep(1)  # rotate 45 degrees
            self.state = "move"
            self.start_time = self.get_clock().now()

        elif self.state == "move":
            msg.linear.x = self.speed
            msg.angular.z = 0.0
            self.pub.publish(msg)

            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if elapsed >= self.side_length / self.speed:
                self.state = "turn"
                self.start_time = self.get_clock().now()

        elif self.state == "turn":
            msg.linear.x = 0.0
            msg.angular.z = self.turn_angle
            self.pub.publish(msg)
            time.sleep(1)  # rotate 90 degrees
            self.step += 1
            self.state = "move"
            self.start_time = self.get_clock().now()

def main(args=None):
    rclpy.init(args=args)
    node = DiamondNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
