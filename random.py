#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class RandomNode(Node):
    def __init__(self):
        super().__init__('random_node')
        self.pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        self.speed = 1.0  # linear speed
        self.turn_speed = math.pi  # rad/s for rotation
        # sequence of (rotation in degrees, forward distance)
        self.sequence = [
            (155, 1.5),
            (115, 1.0),
            (253, 1.0),
            (78, 1.5)
        ]
        self.step = 0
        self.state = "rotate"
        self.timer = self.create_timer(0.1, self.run)
        self.start_time = None

    def run(self):
        if self.step >= len(self.sequence):
            # finished all steps
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.pub.publish(msg)
            return

        rotation_deg, distance = self.sequence[self.step]
        rotation_rad = math.radians(rotation_deg)

        msg = Twist()

        if self.state == "rotate":
            msg.linear.x = 0.0
            msg.angular.z = self.turn_speed
            self.pub.publish(msg)
            if self.start_time is None:
                self.start_time = self.get_clock().now()
            # rotate for time = angle / angular speed
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if elapsed >= abs(rotation_rad) / self.turn_speed:
                self.state = "move"
                self.start_time = self.get_clock().now()

        elif self.state == "move":
            msg.linear.x = self.speed
            msg.angular.z = 0.0
            self.pub.publish(msg)
            if self.start_time is None:
                self.start_time = self.get_clock().now()
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if elapsed >= distance / self.speed:
                self.step += 1
                self.state = "rotate"
                self.start_time = None  # reset timer for next rotation

def main(args=None):
    rclpy.init(args=args)
    node = RandomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
