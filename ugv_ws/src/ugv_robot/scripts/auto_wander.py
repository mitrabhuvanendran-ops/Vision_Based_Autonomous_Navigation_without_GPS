#!/usr/bin/env python3
import math
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class CanyonAutoWanderer(Node):
    def __init__(self):
        super().__init__('canyon_auto_wanderer')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.twist = Twist()
        self.state = "FORWARD"
        self.turn_direction = 1.0
        self.state_timer = 0
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Autonomous canyon wanderer active!")

    def scan_callback(self, msg):
        ranges = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)]
        if not ranges:
            return

        mid = len(msg.ranges) // 2
        span = max(1, len(msg.ranges) // 6)
        front_slice = [r for r in msg.ranges[mid - span : mid + span] if not math.isinf(r) and not math.isnan(r)]
        min_front = min(front_slice) if front_slice else 10.0

        if min_front < 2.0 and self.state == "FORWARD":
            self.state = "AVOID"
            self.turn_direction = random.choice([-1.0, 1.0])
            self.state_timer = 25  # ~2.5 seconds to turn

    def control_loop(self):
        if self.state == "FORWARD":
            self.twist.linear.x = 0.8
            # Slight random weave to capture diverse perspectives and lighting
            t = self.get_clock().now().nanoseconds * 1e-9
            self.twist.angular.z = 0.35 * math.sin(t * 0.7)
        elif self.state == "AVOID":
            self.twist.linear.x = -0.2
            self.twist.angular.z = 1.2 * self.turn_direction
            self.state_timer -= 1
            if self.state_timer <= 0:
                self.state = "FORWARD"

        self.pub.publish(self.twist)

def main():
    rclpy.init()
    node = CanyonAutoWanderer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
