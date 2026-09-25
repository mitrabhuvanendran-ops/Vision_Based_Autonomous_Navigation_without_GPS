#!/usr/bin/env python3
"""Stage 8 - The only node allowed to write /cmd_vel. Passes /cmd_vel_raw
through unless: an obstacle is inside the minimum stopping distance on
/scan, IMU roll/pitch exceeds the safe slope limit, or /plan is stale.
Never bypassed by planner bugs - if in doubt, publish zero."""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan


class SafetyGate(Node):
    def __init__(self):
        super().__init__('safety_gate')
        self.set_parameters([
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])

        self.declare_parameter('min_stopping_distance', 0.45)
        self.declare_parameter('max_slope_rad', 0.5)  # ~28.6 deg
        self.declare_parameter('plan_staleness_s', 3.0)
        self.declare_parameter('front_half_angle_rad', 0.5)

        self.min_stop_dist = self.get_parameter('min_stopping_distance').value
        self.max_slope = self.get_parameter('max_slope_rad').value
        self.plan_staleness = self.get_parameter('plan_staleness_s').value
        self.front_half_angle = self.get_parameter('front_half_angle_rad').value

        self.obstacle_ahead = False
        self.slope_unsafe = False
        self.last_plan_time = None

        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.create_subscription(Path, '/plan', self.plan_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_raw', self.cmd_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("Safety gate active")

    def scan_cb(self, msg):
        n = len(msg.ranges)
        center = n // 2
        span = int(self.front_half_angle / msg.angle_increment) if msg.angle_increment > 0 else n // 4
        front = msg.ranges[max(0, center - span):min(n, center + span)]
        front = [r for r in front if math.isfinite(r) and r > 0]
        self.obstacle_ahead = bool(front) and min(front) < self.min_stop_dist

    def imu_cb(self, msg):
        q = msg.orientation
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        self.slope_unsafe = abs(roll) > self.max_slope or abs(pitch) > self.max_slope

    def plan_cb(self, msg):
        if msg.poses:
            self.last_plan_time = time.monotonic()

    def plan_is_stale(self):
        if self.last_plan_time is None:
            return True
        return (time.monotonic() - self.last_plan_time) > self.plan_staleness

    def cmd_cb(self, msg):
        # Allow pure rotation (pivot turn away) even if obstacle is in front
        if self.obstacle_ahead and msg.linear.x <= 0.05 and abs(msg.angular.z) > 0.1:
            self.cmd_pub.publish(msg)
            return

        if (self.obstacle_ahead and msg.linear.x > 0) or self.slope_unsafe or self.plan_is_stale():
            self.cmd_pub.publish(Twist())
            if self.obstacle_ahead:
                reason = "obstacle ahead (blocking forward movement)"
            elif self.slope_unsafe:
                reason = "unsafe roll/pitch"
            else:
                reason = "stale plan"
            self.get_logger().warn(f"Safety gate override: {reason}", throttle_duration_sec=1.0)
            return
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = SafetyGate()
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
