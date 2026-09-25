#!/usr/bin/env python3
"""Stage 8 - Pure-pursuit follower on /plan, publishing /cmd_vel_raw at ~20Hz.
The safety_gate node (separate, never bypassed) turns this into /cmd_vel."""
import math

import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.set_parameters([
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])

        self.declare_parameter('lookahead', 1.5)
        self.declare_parameter('max_linear', 0.8)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('goal_stop_tolerance', 0.3)

        self.lookahead = self.get_parameter('lookahead').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.goal_stop_tol = self.get_parameter('goal_stop_tolerance').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.path = []
        self.create_subscription(Path, '/plan', self.path_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.create_timer(1.0 / 20.0, self.control_loop)

    def path_cb(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def control_loop(self):
        if len(self.path) < 1:
            self.cmd_pub.publish(Twist())
            return
        try:
            trans = self.tf_buffer.lookup_transform(
                'odom', 'base_footprint', rclpy.time.Time(), timeout=Duration(seconds=0.05))
        except Exception:
            self.cmd_pub.publish(Twist())
            return

        x, y = trans.transform.translation.x, trans.transform.translation.y
        q = trans.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        goal_x, goal_y = self.path[-1]
        dist_to_final = math.hypot(goal_x - x, goal_y - y)
        if dist_to_final < self.goal_stop_tol:
            # Destination reached! Lock wheels to zero velocity continuously.
            self.path = []
            self.cmd_pub.publish(Twist())
            return

        target = self.path[-1]
        for px, py in self.path:
            if math.hypot(px - x, py - y) >= self.lookahead:
                target = (px, py)
                break

        dx = target[0] - x
        dy = target[1] - y
        local_x = dx * math.cos(-yaw) - dy * math.sin(-yaw)
        local_y = dx * math.sin(-yaw) + dy * math.cos(-yaw)

        heading_err = math.atan2(local_y, local_x)
        cmd = Twist()

        # If heading error > 15 deg (0.26 rad), rotate in place first (skid-steer pivot turn)
        PIVOT_THRESH = 0.26
        if abs(heading_err) > PIVOT_THRESH:
            cmd.linear.x = 0.0
            cmd.angular.z = self.max_angular if heading_err > 0 else -self.max_angular
        else:
            # Aligned: drive forward cleanly with proportional steering
            speed_factor = min(1.0, dist_to_final / 2.0)  # smooth slow-down near goal
            cmd.linear.x = self.max_linear * math.cos(heading_err) * speed_factor
            cmd.angular.z = max(-self.max_angular, min(self.max_angular, 2.5 * heading_err))

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = PathFollower()
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
