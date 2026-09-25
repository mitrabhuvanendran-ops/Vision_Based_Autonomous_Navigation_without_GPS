#!/usr/bin/env python3
"""Stage 7.2 - Converts a (bearing_deg, distance_m) dashboard command into a
frozen odom-frame PoseStamped goal + RViz marker."""
import math

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker


class GoalConverter(Node):
    def __init__(self):
        super().__init__('goal_converter')
        self.set_parameters([
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.marker_pub = self.create_publisher(Marker, '/goal_marker', 10)
        self.create_subscription(Float32MultiArray, '/dashboard/goal_polar', self.goal_cb, 10)

        self.get_logger().info("Goal converter active, waiting on /dashboard/goal_polar")

    def goal_cb(self, msg):
        if len(msg.data) < 2:
            self.get_logger().warn("goal_polar needs [bearing_deg, distance_m]")
            return
        bearing_deg, distance_m = msg.data[0], msg.data[1]

        try:
            trans = self.tf_buffer.lookup_transform(
                'odom', 'base_footprint', rclpy.time.Time(), timeout=Duration(seconds=0.2))
        except Exception as e:
            self.get_logger().error(f"Could not get current pose: {e}")
            return

        # Robot-frame offset, then rotate into odom using the pose *now* and
        # freeze it - the goal does not get recomputed as the robot moves.
        theta = math.radians(bearing_deg)
        x_r = distance_m * math.cos(theta)
        y_r = distance_m * math.sin(theta)

        q = trans.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        rx, ry = trans.transform.translation.x, trans.transform.translation.y

        goal_x = rx + x_r * math.cos(yaw) - y_r * math.sin(yaw)
        goal_y = ry + x_r * math.sin(yaw) + y_r * math.cos(yaw)

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'odom'
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)

        marker = Marker()
        marker.header = goal.header
        marker.ns = 'goal'
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = goal.pose
        marker.scale.x = marker.scale.y = marker.scale.z = 0.6
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.0, 1.0, 1.0
        self.marker_pub.publish(marker)

        self.get_logger().info(
            f"New goal: bearing={bearing_deg}deg dist={distance_m}m -> odom ({goal_x:.2f}, {goal_y:.2f})")


def main():
    rclpy.init()
    node = GoalConverter()
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
