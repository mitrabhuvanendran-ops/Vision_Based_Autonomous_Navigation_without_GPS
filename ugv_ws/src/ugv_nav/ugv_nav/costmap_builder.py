#!/usr/bin/env python3
"""Stage 6 - Fuses /seg/mask + depth (back-projected into odom), a
height-above-ground backstop, and /scan into one OccupancyGrid on /costmap.

The height backstop marks any depth point too high above the local ground
plane as occupied even if BiSeNet says traversable - this is the safety net
for the "grass/vegetation-covered rock" failure mode BiSeNet alone can miss.
"""
import math

import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image, LaserScan

FREE, OCCUPIED, UNKNOWN = 0, 100, -1
TRAVERSABLE, NON_TRAVERSABLE, VOID = 0, 1, 255


class CostmapBuilder(Node):
    def __init__(self):
        super().__init__('costmap_builder')
        self.set_parameters([
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])

        self.declare_parameter('resolution', 0.15)
        self.declare_parameter('width_m', 120.0)
        self.declare_parameter('height_m', 120.0)
        self.declare_parameter('height_backstop', 0.22)  # m above local ground -> occupied
        self.declare_parameter('publish_rate', 4.0)
        self.declare_parameter('depth_stride', 3)

        self.res = self.get_parameter('resolution').value
        self.w_m = self.get_parameter('width_m').value
        self.h_m = self.get_parameter('height_m').value
        self.height_backstop = self.get_parameter('height_backstop').value
        self.stride = self.get_parameter('depth_stride').value

        self.grid_w = int(self.w_m / self.res)
        self.grid_h = int(self.h_m / self.res)
        self.origin_x = -self.w_m / 2.0
        self.origin_y = -self.h_m / 2.0
        self.grid = np.full((self.grid_h, self.grid_w), UNKNOWN, dtype=np.int8)

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.camera_k = None
        self.create_subscription(CameraInfo, '/camera/rgb_camera/camera_info', self.info_cb, 10)

        mask_sub = Subscriber(self, Image, '/seg/mask')
        depth_sub = Subscriber(self, Image, '/camera/depth/depth/image_raw')
        self.sync = ApproximateTimeSynchronizer([mask_sub, depth_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.perception_cb)

        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/costmap', qos)
        self.create_timer(1.0 / self.get_parameter('publish_rate').value, self.publish_costmap)

        self.get_logger().info(
            f"Costmap builder active: {self.grid_w}x{self.grid_h} cells @ {self.res}m")

    def info_cb(self, msg):
        self.camera_k = np.array(msg.k).reshape((3, 3))

    def world_to_cell(self, x, y):
        gx = ((x - self.origin_x) / self.res).astype(np.int64)
        gy = ((y - self.origin_y) / self.res).astype(np.int64)
        return gx, gy

    def perception_cb(self, mask_msg, depth_msg):
        if self.camera_k is None:
            return
        try:
            trans = self.tf_buffer.lookup_transform(
                'odom', depth_msg.header.frame_id, rclpy.time.Time(), timeout=Duration(seconds=0.05))
            robot_trans = self.tf_buffer.lookup_transform(
                'odom', 'base_footprint', rclpy.time.Time(), timeout=Duration(seconds=0.05))
        except Exception:
            return

        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        mask = self.bridge.imgmsg_to_cv2(mask_msg, desired_encoding='mono8')

        s = self.stride
        depth_s = depth[::s, ::s]
        mask_s = mask[::s, ::s]
        h, w = depth_s.shape
        vs, us = np.mgrid[0:h, 0:w]
        us = us * s
        vs = vs * s

        fx, fy = self.camera_k[0, 0], self.camera_k[1, 1]
        cx, cy = self.camera_k[0, 2], self.camera_k[1, 2]

        valid = np.isfinite(depth_s) & (depth_s > 0.05) & (depth_s < 25.0)
        z_cam = depth_s[valid]
        x_cam = (us[valid] - cx) * z_cam / fx
        y_cam = (vs[valid] - cy) * z_cam / fy
        cls = mask_s[valid]
        pts_cam = np.stack([x_cam, y_cam, z_cam], axis=1)

        t = trans.transform.translation
        q = trans.transform.rotation
        rot = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        pts_world = pts_cam @ rot.T + np.array([t.x, t.y, t.z])

        ground_z = robot_trans.transform.translation.z
        gx, gy = self.world_to_cell(pts_world[:, 0], pts_world[:, 1])
        in_bounds = (gx >= 0) & (gx < self.grid_w) & (gy >= 0) & (gy < self.grid_h)

        gx, gy, cls, height = gx[in_bounds], gy[in_bounds], cls[in_bounds], pts_world[in_bounds, 2] - ground_z

        free_pts = (cls == TRAVERSABLE) & (height < self.height_backstop)
        occ_pts = (cls == NON_TRAVERSABLE) | (height >= self.height_backstop)

        # Clear free space seen by depth/BiSeNet
        self.grid[gy[free_pts], gx[free_pts]] = FREE
        # Mark obstacles seen by depth/BiSeNet
        self.grid[gy[occ_pts], gx[occ_pts]] = OCCUPIED

        # Always keep robot's immediate footprint clear
        rx, ry = robot_trans.transform.translation.x, robot_trans.transform.translation.y
        rgx, rgy = self.world_to_cell(np.array([rx]), np.array([ry]))
        if 0 <= rgx[0] < self.grid_w and 0 <= rgy[0] < self.grid_h:
            r_mask = (np.abs(np.arange(self.grid_w) - rgx[0]) <= 2)[:, None] & \
                     (np.abs(np.arange(self.grid_h) - rgy[0]) <= 2)[None, :]
            self.grid[r_mask.T] = FREE

    def scan_cb(self, msg):
        try:
            trans = self.tf_buffer.lookup_transform(
                'odom', msg.header.frame_id, rclpy.time.Time(), timeout=Duration(seconds=0.05))
        except Exception:
            return

        t = trans.transform.translation
        q = trans.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        ranges, angles = ranges[valid], angles[valid]

        # Raytrace free space from sensor origin to hit distance
        for r, a in zip(ranges[::4], angles[::4]):
            world_a = a + yaw
            n_steps = max(2, int(r / self.res))
            dists = np.linspace(0.1, r - self.res, n_steps)
            xs = t.x + dists * math.cos(world_a)
            ys = t.y + dists * math.sin(world_a)
            rgx, rgy = self.world_to_cell(xs, ys)
            in_b = (rgx >= 0) & (rgx < self.grid_w) & (rgy >= 0) & (rgy < self.grid_h)
            self.grid[rgy[in_b], rgx[in_b]] = FREE

            # Mark hit point occupied
            hit_x = t.x + r * math.cos(world_a)
            hit_y = t.y + r * math.sin(world_a)
            hgx, hgy = self.world_to_cell(np.array([hit_x]), np.array([hit_y]))
            if 0 <= hgx[0] < self.grid_w and 0 <= hgy[0] < self.grid_h:
                self.grid[hgy[0], hgx[0]] = OCCUPIED

    def publish_costmap(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.info.resolution = self.res
        msg.info.width = self.grid_w
        msg.info.height = self.grid_h
        origin = Pose()
        origin.position.x = self.origin_x
        origin.position.y = self.origin_y
        msg.info.origin = origin
        msg.data = self.grid.flatten().tolist()
        self.costmap_pub.publish(msg)


def main():
    rclpy.init()
    node = CostmapBuilder()
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
