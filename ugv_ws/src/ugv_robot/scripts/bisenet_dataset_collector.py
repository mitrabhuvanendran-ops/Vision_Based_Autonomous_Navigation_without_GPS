#!/usr/bin/env python3
"""Auto-labeling dataset generator for BiSeNet fine-tuning (Stage 4.1).

Ground truth comes from parsing the actual world SDF file's <include> poses
(not a hand-guessed obstacle database, and not the gazebo_ros_state topic,
which this Gazebo Classic build does not reliably publish on). Each known
obstacle's real 3D collision footprint is projected into the camera image
with the camera's real intrinsics + TF pose, and filled to produce a dense
2-class (traversable / non-traversable) mask. Depth is used to void out
sky pixels (no return within sensor range).
"""
import math
import os
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.duration import Duration
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image

TRAVERSABLE = 0
NON_TRAVERSABLE = 1
VOID = 255

# Real collision footprints pulled from each model's model.sdf (Stage 4.1),
# expressed as a local-frame axis-aligned box: full size (dx, dy, dz) and the
# z-offset of the box center relative to the model's <pose> origin.
OBSTACLE_PROFILES = {
    'canyon_boulder_large': {'size': (7.0, 6.0, 5.0), 'z_offset': 2.5},
    'canyon_boulder_med': {'size': (3.2, 2.8, 2.4), 'z_offset': 1.2},
    'saguaro_cactus': {'size': (1.0, 1.0, 3.0), 'z_offset': 1.4},
    'desert_scrub': {'size': (1.1, 1.1, 1.0), 'z_offset': 0.4},
    'bush': {'size': (1.3, 1.3, 1.1), 'z_offset': 0.4},
    'pine_tree': {'size': (2.5, 2.5, 6.0), 'z_offset': 3.0},
    'broadleaf_tree': {'size': (4.0, 4.0, 7.0), 'z_offset': 3.2},
    'fallen_log': {'size': (3.4, 0.4, 0.4), 'z_offset': 0.18},
    'rock_cluster': {'size': (1.4, 1.1, 0.6), 'z_offset': 0.25},
}


def parse_world_obstacles(world_path):
    """Read <include><uri>model://X</uri><pose>x y z r p y</pose></include>
    blocks from a world SDF and resolve each to a known obstacle profile.
    Unknown model types (e.g. "sun") are skipped."""
    tree = ET.parse(world_path)
    obstacles = []
    for inc in tree.getroot().iter('include'):
        uri = inc.findtext('uri', default='')
        model_type = uri.replace('model://', '').strip()
        profile = OBSTACLE_PROFILES.get(model_type)
        if profile is None:
            continue
        pose_text = inc.findtext('pose', default='0 0 0 0 0 0')
        x, y, z, roll, pitch, yaw = [float(v) for v in pose_text.split()]
        obstacles.append({
            'center': np.array([x, y, z + profile['z_offset']]),
            'size': profile['size'],
            'yaw': yaw,
        })
    return obstacles


def project_box_polygon(center, size, yaw, cam_pos, cam_rot_mat, k, img_w, img_h):
    """Project a yaw-rotated 3D box into image space and return the 2D convex
    hull of its visible corners, or None if entirely out of frame/behind."""
    dx, dy, dz = size
    local_corners = [
        np.array([sx * dx / 2.0, sy * dy / 2.0, sz * dz / 2.0])
        for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)
    ]
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    yaw_rot = np.array([[cos_y, -sin_y, 0], [sin_y, cos_y, 0], [0, 0, 1]])
    corners_world = [center + yaw_rot @ c for c in local_corners]
    corners_cam = [cam_rot_mat.T @ (pt - cam_pos) for pt in corners_world]

    if all(c[2] <= 0.2 for c in corners_cam):
        return None

    pts = []
    for c in corners_cam:
        if c[2] <= 0.2:
            continue
        proj = k @ c
        u, v = proj[0] / proj[2], proj[1] / proj[2]
        pts.append([np.clip(u, -img_w, 2 * img_w), np.clip(v, -img_h, 2 * img_h)])

    if len(pts) < 3:
        return None
    pts = np.array(pts, dtype=np.float32)
    hull = cv2.convexHull(pts)
    return hull.astype(np.int32)


class GroundTruthBiSeNetCollector(Node):
    def __init__(self):
        super().__init__('bisenet_dataset_collector')
        self.set_parameters([
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])

        self.declare_parameter('world_file', os.path.expanduser(
            '~/ugv_ws/src/ugv_terrain/worlds/forest.world'))
        world_path = self.get_parameter('world_file').value
        self.obstacles = parse_world_obstacles(world_path)
        self.get_logger().info(
            f"Loaded {len(self.obstacles)} ground-truth obstacles from {world_path}")

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.camera_k = None
        self.img_w, self.img_h = 640, 480
        self.create_subscription(CameraInfo, '/camera/rgb_camera/camera_info', self.info_cb, 10)

        self.max_range = 29.0  # matches the RGB-D clip far range; beyond this = void/sky
        self.min_dist_thresh = 0.35
        self.min_yaw_thresh = 0.26
        self.last_saved_x = None
        self.last_saved_y = None
        self.last_saved_yaw = None

        self.base_dir = os.path.expanduser('~/ugv_ws/bisenet_dataset')
        self.img_dir = os.path.join(self.base_dir, 'images')
        self.mask_dir = os.path.join(self.base_dir, 'masks')
        os.makedirs(self.img_dir, exist_ok=True)
        os.makedirs(self.mask_dir, exist_ok=True)
        self.saved_idx = 0

        rgb_sub = Subscriber(self, Image, '/camera/rgb_camera/image_raw')
        depth_sub = Subscriber(self, Image, '/camera/depth/depth/image_raw')
        self.sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=10, slop=0.05)
        self.sync.registerCallback(self.frame_cb)

        self.latest_rgb = None
        self.latest_depth = None
        self.get_logger().info("Ground-truth dataset collector active. Drive the UGV to collect frames.")

    def info_cb(self, msg):
        self.camera_k = np.array(msg.k).reshape((3, 3))
        self.img_w, self.img_h = msg.width, msg.height

    def frame_cb(self, rgb_msg, depth_msg):
        if self.camera_k is None:
            return

        try:
            trans = self.tf_buffer.lookup_transform(
                'odom', 'camera_optical_link', rclpy.time.Time(), timeout=Duration(seconds=0.05))
        except Exception:
            return

        curr_x = trans.transform.translation.x
        curr_y = trans.transform.translation.y
        q = trans.transform.rotation
        curr_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        if self.last_saved_x is not None:
            dist_moved = math.hypot(curr_x - self.last_saved_x, curr_y - self.last_saved_y)
            yaw_diff = abs(math.atan2(math.sin(curr_yaw - self.last_saved_yaw),
                                       math.cos(curr_yaw - self.last_saved_yaw)))
            if dist_moved < self.min_dist_thresh and yaw_diff < self.min_yaw_thresh:
                return

        self.last_saved_x, self.last_saved_y, self.last_saved_yaw = curr_x, curr_y, curr_yaw
        self.save_frame(rgb_msg, depth_msg, trans)

    def save_frame(self, rgb_msg, depth_msg, trans):
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')

        cam_pos = np.array([trans.transform.translation.x,
                             trans.transform.translation.y,
                             trans.transform.translation.z])
        q = trans.transform.rotation
        cam_rot = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

        mask = np.full((self.img_h, self.img_w), TRAVERSABLE, dtype=np.uint8)

        invalid_depth = ~np.isfinite(depth) | (depth <= 0.0) | (depth >= self.max_range)
        mask[invalid_depth] = VOID

        n_obstacles = 0
        for obs in self.obstacles:
            dist = np.linalg.norm(obs['center'] - cam_pos)
            if dist > 40.0:
                continue
            poly = project_box_polygon(
                obs['center'], obs['size'], obs['yaw'], cam_pos, cam_rot,
                self.camera_k, self.img_w, self.img_h)
            if poly is not None:
                cv2.fillConvexPoly(mask, poly, NON_TRAVERSABLE)
                n_obstacles += 1

        fname = f"canyon_{self.saved_idx:05d}.png"
        cv2.imwrite(os.path.join(self.img_dir, fname), rgb)
        cv2.imwrite(os.path.join(self.mask_dir, fname), mask)
        self.get_logger().info(
            f"Saved frame #{self.saved_idx:05d} pos=({cam_pos[0]:.1f},{cam_pos[1]:.1f}) "
            f"obstacles_in_view={n_obstacles}")
        self.saved_idx += 1


def main():
    rclpy.init()
    node = GroundTruthBiSeNetCollector()
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
