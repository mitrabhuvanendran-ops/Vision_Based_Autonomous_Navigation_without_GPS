#!/usr/bin/env python3
import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import tf2_ros
from scipy.spatial.transform import Rotation as R

OBJECT_PROFILES = {
    0: {"name": "rock", "size": (3.2, 3.0, 2.5)},
    1: {"name": "cactus", "size": (0.45, 0.45, 2.8)},
    2: {"name": "scrub", "size": (0.8, 0.8, 0.7)}
}

class AutoYoloDatasetGenerator(Node):
    def __init__(self):
        super().__init__('yolo_dataset_generator')
        
        # Enforce simulation time
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.camera_k = None
        self.img_w = 640
        self.img_h = 480
        self.frame_idx = 0
        self.saved_count = 0

        self.out_dir = os.path.expanduser('~/ugv_ws/yolo_dataset')
        self.img_dir = os.path.join(self.out_dir, 'images')
        self.lbl_dir = os.path.join(self.out_dir, 'labels')
        os.makedirs(self.img_dir, exist_ok=True)
        os.makedirs(self.lbl_dir, exist_ok=True)

        self.create_subscription(CameraInfo, '/camera/rgb_camera/camera_info', self.info_cb, 10)
        self.create_subscription(Image, '/camera/rgb_camera/image_raw', self.image_cb, 10)

        self.objects = self.generate_obstacle_database()
        self.get_logger().info(f"YOLO Dataset Generator active with sim_time enabled!")
        self.get_logger().info(f"Database contains {len(self.objects)} obstacle coordinates.")
        self.get_logger().info(f"Saving dataset into: {self.out_dir}")

    def generate_obstacle_database(self):
        import random
        random.seed(42)
        objs = []
        # Boulder positions
        for angle in np.linspace(0, 2*np.pi, 20, endpoint=False):
            r = random.uniform(22.0, 36.0)
            objs.append((0, r * np.cos(angle), r * np.sin(angle), 1.2))
        # Cacti positions
        for _ in range(25):
            r = random.uniform(14.0, 26.0)
            a = random.uniform(0, 6.28)
            objs.append((1, r * np.cos(a), r * np.sin(a), 1.4))
        # Scrub positions
        for _ in range(30):
            r = random.uniform(16.0, 35.0)
            a = random.uniform(0, 6.28)
            objs.append((2, r * np.cos(a), r * np.sin(a), 0.35))
        return objs

    def info_cb(self, msg):
        if self.camera_k is None:
            self.get_logger().info("Received CameraInfo calibration.")
        self.camera_k = np.array(msg.k).reshape((3, 3))
        self.img_w = msg.width
        self.img_h = msg.height

    def project_3d_box(self, center, size, cam_pos, cam_rot_mat):
        cx, cy, cz = center
        dx, dy, dz = size
        corners_world = [
            np.array([cx + sx*dx/2.0, cy + sy*dy/2.0, cz + sz*dz/2.0])
            for sx in [-1, 1] for sy in [-1, 1] for sz in [-1, 1]
        ]

        corners_cam = [cam_rot_mat.T @ (pt - cam_pos) for pt in corners_world]

        # Ignore if behind or closer than 0.2m to lens
        if any(c[2] <= 0.2 for c in corners_cam):
            return None

        u_pts, v_pts = [], []
        for c in corners_cam:
            proj = self.camera_k @ c
            u = proj[0] / proj[2]
            v = proj[1] / proj[2]
            u_pts.append(u)
            v_pts.append(v)

        xmin, xmax = max(0, min(u_pts)), min(self.img_w, max(u_pts))
        ymin, ymax = max(0, min(v_pts)), min(self.img_h, max(v_pts))

        # Filter out bounding boxes out of screen or smaller than 12 pixels
        if xmax <= xmin + 12 or ymax <= ymin + 12:
            return None

        x_center = ((xmin + xmax) / 2.0) / self.img_w
        y_center = ((ymin + ymax) / 2.0) / self.img_h
        width = (xmax - xmin) / self.img_w
        height = (ymax - ymin) / self.img_h

        return (x_center, y_center, width, height)

    def image_cb(self, msg):
        self.frame_idx += 1
        if self.camera_k is None or self.frame_idx % 3 != 0:
            return

        try:
            # Lookup transform with a 0.05s buffer timeout
            trans = self.tf_buffer.lookup_transform(
                'odom', 'camera_optical_link', rclpy.time.Time(), timeout=Duration(seconds=0.05))
        except Exception as e:
            return

        t = trans.transform.translation
        cam_pos = np.array([t.x, t.y, t.z])
        q = trans.transform.rotation

        r = R.from_quat([q.x, q.y, q.z, q.w])
        cam_rot = r.as_matrix()

        yolo_annotations = []
        for class_id, ox, oy, oz in self.objects:
            dist = np.linalg.norm(np.array([ox, oy, oz]) - cam_pos)
            if dist > 35.0:
                continue

            size = OBJECT_PROFILES[class_id]["size"]
            bbox = self.project_3d_box((ox, oy, oz), size, cam_pos, cam_rot)
            if bbox is not None:
                yolo_annotations.append(f"{class_id} {bbox[0]:.6f} {bbox[1]:.6f} {bbox[2]:.6f} {bbox[3]:.6f}")

        if len(yolo_annotations) > 0:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            img_file = os.path.join(self.img_dir, f"img_{self.saved_count:05d}.jpg")
            lbl_file = os.path.join(self.lbl_dir, f"img_{self.saved_count:05d}.txt")

            cv2.imwrite(img_file, cv_img)
            with open(lbl_file, 'w') as f:
                f.write("\n".join(yolo_annotations))

            self.get_logger().info(f"Captured #{self.saved_count:04d} -> {len(yolo_annotations)} objects labeled.")
            self.saved_count += 1

def main():
    rclpy.init()
    node = AutoYoloDatasetGenerator()
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
