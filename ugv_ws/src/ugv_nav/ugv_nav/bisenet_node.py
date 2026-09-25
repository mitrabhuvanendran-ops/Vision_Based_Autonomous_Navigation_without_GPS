#!/usr/bin/env python3
"""Stage 4.3 - Runs the fine-tuned 2-class BiSeNetV2 on the live RGB feed
and publishes a same-timestamp/same-frame traversability mask."""
import os
import sys

import cv2
import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

sys.path.insert(0, os.path.expanduser('~/OFFSEG-main/Segmentation/BiSeNet'))
from lib.models.bisenetv2 import BiSeNetV2  # noqa: E402

N_CLASSES = 2
MEAN = np.array([0.485, 0.456, 0.406])
STD = np.array([0.229, 0.224, 0.225])


class BiSeNetNode(Node):
    def __init__(self):
        super().__init__('bisenet_node')
        self.set_parameters([
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])
        self.declare_parameter('model_path', os.path.expanduser(
            '~/ugv_ws/models/bisenet_sim_2class.pth'))
        self.declare_parameter('infer_size', 512)

        self.bridge = CvBridge()
        self.infer_size = self.get_parameter('infer_size').value
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'

        model_path = self.get_parameter('model_path').value
        rugd_path = os.path.expanduser('~/OFFSEG-main/Models/BiSeNet_RUGD/model_final.pth')
        self.model = None
        self.model_classes = 2

        if os.path.exists(model_path):
            self.model = BiSeNetV2(n_classes=N_CLASSES, output_aux=False).to(self.device)
            ckpt = torch.load(model_path, map_location=self.device)
            state_dict = ckpt.get('state_dict', ckpt) if isinstance(ckpt, dict) else ckpt
            self.model.load_state_dict(state_dict)
            self.model.eval()
            self.get_logger().info(f"Loaded fine-tuned BiSeNet from {model_path} on {self.device}")
        elif os.path.exists(rugd_path):
            self.model_classes = 4
            self.model = BiSeNetV2(n_classes=4, output_aux=False).to(self.device)
            ckpt = torch.load(rugd_path, map_location=self.device)
            state_dict = ckpt.get('state_dict', ckpt) if isinstance(ckpt, dict) else ckpt
            self.model.load_state_dict(state_dict)
            self.model.eval()
            self.get_logger().info(f"Loaded pretrained RUGD BiSeNet (4-class) from {rugd_path} on {self.device}")
        else:
            self.get_logger().warn(
                f"No BiSeNet weights found at {model_path} or {rugd_path}. Using fast terrain heuristic fallback.")

        self.mask_pub = self.create_publisher(Image, '/seg/mask', 10)
        self.create_subscription(Image, '/camera/rgb_camera/image_raw', self.image_cb, 10)

    def image_cb(self, msg):
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = bgr.shape[:2]

        if self.model is not None:
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            resized = cv2.resize(rgb, (self.infer_size, self.infer_size), interpolation=cv2.INTER_LINEAR)
            normed = (resized.astype(np.float32) / 255.0 - MEAN) / STD
            tensor = torch.from_numpy(normed.transpose(2, 0, 1)).float().unsqueeze(0).to(self.device)

            with torch.no_grad():
                pred = self.model(tensor)
            pred = pred.squeeze(0).byte().cpu().numpy()
            pred = cv2.resize(pred, (w, h), interpolation=cv2.INTER_NEAREST)

            if self.model_classes == 4:
                # Map RUGD 4 classes (0: traversable, >0: non-traversable/sky/obstacle)
                mask = np.where(pred == 0, np.uint8(0), np.uint8(1))
            else:
                mask = pred
        else:
            # Fallback robust terrain heuristic (boulders/rocks/cacti vs sand)
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            mask = np.zeros((h, w), dtype=np.uint8)

            # Boulders / dark rocks (low saturation + low value or distinct hue)
            sat = hsv[:, :, 1]
            val = hsv[:, :, 2]
            lap = cv2.Laplacian(gray, cv2.CV_8U)

            # High texture (edges of rocks/scrub) or dark rock shadows / green cacti
            mask[(lap > 35) | (val < 75) | ((hsv[:, :, 0] > 35) & (hsv[:, :, 0] < 85) & (sat > 40))] = 1

            # Morphological filtering to clean up noise
            mask = cv2.medianBlur(mask, 5)

        out_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        out_msg.header = msg.header  # same timestamp/frame as the RGB input
        self.mask_pub.publish(out_msg)


def main():
    rclpy.init()
    node = BiSeNetNode()
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
