#!/usr/bin/env python3
# front_cam_bridge.py
# 把 NX 前置相机接入 rviz:
#   1) 订阅 /front_camera/image_compressed (jpeg, best_effort, 低带宽过 wifi)
#   2) PC 本地 cv2 解码成 raw Image 发布到 /front_camera/view (local, 不占 wifi)
#   3) 发布 /front_camera/camera_info (K 来自 /ota/calibration_results.yaml)
# rviz 里 Image display 直接指向 /front_camera/view (transport=raw)。
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

CAM_TOPIC = "/front_camera/image_compressed"
CAM_INFO_TOPIC = "/front_camera/camera_info"
VIEW_TOPIC = "/front_camera/view"

# 来自 /ota/calibration_results.yaml (front_camera, 1920x1080)
K = [794.1894973587571, 0.0, 943.773506082167,
     0.0, 792.7505146085679, 537.671363469499,
     0.0, 0.0, 1.0]
D = [-0.10775359962782109, 0.012524171396792294,
     -0.00012337856148119088, -0.0001484348474226288,
     -0.00016140441379733532]
WIDTH, HEIGHT = 1920, 1080


class FrontCamBridge(Node):
    def __init__(self):
        super().__init__("front_cam_bridge")
        best = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                          durability=DurabilityPolicy.VOLATILE,
                          history=HistoryPolicy.KEEP_LAST)
        # raw 输出与 camera_info 用 RELIABLE(对 rviz 的 raw 订阅和 transient 都兼容)
        reliable = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE,
                              history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(CompressedImage, CAM_TOPIC,
                                            self.on_image, best)
        self.pub = self.create_publisher(Image, VIEW_TOPIC, reliable)
        self.info_pub = self.create_publisher(
            CameraInfo, CAM_INFO_TOPIC,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.cam_info = self._build_cam_info()
        self.timer = self.create_timer(1.0, self.publish_info)
        self.frames = 0
        self.get_logger().info("bridge ready: %s -> %s (decode+raw)" % (CAM_TOPIC, VIEW_TOPIC))

    def _build_cam_info(self):
        ci = CameraInfo()
        ci.header.frame_id = "front_camera"
        ci.width = WIDTH
        ci.height = HEIGHT
        ci.distortion_model = "plumb_bob"
        ci.d = list(D)
        ci.k = list(K)
        ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        ci.p = [K[0], 0.0, K[2], 0.0,
                0.0, K[4], K[5], 0.0,
                0.0, 0.0, 1.0, 0.0]
        return ci

    def on_image(self, msg):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        except Exception:
            return
        if bgr is None:
            return
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        im = Image()
        im.header.stamp = self.get_clock().now().to_msg()
        im.header.frame_id = "front_camera"
        im.height, im.width = rgb.shape[:2]
        im.encoding = "rgb8"
        im.is_bigendian = False
        im.step = rgb.shape[1] * 3
        im.data = rgb.tobytes()
        self.pub.publish(im)
        self.frames += 1
        if self.frames % 50 == 0:
            self.get_logger().info("decoded %d frames, size=%dx%d" % (self.frames, im.width, im.height))

    def publish_info(self):
        self.cam_info.header.stamp = self.get_clock().now().to_msg()
        self.info_pub.publish(self.cam_info)


def main():
    rclpy.init()
    rclpy.spin(FrontCamBridge())
    rclpy.shutdown()


if __name__ == "__main__":
    main()