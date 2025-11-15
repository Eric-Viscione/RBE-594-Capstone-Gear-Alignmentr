#!/usr/bin/env python3
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
)

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped

from .tag_pose_estimator import TagPoseEstimator, DetectorConfig


class MinimalTagPose(Node):
    """
    Thin ROS2 wrapper around TagPoseEstimator:
      - Subscribes to image + CameraInfo
      - Uses TagPoseEstimator to compute tag pose
      - Publishes /tag_pose_cam and /tag_pose_world
    """

    def __init__(self):
        super().__init__('minimal_tag_pose')
        self.get_logger().info(f"RUNNING FILE: {__file__}")
        default_cfg = DetectorConfig()
        # ---- Parameters ----
        default_cam_topic = f"/tag_pose_cam"
        default_world_topic = f"/tag_pose_world"
        self.declare_parameter('long_axis_cam_topic', '/tag_long_axis/cam')
        self.declare_parameter('long_axis_world_topic', '/tag_long_axis/world')
        self.declare_parameter('pose_topic_cam', default_cam_topic)
        self.declare_parameter('pose_topic_world', default_world_topic)
        self.declare_parameter('image_topic', '/camera_feed/image')
        self.declare_parameter('camera_info_topic', '/camera_feed/camera_info')
        self.declare_parameter('camera_optical_frame', 'camera_color_optical_frame')
        self.declare_parameter('color_mode', default_cfg.color_mode)
        self.declare_parameter('hsv_low', list(default_cfg.hsv_low)) 
        self.declare_parameter('hsv_high', list(default_cfg.hsv_high))
        self.declare_parameter('rect_w', 0.02)   # short side
        self.declare_parameter('rect_h', 0.05)   # long side
        self.declare_parameter('debug_images_dir', '~/RBE594_ws/debug_images')
        self.declare_parameter('save_debug_images', True)
        self.declare_parameter('debug_save_every_n', 5)

        self.image_topic =          self.get_parameter('image_topic').value
        color_mode =                self.get_parameter('color_mode').value
        hsv_low =                   self.get_parameter('hsv_low').value
        hsv_high =                  self.get_parameter('hsv_high').value
        self.pose_topic_cam =       self.get_parameter('pose_topic_cam').value       
        self.pose_topic_world =     self.get_parameter('pose_topic_world').value
        self.long_axis_cam_topic = self.get_parameter('long_axis_cam_topic').value
        self.long_axis_world_topic = self.get_parameter('long_axis_world_topic').value   
        self.image_topic = self.get_parameter('image_topic').value
        self.caminfo_topic = self.get_parameter('camera_info_topic').value
        self.cam_frame = self.get_parameter('camera_optical_frame').value

        subfolder_name = self.get_name().split('_')[0]
        rect_w = float(self.get_parameter('rect_w').value)
        rect_h = float(self.get_parameter('rect_h').value)

        debug_dir = self.get_parameter('debug_images_dir').value
        save_debug = bool(self.get_parameter('save_debug_images').value)
        debug_every = int(self.get_parameter('debug_save_every_n').value)

        # ---- Camera intrinsics state ----
        self.bridge = CvBridge()
        self.have_caminfo = False
        self.K = np.eye(3, dtype=np.float64)
        self.D = np.zeros(5, dtype=np.float64)

        R_world_cam = np.diag([1.0, 1.0, -1.0]).astype(np.float64)
        t_world_cam = np.array([0.0, 0.0, 2.0], dtype=np.float64)

        cfg = DetectorConfig(
                rect_w=rect_w,
                rect_h=rect_h,
                R_world_cam=R_world_cam,
                t_world_cam=t_world_cam,
                color_mode=color_mode,
                hsv_low=tuple(hsv_low),
                hsv_high=tuple(hsv_high))
        
        self.estimator = TagPoseEstimator(
            logger=self.get_logger(),
            config=cfg,
            debug_dir=debug_dir,
            subfolder_name=subfolder_name,
            save_debug=save_debug,
            debug_every=debug_every,
        )

        # ---- Publishers ----
        self.pose_cam_pub = self.create_publisher(PoseStamped, self.pose_topic_cam, 10)
        self.pose_world_pub = self.create_publisher(PoseStamped, self.pose_topic_world, 10)
        self.long_axis_cam_pub = self.create_publisher(PoseStamped, self.long_axis_cam_topic, 10)
        self.long_axis_world_pub = self.create_publisher(PoseStamped, self.long_axis_world_topic, 10)

        self.get_logger().info(f"Publishing long axis to {self.long_axis_cam_topic} and {self.long_axis_world_topic}")
        
        self.get_logger().info(f"Publishing to {self.pose_topic_cam} and {self.pose_topic_world}")

        # ---- Subscriptions ----
        caminfo_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(CameraInfo, self.caminfo_topic, self._caminfo_cb, caminfo_qos)
        self.create_subscription(Image, self.image_topic, self._image_cb, qos_profile_sensor_data)

        self.get_logger().info(f"Listening to {self.image_topic} and {self.caminfo_topic}")

    # ---------- CameraInfo callback ----------

    def _caminfo_cb(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64).reshape(-1)
        self.cam_frame = msg.header.frame_id
        if not self.have_caminfo:
            self.get_logger().info(f"CameraInfo received. frame={self.cam_frame}\nK=\n{self.K}")
        self.have_caminfo = True

    # ---------- Image callback ----------

    def _image_cb(self, msg: Image):
        if not self.have_caminfo:
            return

        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        ps_cam, ps_world, ps_long_cam, ps_long_world = self.estimator.process_frame(
            img_bgr=img,
            K=self.K,
            D=self.D,
            cam_frame=self.cam_frame,
            stamp=msg.header.stamp,
        )

        if ps_cam is None or ps_world is None:
            return

        # publish
        self.pose_cam_pub.publish(ps_cam)
        self.pose_world_pub.publish(ps_world)
        self.long_axis_cam_pub.publish(ps_long_cam)
        self.long_axis_world_pub.publish(ps_long_world)


def main():
    rclpy.init()
    rclpy.spin(MinimalTagPose())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
