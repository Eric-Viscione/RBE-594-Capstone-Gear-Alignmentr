#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import apriltag
import numpy as np
import inspect


class AprilTag(Node):
    """
    Subscribes to /camera_feed/image, detects AprilTags,
    """

    def __init__(self):
        super().__init__('apriltag')
        self.bridge = CvBridge()

        desired_opts = { ##options to define the detector and tags
            'families': 'tag36h11',
            'nthreads': 1,
            'quad_decimate': 1.0,
            'refine_edges': True,
            'refine_decode': False,
            'refine_pose': False,
            'debug': False,
            'quad_contours': True,
            'decode_sharpening': 0.25,
            'quad_sigma': 0.0,
        }

        sig = inspect.signature(apriltag.DetectorOptions.__init__)
        supported = set(p.name for p in sig.parameters.values())
        filtered = {k: v for k, v in desired_opts.items() if k in supported}

        options = apriltag.DetectorOptions(**filtered)
        self.detector = apriltag.Detector(options)

        self.subscription = self.create_subscription(
            Image, '/camera_feed/image', self.image_cb, 10
        )
        self.get_logger().info('AprilTag detector listening on /camera_feed/image')

    def image_cb(self, msg: Image):
        try:
            try:
                gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            except Exception:
                bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

            if gray.dtype != np.uint8:
                gray = gray.astype(np.uint8, copy=False)

            detections = self.detector.detect(gray)
            print('yes' if len(detections) > 0 else 'no', flush=True)

        except Exception as e:
            # Keep node alive; just warn
            self.get_logger().warn(f'Image processing error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = AprilTag()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
