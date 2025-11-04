#!/usr/bin/env python3
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped


# ---------- tiny math helpers ----------
def rotmat_to_quat(R):
    # 3x3 rotation -> (x,y,z,w)
    m00,m01,m02 = R[0]; m10,m11,m12 = R[1]; m20,m21,m22 = R[2]
    tr = m00 + m11 + m22
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (m21 - m12) / S
        qy = (m02 - m20) / S
        qz = (m10 - m01) / S
    elif m00 > m11 and m00 > m22:
        S = np.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / S
        qx = 0.25 * S
        qy = (m01 + m10) / S
        qz = (m02 + m20) / S
    elif m11 > m22:
        S = np.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / S
        qx = (m01 + m10) / S
        qy = 0.25 * S
        qz = (m12 + m21) / S
    else:
        S = np.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / S
        qx = (m02 + m20) / S
        qy = (m12 + m21) / S
        qz = 0.25 * S
    return qx,qy,qz,qw

def order_corners(pts):
    # Ensure TL, TR, BR, BL order clockwise
    c = pts.mean(axis=0)
    ang = np.arctan2(pts[:,1]-c[1], pts[:,0]-c[0])
    pts = pts[np.argsort(ang)]
    start = np.argmin(pts.sum(axis=1))
    pts = np.roll(pts, -start, axis=0)
    v1, v2 = pts[1]-pts[0], pts[2]-pts[1]
    if (v1[0]*v2[1]-v1[1]*v2[0]) < 0:
        pts = np.array([pts[0], pts[3], pts[2], pts[1]])
    return pts.astype(np.float32)
# ---------------------------------------


class MinimalTagPose(Node):
    """
    Subscribes:
      - image_topic (Image)
      - camera_info_topic (CameraInfo)
    Publishes:
      - /tag_pose_cam (PoseStamped)   # pose of tag in camera optical frame
    Parameters:
      - image_topic (default: /camera_feed/image)
      - camera_info_topic (default: /camera_feed/camera_info)
      - tag_size [meters] (default: 0.05)   # edge length of your square tag
      - camera_optical_frame (default: camera_color_optical_frame)
    """

    def __init__(self):
        super().__init__('minimal_tag_pose')
        self.declare_parameter('image_topic', '/camera_feed/image')
        self.declare_parameter('camera_info_topic', '/camera_feed/camera_info')
        self.declare_parameter('tag_size', 0.05)  # 5 cm default
        self.declare_parameter('camera_optical_frame', 'camera_color_optical_frame')

        self.image_topic = self.get_parameter('image_topic').value
        self.caminfo_topic = self.get_parameter('camera_info_topic').value
        self.tag_size = float(self.get_parameter('tag_size').value)
        self.cam_frame = self.get_parameter('camera_optical_frame').value

        self.bridge = CvBridge()
        self.have_caminfo = False
        self.K = np.eye(3, dtype=np.float64)
        self.D = np.zeros(5, dtype=np.float64)

        # 3D model points for a square lying on Z=0 in tag frame (units: meters)
        s = self.tag_size / 2.0
        self.obj_pts = np.array([[-s,-s,0],[ s,-s,0],[ s, s,0],[-s, s,0]], dtype=np.float32)

        # Publisher: pose in camera frame
        self.pose_pub = self.create_publisher(PoseStamped, '/tag_pose_cam', 10)

        # CameraInfo: use RELIABLE QoS
        caminfo_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(CameraInfo, self.caminfo_topic, self._caminfo_cb, caminfo_qos)

        # Image: sensor data QoS
        self.create_subscription(Image, self.image_topic, self._image_cb, qos_profile_sensor_data)

        self.get_logger().info(f"Listening to {self.image_topic} and {self.caminfo_topic}")

    # ---- Camera intrinsics handler ----
    def _caminfo_cb(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3,3)
        self.D = np.array(msg.d, dtype=np.float64).reshape(-1)
        if not self.have_caminfo:
            self.get_logger().info(f"CameraInfo received.\nK=\n{self.K}")
        self.have_caminfo = True

    # ---- Image handler: detect square, PnP, publish pose ----
    def _image_cb(self, msg: Image):
        if not self.have_caminfo:
            return

        # 1) BGR -> Gray
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # 2) Simple, robust thresholding
        bw = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 15, 5
        )

        # 3) Find contours and pick the best square
        cnts, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        best_area = 0.0
        for c in cnts:
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.01*peri, True)  # tighter epsilon keeps corners
            if len(approx) != 4 or not cv2.isContourConvex(approx):
                continue
            area = cv2.contourArea(approx)
            (w, h) = cv2.minAreaRect(approx)[1]
            if min(w, h) <= 0:
                continue
            ratio = max(w, h) / max(1e-6, min(w, h))  # allow some skew
            if area > best_area and ratio < 10.0 and area > 5:
                best = approx.reshape(-1, 2).astype(np.float32)
                best_area = area

        if best is None:
            return

        img_pts = order_corners(best)

        # 4) PnP for pose in camera frame
        try:
            ok, rvec, tvec = cv2.solvePnP(self.obj_pts, img_pts, self.K, self.D,
                                          flags=cv2.SOLVEPNP_IPPE_SQUARE)
        except Exception:
            ok, rvec, tvec = cv2.solvePnP(self.obj_pts, img_pts, self.K, self.D,
                                          flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return

        R, _ = cv2.Rodrigues(rvec)
        qx,qy,qz,qw = rotmat_to_quat(R)

        # 5) Publish PoseStamped in the camera optical frame
        ps = PoseStamped()
        ps.header.stamp = msg.header.stamp
        ps.header.frame_id = self.cam_frame
        ps.pose.position.x = float(tvec[0])
        ps.pose.position.y = float(tvec[1])
        ps.pose.position.z = float(tvec[2])
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        self.pose_pub.publish(ps)
        # (Optional tiny log)
        self.get_logger().info(
            f"tag@cam: xyz=({ps.pose.position.x:.3f},{ps.pose.position.y:.3f},{ps.pose.position.z:.3f})"
        )


def main():
    rclpy.init()
    rclpy.spin(MinimalTagPose())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
