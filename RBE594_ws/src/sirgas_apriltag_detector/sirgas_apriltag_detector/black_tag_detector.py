#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data 
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import Image as ImageMsg




def rotmat_to_quaternion(R):
    # R: 3x3 rotation matrix -> (x, y, z, w)
    m00,m01,m02 = R[0]; m10,m11,m12 = R[1]; m20,m21,m22 = R[2]
    tr = m00 + m11 + m22
    if tr > 0:
        S = (tr + 1.0) ** 0.5 * 2.0
        qw = 0.25 * S
        qx = (m21 - m12) / S
        qy = (m02 - m20) / S
        qz = (m10 - m01) / S
    elif (m00 > m11) and (m00 > m22):
        S = (1.0 + m00 - m11 - m22) ** 0.5 * 2.0
        qw = (m21 - m12) / S
        qx = 0.25 * S
        qy = (m01 + m10) / S
        qz = (m02 + m20) / S
    elif m11 > m22:
        S = (1.0 + m11 - m00 - m22) ** 0.5 * 2.0
        qw = (m02 - m20) / S
        qx = (m01 + m10) / S
        qy = 0.25 * S
        qz = (m12 + m21) / S
    else:
        S = (1.0 + m22 - m00 - m11) ** 0.5 * 2.0
        qw = (m10 - m01) / S
        qx = (m02 + m20) / S
        qy = (m12 + m21) / S
        qz = 0.25 * S
    return (qx, qy, qz, qw)

def order_corners(pts):
    c = pts.mean(axis=0)
    ang = np.arctan2(pts[:,1]-c[1], pts[:,0]-c[0])
    pts = pts[np.argsort(ang)]
    start = np.argmin(pts.sum(axis=1))
    pts = np.roll(pts, -start, axis=0)
    # enforce clockwise TL,TR,BR,BL
    v1, v2 = pts[1]-pts[0], pts[2]-pts[1]
    if (v1[0]*v2[1]-v1[1]*v2[0]) < 0:
        pts = np.array([pts[0], pts[3], pts[2], pts[1]])
    return pts.astype(np.float32)

class BlackTagPose(Node):
    def __init__(self):
        super().__init__('black_tag_pose')
        self.declare_parameter('image_topic', '/camera_feed/image')
        self.declare_parameter('camera_info_topic', '/camera_feed/camera_info')
        self.declare_parameter('tag_size', 0.50)  # meters
        self.declare_parameter('camera_optical_frame', 'camera_color_optical_frame')
        self.declare_parameter('tag_frame', 'first_gear_apriltag')
        
        ##debug parameters
        self.debug_pub = self.create_publisher(ImageMsg, '/black_tag/debug/image', 1)
        self.declare_parameter('debug_dir', '/tmp')
        self.debug_dir = self.get_parameter('debug_dir').value

        self.bridge = CvBridge()
        fx, fy = 554.2547, 554.2547
        cx, cy = 320.0, 240.0
        self.K = np.array([[fx, 0.0, cx],
                           [0.0, fy, cy],
                           [0.0, 0.0, 1.0]], dtype=np.float64)
        self.D = np.zeros(5, dtype=np.float64)   # no distortion in sim

        self.tag_size = float(self.get_parameter('tag_size').value)
        self.cam_frame = self.get_parameter('camera_optical_frame').value
        self.tag_frame = self.get_parameter('tag_frame').value

        self.pose_pub = self.create_publisher(PoseStamped, '/black_tag/pose', 10)
        self.tfbr = TransformBroadcaster(self)
        self.create_subscription(CameraInfo,
            self.get_parameter('camera_info_topic').value, self._caminfo_cb, qos_profile_sensor_data)

        self.create_subscription(Image,
            self.get_parameter('image_topic').value, self._image_cb, qos_profile_sensor_data)

        s = self.tag_size/2.0
        self.obj = np.array([[-s,-s,0],[ s,-s,0],[ s, s,0],[-s, s,0]], dtype=np.float32)

        self.get_logger().info('black_tag_detector: using HARD-CODED intrinsics (fx,fy,cx,cy).')
        self.get_logger().info(f"image_topic param: {self.get_parameter('image_topic').value}")
        self.get_logger().info(f"camera_info_topic param: {self.get_parameter('camera_info_topic').value}")
    def _caminfo_cb(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3,3)
        self.D = np.array(msg.d, dtype=np.float64).reshape(-1)

    def _image_cb(self, msg: Image):
        if not hasattr(self, '_seen_img'):
            self.get_logger().info(f"image received (encoding={msg.encoding}, {msg.width}x{msg.height})")
            self._seen_img = True
        # if self.K is None:
        #     return
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Threshold for “black” squares (tune 20..50 as needed)
        _, bw = cv2.threshold(gray, 30, 255, cv2.THRESH_BINARY_INV)
        bw = cv2.medianBlur(bw, 5)

        cnts, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            print("No contours")
            return

        best = None; best_area = 0.0
        for c in cnts:
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.02*peri, True)
            if len(approx) != 4 or not cv2.isContourConvex(approx):
                continue
            area = cv2.contourArea(approx)
            if area < 200:
                continue
            rect = cv2.minAreaRect(approx)
            w, h = rect[1]
            if min(w,h) <= 0:
                continue
            ratio = max(w,h)/max(1e-6, min(w,h))
            if ratio < 1.3 and area > best_area:
                best = approx.reshape(-1,2).astype(np.float32)
                best_area = area

        if best is None:
            return

        img_pts = order_corners(best)
        try:
            ok, rvec, tvec = cv2.solvePnP(self.obj, img_pts, self.K, self.D,
                                          flags=cv2.SOLVEPNP_IPPE_SQUARE)
        except Exception:
            ok, rvec, tvec = cv2.solvePnP(self.obj, img_pts, self.K, self.D,
                                          flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return

        R,_ = cv2.Rodrigues(rvec)
        T = np.eye(4); T[:3,:3] = R; T[:3,3] = tvec.ravel()
        R,_ = cv2.Rodrigues(rvec)
        qx, qy, qz, qw = rotmat_to_quaternion(R)    

        ps = PoseStamped()
        ps.header.stamp = msg.header.stamp
        ps.header.frame_id = self.cam_frame
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = tvec.ravel().tolist()
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = qx,qy,qz,qw
        
        self.pose_pub.publish(ps)

        tf = TransformStamped()
        tf.header = ps.header
        tf.child_frame_id = self.tag_frame
        tf.transform.translation.x = ps.pose.position.x
        tf.transform.translation.y = ps.pose.position.y
        tf.transform.translation.z = ps.pose.position.z
        tf.transform.rotation = ps.pose.orientation
        self.tfbr.sendTransform(tf)
        # Log contours only once (first frame we see contours)
        if not hasattr(self, "_dumped_contours"):
            self._dumped_contours = True
            self.get_logger().info(f"found {len(cnts)} raw contours; logging top 10 by area")
            # sort largest first
            cnts_sorted = sorted(cnts, key=cv2.contourArea, reverse=True)[:10]
            for i, c in enumerate(cnts_sorted):
                area = cv2.contourArea(c)
                peri = cv2.arcLength(c, True)
                approx = cv2.approxPolyDP(c, 0.02*peri, True)
                rect = cv2.minAreaRect(c)       # ((cx,cy),(w,h),angle)
                (cx, cy), (w, h), angle = rect
                self.get_logger().info(
                    f"[c{i}] area={area:.1f} peri={peri:.1f} verts={len(approx)} "
                    f"rect: center=({cx:.1f},{cy:.1f}) size=({w:.1f},{h:.1f}) angle={angle:.1f}"
                )
                # if quad, print the 4 points
                if len(approx) == 4:
                    pts = approx.reshape(-1,2)
                    self.get_logger().info(
                        f"[c{i}] quad pts: " +
                        ", ".join([f"({float(x):.1f},{float(y):.1f})" for (x,y) in pts])
                    )
def main():
    rclpy.init()
    rclpy.spin(BlackTagPose())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
