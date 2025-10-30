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
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped


def transform_to_mat44(tf_msg):
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1-2*(y*y+z*z),   2*(x*y - z*w),   2*(x*z + y*w)],
        [  2*(x*y + z*w), 1-2*(x*x+z*z),   2*(y*z - x*w)],
        [  2*(x*z - y*w),   2*(y*z + x*w), 1-2*(x*x+y*y)]
    ], dtype=np.float64)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3,  3] = [t.x, t.y, t.z]
    return T
def rotmat_to_rpy_zyx(R):
    # returns roll, pitch, yaw (ZYX convention)
    sy = -R[2,0]
    if abs(sy) < 1.0:
        pitch = np.arcsin(sy)
        roll  = np.arctan2(R[2,1], R[2,2])
        yaw   = np.arctan2(R[1,0], R[0,0])
    else:
        pitch = np.pi/2 * np.sign(sy)
        roll  = np.arctan2(-R[0,1], -R[0,2])
        yaw   = 0.0
    return roll, pitch, yaw
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
        self.declare_parameter('detected_tag_frame', 'first_gear_apriltag_detected')        
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
        # self.cam_frame = self.get_parameter('camera_optical_frame').value
        # self.cam_frame = self.get_parameter('sim_cam/camera_link/camera').value
        # self.cam_frame = 'sim_cam/camera_link/camera'
        self.declare_parameter('world_frame', 'world')
        self.world_frame = self.get_parameter('world_frame').value
        self.cam_frame = 'camera_link'
        self.tag_frame = self.get_parameter('detected_tag_frame').value

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
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_pub = self.create_publisher(PoseStamped, '/black_tag/pose', 10)
        self.pose_world_pub = self.create_publisher(PoseStamped, '/black_tag/pose_world', 10)  # <-- add this
        self.tfbr = TransformBroadcaster(self)
    def _publish_world_pose(self, msg_stamp, R_cam_tag, t_cam_tag):
        # 1) Build T_cam_tag
        T_cam_tag = np.eye(4)
        T_cam_tag[:3,:3] = R_cam_tag
        T_cam_tag[:3, 3] = t_cam_tag.reshape(3)

        # 2) Get T_world_cam (use your world/base frame & your camera optical frame)
        target_world = 'peg_board'  # or 'map' / your base frame
        cam_frame    = self.cam_frame  # e.g., 'camera_color_optical_frame'
        try:
            tf_w_c = self.tf_buffer.lookup_transform(self.world_frame, self.cam_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'No TF {target_world}->{cam_frame}: {e}')
            return

        T_w_c = transform_to_mat44(tf_w_c)

        # 3) Compose
        T_w_tag = T_w_c @ T_cam_tag
        R_w_tag = T_w_tag[:3,:3]
        t_w_tag = T_w_tag[:3, 3]

        # 4) Convert to rpy + quaternion and publish
        roll, pitch, yaw = rotmat_to_rpy_zyx(R_w_tag)
        qx, qy, qz, qw = rotmat_to_quaternion(R_w_tag)

        ps = PoseStamped()
        ps.header.stamp = msg_stamp
        ps.header.frame_id = target_world
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = t_w_tag.tolist()
        ps.pose.orientation.x = qx; ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz; ps.pose.orientation.w = qw
        self.pose_pub.publish(ps)

        # Optional: log RPY
        self.get_logger().info(
            f"tag in {target_world}: "
            f"xyz=({t_w_tag[0]:.3f},{t_w_tag[1]:.3f},{t_w_tag[2]:.3f}) "
            f"rpy=({roll:.3f},{pitch:.3f},{yaw:.3f})"
        )
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

        # Threshold for “black” squares 
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

        R, _ = cv2.Rodrigues(rvec)
        qx, qy, qz, qw = rotmat_to_quaternion(R)

        # ---------- ADD: world-frame composition ----------
        # Build T_cam_tag from solvePnP
        T_cam_tag = np.eye(4, dtype=np.float64)
        T_cam_tag[:3, :3] = R
        T_cam_tag[:3,  3] = tvec.reshape(3)

        # Lookup world->camera (make sure self.world_frame and self.cam_frame exist in TF)
        try:
            tf_w_c = self.tf_buffer.lookup_transform(self.world_frame, self.cam_frame, rclpy.time.Time())
        except Exception as e:
            # If TF not ready yet, just skip this frame
            if not hasattr(self, "_warned_tf"):
                self.get_logger().warn(f"No TF {self.world_frame}->{self.cam_frame}: {e}")
                self._warned_tf = True
            # still publish camera-frame pose if you want; then return or continue
            pass
        else:
            T_w_c   = transform_to_mat44(tf_w_c)
            T_w_tag = T_w_c @ T_cam_tag
            R_w_tag = T_w_tag[:3, :3]
            t_w_tag = T_w_tag[:3,  3]

            qx_w, qy_w, qz_w, qw_w = rotmat_to_quaternion(R_w_tag)

            # Publish world-frame pose
            psw = PoseStamped()
            psw.header.stamp = msg.header.stamp
            psw.header.frame_id = self.world_frame
            psw.pose.position.x, psw.pose.position.y, psw.pose.position.z = t_w_tag.tolist()
            psw.pose.orientation.x = qx_w; psw.pose.orientation.y = qy_w
            psw.pose.orientation.z = qz_w; psw.pose.orientation.w = qw_w
            self.pose_world_pub.publish(psw)

            # Broadcast TF for the detected tag in world
            tfw = TransformStamped()
            tfw.header = psw.header
            tfw.child_frame_id = self.tag_frame            # e.g., 'first_gear_apriltag_detected'
            tfw.transform.translation.x = psw.pose.position.x
            tfw.transform.translation.y = psw.pose.position.y
            tfw.transform.translation.z = psw.pose.position.z
            tfw.transform.rotation = psw.pose.orientation
            self.tfbr.sendTransform(tfw)
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
