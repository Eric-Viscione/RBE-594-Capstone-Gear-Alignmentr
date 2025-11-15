#!/usr/bin/env python3
import numpy as np
import cv2
import os
from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # <-- registers Pose/PoseStamped converters with tf2
# from tf2_geometry_msgs import do_transform_pose  # apt: ros-<distro>-tf2-geometry-msgs
from rclpy.time import Time
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

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
        self.get_logger().info(f"RUNNING FILE: {__file__}")
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
        ##SEts up frames for world transform
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        #Pbulsihed world pub
        self.pose_world_pub = self.create_publisher(PoseStamped, '/tag_pose_world', 10)

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
        # self.R_world_cam = np.eye(3, dtype=np.float64)
        self.R_world_cam = np.diag([1.0, 1.0, -1.0]).astype(np.float64)
        self.t_world_cam = np.array([0.0, 0.0, 2.0], dtype=np.float64)
        self.declare_parameter('debug_images_dir', '~/RBE594_ws/debug_images')
        self.declare_parameter('save_debug_images', False)          # master switch
        self.declare_parameter('debug_save_every_n', 2)            # 1 = every frame

        self.debug_dir = os.path.expanduser(self.get_parameter('debug_images_dir').value)
        self.save_debug = bool(self.get_parameter('save_debug_images').value)
        self.debug_every = int(self.get_parameter('debug_save_every_n').value)
        self._dbg_count = 0
        self.declare_parameter('rect_w', 0.02)   # tag_size  (short)
        self.declare_parameter('rect_h', 0.05)   # tag_size + extra_len (long)
        self.rect_w = float(self.get_parameter('rect_w').value)
        self.rect_h = float(self.get_parameter('rect_h').value)
        if self.save_debug:
            os.makedirs(self.debug_dir, exist_ok=True)
            self.get_logger().info(f"Saving debug images to: {self.debug_dir}")

    # ---- Camera intrinsics handler ----
    def _caminfo_cb(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3,3)
        self.D = np.array(msg.d, dtype=np.float64).reshape(-1)
        self.cam_frame = msg.header.frame_id          # <-- use the real frame name
        if not self.have_caminfo:
            self.get_logger().info(f"CameraInfo received. frame={self.cam_frame}\nK=\n{self.K}")
        self.have_caminfo = True

    # ---- Image handler: detect square, PnP, publish pose ----
    def build_rect_model_pts(self, w, h):
        sx, sy = w/2.0, h/2.0
        # TL, TR, BR, BL model corners in tag frame (Z=0)
        return np.array([[-sx,-sy,0],
                        [ sx,-sy,0],
                        [ sx, sy,0],
                        [-sx, sy,0]], dtype=np.float32)

    def pick_long_edge_from_image(self, img_pts):
        """
        img_pts must be TL,TR,BR,BL (your order_corners does that).
        We measure projected lengths under perspective:
        - TL->TR is 'image-X' edge
        - TR->BR is 'image-Y' edge
        Returns ('x' or 'y'), and the two lengths.
        """
        e_x = float(np.linalg.norm(img_pts[1] - img_pts[0]))  # TL->TR
        e_y = float(np.linalg.norm(img_pts[2] - img_pts[1]))  # TR->BR
        return ('x', e_x, e_y) if e_x >= e_y else ('y', e_x, e_y)
    def _image_cb(self, msg: Image):
        if not self.have_caminfo:
            return

        # 1) BGR -> Gray
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # normalize then blur a touch
        norm  = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX)
        blur  = cv2.medianBlur(norm, 5)

        # choose a dark cutoff. start at 60–90 and adjust.
        t_dark = 50
        bw = (blur < t_dark).astype(np.uint8) * 255

        # fill + clean
        k_close = cv2.getStructuringElement(cv2.MORPH_RECT, (13,13))
        bw_m = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, k_close, iterations=1)
        k_open  = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        bw_m = cv2.morphologyEx(bw_m, cv2.MORPH_OPEN,  k_open,  iterations=1)

        bw_m = bw_m.astype(np.uint8)

        # 3) Find contours and pick best quadrilateral (square-ish + size bounds)
        cnts, _ = cv2.findContours(bw_m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        H, W = gray.shape[:2]
        img_area = W * H

        min_area = 10          # very small for debugging
        max_area = 0.5 * img_area
        max_ratio = 20.0       # allow very long skinny bars
        overlay_all = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        best = None
        best_score = -1.0

        for c in cnts:
            area = cv2.contourArea(c)
            if area < min_area or area > max_area:
                continue

            # oriented rectangle that best fits the contour
            rect = cv2.minAreaRect(c)        # ((cx,cy),(w_rect,h_rect),theta)
            (w_rect, h_rect) = rect[1]

            if min(w_rect, h_rect) < 1e-3:
                continue

            ratio = max(w_rect, h_rect) / max(1e-6, min(w_rect, h_rect))
            self.get_logger().info(
                f"candidate: area={area:.1f}, w_rect={w_rect:.1f}, h_rect={h_rect:.1f}, aspect={ratio:.2f}"
            )

            if ratio > max_ratio:
                continue

            # get the 4 corner points of this oriented rectangle
            box = cv2.boxPoints(rect)            # 4×2 float
            box = box.astype(np.float32)

            # simple score: larger area wins
            score = area

            # draw on debug overlay
            cv2.polylines(overlay_all, [box.astype(int)], True, (0,255,255), 2)

            if score > best_score:
                best_score = score
                best = box


        if best is None:
            # optional: save negatives for debugging
            if self.save_debug:
                self._dbg_count += 1
                if self._dbg_count % self.debug_every == 0:
                    ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                    base = os.path.join(self.debug_dir, f"frame_{ts}")
                    try:
                        cv2.imwrite(base + "_orig.png", img)
                        cv2.imwrite(base + "_blur.png", blur)
                        cv2.imwrite(base + "_bw.png", bw)
                        cv2.imwrite(base + "_bw_morph.png", bw_m)
                        cv2.imwrite(base + "_candidates.png", overlay_all)
                    except Exception as e:
                        self.get_logger().warn(f"Debug save (no-quad) failed: {e}")
            return

        # Ensure TL,TR,BR,BL order
        img_pts = order_corners(best)

        # 4) Decide which IMAGE edge is longer (robust under skew)
        long_edge_img, e_x, e_y = self.pick_long_edge_from_image(img_pts)

        # 5) Choose rectangular MODEL so that TL→TR corresponds to the *longer* observed image edge
        model_x_is_long = (self.rect_w >= self.rect_h)
        w_use, h_use = self.rect_w, self.rect_h
        if (long_edge_img == 'x') != model_x_is_long:
            # swap to keep model X = long edge, Y = short edge
            w_use, h_use = self.rect_h, self.rect_w
        obj_pts = self.build_rect_model_pts(w_use, h_use)

        # 6) PnP in camera frame
        try:
            ok, rvec, tvec = cv2.solvePnP(
                obj_pts, img_pts, self.K, self.D,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
        except Exception:
            ok, rvec, tvec = cv2.solvePnP(
                obj_pts, img_pts, self.K, self.D,
                flags=cv2.SOLVEPNP_ITERATIVE
            )
        if not ok:
            return

        R, _ = cv2.Rodrigues(rvec)

        # 7) Publish pose in camera frame
        qx, qy, qz, qw = rotmat_to_quat(R)
        ps_cam = PoseStamped()
        ps_cam.header.stamp = msg.header.stamp
        ps_cam.header.frame_id = self.cam_frame
        ps_cam.pose.position.x = float(tvec[0])
        ps_cam.pose.position.y = float(tvec[1])
        ps_cam.pose.position.z = float(tvec[2])
        ps_cam.pose.orientation.x = qx
        ps_cam.pose.orientation.y = qy
        ps_cam.pose.orientation.z = qz
        ps_cam.pose.orientation.w = qw
        self.pose_pub.publish(ps_cam)
        self.get_logger().info(
            f"tag@cam: xyz=({ps_cam.pose.position.x:.3f},"
            f"{ps_cam.pose.position.y:.3f},{ps_cam.pose.position.z:.3f})"
        )

        # 8) Long-edge unit direction (camera & world)
        long_axis_cam = R[:, 0] if (w_use >= h_use) else R[:, 1]
        long_axis_world = self.R_world_cam @ long_axis_cam

        self.get_logger().info(
            f"long_edge_img={long_edge_img} (e_x={e_x:.1f}, e_y={e_y:.1f}), "
            f"model_used w={w_use:.3f} h={h_use:.3f}"
        )
        self.get_logger().info(
            f"long_axis_cam=({long_axis_cam[0]:.3f},{long_axis_cam[1]:.3f},{long_axis_cam[2]:.3f}), "
            f"long_axis_world=({long_axis_world[0]:.3f},{long_axis_world[1]:.3f},{long_axis_world[2]:.3f})"
        )

        # 9) Compose to world and publish world pose (using your fixed world←cam)
        T_world_cam = np.eye(4, dtype=np.float64)
        T_world_cam[:3, :3] = self.R_world_cam
        T_world_cam[:3, 3] = self.t_world_cam

        T_cam_tag = np.eye(4, dtype=np.float64)
        T_cam_tag[:3, :3] = R
        T_cam_tag[:3, 3] = tvec.flatten()

        T_world_tag = T_world_cam @ T_cam_tag
        R_world_tag = T_world_tag[:3, :3]
        t_world_tag = T_world_tag[:3, 3]
        qx_w, qy_w, qz_w, qw_w = rotmat_to_quat(R_world_tag)

        ps_world = PoseStamped()
        ps_world.header.stamp = msg.header.stamp
        ps_world.header.frame_id = 'world'
        ps_world.pose.position.x = float(t_world_tag[0])
        ps_world.pose.position.y = float(t_world_tag[1])
        ps_world.pose.position.z = float(t_world_tag[2])
        ps_world.pose.orientation.x = qx_w
        ps_world.pose.orientation.y = qy_w
        ps_world.pose.orientation.z = qz_w
        ps_world.pose.orientation.w = qw_w
        self.pose_world_pub.publish(ps_world)
        self.get_logger().info(
            f"tag@world: xyz=({ps_world.pose.position.x:.3f},"
            f"{ps_world.pose.position.y:.3f},{ps_world.pose.position.z:.3f})"
        )

        # 10) Overlays (green = final quad, magenta = detected long edge)
        overlay = img.copy()
        pts = img_pts.astype(int)
        cv2.polylines(overlay, [pts], True, (0, 255, 0), 2)
        for (x, y) in pts:
            cv2.circle(overlay, (x, y), 3, (255, 0, 0), -1)
        if long_edge_img == 'x':
            cv2.line(overlay, tuple(pts[0]), tuple(pts[1]), (255, 0, 255), 2)  # TL->TR
        else:
            cv2.line(overlay, tuple(pts[1]), tuple(pts[2]), (255, 0, 255), 2)  # TR->BR
        vx = long_axis_cam[0]
        vy = long_axis_cam[1]

        angle_cam = np.arctan2(vy, vx)   # radians
        self.get_logger().info(f"long_edge_angle_cam = {angle_cam:.3f} rad")

        # 11) Save debug images (throttled)
        if self.save_debug:
            self._dbg_count += 1
            if self._dbg_count % self.debug_every == 0:
                ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                base = os.path.join(self.debug_dir, f"frame_{ts}")
                try:
                    cv2.imwrite(base + "_orig.png", img)
                    cv2.imwrite(base + "_blur.png", blur)
                    cv2.imwrite(base + "_bw.png", bw)
                    cv2.imwrite(base + "_bw_morph.png", bw_m)
                    cv2.imwrite(base + "_candidates.png", overlay_all)
                    cv2.imwrite(base + "_overlay.png", overlay)

                    txt_cam = (f"cam=({ps_cam.pose.position.x:.3f},"
                            f"{ps_cam.pose.position.y:.3f},{ps_cam.pose.position.z:.3f})")
                    txt_wld = (f"world=({ps_world.pose.position.x:.3f},"
                            f"{ps_world.pose.position.y:.3f},{ps_world.pose.position.z:.3f})")
                    overlay_annot = overlay.copy()
                    cv2.putText(overlay_annot, txt_cam, (10, 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    cv2.putText(overlay_annot, txt_wld, (10, 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    cv2.imwrite(base + "_overlay_annot.png", overlay_annot)

                    self.get_logger().debug(f"Saved debug images: {base}_*.png")
                except Exception as e:
                    self.get_logger().warn(f"Debug save failed: {e}")





def main():
    rclpy.init()
    rclpy.spin(MinimalTagPose())
    rclpy.shutdown()


if __name__ == "__main__":
    main()