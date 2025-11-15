# tag_pose_estimator.py
import os
from dataclasses import dataclass, field
from datetime import datetime

import numpy as np
import cv2

from geometry_msgs.msg import PoseStamped

from .helpers import (
    rotmat_to_quat,
    order_corners,
    build_rect_model_pts,
    pick_long_edge_from_image,
)


@dataclass
class DetectorConfig:
    # thresholding
    t_dark: int = 20                     # dark cutoff in [0,255]
    close_kernel: tuple = (3, 3)
    open_kernel: tuple = (1, 1)
    color_mode: str = "hsv"

    # HSV range if using color_mode="hsv"
    hsv_low:  tuple = (100, 100, 100)   # a bit above yellow
    hsv_high: tuple = (130, 255, 255) 
    
    
    # contour filters
    min_area_frac: float = 0.00001        # min area as fraction of image
    max_area_frac: float = 0.6           # max area as fraction of image
    max_ratio: float = 10.0               # max aspect ratio (long/short)
    eps_frac: float = 0.05             # approxPolyDP epsilon fraction

    # rectangle physical size (meters)
    rect_w: float = 0.02                 # short side
    rect_h: float = 0.05                 # long side

    # debugging
    enable_debug_log: bool = True       # <-- NEW: controls extra prints
    max_debug_contours: int = 5          # how many top contours to log

    # world <- camera transform (3x3 R, 3x1 t)
    R_world_cam: np.ndarray = field(
        default_factory=lambda: np.diag([1.0, 1.0, -1.0]).astype(np.float64)
    )
    t_world_cam: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.0, 2.0], dtype=np.float64)
    )



class TagPoseEstimator:
    def __init__(self,
                 config: DetectorConfig,
                 logger=None,
                 debug_dir: str = None,
                 subfolder_name: str = None, 
                 save_debug: bool = True,
                 debug_every: int = 5):
        self.cfg = config
        self.logger = logger
        self.log = logger 
        self.debug_dir = os.path.expanduser(debug_dir) if debug_dir else None
        self.save_debug = save_debug and (self.debug_dir is not None)
        self.debug_every = max(1, int(debug_every))
        self._dbg_count = 0

        if self.save_debug:
            base_dir = os.path.expanduser(debug_dir)
            if subfolder_name:
                # Append the subfolder name (e.g., 'green' or 'black') to the path
                self.debug_dir = os.path.join(base_dir, subfolder_name)
            else:
                self.debug_dir = base_dir
                
            os.makedirs(self.debug_dir, exist_ok=True)
            self.log.info(f"Saving debug images to: {self.debug_dir}")
    def _log(self, msg: str):
        if self.cfg.enable_debug_log:
            self.log.info(msg)

    # ------------- debug when no detection -------------
    def _debug_no_detection(self, gray, bw_m, contours):
        """
        Called when no quadrilateral passes the filters.
        Logs useful stats about contours and thresholds.
        """
        if not self.cfg.enable_debug_log:
            return

        H, W = gray.shape[:2]
        n = len(contours)
        self._log(
            f"[DEBUG] No valid quad found. image={W}x{H}, "
            f"num_contours={n}"
        )

        # convert area thresholds from fractions to pixels
        img_area = float(W * H)
        min_area_px = self.cfg.min_area_frac * img_area
        max_area_px = self.cfg.max_area_frac * img_area
        self._log(
            f"[DEBUG] area thresholds: "
            f"min_area_frac={self.cfg.min_area_frac:.6f} "
            f"({min_area_px:.1f} px), "
            f"max_area_frac={self.cfg.max_area_frac:.3f} "
            f"({max_area_px:.1f} px), "
            f"max_ratio={self.cfg.max_ratio:.2f}"
        )

        # summarize top-K contours by area
        contour_stats = []
        for c in contours:
            area = cv2.contourArea(c)
            (w_rect, h_rect) = cv2.minAreaRect(c)[1]
            if min(w_rect, h_rect) <= 1e-6:
                ratio = float("inf")
            else:
                ratio = max(w_rect, h_rect) / max(1e-6, min(w_rect, h_rect))

            contour_stats.append((area, w_rect, h_rect, ratio))

        contour_stats.sort(key=lambda x: x[0], reverse=True)
        max_k = min(self.cfg.max_debug_contours, len(contour_stats))

        for i in range(max_k):
            area, w_rect, h_rect, ratio = contour_stats[i]
            reason = []
            if area < min_area_px:
                reason.append("area < min")
            if area > max_area_px:
                reason.append("area > max")
            if ratio > self.cfg.max_ratio:
                reason.append("ratio > max_ratio")
            if not reason:
                reason.append("failed quad/convex filter or scoring")

            self._log(
                f"[DEBUG] contour {i}: "
                f"area={area:.1f}, w_rect={w_rect:.1f}, "
                f"h_rect={h_rect:.1f}, ratio={ratio:.2f} "
                f"-> {'; '.join(reason)}"
            )
    # ---------- main entry point ----------

    def process_frame(self, img_bgr, K, D, cam_frame: str, stamp):
        """
        Inputs:
        - img_bgr   : OpenCV BGR image
        - K, D      : intrinsics (3x3, vector)
        - cam_frame : name of camera frame (string)
        - stamp     : ROS2 time stamp
        Returns:
        - ps_cam    : PoseStamped in camera frame (or None)
        - ps_world  : PoseStamped in 'world' frame (or None)
        """
        # 1) preprocess
        gray, norm, blur = self._preprocess(img_bgr)
        bw, bw_m = self._threshold_and_morph(blur, img_bgr=img_bgr)


        # 3) find best quadrilateral + have access to all contours
        best_quad, overlay_all, cnts = self._find_best_quad(gray, bw_m)

        # 4) if nothing detected, dump debug and log why
        if best_quad is None:
            self._maybe_save_debug(
                img_bgr, blur, bw, bw_m, overlay_all,
                ps_cam=None, ps_world=None, overlay=None
            )
            self._debug_no_detection(gray, bw_m, cnts)
            return None, None, None, None

        # 5) TL,TR,BR,BL ordering
        img_pts = order_corners(best_quad)

        # 6) Decide which image edge is longer (handles skew)
        long_edge_img, e_x, e_y = pick_long_edge_from_image(img_pts)

        # 7) Build physical rectangle model consistent with long edge
        obj_pts, long_axis_idx, w_used, h_used = self._build_rect_model(long_edge_img)

        # 8) PnP
        R, tvec = self._solve_pnp(obj_pts, img_pts, K, D)
        if R is None:
            return None, None, None, None

        # 9) Pose in camera
        ps_cam, long_axis_cam, long_axis_world = self._pose_in_camera(
            R, tvec, cam_frame, stamp, long_axis_idx
        )

        # 9b) Build Stamped Vector Messages (using PoseStamped position for the vector)
        ps_long_cam = self._build_vector_stamped(
            long_axis_cam, cam_frame, stamp, is_world_vector=False
        )
        ps_long_world = self._build_vector_stamped(
            long_axis_world, 'world', stamp, is_world_vector=True
        )

        # 10) Pose in world
        ps_world = self._pose_in_world(R, tvec, stamp)

        # 11) Logging pose + orientation info
        self.log.info(
            f"tag@cam: xyz=({ps_cam.pose.position.x:.3f},"
            f"{ps_cam.pose.position.y:.3f},{ps_cam.pose.position.z:.3f})"
        )
        self.log.info(
            f"long_edge_img={long_edge_img} (e_x={e_x:.1f}, e_y={e_y:.1f}), "
            f"model_used w={w_used:.3f} h={h_used:.3f}"
        )
        self.log.info(
            f"long_axis_cam=({long_axis_cam[0]:.3f},"
            f"{long_axis_cam[1]:.3f},{long_axis_cam[2]:.3f}), "
            f"long_axis_world=({long_axis_world[0]:.3f},"
            f"{long_axis_world[1]:.3f},{long_axis_world[2]:.3f})"
        )

        # 12) Build overlay for debug
        overlay = self._make_overlay(img_bgr, img_pts, long_edge_img)

        # 13) Save debug images (if enabled)
        self._maybe_save_debug(
                img_bgr, blur, bw, bw_m, overlay_all,
                ps_cam=ps_cam, ps_world=ps_world, 
                ps_long_cam=ps_long_cam, ps_long_world=ps_long_world, # <--- FIX: Add these arguments
                overlay=overlay
            )

        return ps_cam, ps_world, ps_long_cam, ps_long_world

    def _build_vector_stamped(self, vector: np.ndarray, frame_id: str, stamp, is_world_vector: bool) -> PoseStamped:
            """Helper to package the long axis vector into a PoseStamped message."""
            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = frame_id
            
            # The vector components are stored in the position field
            ps.pose.position.x = float(vector[0])
            ps.pose.position.y = float(vector[1])
            ps.pose.position.z = float(vector[2])
            
            # Orientation is left as identity/zero since we only care about the vector
            ps.pose.orientation.x = 0.0
            ps.pose.orientation.y = 0.0
            ps.pose.orientation.z = 0.0
            ps.pose.orientation.w = 1.0
            
            # Log vector for verification
            self.log.info(
                f"{'World' if is_world_vector else 'Cam'} Long Axis Vector: "
                f"({vector[0]:.3f}, {vector[1]:.3f}, {vector[2]:.3f})"
            )
            
            return ps

    def _preprocess(self, img_bgr):
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        norm = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX)
        # blur = cv2.medianBlur(norm, 5)
        blur = norm.copy()
        return gray, norm, blur

    def _threshold_and_morph(self, blur, img_bgr=None):
        """
        Produce a binary image 'bw' where the tag is white (255) and background black (0),
        using either:
        - dark/bright grayscale thresholding, or
        - HSV color thresholding.
        """
        mode = getattr(self.cfg, "color_mode", "dark")

        if mode == "hsv" and img_bgr is not None:
            # --- HSV color thresholding ---
            hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
            low = np.array(self.cfg.hsv_low, dtype=np.uint8)
            high = np.array(self.cfg.hsv_high, dtype=np.uint8)
            bw = cv2.inRange(hsv, low, high)  # tag-colored = white, others black
        else:
            # --- grayscale thresholding ---
            t = self.cfg.t_dark
            if mode == "bright":
                # detect light tag on dark background
                bw = (blur > t).astype(np.uint8) * 255
            else:
                # default: detect dark tag on light background
                bw = (blur < t).astype(np.uint8) * 255

        # morphology to clean up
        k_close = cv2.getStructuringElement(cv2.MORPH_RECT, self.cfg.close_kernel)
        bw_m = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, k_close, iterations=1)

        k_open = cv2.getStructuringElement(cv2.MORPH_RECT, self.cfg.open_kernel)
        bw_m = cv2.morphologyEx(bw_m, cv2.MORPH_OPEN, k_open, iterations=1)

        return bw, bw_m.astype(np.uint8)


    def _find_best_quad(self, gray, bw_m):
        cnts, _ = cv2.findContours(bw_m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        H, W = gray.shape[:2]

        min_area_px = max(1, int(self.cfg.min_area_frac * W * H))
        max_area_px = max(1, int(self.cfg.max_area_frac * W * H))

        overlay_all = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        best = None
        best_score = -1.0

        for c in cnts:
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, self.cfg.eps_frac * peri, True)

            # draw all contours faintly
            cv2.drawContours(overlay_all, [c], -1, (80, 80, 80), 1)

            if len(approx) != 4:
                continue
            if not cv2.isContourConvex(approx):
                continue

            area = cv2.contourArea(approx)
            if area < min_area_px or area > max_area_px:
                continue

            (w_rect, h_rect) = cv2.minAreaRect(approx)[1]
            if min(w_rect, h_rect) <= 1e-6:
                continue
            ratio = max(w_rect, h_rect) / max(1e-6, min(w_rect, h_rect))
            self.log.info(
                f"candidate: area={area:.1f}, w_rect={w_rect:.1f}, "
                f"h_rect={h_rect:.1f}, aspect={ratio:.2f}"
            )
            if ratio > self.cfg.max_ratio:
                continue

            squareness = 1.0 / (1.0 + abs(1.0 - ratio))
            score = squareness * area

            cv2.polylines(overlay_all, [approx], True, (0, 255, 255), 2)

            if score > best_score:
                best_score = score
                best = approx.reshape(-1, 2).astype(np.float32)

        # NOTE: return contours as third value
        return best, overlay_all, cnts

    def _build_rect_model(self, long_edge_img: str):
        """
        Decides which physical dimension is "X" vs "Y" in the tag model
        so that X corresponds to the long edge in the image.
        """
        w_phys = self.cfg.rect_w
        h_phys = self.cfg.rect_h
        model_x_is_long = (w_phys >= h_phys)

        # Decide which image edge was longer
        # long_edge_img == 'x' means TL->TR is longer in pixels
        if (long_edge_img == 'x') != model_x_is_long:
            # swap so that model X = longer side
            w_used, h_used = h_phys, w_phys
        else:
            w_used, h_used = w_phys, h_phys

        obj_pts = build_rect_model_pts(w_used, h_used)

        # If X is long: we interpret R[:,0] as long axis.
        # If Y is long: we interpret R[:,1] as long axis.
        long_axis_idx = 0 if w_used >= h_used else 1

        return obj_pts, long_axis_idx, w_used, h_used

    def _solve_pnp(self, obj_pts, img_pts, K, D):
        try:
            ok, rvec, tvec = cv2.solvePnP(
                obj_pts, img_pts, K, D,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
        except Exception:
            ok, rvec, tvec = cv2.solvePnP(
                obj_pts, img_pts, K, D,
                flags=cv2.SOLVEPNP_ITERATIVE
            )
        if not ok:
            return None, None
        R, _ = cv2.Rodrigues(rvec)
        return R, tvec

    def _pose_in_camera(self, R, tvec, cam_frame, stamp, long_axis_idx):
        qx, qy, qz, qw = rotmat_to_quat(R)

        ps_cam = PoseStamped()
        ps_cam.header.stamp = stamp
        ps_cam.header.frame_id = cam_frame

        ps_cam.pose.position.x = float(tvec[0])
        ps_cam.pose.position.y = float(tvec[1])
        ps_cam.pose.position.z = float(tvec[2])

        ps_cam.pose.orientation.x = qx
        ps_cam.pose.orientation.y = qy
        ps_cam.pose.orientation.z = qz
        ps_cam.pose.orientation.w = qw

        long_axis_cam = R[:, long_axis_idx]
        long_axis_world = self.cfg.R_world_cam @ long_axis_cam

        return ps_cam, long_axis_cam, long_axis_world

    def _pose_in_world(self, R, tvec, stamp):
        T_world_cam = np.eye(4, dtype=np.float64)
        T_world_cam[:3, :3] = self.cfg.R_world_cam
        T_world_cam[:3, 3] = self.cfg.t_world_cam

        T_cam_tag = np.eye(4, dtype=np.float64)
        T_cam_tag[:3, :3] = R
        T_cam_tag[:3, 3] = tvec.flatten()

        T_world_tag = T_world_cam @ T_cam_tag
        R_world_tag = T_world_tag[:3, :3]
        t_world_tag = T_world_tag[:3, 3]
        qx_w, qy_w, qz_w, qw_w = rotmat_to_quat(R_world_tag)

        ps_world = PoseStamped()
        ps_world.header.stamp = stamp
        ps_world.header.frame_id = 'world'
        ps_world.pose.position.x = float(t_world_tag[0])
        ps_world.pose.position.y = float(t_world_tag[1])
        ps_world.pose.position.z = float(t_world_tag[2])
        ps_world.pose.orientation.x = qx_w
        ps_world.pose.orientation.y = qy_w
        ps_world.pose.orientation.z = qz_w
        ps_world.pose.orientation.w = qw_w

        return ps_world

    def _make_overlay(self, img_bgr, img_pts, long_edge_img):
        overlay = img_bgr.copy()
        pts = img_pts.astype(int)
        cv2.polylines(overlay, [pts], True, (0, 255, 0), 2)
        for (x, y) in pts:
            cv2.circle(overlay, (x, y), 3, (255, 0, 0), -1)

        if long_edge_img == 'x':
            cv2.line(overlay, tuple(pts[0]), tuple(pts[1]), (255, 0, 255), 2)
        else:
            cv2.line(overlay, tuple(pts[1]), tuple(pts[2]), (255, 0, 255), 2)
        return overlay


    def _maybe_save_debug(self, img_bgr, blur, bw, bw_m, overlay_all,
                          ps_cam=None, ps_world=None, 
                          ps_long_cam=None, ps_long_world=None, # MODIFIED: added new arguments
                          overlay=None):
        if not self.save_debug:
            return
        self._dbg_count += 1
        if self._dbg_count % self.debug_every != 0:
            return

        ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        base = os.path.join(self.debug_dir, f"frame_{ts}")

        try:
            cv2.imwrite(base + "_orig.png", img_bgr)
            cv2.imwrite(base + "_blur.png", blur)
            cv2.imwrite(base + "_bw.png", bw)
            cv2.imwrite(base + "_bw_morph.png", bw_m)
            cv2.imwrite(base + "_candidates.png", overlay_all)
            if overlay is not None:
                cv2.imwrite(base + "_overlay.png", overlay)

            # annotate overlay with pose if available
            if overlay is not None and ps_cam is not None and ps_world is not None:
                overlay_annot = overlay.copy()
                txt_cam = (
                    f"cam=({ps_cam.pose.position.x:.3f},"
                    f"{ps_cam.pose.position.y:.3f},{ps_cam.pose.position.z:.3f})"
                )
                txt_wld = (
                    f"world=({ps_world.pose.position.x:.3f},"
                    f"{ps_world.pose.position.y:.3f},{ps_world.pose.position.z:.3f})"
                )
                if ps_long_world is not None:
                    txt_long = (
                        f"axis_wld=({ps_long_world.pose.position.x:.3f},"
                        f"{ps_long_world.pose.position.y:.3f},{ps_long_world.pose.position.z:.3f})"
                    )
                cv2.putText(
                    overlay_annot, txt_long, (10, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
                )
                cv2.putText(
                    overlay_annot, txt_cam, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2
                )
                cv2.putText(
                    overlay_annot, txt_wld, (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2
                )
                cv2.imwrite(base + "_overlay_annot.png", overlay_annot)

            self.log.debug(f"Saved debug images: {base}_*.png")
        except Exception as e:
            self.log.warn(f"Debug save failed: {e}")
