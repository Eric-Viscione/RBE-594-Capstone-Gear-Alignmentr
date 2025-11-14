# tag_geom.py
import numpy as np


def rotmat_to_quat(R: np.ndarray):
    """
    3x3 rotation matrix -> quaternion (x, y, z, w).
    """
    m00, m01, m02 = R[0]
    m10, m11, m12 = R[1]
    m20, m21, m22 = R[2]
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

    return qx, qy, qz, qw


def order_corners(pts: np.ndarray) -> np.ndarray:
    """
    Given 4 points (unordered), return them as TL, TR, BR, BL.
    """
    pts = pts.reshape(-1, 2)
    c = pts.mean(axis=0)
    ang = np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0])
    pts = pts[np.argsort(ang)]

    start = np.argmin(pts.sum(axis=1))
    pts = np.roll(pts, -start, axis=0)

    v1 = pts[1] - pts[0]
    v2 = pts[2] - pts[1]
    cross = v1[0] * v2[1] - v1[1] * v2[0]
    if cross < 0:
        pts = np.array([pts[0], pts[3], pts[2], pts[1]])

    return pts.astype(np.float32)


def build_rect_model_pts(width: float, height: float) -> np.ndarray:
    """
    3D model points (Z=0) for a rectangle of size width x height centered at origin.
    Returns corners in TL, TR, BR, BL.
    """
    sx, sy = width / 2.0, height / 2.0
    return np.array(
        [
            [-sx, -sy, 0],  # TL
            [ sx, -sy, 0],  # TR
            [ sx,  sy, 0],  # BR
            [-sx,  sy, 0],  # BL
        ],
        dtype=np.float32
    )


def pick_long_edge_from_image(img_pts: np.ndarray):
    """
    img_pts must be TL,TR,BR,BL.
    Measures TL->TR (x-edge) and TR->BR (y-edge) in image pixels.
    Returns:
      (which, e_x, e_y) where which is 'x' or 'y'.
    """
    e_x = float(np.linalg.norm(img_pts[1] - img_pts[0]))  # TL->TR
    e_y = float(np.linalg.norm(img_pts[2] - img_pts[1]))  # TR->BR

    if e_x >= e_y:
        return 'x', e_x, e_y
    else:
        return 'y', e_x, e_y
