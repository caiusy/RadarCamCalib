"""
homography_solver.py - Compute homography from matched radar-image point pairs.

Uses OpenCV's findHomography with RANSAC for robust estimation.
The homography H maps radar BEV coordinates to image pixel coordinates:

    [u, v, 1]^T ≈ H · [x, y, 1]^T

where (x, y) are radar coordinates and (u, v) are image pixels.
"""

import numpy as np
import cv2
from typing import Tuple, Optional


def compute_homography(
    radar_points: np.ndarray,
    image_points: np.ndarray,
    ransac_thresh: float = 5.0
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """Compute homography matrix from matched point pairs using RANSAC.

    Args:
        radar_points: Nx2 array of radar BEV coordinates [x, y].
        image_points: Nx2 array of corresponding image coordinates [u, v].
        ransac_thresh: RANSAC reprojection threshold in pixels.

    Returns:
        Tuple of (H, inlier_mask):
            - H: 3x3 homography matrix, or None if computation fails.
            - inlier_mask: Nx1 binary mask of inlier points, or None.

    Raises:
        ValueError: If fewer than 4 point pairs are provided.
    """
    if len(radar_points) < 4 or len(image_points) < 4:
        raise ValueError(
            f"At least 4 point pairs required, got {len(radar_points)}"
        )

    if len(radar_points) != len(image_points):
        raise ValueError(
            f"Point count mismatch: {len(radar_points)} radar vs {len(image_points)} image"
        )

    # Ensure float64 for OpenCV
    src = np.array(radar_points, dtype=np.float64).reshape(-1, 1, 2)
    dst = np.array(image_points, dtype=np.float64).reshape(-1, 1, 2)

    H, mask = cv2.findHomography(src, dst, cv2.RANSAC, ransac_thresh)

    if H is None:
        return None, None

    return H, mask


def compute_reprojection_errors(
    radar_points: np.ndarray,
    image_points: np.ndarray,
    H: np.ndarray
) -> np.ndarray:
    """Compute per-point reprojection errors using a homography matrix.

    Projects each radar point through H and measures Euclidean distance
    to the corresponding image point.

    Args:
        radar_points: Nx2 array of radar BEV coordinates [x, y].
        image_points: Nx2 array of corresponding image coordinates [u, v].
        H: 3x3 homography matrix (radar -> image).

    Returns:
        N-element array of reprojection errors in pixels.
    """
    n = len(radar_points)
    if n == 0:
        return np.array([], dtype=np.float64)

    # Build homogeneous coordinates
    ones = np.ones((n, 1), dtype=np.float64)
    pts_h = np.hstack([radar_points, ones])  # Nx3

    # Project: [u', v', w'] = H @ [x, y, 1]
    projected = (H @ pts_h.T).T  # Nx3

    # Normalize by w
    w = projected[:, 2:3]
    w = np.where(np.abs(w) < 1e-10, 1e-10, w)  # Avoid division by zero
    projected_uv = projected[:, :2] / w  # Nx2

    # Euclidean distance
    errors = np.linalg.norm(projected_uv - image_points, axis=1)
    return errors


def invert_homography(H: np.ndarray) -> np.ndarray:
    """Compute the inverse of a homography matrix.

    Args:
        H: 3x3 homography matrix.

    Returns:
        3x3 inverse homography matrix.

    Raises:
        ValueError: If the matrix is singular.
    """
    det = np.linalg.det(H)
    if abs(det) < 1e-10:
        raise ValueError("Homography matrix is singular, cannot invert")

    return np.linalg.inv(H)
