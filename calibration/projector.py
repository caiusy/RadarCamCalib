"""
projector.py - Project points between radar BEV and image coordinates.

Uses the computed homography matrix H to transform:
    - Radar -> Image: [u, v, 1]^T ≈ H · [x, y, 1]^T
    - Image -> Radar: [x, y, 1]^T ≈ H_inv · [u, v, 1]^T
"""

import numpy as np
from typing import Optional, Tuple


def project_radar_to_image(
    radar_points: np.ndarray,
    H: np.ndarray
) -> np.ndarray:
    """Project radar BEV points to image pixel coordinates.

    Args:
        radar_points: Nx2 array of radar [x, y] coordinates.
        H: 3x3 homography matrix (radar -> image).

    Returns:
        Nx2 array of image [u, v] coordinates.
    """
    if len(radar_points) == 0:
        return np.empty((0, 2), dtype=np.float64)

    n = len(radar_points)
    ones = np.ones((n, 1), dtype=np.float64)
    pts_h = np.hstack([radar_points, ones])  # Nx3

    projected = (H @ pts_h.T).T  # Nx3

    # Normalize by homogeneous coordinate
    w = projected[:, 2:3]
    w = np.where(np.abs(w) < 1e-10, 1e-10, w)
    image_points = projected[:, :2] / w

    return image_points


def project_image_to_radar(
    image_points: np.ndarray,
    H_inv: np.ndarray
) -> np.ndarray:
    """Project image pixel coordinates to radar BEV coordinates.

    Args:
        image_points: Nx2 array of image [u, v] coordinates.
        H_inv: 3x3 inverse homography matrix (image -> radar).

    Returns:
        Nx2 array of radar [x, y] coordinates.
    """
    if len(image_points) == 0:
        return np.empty((0, 2), dtype=np.float64)

    n = len(image_points)
    ones = np.ones((n, 1), dtype=np.float64)
    pts_h = np.hstack([image_points, ones])  # Nx3

    projected = (H_inv @ pts_h.T).T  # Nx3

    w = projected[:, 2:3]
    w = np.where(np.abs(w) < 1e-10, 1e-10, w)
    radar_points = projected[:, :2] / w

    return radar_points


def project_single_point(
    x: float, y: float, H: np.ndarray
) -> Tuple[float, float]:
    """Project a single point through a homography.

    Args:
        x: Input x coordinate.
        y: Input y coordinate.
        H: 3x3 homography matrix.

    Returns:
        Tuple of (projected_x, projected_y).
    """
    pt = np.array([x, y, 1.0], dtype=np.float64)
    result = H @ pt
    w = result[2] if abs(result[2]) > 1e-10 else 1e-10
    return (result[0] / w, result[1] / w)


def filter_points_in_image(
    image_points: np.ndarray,
    image_width: int,
    image_height: int,
    margin: int = 0
) -> Tuple[np.ndarray, np.ndarray]:
    """Filter projected points that fall within image bounds.

    Args:
        image_points: Nx2 array of [u, v] coordinates.
        image_width: Image width in pixels.
        image_height: Image height in pixels.
        margin: Extra margin around image bounds (pixels).

    Returns:
        Tuple of (filtered_points, valid_mask):
            - filtered_points: Mx2 array of points within bounds.
            - valid_mask: N-element boolean mask.
    """
    if len(image_points) == 0:
        return np.empty((0, 2), dtype=np.float64), np.array([], dtype=bool)

    u = image_points[:, 0]
    v = image_points[:, 1]

    valid = (
        (u >= -margin) & (u < image_width + margin) &
        (v >= -margin) & (v < image_height + margin)
    )

    return image_points[valid], valid
