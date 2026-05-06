"""
image_overlay.py - Draw overlays on camera images using OpenCV.

Pure rendering functions that take an image array and draw radar projections,
matched points, labels, etc. No PyQt dependency.
"""

import cv2
import numpy as np
from typing import List, Optional, Tuple

from data.data_models import RadarObject, MatchedPoint
from .color_map import hex_to_bgr, get_class_color, get_pair_color


def draw_projected_radar(
    image: np.ndarray,
    radar_objects: List[RadarObject],
    projected_points: np.ndarray,
    show_labels: bool = True,
    marker_radius: int = 10,
    marker_color: str = '#FF00FF',
) -> np.ndarray:
    """Draw projected radar points onto an image.

    Args:
        image: BGR image array (will be copied, not modified in-place).
        radar_objects: List of RadarObject instances.
        projected_points: Nx2 array of projected [u, v] pixel coordinates.
        show_labels: Whether to draw track ID labels.
        marker_radius: Circle radius in pixels.
        marker_color: Hex color for markers.

    Returns:
        Image with overlays drawn.
    """
    img = image.copy()
    color = hex_to_bgr(marker_color)

    for i, obj in enumerate(radar_objects):
        if i >= len(projected_points):
            break

        u, v = int(projected_points[i, 0]), int(projected_points[i, 1])

        # Skip out-of-bounds points
        h, w = img.shape[:2]
        if u < -100 or u > w + 100 or v < -100 or v > h + 100:
            continue

        # Draw circle marker
        cv2.circle(img, (u, v), marker_radius, color, 2)

        if show_labels:
            label = f"R{obj.track_id}"
            cv2.putText(
                img, label,
                (u + marker_radius + 4, v - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1,
                cv2.LINE_AA,
            )

    return img


def draw_matched_points(
    image: np.ndarray,
    matched_points: List[MatchedPoint],
    image_points_only: bool = True,
) -> np.ndarray:
    """Draw matched point pairs on an image.

    Args:
        image: BGR image array (will be copied).
        matched_points: List of MatchedPoint instances.
        image_points_only: If True, draw only image points (not radar projection).

    Returns:
        Image with matched points drawn.
    """
    img = image.copy()

    for i, mp in enumerate(matched_points):
        color = hex_to_bgr(get_pair_color(i))

        u, v = int(mp.image_u), int(mp.image_v)

        # Draw crosshair
        size = 8
        cv2.line(img, (u - size, v), (u + size, v), color, 2)
        cv2.line(img, (u, v - size), (u, v + size), color, 2)
        cv2.circle(img, (u, v), size + 2, color, 1)

        # Label
        label = f"P{i+1}"
        if mp.reprojection_error is not None:
            label += f" ({mp.reprojection_error:.1f}px)"
        cv2.putText(
            img, label,
            (u + size + 6, v + 4),
            cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1,
            cv2.LINE_AA,
        )

    return img


def draw_radar_on_image(
    image: np.ndarray,
    radar_objects: List[RadarObject],
    H: np.ndarray,
    show_labels: bool = True,
) -> np.ndarray:
    """Project radar objects and draw them on an image using homography.

    Convenience function combining projection and drawing.

    Args:
        image: BGR image array.
        radar_objects: List of RadarObject instances.
        H: 3x3 homography matrix (radar → image).
        show_labels: Whether to show track IDs.

    Returns:
        Image with projected radar points drawn.
    """
    if not radar_objects:
        return image.copy()

    # Build point array
    pts = np.array([[obj.x, obj.y] for obj in radar_objects], dtype=np.float64)

    # Project
    from calibration.projector import project_radar_to_image
    projected = project_radar_to_image(pts, H)

    return draw_projected_radar(image, radar_objects, projected, show_labels)
