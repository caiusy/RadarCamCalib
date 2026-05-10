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


def draw_vision_bboxes(
    image: np.ndarray,
    vision_objects,
    color: tuple = (0, 200, 0),
    thickness: int = 2,
) -> np.ndarray:
    """Draw vision detection bounding boxes on an image.

    Args:
        image: BGR image array.
        vision_objects: List of VisionObject instances.
        color: BGR color for bboxes.
        thickness: Line thickness.

    Returns:
        Image with bboxes drawn.
    """
    img = image.copy()

    for obj in vision_objects:
        x1, y1, x2, y2 = int(obj.bbox[0]), int(obj.bbox[1]), int(obj.bbox[2]), int(obj.bbox[3])
        cv2.rectangle(img, (x1, y1), (x2, y2), color, thickness, cv2.LINE_AA)

        # Label with class and score
        label = f"{obj.obj_class} {obj.score:.2f}"
        cv2.putText(
            img, label, (x1, y1 - 5),
            cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1, cv2.LINE_AA,
        )

        # Draw bottom center marker
        bc_u, bc_v = obj.bottom_center
        cv2.circle(img, (int(bc_u), int(bc_v)), 3, (0, 255, 255), -1)

    return img


def draw_association_lines(
    image: np.ndarray,
    matches,
    radar_proj_uv: np.ndarray,
    vision_objects,
    match_color: tuple = (255, 200, 0),
    thickness: int = 2,
) -> np.ndarray:
    """Draw matching lines between radar projections and vision bbox centers.

    Args:
        image: BGR image array.
        matches: List of Match instances.
        radar_proj_uv: Nx2 array of radar projected [u, v] in image.
        vision_objects: List of VisionObject instances.
        match_color: BGR color for match lines.
        thickness: Line thickness.

    Returns:
        Image with matching lines drawn.
    """
    img = image.copy()

    for m in matches:
        r_pt = (int(radar_proj_uv[m.radar_idx, 0]), int(radar_proj_uv[m.radar_idx, 1]))
        v_bc = vision_objects[m.vision_idx].bottom_center
        v_pt = (int(v_bc[0]), int(v_bc[1]))

        cv2.line(img, r_pt, v_pt, match_color, thickness, cv2.LINE_AA)

        # Draw cost label at midpoint
        mid_x = (r_pt[0] + v_pt[0]) // 2
        mid_y = (r_pt[1] + v_pt[1]) // 2
        cv2.putText(
            img, f"{m.cost:.2f}",
            (mid_x, mid_y - 5),
            cv2.FONT_HERSHEY_SIMPLEX, 0.3, match_color, 1, cv2.LINE_AA,
        )

    return img
