"""
radar_drawer.py - High-level radar drawing utilities.

Combines image overlay and BEV rendering for common use cases.
"""

import cv2
import numpy as np
from typing import List, Dict, Optional

from data.data_models import RadarObject, MatchedPoint
from calibration.projector import project_radar_to_image, filter_points_in_image
from .image_overlay import draw_projected_radar, draw_matched_points
from .bev_renderer import render_bev_frame


def draw_radar_overlay(
    image: np.ndarray,
    radar_objects: List[RadarObject],
    H: np.ndarray,
    matched_points: Optional[List[MatchedPoint]] = None,
    show_labels: bool = True,
) -> np.ndarray:
    """Draw radar projections and matched points on an image.

    Combines homography projection with rendering.

    Args:
        image: BGR image array.
        radar_objects: List of radar objects to project.
        H: Homography matrix (radar → image).
        matched_points: Optional matched points to draw.
        show_labels: Whether to show track ID labels.

    Returns:
        Image with all overlays drawn.
    """
    result = image.copy()

    if radar_objects:
        pts = np.array([[o.x, o.y] for o in radar_objects], dtype=np.float64)
        projected = project_radar_to_image(pts, H)

        # Filter to image bounds
        h, w = result.shape[:2]
        in_bounds, valid_mask = filter_points_in_image(projected, w, h, margin=50)

        # Draw only in-bounds objects
        valid_objects = [o for o, v in zip(radar_objects, valid_mask) if v]
        result = draw_projected_radar(
            result, valid_objects, in_bounds, show_labels
        )

    if matched_points:
        result = draw_matched_points(result, matched_points)

    return result
