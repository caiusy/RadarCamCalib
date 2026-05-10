"""
bev_renderer.py - Render Bird's Eye View (BEV) images using OpenCV.

Pure rendering functions for creating BEV visualizations of radar data,
trajectories, and range polygons. No PyQt dependency.
"""

import cv2
import numpy as np
from typing import Dict, List, Optional, Tuple

from data.data_models import RadarObject
from trajectory.trajectory_models import Trajectory
from .color_map import hex_to_bgr, get_class_color, get_pair_color


def _bev_to_pixel(
    x: float, y: float,
    x_range: Tuple[float, float],
    y_range: Tuple[float, float],
    img_width: int,
    img_height: int,
) -> Tuple[int, int]:
    """Convert BEV meter coordinates to pixel coordinates.

    BEV: x=lateral (right positive), y=forward (up positive).
    Image: origin at top-left, y increases downward.

    Args:
        x, y: BEV coordinates in meters.
        x_range: (x_min, x_max) in meters.
        y_range: (y_min, y_max) in meters.
        img_width, img_height: Output image dimensions.

    Returns:
        (px, py) pixel coordinates.
    """
    x_min, x_max = x_range
    y_min, y_max = y_range

    px = int((x - x_min) / (x_max - x_min) * img_width)
    # Y is flipped: forward (higher y) is at top of image
    py = int((1.0 - (y - y_min) / (y_max - y_min)) * img_height)

    return px, py


def render_bev_frame(
    radar_objects: List[RadarObject],
    x_range: Tuple[float, float] = (-30, 30),
    y_range: Tuple[float, float] = (0, 200),
    img_size: Tuple[int, int] = (600, 800),
    show_grid: bool = True,
    show_labels: bool = True,
) -> np.ndarray:
    """Render a single radar frame as a BEV image.

    Args:
        radar_objects: List of RadarObject to render.
        x_range: Lateral range (min, max) in meters.
        y_range: Forward range (min, max) in meters.
        img_size: Output image (width, height).
        show_grid: Whether to draw grid lines.
        show_labels: Whether to draw track ID labels.

    Returns:
        BGR image array.
    """
    w, h = img_size
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:] = (30, 30, 30)  # Dark background

    if show_grid:
        _draw_grid(img, x_range, y_range, w, h)

    for obj in radar_objects:
        color = hex_to_bgr(get_class_color(obj.obj_class))
        px, py = _bev_to_pixel(obj.x, obj.y, x_range, y_range, w, h)

        if 0 <= px < w and 0 <= py < h:
            cv2.circle(img, (px, py), 5, color, -1)

            if show_labels:
                label = f"#{obj.track_id}"
                cv2.putText(
                    img, label, (px + 7, py - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1,
                    cv2.LINE_AA,
                )

    return img


def render_bev_trajectories(
    trajectories: Dict[int, Trajectory],
    x_range: Tuple[float, float] = (-30, 30),
    y_range: Tuple[float, float] = (0, 200),
    img_size: Tuple[int, int] = (600, 800),
    show_grid: bool = True,
    current_frame_index: Optional[int] = None,
) -> np.ndarray:
    """Render multiple trajectories on a BEV image.

    Args:
        trajectories: Dict of track_id → Trajectory.
        x_range: Lateral range in meters.
        y_range: Forward range in meters.
        img_size: Output image (width, height).
        show_grid: Whether to draw grid lines.
        current_frame_index: If set, highlight points at this frame.

    Returns:
        BGR image array.
    """
    w, h = img_size
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:] = (30, 30, 30)

    if show_grid:
        _draw_grid(img, x_range, y_range, w, h)

    for i, (tid, traj) in enumerate(trajectories.items()):
        color = hex_to_bgr(get_pair_color(i))

        # Draw trajectory path
        pts_px = []
        for pt in traj.points:
            px, py = _bev_to_pixel(pt.x, pt.y, x_range, y_range, w, h)
            pts_px.append((px, py))

        # Draw path lines
        for j in range(len(pts_px) - 1):
            cv2.line(img, pts_px[j], pts_px[j + 1], color, 1, cv2.LINE_AA)

        # Draw points
        for j, pt in enumerate(traj.points):
            px, py = pts_px[j]
            radius = 3

            # Highlight current frame
            if current_frame_index is not None and pt.frame_index == current_frame_index:
                radius = 6
                cv2.circle(img, (px, py), radius + 2, (255, 255, 255), 2)

            cv2.circle(img, (px, py), radius, color, -1)

        # Label at first point
        if pts_px:
            cv2.putText(
                img, f"T{tid}", (pts_px[0][0] + 8, pts_px[0][1] - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1, cv2.LINE_AA,
            )

    return img


def render_range_polygon(
    img: np.ndarray,
    polygons: List[dict],
    x_range: Tuple[float, float],
    y_range: Tuple[float, float],
    color: Tuple[int, int, int] = (0, 255, 128),
    alpha: float = 0.3,
) -> np.ndarray:
    """Draw range estimation polygons on a BEV image.

    Args:
        img: BEV image to draw on (will be copied).
        polygons: List of polygon dicts with "polygon" key.
        x_range, y_range: BEV coordinate ranges.
        color: BGR color for polygon edges.
        alpha: Fill transparency (0-1).

    Returns:
        Image with range polygons drawn.
    """
    result = img.copy()
    h, w = result.shape[:2]

    for poly_dict in polygons:
        vertices = poly_dict.get("polygon", [])
        if len(vertices) < 3:
            continue

        pts_px = []
        for vx, vy in vertices:
            px, py = _bev_to_pixel(vx, vy, x_range, y_range, w, h)
            pts_px.append([px, py])

        pts_arr = np.array(pts_px, dtype=np.int32)

        # Semi-transparent fill
        overlay = result.copy()
        cv2.fillPoly(overlay, [pts_arr], color)
        cv2.addWeighted(overlay, alpha, result, 1 - alpha, 0, result)

        # Solid edge
        cv2.polylines(result, [pts_arr], True, color, 2, cv2.LINE_AA)

        # Area label
        area = poly_dict.get("area_m2", 0)
        cx = int(np.mean([p[0] for p in pts_px]))
        cy = int(np.mean([p[1] for p in pts_px]))
        cv2.putText(
            result, f"{area:.0f}m²", (cx - 20, cy),
            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA,
        )

    return result


def _draw_grid(
    img: np.ndarray,
    x_range: Tuple[float, float],
    y_range: Tuple[float, float],
    w: int, h: int,
):
    """Draw grid lines on BEV image."""
    x_min, x_max = x_range
    y_min, y_max = y_range

    grid_color = (50, 50, 50)
    axis_color = (80, 80, 80)

    # Vertical lines (constant x)
    x_step = 10 if (x_max - x_min) <= 100 else 20
    for x in range(int(x_min), int(x_max) + 1, x_step):
        px, _ = _bev_to_pixel(x, y_min, x_range, y_range, w, h)
        _, py0 = _bev_to_pixel(x, y_min, x_range, y_range, w, h)
        _, py1 = _bev_to_pixel(x, y_max, x_range, y_range, w, h)
        c = axis_color if x == 0 else grid_color
        cv2.line(img, (px, py0), (px, py1), c, 1)

        # Label
        cv2.putText(
            img, f"{x}m", (px + 2, py0 - 5),
            cv2.FONT_HERSHEY_SIMPLEX, 0.3, (100, 100, 100), 1,
        )

    # Horizontal lines (constant y)
    y_step = 20 if (y_max - y_min) <= 200 else 50
    for y in range(int(y_min), int(y_max) + 1, y_step):
        _, py = _bev_to_pixel(x_min, y, x_range, y_range, w, h)
        px0, _ = _bev_to_pixel(x_min, y, x_range, y_range, w, h)
        px1, _ = _bev_to_pixel(x_max, y, x_range, y_range, w, h)
        c = axis_color if y == 0 else grid_color
        cv2.line(img, (px0, py), (px1, py), c, 1)

        cv2.putText(
            img, f"{y}m", (px0 + 2, py - 3),
            cv2.FONT_HERSHEY_SIMPLEX, 0.3, (100, 100, 100), 1,
        )

    # Ego marker at (0, 0)
    ego_px, ego_py = _bev_to_pixel(0, 0, x_range, y_range, w, h)
    cv2.rectangle(
        img,
        (ego_px - 4, ego_py - 8),
        (ego_px + 4, ego_py),
        (0, 120, 200), -1,
    )


def render_vision_bev_points(
    img: np.ndarray,
    vision_bev_pts: np.ndarray,
    x_range: Tuple[float, float],
    y_range: Tuple[float, float],
    color: Tuple[int, int, int] = (0, 200, 0),
    radius: int = 4,
) -> np.ndarray:
    """Draw back-projected vision points on a BEV image.

    Args:
        img: BEV image to draw on (will be copied).
        vision_bev_pts: Nx2 array of BEV [x, y] coordinates.
        x_range, y_range: BEV coordinate ranges.
        color: BGR color for the points.
        radius: Point radius.

    Returns:
        Image with vision BEV points drawn.
    """
    result = img.copy()
    h, w = result.shape[:2]

    for pt in vision_bev_pts:
        px, py = _bev_to_pixel(pt[0], pt[1], x_range, y_range, w, h)
        if 0 <= px < w and 0 <= py < h:
            cv2.drawMarker(
                result, (px, py), color,
                cv2.MARKER_DIAMOND, radius * 2, 2, cv2.LINE_AA,
            )

    return result


def render_effective_zones(
    img: np.ndarray,
    zone_data: dict,
    x_range: Tuple[float, float],
    y_range: Tuple[float, float],
) -> np.ndarray:
    """Draw effective and blind zone polygons on a BEV image.

    Uses the output of compute_effective_detection_area.

    Args:
        img: BEV image to draw on (will be copied).
        zone_data: Dict with keys 'radar_effective_detection_polygon',
            'radar_blind_zone_polygon', 'radar_observed_coverage_polygon'.
        x_range, y_range: BEV coordinate ranges.

    Returns:
        Image with zone polygons drawn.
    """
    result = img.copy()
    h, w = result.shape[:2]

    # Effective zone — green, semi-transparent
    _draw_zone_polygons(
        result,
        zone_data.get("radar_effective_detection_polygon", []),
        x_range, y_range, w, h,
        color=(0, 180, 0), alpha=0.2,
    )

    # Blind zone — red, semi-transparent
    _draw_zone_polygons(
        result,
        zone_data.get("radar_blind_zone_polygon", []),
        x_range, y_range, w, h,
        color=(0, 0, 200), alpha=0.3,
    )

    return result


def _draw_zone_polygons(
    img: np.ndarray,
    polygons: list,
    x_range, y_range, w, h,
    color, alpha,
):
    """Draw zone polygons with semi-transparent fill."""
    for poly_vertices in polygons:
        if len(poly_vertices) < 3:
            continue

        pts_px = []
        for vertex in poly_vertices:
            vx, vy = vertex[0], vertex[1]
            px, py = _bev_to_pixel(vx, vy, x_range, y_range, w, h)
            pts_px.append([px, py])

        pts_arr = np.array(pts_px, dtype=np.int32)

        overlay = img.copy()
        cv2.fillPoly(overlay, [pts_arr], color)
        cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)
        cv2.polylines(img, [pts_arr], True, color, 1, cv2.LINE_AA)
