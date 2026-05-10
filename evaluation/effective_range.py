"""
effective_range.py - Radar effective detection area and blind zone analysis.

Computes grid-based recall analysis to identify:
1. radar_observed_coverage_polygon - where radar has any detections
2. radar_effective_detection_polygon - where radar reliably detects (high recall)
3. radar_blind_zone_polygon - where radar fails to detect (low recall)
"""

import json
import numpy as np
from pathlib import Path
from typing import Dict, Any, List, Optional, Tuple

from calibration.projector import project_image_to_radar
from vision.vision_models import VisionObject, extract_vision_bottom_centers
from association.association_models import Match


# Default configuration
DEFAULT_CONFIG = {
    "grid_size": 5.0,
    "x_range": [-20, 20],
    "y_range": [0, 200],
    "min_vision_count": 3,
    "min_recall": 0.6,
    "blind_zone_max_recall": 0.2,
}


def compute_effective_detection_area(
    all_matches: List[List[Match]],
    all_vision_objects: List[List[VisionObject]],
    all_radar_bev: List[np.ndarray],
    H_inv: np.ndarray,
    config: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """Compute effective detection area from multi-frame association results.

    Pipeline:
    1. Back-project all vision bbox bottom centers to BEV.
    2. Grid: count vision_count per cell.
    3. Grid: count matched_count per cell.
    4. Compute per-cell recall = matched_count / vision_count.
    5. Effective zone: vision_count >= min AND recall >= min_recall.
    6. Blind zone: vision_count >= min AND recall <= blind_zone_max_recall.
    7. Convert grid masks to polygons.

    Args:
        all_matches: List of per-frame Match lists.
        all_vision_objects: List of per-frame VisionObject lists.
        all_radar_bev: List of per-frame Nx2 radar BEV point arrays.
        H_inv: 3x3 inverse homography (image → radar BEV).
        config: Configuration dictionary.

    Returns:
        Dictionary with polygon data and statistics.
    """
    cfg = {**DEFAULT_CONFIG}
    if config:
        cfg.update(config)

    grid_size = cfg["grid_size"]
    x_min, x_max = cfg["x_range"]
    y_min, y_max = cfg["y_range"]
    min_vc = cfg["min_vision_count"]
    min_recall = cfg["min_recall"]
    blind_max = cfg["blind_zone_max_recall"]

    nx = int(np.ceil((x_max - x_min) / grid_size))
    ny = int(np.ceil((y_max - y_min) / grid_size))

    vision_grid = np.zeros((nx, ny), dtype=int)
    matched_grid = np.zeros((nx, ny), dtype=int)
    radar_grid = np.zeros((nx, ny), dtype=int)

    # Accumulate across all frames
    for frame_idx in range(len(all_vision_objects)):
        vobjs = all_vision_objects[frame_idx]
        matches = all_matches[frame_idx] if frame_idx < len(all_matches) else []

        # Vision bottom centers → BEV
        bottom_uv = extract_vision_bottom_centers(vobjs)
        if len(bottom_uv) > 0:
            v_bev = project_image_to_radar(bottom_uv, H_inv)
            for j in range(len(v_bev)):
                vx, vy = v_bev[j]
                ix = int((vx - x_min) / grid_size)
                iy = int((vy - y_min) / grid_size)
                if 0 <= ix < nx and 0 <= iy < ny:
                    vision_grid[ix, iy] += 1

        # Radar points
        if frame_idx < len(all_radar_bev) and len(all_radar_bev[frame_idx]) > 0:
            rbev = all_radar_bev[frame_idx]
            for k in range(len(rbev)):
                rx, ry = rbev[k]
                ix = int((rx - x_min) / grid_size)
                iy = int((ry - y_min) / grid_size)
                if 0 <= ix < nx and 0 <= iy < ny:
                    radar_grid[ix, iy] += 1

        # Matched radar points
        for m in matches:
            if frame_idx < len(all_radar_bev) and m.radar_idx < len(all_radar_bev[frame_idx]):
                rx, ry = all_radar_bev[frame_idx][m.radar_idx]
                ix = int((rx - x_min) / grid_size)
                iy = int((ry - y_min) / grid_size)
                if 0 <= ix < nx and 0 <= iy < ny:
                    matched_grid[ix, iy] += 1

    # Compute recall grid
    recall_grid = np.zeros((nx, ny), dtype=np.float64)
    has_data = vision_grid > 0
    recall_grid[has_data] = matched_grid[has_data] / vision_grid[has_data]

    # Zone masks
    sufficient_data = vision_grid >= min_vc
    observed_mask = radar_grid > 0
    effective_mask = sufficient_data & (recall_grid >= min_recall)
    blind_mask = sufficient_data & (recall_grid <= blind_max)

    # Convert masks to polygons
    observed_polys = _mask_to_rect_polygons(observed_mask, x_min, y_min, grid_size)
    effective_polys = _mask_to_rect_polygons(effective_mask, x_min, y_min, grid_size)
    blind_polys = _mask_to_rect_polygons(blind_mask, x_min, y_min, grid_size)

    result = {
        "version": "1.0",
        "type": "radar_effective_detection_area",
        "config": cfg,
        "radar_observed_coverage_polygon": observed_polys,
        "radar_effective_detection_polygon": effective_polys,
        "radar_blind_zone_polygon": blind_polys,
        "stats": {
            "total_grid_cells": int(nx * ny),
            "observed_cells": int(observed_mask.sum()),
            "effective_cells": int(effective_mask.sum()),
            "blind_zone_cells": int(blind_mask.sum()),
            "total_vision_detections": int(vision_grid.sum()),
            "total_matched": int(matched_grid.sum()),
            "overall_recall": float(matched_grid.sum() / vision_grid.sum())
                if vision_grid.sum() > 0 else 0.0,
        },
    }

    return result


def _mask_to_rect_polygons(
    mask: np.ndarray,
    x_min: float,
    y_min: float,
    grid_size: float,
) -> List[List[List[float]]]:
    """Convert a 2D boolean mask to rectangular polygon vertices.

    Each True cell becomes a rectangle polygon. Adjacent cells are
    kept as separate rectangles for simplicity.

    Args:
        mask: NxM boolean mask.
        x_min: X origin of grid.
        y_min: Y origin of grid.
        grid_size: Cell size in meters.

    Returns:
        List of polygons, each polygon is a list of [x, y] vertices.
    """
    polygons = []
    nx, ny = mask.shape

    # Group connected cells into simple polygons using contours
    try:
        import cv2
        mask_uint8 = mask.astype(np.uint8) * 255
        # Transpose because cv2 uses (rows=y, cols=x)
        mask_img = mask_uint8.T
        contours, _ = cv2.findContours(
            mask_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        for cnt in contours:
            if len(cnt) < 3:
                continue
            poly = []
            for pt in cnt:
                px, py = pt[0]
                real_x = x_min + px * grid_size
                real_y = y_min + py * grid_size
                poly.append([round(real_x, 2), round(real_y, 2)])
            if len(poly) >= 3:
                polygons.append(poly)
    except Exception:
        # Fallback: output individual cell rectangles
        for ix in range(nx):
            for iy in range(ny):
                if mask[ix, iy]:
                    x0 = x_min + ix * grid_size
                    y0 = y_min + iy * grid_size
                    x1 = x0 + grid_size
                    y1 = y0 + grid_size
                    polygons.append([
                        [x0, y0], [x1, y0], [x1, y1], [x0, y1],
                    ])

    return polygons


def save_effective_range_json(path: Path, result: Dict[str, Any]):
    """Save effective detection area result to JSON.

    Args:
        path: Output file path.
        result: Result dictionary from compute_effective_detection_area.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    with open(path, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2, ensure_ascii=False)

    print(f"[EffectiveRange] Saved to {path}")
