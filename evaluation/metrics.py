"""
metrics.py - Compute evaluation metrics for radar-vision association.

Computes:
- radar_recall, radar_fp_rate
- BEV error statistics (mean, lateral, longitudinal, percentiles)
- Distance-binned metrics
- Lane-segmented metrics
- Grid-level metrics
"""

import numpy as np
from typing import List, Dict, Any, Optional

from data.data_models import RadarObject
from vision.vision_models import VisionObject, extract_vision_bottom_centers
from calibration.projector import project_image_to_radar
from association.association_models import Match, AssociationResult

from .eval_models import (
    EvaluationResult, DistanceBinMetrics, LaneMetrics, GridMetrics,
)

# Default distance bins for evaluation
DEFAULT_DISTANCE_BINS = [
    (0, 30, "0-30m"),
    (30, 60, "30-60m"),
    (60, 100, "60-100m"),
    (100, 150, "100-150m"),
    (150, 300, "150m+"),
]

# Default lane definitions (lateral x ranges)
DEFAULT_LANES = [
    ("left_2", [-14.0, -7.0]),
    ("left_1", [-7.0, -3.5]),
    ("center", [-3.5, 3.5]),
    ("right_1", [3.5, 7.0]),
    ("right_2", [7.0, 14.0]),
]


def compute_association_metrics(
    association_result: AssociationResult,
    radar_objects: List[RadarObject],
    vision_objects: List[VisionObject],
    H_inv: np.ndarray,
    config: Optional[Dict[str, Any]] = None,
) -> EvaluationResult:
    """Compute comprehensive evaluation metrics from association result.

    Args:
        association_result: Output from associate_radar_vision.
        radar_objects: List of radar objects used in association.
        vision_objects: List of vision objects used in association.
        H_inv: 3x3 inverse homography (image -> radar BEV).
        config: Optional config with keys:
            - distance_bins: List of (min, max, label) tuples.
            - lanes: List of (lane_id, [x_min, x_max]) tuples.
            - grid_size: Grid cell size for grid metrics (meters).
            - grid_x_range: [x_min, x_max] for grid.
            - grid_y_range: [y_min, y_max] for grid.

    Returns:
        EvaluationResult with all metrics computed.
    """
    cfg = config or {}

    matches = association_result.matches
    unmatched_radar = association_result.unmatched_radar
    unmatched_vision = association_result.unmatched_vision

    n_radar = len(radar_objects)
    n_vision = len(vision_objects)
    n_matched = len(matches)

    # --- Basic metrics ---
    radar_recall = n_matched / n_vision if n_vision > 0 else 0.0
    radar_fp_rate = len(unmatched_radar) / n_radar if n_radar > 0 else 0.0

    # --- Compute BEV errors for matched pairs ---
    radar_bev = np.array(
        [[obj.x, obj.y] for obj in radar_objects], dtype=np.float64,
    ) if radar_objects else np.empty((0, 2))

    vision_bottom_uv = extract_vision_bottom_centers(vision_objects)
    vision_bev = (
        project_image_to_radar(vision_bottom_uv, H_inv)
        if len(vision_bottom_uv) > 0
        else np.empty((0, 2))
    )

    bev_errors = []
    lateral_errors = []
    longitudinal_errors = []

    for m in matches:
        r_pt = radar_bev[m.radar_idx]
        v_pt = vision_bev[m.vision_idx]
        dx = abs(r_pt[0] - v_pt[0])
        dy = abs(r_pt[1] - v_pt[1])
        bev_errors.append(np.sqrt(dx ** 2 + dy ** 2))
        lateral_errors.append(dx)
        longitudinal_errors.append(dy)

    bev_errors_arr = np.array(bev_errors) if bev_errors else np.array([0.0])
    lat_errors_arr = np.array(lateral_errors) if lateral_errors else np.array([0.0])
    lon_errors_arr = np.array(longitudinal_errors) if longitudinal_errors else np.array([0.0])

    mean_bev = float(np.mean(bev_errors_arr)) if n_matched > 0 else 0.0
    mean_lat = float(np.mean(lat_errors_arr)) if n_matched > 0 else 0.0
    mean_lon = float(np.mean(lon_errors_arr)) if n_matched > 0 else 0.0
    p50_bev = float(np.percentile(bev_errors_arr, 50)) if n_matched > 0 else 0.0
    p90_bev = float(np.percentile(bev_errors_arr, 90)) if n_matched > 0 else 0.0
    p95_bev = float(np.percentile(bev_errors_arr, 95)) if n_matched > 0 else 0.0

    # --- Distance-binned metrics ---
    dist_bins_cfg = cfg.get("distance_bins", DEFAULT_DISTANCE_BINS)
    distance_bins = _compute_distance_bin_metrics(
        matches, radar_objects, vision_objects,
        radar_bev, vision_bev, dist_bins_cfg,
    )

    # --- Lane metrics ---
    lanes_cfg = cfg.get("lanes", DEFAULT_LANES)
    lane_metrics = _compute_lane_metrics(
        matches, radar_objects, vision_objects,
        radar_bev, vision_bev, lanes_cfg,
    )

    # --- Grid metrics ---
    grid_size = cfg.get("grid_size", 5.0)
    grid_x_range = cfg.get("grid_x_range", [-15, 15])
    grid_y_range = cfg.get("grid_y_range", [0, 200])
    grid_metrics = _compute_grid_metrics(
        matches, radar_objects, vision_objects,
        radar_bev, vision_bev, grid_size, grid_x_range, grid_y_range,
    )

    return EvaluationResult(
        radar_recall=radar_recall,
        radar_fp_rate=radar_fp_rate,
        mean_bev_error=mean_bev,
        mean_lateral_error=mean_lat,
        mean_longitudinal_error=mean_lon,
        p50_bev_error=p50_bev,
        p90_bev_error=p90_bev,
        p95_bev_error=p95_bev,
        total_radar=n_radar,
        total_vision=n_vision,
        total_matched=n_matched,
        distance_bins=distance_bins,
        lane_metrics=lane_metrics,
        grid_metrics=grid_metrics,
    )


def _compute_distance_bin_metrics(
    matches, radar_objects, vision_objects,
    radar_bev, vision_bev, bins,
) -> List[DistanceBinMetrics]:
    """Compute metrics per distance range bin."""
    results = []
    for d_min, d_max, label in bins:
        radar_in_bin = [
            i for i, obj in enumerate(radar_objects) if d_min <= obj.y < d_max
        ]
        vision_in_bin = []
        for j in range(len(vision_objects)):
            if j < len(vision_bev):
                if d_min <= vision_bev[j, 1] < d_max:
                    vision_in_bin.append(j)

        matches_in_bin = [m for m in matches if m.radar_idx in set(radar_in_bin)]
        n_vis = len(vision_in_bin)
        n_rad = len(radar_in_bin)
        n_match = len(matches_in_bin)

        bev_errs, lat_errs, lon_errs = [], [], []
        for m in matches_in_bin:
            r_pt = radar_bev[m.radar_idx]
            v_pt = vision_bev[m.vision_idx]
            dx = abs(r_pt[0] - v_pt[0])
            dy = abs(r_pt[1] - v_pt[1])
            bev_errs.append(np.sqrt(dx ** 2 + dy ** 2))
            lat_errs.append(dx)
            lon_errs.append(dy)

        matched_vis_set = {m.vision_idx for m in matches_in_bin} & set(vision_in_bin)
        recall = len(matched_vis_set) / n_vis if n_vis > 0 else 0.0
        fp_rate = (n_rad - n_match) / n_rad if n_rad > 0 else 0.0

        results.append(DistanceBinMetrics(
            range_str=label, range_min=d_min, range_max=d_max,
            total_vision=n_vis, total_radar=n_rad, matched_count=n_match,
            recall=recall, fp_rate=fp_rate,
            mean_bev_error=float(np.mean(bev_errs)) if bev_errs else 0.0,
            mean_lateral_error=float(np.mean(lat_errs)) if lat_errs else 0.0,
            mean_longitudinal_error=float(np.mean(lon_errs)) if lon_errs else 0.0,
        ))
    return results


def _compute_lane_metrics(
    matches, radar_objects, vision_objects,
    radar_bev, vision_bev, lanes,
) -> List[LaneMetrics]:
    """Compute metrics per lane segment."""
    results = []
    for lane_id, (x_min, x_max) in lanes:
        vision_in_lane = [
            j for j in range(len(vision_objects))
            if j < len(vision_bev) and x_min <= vision_bev[j, 0] < x_max
        ]
        matches_in_lane = [
            m for m in matches
            if m.radar_idx < len(radar_bev) and x_min <= radar_bev[m.radar_idx, 0] < x_max
        ]
        n_vis = len(vision_in_lane)
        n_match = len(matches_in_lane)

        bev_errs = []
        for m in matches_in_lane:
            bev_errs.append(float(np.linalg.norm(
                radar_bev[m.radar_idx] - vision_bev[m.vision_idx]
            )))

        matched_vis_set = {m.vision_idx for m in matches_in_lane} & set(vision_in_lane)
        recall = len(matched_vis_set) / n_vis if n_vis > 0 else 0.0

        results.append(LaneMetrics(
            lane_id=lane_id, x_range=[x_min, x_max],
            total_vision=n_vis, matched_count=n_match,
            recall=recall,
            mean_bev_error=float(np.mean(bev_errs)) if bev_errs else 0.0,
        ))
    return results


def _compute_grid_metrics(
    matches, radar_objects, vision_objects,
    radar_bev, vision_bev, grid_size, x_range, y_range,
) -> GridMetrics:
    """Compute grid-level metrics."""
    x_min, x_max = x_range
    y_min, y_max = y_range
    nx = int(np.ceil((x_max - x_min) / grid_size))
    ny = int(np.ceil((y_max - y_min) / grid_size))

    vision_grid = np.zeros((nx, ny), dtype=int)
    match_grid = np.zeros((nx, ny), dtype=int)
    match_bev_sum = np.zeros((nx, ny), dtype=np.float64)

    for j in range(len(vision_objects)):
        if j < len(vision_bev):
            vx, vy = vision_bev[j]
            ix = int((vx - x_min) / grid_size)
            iy = int((vy - y_min) / grid_size)
            if 0 <= ix < nx and 0 <= iy < ny:
                vision_grid[ix, iy] += 1

    for m in matches:
        if m.radar_idx < len(radar_bev):
            rx, ry = radar_bev[m.radar_idx]
            ix = int((rx - x_min) / grid_size)
            iy = int((ry - y_min) / grid_size)
            if 0 <= ix < nx and 0 <= iy < ny:
                match_grid[ix, iy] += 1
                if m.vision_idx < len(vision_bev):
                    match_bev_sum[ix, iy] += float(np.linalg.norm(
                        radar_bev[m.radar_idx] - vision_bev[m.vision_idx]
                    ))

    cells = []
    for ix in range(nx):
        for iy in range(ny):
            vc = int(vision_grid[ix, iy])
            mc = int(match_grid[ix, iy])
            if vc > 0 or mc > 0:
                recall = mc / vc if vc > 0 else 0.0
                mean_err = match_bev_sum[ix, iy] / mc if mc > 0 else 0.0
                cells.append({
                    "x": round(x_min + (ix + 0.5) * grid_size, 1),
                    "y": round(y_min + (iy + 0.5) * grid_size, 1),
                    "vision_count": vc, "matched_count": mc,
                    "recall": round(recall, 4),
                    "mean_bev_error": round(mean_err, 4),
                })

    return GridMetrics(
        grid_size=grid_size,
        x_range=list(x_range), y_range=list(y_range),
        cells=cells,
    )
