"""
range_estimator.py - High-level radar range estimation pipeline.

Orchestrates the full pipeline:
1. Basic point filtering
2. DBSCAN noise removal
3. Grid density computation
4. Valid mask extraction
5. Contour -> polygon extraction
"""

import numpy as np
from typing import List, Dict, Any

from .point_filter import basic_filter_points, dbscan_filter_points
from .density_grid import build_occupancy_grid, extract_valid_grid
from .contour_extractor import mask_to_polygons


def estimate_radar_range(
    points: np.ndarray,
    config: Dict[str, Any] = None,
) -> Dict[str, Any]:
    """Estimate radar detection range from trajectory points.

    Full pipeline from raw points to polygon output.

    Args:
        points: Nx2 array of all radar trajectory [x, y] points.
        config: Configuration dict with keys:
            - x_range: [x_min, x_max], default [-100, 100]
            - y_range: [y_min, y_max], default [0, 300]
            - resolution: Grid cell size in meters, default 1.0
            - min_count: Min detections per grid cell, default 3
            - dbscan_eps: DBSCAN epsilon, default 3.0
            - dbscan_min_samples: DBSCAN min_samples, default 10
            - min_area_m2: Min polygon area in m², default 20.0

    Returns:
        Dictionary with:
            - "range_polygons": List of polygon dicts.
            - "method": Method parameters used.
            - "stats": Statistics (total_points, filtered_points, etc.).
    """
    if config is None:
        config = {}

    # Extract config with defaults
    x_range = tuple(config.get("x_range", [-100, 100]))
    y_range = tuple(config.get("y_range", [0, 300]))
    resolution = config.get("resolution", 1.0)
    min_count = config.get("min_count", 3)
    dbscan_eps = config.get("dbscan_eps", 3.0)
    dbscan_min_samples = config.get("dbscan_min_samples", 10)
    min_area_m2 = config.get("min_area_m2", 20.0)

    print(f"[RangeEstimator] Starting with {len(points)} raw points")

    # Step 1: Basic range filter
    filtered = basic_filter_points(points, x_range, y_range)

    # Step 2: DBSCAN noise removal
    denoised = dbscan_filter_points(filtered, dbscan_eps, dbscan_min_samples)

    # Step 3: Build occupancy grid
    grid, x_min, y_min = build_occupancy_grid(
        denoised, x_range, y_range, resolution
    )

    # Step 4: Extract valid cells
    valid_mask = extract_valid_grid(grid, min_count)

    # Step 5: Extract polygons
    polygons = mask_to_polygons(
        valid_mask, x_min, y_min, resolution, min_area_m2
    )

    # Build result
    method_info = {
        "name": "grid_density",
        "x_range": list(x_range),
        "y_range": list(y_range),
        "resolution": resolution,
        "min_count": min_count,
        "min_track_len": config.get("min_track_len", 5),
        "dbscan_eps": dbscan_eps,
        "dbscan_min_samples": dbscan_min_samples,
    }

    stats = {
        "total_input_points": len(points),
        "after_basic_filter": len(filtered),
        "after_dbscan": len(denoised),
        "num_polygons": len(polygons),
    }

    result = {
        "version": "1.0",
        "type": "radar_detection_range",
        "coordinate_system": {
            "x": "lateral distance, unit: m",
            "y": "forward distance, unit: m",
        },
        "method": method_info,
        "range_polygons": polygons,
        "stats": stats,
    }

    print(f"[RangeEstimator] Result: {len(polygons)} polygons")
    return result
