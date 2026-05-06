"""
point_filter.py - Filter radar points for range estimation.

Provides basic range filtering and DBSCAN-based noise removal.
"""

import numpy as np
from typing import Tuple, List

from sklearn.cluster import DBSCAN


def basic_filter_points(
    points: np.ndarray,
    x_range: Tuple[float, float] = (-100, 100),
    y_range: Tuple[float, float] = (0, 300),
) -> np.ndarray:
    """Filter points by coordinate range to remove obvious outliers.

    Args:
        points: Nx2 array of [x, y] coordinates.
        x_range: (x_min, x_max) lateral range in meters.
        y_range: (y_min, y_max) forward range in meters.

    Returns:
        Filtered Nx2 array of points within range.
    """
    if len(points) == 0:
        return points

    mask = (
        (points[:, 0] >= x_range[0]) & (points[:, 0] <= x_range[1]) &
        (points[:, 1] >= y_range[0]) & (points[:, 1] <= y_range[1])
    )

    filtered = points[mask]
    removed = len(points) - len(filtered)
    if removed > 0:
        print(f"[PointFilter] Basic filter removed {removed} points "
              f"(x: {x_range}, y: {y_range})")
    return filtered


def dbscan_filter_points(
    points: np.ndarray,
    eps: float = 3.0,
    min_samples: int = 10,
) -> np.ndarray:
    """Remove isolated noise points using DBSCAN clustering.

    Points labeled as noise (label == -1) by DBSCAN are removed.

    Args:
        points: Nx2 array of [x, y] coordinates.
        eps: Maximum distance between two samples in the same neighborhood.
        min_samples: Minimum number of samples in a neighborhood for a core point.

    Returns:
        Filtered array with noise points removed.
    """
    if len(points) < min_samples:
        print(f"[PointFilter] Too few points ({len(points)}) for DBSCAN, skipping")
        return points

    db = DBSCAN(eps=eps, min_samples=min_samples)
    labels = db.fit_predict(points)

    # Keep only non-noise points (label != -1)
    valid_mask = labels != -1
    filtered = points[valid_mask]

    n_noise = np.sum(~valid_mask)
    n_clusters = len(set(labels)) - (1 if -1 in labels else 0)
    print(f"[PointFilter] DBSCAN: {n_clusters} clusters, "
          f"{n_noise} noise points removed, {len(filtered)} remaining")

    return filtered
