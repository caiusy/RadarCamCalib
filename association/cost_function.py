"""
cost_function.py - Cost matrix computation for radar-vision association.

Computes normalized BEV and pixel distances, then combines them
into a weighted cost matrix for Hungarian matching.

Cost formula:
    cost = bev_weight * normalized_bev_distance + pixel_weight * normalized_pixel_distance
"""

import numpy as np
from typing import Tuple


def compute_bev_distances(
    radar_pts: np.ndarray,
    vision_bev_pts: np.ndarray,
) -> np.ndarray:
    """Compute pairwise BEV Euclidean distances.

    Args:
        radar_pts: Nx2 array of radar BEV [x, y] coordinates.
        vision_bev_pts: Mx2 array of vision back-projected BEV [x, y].

    Returns:
        NxM distance matrix in meters.
    """
    n_radar = len(radar_pts)
    n_vision = len(vision_bev_pts)

    if n_radar == 0 or n_vision == 0:
        return np.zeros((n_radar, n_vision), dtype=np.float64)

    diff = radar_pts[:, np.newaxis, :] - vision_bev_pts[np.newaxis, :, :]
    distances = np.linalg.norm(diff, axis=2)
    return distances


def compute_pixel_distances(
    radar_proj_uv: np.ndarray,
    vision_bottom_uv: np.ndarray,
) -> np.ndarray:
    """Compute pairwise pixel Euclidean distances.

    Args:
        radar_proj_uv: Nx2 array of radar projected [u, v] in image.
        vision_bottom_uv: Mx2 array of vision bbox bottom centers [u, v].

    Returns:
        NxM distance matrix in pixels.
    """
    n_radar = len(radar_proj_uv)
    n_vision = len(vision_bottom_uv)

    if n_radar == 0 or n_vision == 0:
        return np.zeros((n_radar, n_vision), dtype=np.float64)

    diff = radar_proj_uv[:, np.newaxis, :] - vision_bottom_uv[np.newaxis, :, :]
    distances = np.linalg.norm(diff, axis=2)
    return distances


def normalize_matrix(matrix: np.ndarray) -> np.ndarray:
    """Normalize a distance matrix to [0, 1] range.

    Uses min-max normalization. If all values are the same,
    returns a zero matrix.

    Args:
        matrix: NxM distance matrix.

    Returns:
        NxM normalized matrix.
    """
    if matrix.size == 0:
        return matrix.copy()

    min_val = matrix.min()
    max_val = matrix.max()
    range_val = max_val - min_val

    if range_val < 1e-10:
        return np.zeros_like(matrix)

    return (matrix - min_val) / range_val


def compute_cost_matrix(
    bev_distances: np.ndarray,
    pixel_distances: np.ndarray,
    gate_matrix: np.ndarray,
    bev_weight: float = 0.7,
    pixel_weight: float = 0.3,
) -> np.ndarray:
    """Compute weighted cost matrix from BEV and pixel distances.

    Applies gating mask by setting gated-out pairs to infinity.

    Args:
        bev_distances: NxM BEV distance matrix.
        pixel_distances: NxM pixel distance matrix.
        gate_matrix: NxM boolean gate matrix.
        bev_weight: Weight for BEV distance component (default 0.7).
        pixel_weight: Weight for pixel distance component (default 0.3).

    Returns:
        NxM cost matrix. Values in [0, 1] for valid pairs, np.inf for gated-out.
    """
    # Normalize distances
    norm_bev = normalize_matrix(bev_distances)
    norm_pixel = normalize_matrix(pixel_distances)

    # Weighted combination
    cost = bev_weight * norm_bev + pixel_weight * norm_pixel

    # Apply gating: set gated-out pairs to infinity
    cost[~gate_matrix] = np.inf

    return cost
