"""
hungarian_matcher.py - Hungarian (Kuhn-Munkres) matching for radar-vision association.

Uses scipy.optimize.linear_sum_assignment to find the optimal
minimum-cost assignment between radar and vision detections.
"""

import numpy as np
from typing import List
from scipy.optimize import linear_sum_assignment

from .association_models import Match, AssociationResult


def hungarian_match(
    cost_matrix: np.ndarray,
    gate_matrix: np.ndarray,
    bev_distances: np.ndarray,
    pixel_distances: np.ndarray,
    max_cost: float = 0.8,
) -> AssociationResult:
    """Perform Hungarian matching on a cost matrix.

    Args:
        cost_matrix: NxM cost matrix (radar × vision).
            Values should be in [0, 1] for valid pairs, np.inf for gated-out.
        gate_matrix: NxM boolean gate matrix.
        bev_distances: NxM raw BEV distance matrix (for result metadata).
        pixel_distances: NxM raw pixel distance matrix (for result metadata).
        max_cost: Maximum allowed cost for a valid match.
            Matches above this threshold are discarded.

    Returns:
        AssociationResult with matches, unmatched_radar, unmatched_vision.
    """
    n_radar, n_vision = cost_matrix.shape

    if n_radar == 0 or n_vision == 0:
        return AssociationResult(
            matches=[],
            unmatched_radar=list(range(n_radar)),
            unmatched_vision=list(range(n_vision)),
            cost_matrix=cost_matrix,
            gate_matrix=gate_matrix,
        )

    # Replace inf with a large finite value for scipy
    cost_finite = cost_matrix.copy()
    large_val = 1e6
    cost_finite[np.isinf(cost_finite)] = large_val

    # Run Hungarian algorithm
    row_indices, col_indices = linear_sum_assignment(cost_finite)

    # Filter matches by max_cost and gate
    matches = []
    matched_radar = set()
    matched_vision = set()

    for r, c in zip(row_indices, col_indices):
        if cost_matrix[r, c] <= max_cost and gate_matrix[r, c]:
            match = Match(
                radar_idx=int(r),
                vision_idx=int(c),
                cost=float(cost_matrix[r, c]),
                bev_distance=float(bev_distances[r, c]),
                pixel_distance=float(pixel_distances[r, c]),
            )
            matches.append(match)
            matched_radar.add(r)
            matched_vision.add(c)

    # Determine unmatched
    unmatched_radar = [i for i in range(n_radar) if i not in matched_radar]
    unmatched_vision = [j for j in range(n_vision) if j not in matched_vision]

    return AssociationResult(
        matches=matches,
        unmatched_radar=unmatched_radar,
        unmatched_vision=unmatched_vision,
        cost_matrix=cost_matrix,
        gate_matrix=gate_matrix,
    )
