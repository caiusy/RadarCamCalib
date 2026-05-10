"""
gating.py - Gating functions for radar-vision association.

Provides three gating stages to filter out unlikely matches
before the cost matrix is constructed:
1. Image gating: radar projection falls inside expanded vision bbox
2. BEV gating: BEV distance within threshold
3. Class compatibility: matching object types
"""

import numpy as np
from typing import List, Optional


# Default class compatibility map
# Each key maps to compatible classes
CLASS_COMPAT = {
    "vehicle": {"vehicle", "car", "truck", "bus"},
    "car": {"vehicle", "car"},
    "truck": {"vehicle", "truck"},
    "bus": {"vehicle", "bus"},
    "cyclist": {"cyclist", "non_motor"},
    "pedestrian": {"pedestrian"},
    "unknown": {"vehicle", "car", "truck", "bus", "cyclist", "pedestrian", "unknown"},
}


def image_gating(
    radar_proj_uv: np.ndarray,
    vision_bboxes: np.ndarray,
    margin: float = 20.0,
) -> np.ndarray:
    """Check if radar projection points fall inside expanded vision bboxes.

    Args:
        radar_proj_uv: Nx2 array of radar projected [u, v] in image.
        vision_bboxes: Mx4 array of vision bboxes [x1, y1, x2, y2].
        margin: Bbox expansion margin in pixels.

    Returns:
        NxM boolean matrix. gate[i, j] = True if radar i is inside
        expanded bbox j.
    """
    n_radar = len(radar_proj_uv)
    n_vision = len(vision_bboxes)

    if n_radar == 0 or n_vision == 0:
        return np.zeros((n_radar, n_vision), dtype=bool)

    # Expand bboxes
    expanded = vision_bboxes.copy()
    expanded[:, 0] -= margin   # x1
    expanded[:, 1] -= margin   # y1
    expanded[:, 2] += margin   # x2
    expanded[:, 3] += margin   # y2

    # Vectorized check: radar_u in [x1, x2] AND radar_v in [y1, y2]
    # radar_proj_uv: (N, 2), expanded: (M, 4)
    u = radar_proj_uv[:, 0][:, np.newaxis]  # (N, 1)
    v = radar_proj_uv[:, 1][:, np.newaxis]  # (N, 1)

    x1 = expanded[:, 0][np.newaxis, :]  # (1, M)
    y1 = expanded[:, 1][np.newaxis, :]
    x2 = expanded[:, 2][np.newaxis, :]
    y2 = expanded[:, 3][np.newaxis, :]

    gate = (u >= x1) & (u <= x2) & (v >= y1) & (v <= y2)  # (N, M)
    return gate


def bev_gating(
    radar_pts_bev: np.ndarray,
    vision_pts_bev: np.ndarray,
    threshold: float = 5.0,
) -> np.ndarray:
    """Check if BEV distance between radar and vision points is within threshold.

    Args:
        radar_pts_bev: Nx2 array of radar BEV [x, y] coordinates.
        vision_pts_bev: Mx2 array of vision back-projected BEV [x, y].
        threshold: Maximum BEV distance in meters.

    Returns:
        NxM boolean matrix. gate[i, j] = True if distance < threshold.
    """
    n_radar = len(radar_pts_bev)
    n_vision = len(vision_pts_bev)

    if n_radar == 0 or n_vision == 0:
        return np.zeros((n_radar, n_vision), dtype=bool)

    # Compute pairwise distances
    # radar: (N, 1, 2), vision: (1, M, 2)
    diff = radar_pts_bev[:, np.newaxis, :] - vision_pts_bev[np.newaxis, :, :]
    distances = np.linalg.norm(diff, axis=2)  # (N, M)

    return distances < threshold


def class_compatibility(
    radar_classes: List[str],
    vision_classes: List[str],
    compat_map: Optional[dict] = None,
) -> np.ndarray:
    """Check class compatibility between radar and vision objects.

    Args:
        radar_classes: List of N radar object class strings.
        vision_classes: List of M vision object class strings.
        compat_map: Optional custom compatibility map. Uses default if None.

    Returns:
        NxM boolean matrix. gate[i, j] = True if classes are compatible.
    """
    if compat_map is None:
        compat_map = CLASS_COMPAT

    n_radar = len(radar_classes)
    n_vision = len(vision_classes)

    if n_radar == 0 or n_vision == 0:
        return np.zeros((n_radar, n_vision), dtype=bool)

    gate = np.zeros((n_radar, n_vision), dtype=bool)

    for i, rc in enumerate(radar_classes):
        compat_set = compat_map.get(rc, {rc, "unknown"})
        for j, vc in enumerate(vision_classes):
            gate[i, j] = vc in compat_set or rc in compat_map.get(vc, {vc, "unknown"})

    return gate


def combined_gating(
    image_gate: np.ndarray,
    bev_gate: np.ndarray,
    class_gate: np.ndarray,
) -> np.ndarray:
    """Combine multiple gating matrices with logical AND.

    A pair passes the combined gate only if it passes ALL individual gates.

    Args:
        image_gate: NxM boolean matrix from image_gating.
        bev_gate: NxM boolean matrix from bev_gating.
        class_gate: NxM boolean matrix from class_compatibility.

    Returns:
        NxM boolean matrix — combined gate.
    """
    return image_gate & bev_gate & class_gate
