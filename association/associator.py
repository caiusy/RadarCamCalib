"""
associator.py - Full radar-vision association pipeline.

Orchestrates the complete association workflow:
1. Project radar points to image coordinates via homography
2. Compute vision bbox bottom centers
3. Back-project vision bottom centers to BEV via inverse homography
4. Apply image, BEV, and class gating
5. Compute cost matrix
6. Run Hungarian matching
"""

import numpy as np
from typing import Dict, Any, List, Optional

from data.data_models import RadarObject
from vision.vision_models import VisionObject, extract_vision_bottom_centers
from calibration.projector import project_radar_to_image, project_image_to_radar

from .gating import image_gating, bev_gating, class_compatibility, combined_gating
from .cost_function import compute_bev_distances, compute_pixel_distances, compute_cost_matrix
from .hungarian_matcher import hungarian_match
from .association_models import AssociationResult


# Default association configuration
DEFAULT_CONFIG = {
    "bev_threshold": 15.0,      # BEV gating threshold (meters)
    "image_margin": 50.0,       # Bbox expansion margin (pixels)
    "bev_weight": 0.7,          # Cost weight for BEV distance
    "pixel_weight": 0.3,        # Cost weight for pixel distance
    "max_cost": 0.9,            # Maximum cost for valid match
}


def associate_radar_vision(
    radar_objects: List[RadarObject],
    vision_objects: List[VisionObject],
    H: np.ndarray,
    H_inv: np.ndarray,
    config: Optional[Dict[str, Any]] = None,
) -> AssociationResult:
    """Run the full radar-vision association pipeline.

    Pipeline steps:
    1. Extract radar BEV points and project to image via H.
    2. Extract vision bbox bottom centers.
    3. Back-project vision bottom centers to BEV via H_inv.
    4. Apply image gating (radar proj inside expanded bbox).
    5. Apply BEV gating (BEV distance within threshold).
    6. Apply class compatibility gating.
    7. Combine gates and compute cost matrix.
    8. Run Hungarian matching.

    Args:
        radar_objects: List of radar detected objects.
        vision_objects: List of vision detected objects.
        H: 3x3 homography matrix (radar BEV → image).
        H_inv: 3x3 inverse homography matrix (image → radar BEV).
        config: Association configuration dict. Uses defaults if None.

    Returns:
        AssociationResult with matches, unmatched_radar, unmatched_vision.
    """
    cfg = {**DEFAULT_CONFIG}
    if config:
        cfg.update(config)

    n_radar = len(radar_objects)
    n_vision = len(vision_objects)

    # Handle edge cases
    if n_radar == 0 or n_vision == 0:
        return AssociationResult(
            matches=[],
            unmatched_radar=list(range(n_radar)),
            unmatched_vision=list(range(n_vision)),
        )

    # --- Step 1: Extract radar points and project to image ---
    radar_bev = np.array(
        [[obj.x, obj.y] for obj in radar_objects],
        dtype=np.float64,
    )
    radar_proj_uv = project_radar_to_image(radar_bev, H)

    # --- Step 2: Extract vision bbox bottom centers ---
    vision_bottom_uv = extract_vision_bottom_centers(vision_objects)

    # --- Step 3: Back-project vision bottom centers to BEV ---
    vision_bev = project_image_to_radar(vision_bottom_uv, H_inv)

    # --- Step 4: Image gating ---
    vision_bboxes = np.array(
        [obj.bbox for obj in vision_objects],
        dtype=np.float64,
    )
    img_gate = image_gating(
        radar_proj_uv, vision_bboxes, margin=cfg["image_margin"]
    )

    # --- Step 5: BEV gating ---
    bev_gate = bev_gating(
        radar_bev, vision_bev, threshold=cfg["bev_threshold"]
    )

    # --- Step 6: Class compatibility gating ---
    radar_classes = [obj.obj_class for obj in radar_objects]
    vision_classes = [obj.obj_class for obj in vision_objects]
    cls_gate = class_compatibility(radar_classes, vision_classes)

    # --- Step 7: Combine gates and compute cost matrix ---
    gate = combined_gating(img_gate, bev_gate, cls_gate)

    bev_dist = compute_bev_distances(radar_bev, vision_bev)
    pixel_dist = compute_pixel_distances(radar_proj_uv, vision_bottom_uv)

    cost = compute_cost_matrix(
        bev_dist, pixel_dist, gate,
        bev_weight=cfg["bev_weight"],
        pixel_weight=cfg["pixel_weight"],
    )

    # --- Step 8: Hungarian matching ---
    result = hungarian_match(
        cost, gate, bev_dist, pixel_dist,
        max_cost=cfg["max_cost"],
    )

    return result
