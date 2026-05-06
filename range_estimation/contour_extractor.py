"""
contour_extractor.py - Extract polygons from binary occupancy masks.

Uses OpenCV morphological operations and contour detection to
convert grid-based detection masks into polygon representations.
"""

import numpy as np
import cv2
from typing import List, Tuple


def mask_to_polygons(
    valid_mask: np.ndarray,
    x_min: float,
    y_min: float,
    resolution: float,
    min_area_m2: float = 20.0,
    morph_kernel_size: int = 5,
) -> List[dict]:
    """Convert a binary mask to polygon(s) in real-world coordinates.

    Pipeline:
    1. Apply morphological closing to fill small holes.
    2. Find contours using OpenCV.
    3. Convert contour pixels back to BEV meter coordinates.
    4. Filter by minimum area.

    Args:
        valid_mask: Binary mask (uint8, 255=valid).
        x_min: X offset of the grid origin (meters).
        y_min: Y offset of the grid origin (meters).
        resolution: Grid cell size (meters).
        min_area_m2: Minimum polygon area to keep (square meters).
        morph_kernel_size: Kernel size for morphological closing.

    Returns:
        List of polygon dicts, each with keys:
            - "polygon": List of [x, y] vertices.
            - "area_m2": Polygon area in square meters.
    """
    if valid_mask is None or valid_mask.size == 0:
        return []

    # 1. Morphological closing to fill gaps
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (morph_kernel_size, morph_kernel_size)
    )
    closed = cv2.morphologyEx(valid_mask, cv2.MORPH_CLOSE, kernel)

    # 2. Find contours (external only)
    contours, _ = cv2.findContours(
        closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        print("[ContourExtractor] No contours found")
        return []

    # 3. Convert each contour to real-world coordinates
    polygons = []
    for contour in contours:
        # Compute area in pixel units, convert to m^2
        pixel_area = cv2.contourArea(contour)
        area_m2 = pixel_area * (resolution ** 2)

        if area_m2 < min_area_m2:
            continue

        # Simplify contour
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Convert pixel coords to BEV meters
        vertices = []
        for pt in approx.reshape(-1, 2):
            bev_x = pt[0] * resolution + x_min
            bev_y = pt[1] * resolution + y_min
            vertices.append([round(bev_x, 2), round(bev_y, 2)])

        polygons.append({
            "polygon": vertices,
            "area_m2": round(area_m2, 2),
        })

    # Sort by area descending
    polygons.sort(key=lambda p: p["area_m2"], reverse=True)

    print(f"[ContourExtractor] Extracted {len(polygons)} polygons "
          f"(min_area={min_area_m2} m²)")

    return polygons
