"""
density_grid.py - Grid-based density analysis for radar range estimation.

Divides the BEV space into a regular grid and counts radar detections
per cell to determine the effective detection area.
"""

import numpy as np
from typing import Tuple


def build_occupancy_grid(
    points: np.ndarray,
    x_range: Tuple[float, float] = (-100, 100),
    y_range: Tuple[float, float] = (0, 300),
    resolution: float = 1.0,
) -> Tuple[np.ndarray, float, float]:
    """Build a 2D occupancy grid counting points per cell.

    Args:
        points: Nx2 array of [x, y] coordinates.
        x_range: (x_min, x_max) range in meters.
        y_range: (y_min, y_max) range in meters.
        resolution: Grid cell size in meters.

    Returns:
        Tuple of (grid, x_min, y_min):
            - grid: 2D array of point counts (rows=y, cols=x).
            - x_min: Minimum x value of the grid.
            - y_min: Minimum y value of the grid.
    """
    x_min, x_max = x_range
    y_min, y_max = y_range

    # Grid dimensions
    nx = int(np.ceil((x_max - x_min) / resolution))
    ny = int(np.ceil((y_max - y_min) / resolution))

    grid = np.zeros((ny, nx), dtype=np.int32)

    if len(points) == 0:
        return grid, x_min, y_min

    # Map points to grid indices
    ix = ((points[:, 0] - x_min) / resolution).astype(int)
    iy = ((points[:, 1] - y_min) / resolution).astype(int)

    # Clip to valid range
    ix = np.clip(ix, 0, nx - 1)
    iy = np.clip(iy, 0, ny - 1)

    # Accumulate counts
    for i in range(len(points)):
        grid[iy[i], ix[i]] += 1

    total_cells = nx * ny
    occupied = np.sum(grid > 0)
    print(f"[DensityGrid] Grid {nx}x{ny}, {occupied}/{total_cells} cells occupied")

    return grid, x_min, y_min


def extract_valid_grid(
    grid: np.ndarray,
    min_count: int = 3,
) -> np.ndarray:
    """Create a binary mask of grid cells with sufficient detections.

    Args:
        grid: 2D array of point counts per cell.
        min_count: Minimum number of detections to consider a cell valid.

    Returns:
        Binary mask (uint8) where 255 = valid, 0 = invalid.
    """
    valid = (grid >= min_count).astype(np.uint8) * 255

    n_valid = np.sum(valid > 0)
    print(f"[DensityGrid] {n_valid} valid cells (min_count={min_count})")

    return valid
