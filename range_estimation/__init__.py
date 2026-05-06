"""
range_estimation - Radar detection range estimation from trajectory data.

Pipeline:
1. Collect all radar trajectory points
2. Basic range filtering
3. DBSCAN noise removal
4. Grid density analysis
5. Morphological processing
6. Contour extraction -> polygon
"""

from .point_filter import basic_filter_points, dbscan_filter_points
from .density_grid import build_occupancy_grid, extract_valid_grid
from .contour_extractor import mask_to_polygons
from .range_estimator import estimate_radar_range
