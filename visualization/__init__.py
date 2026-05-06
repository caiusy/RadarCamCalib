"""
visualization - Drawing utilities for radar-camera calibration visualization.

Provides pure rendering functions (no UI framework dependency where possible):
- Image overlay drawing (OpenCV-based)
- BEV radar rendering (OpenCV-based)
- Color mapping utilities
"""

from .image_overlay import draw_projected_radar, draw_matched_points
from .bev_renderer import render_bev_frame, render_bev_trajectories
from .color_map import get_class_color, get_pair_color, PAIR_COLORS
