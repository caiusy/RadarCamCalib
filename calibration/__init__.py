"""
calibration - Homography-based radar-camera calibration.

This package provides:
- Homography computation with RANSAC
- Point projection between radar BEV and image coordinates
- Calibration session management
- Calibration I/O (save/load JSON)
"""

from .homography_solver import compute_homography, compute_reprojection_errors, invert_homography
from .projector import project_radar_to_image, project_image_to_radar, filter_points_in_image
from .calibration_session import CalibrationSession
from .calibration_io import save_calibration_json, load_calibration_json
