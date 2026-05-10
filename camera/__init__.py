"""
camera - Camera parameter management for radar-camera calibration system.

This package provides:
- Camera intrinsic model (K matrix, distortion coefficients)
- Camera-radar extrinsic parameters
- Camera parameter I/O (save/load JSON)
- Enhanced calibration JSON with camera parameters
"""

from .camera_model import (
    CameraModel,
    get_default_camera_model,
    project_points_pinhole,
    undistort_points,
)
from .camera_io import (
    save_camera_params,
    load_camera_params,
    save_enhanced_calibration_json,
)
