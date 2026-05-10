"""
camera_io.py - Camera parameter I/O and enhanced calibration JSON.

Provides save/load for standalone camera parameter files,
and an enhanced calibration JSON format that includes camera intrinsics.
"""

import json
from pathlib import Path
from typing import Optional
import numpy as np

from .camera_model import CameraModel
from data.data_models import CalibrationResult


def save_camera_params(path: Path, model: CameraModel):
    """Save camera parameters to a JSON file.

    Args:
        path: Output file path.
        model: CameraModel instance to save.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    data = {
        "version": "1.0",
        "type": "camera_parameters",
        **model.to_dict(),
    }

    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)

    print(f"[CameraIO] Saved camera params to {path}")


def load_camera_params(path: Path) -> CameraModel:
    """Load camera parameters from a JSON file.

    Args:
        path: Path to the camera parameter JSON file.

    Returns:
        CameraModel instance.

    Raises:
        FileNotFoundError: If the file does not exist.
        ValueError: If required fields are missing.
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"Camera params file not found: {path}")

    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    if "camera_matrix" not in data:
        raise ValueError("Missing 'camera_matrix' field in camera params file")

    return CameraModel.from_dict(data)


def save_enhanced_calibration_json(
    path: Path,
    result: CalibrationResult,
    camera_model: Optional[CameraModel] = None,
    camera_sensor_id: int = 1,
    radar_sensor_id: int = 2,
):
    """Save enhanced calibration result JSON with camera parameters.

    Extends the standard calibration JSON format with camera intrinsics,
    distortion coefficients, image size, and radar-to-camera extrinsics.

    Args:
        path: Output file path.
        result: CalibrationResult with homography data.
        camera_model: Optional CameraModel with intrinsics/extrinsics.
        camera_sensor_id: Camera sensor ID for metadata.
        radar_sensor_id: Radar sensor ID for metadata.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    data = {
        "version": "2.0",
        "calibration_type": "radar_to_camera_homography",
        "camera_sensor_id": camera_sensor_id,
        "radar_sensor_id": radar_sensor_id,
        "coordinate_system": {
            "radar": {
                "x": "lateral distance, unit: m",
                "y": "forward distance, unit: m",
            },
            "image": {
                "u": "horizontal pixel coordinate",
                "v": "vertical pixel coordinate",
            },
        },
        "homography": result.H.tolist(),
        "inverse_homography": result.H_inv.tolist(),
        "matched_points": [p.to_dict() for p in result.matched_points],
        "mean_reprojection_error": round(result.mean_error, 4),
        "max_reprojection_error": round(result.max_error, 4),
    }

    # Add camera parameters if available
    if camera_model is not None:
        data["camera_parameters"] = camera_model.to_dict()

    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)

    print(f"[CameraIO] Saved enhanced calibration to {path}")
