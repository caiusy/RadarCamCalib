"""
calibration_io.py - Save and load calibration and range JSON files.

Handles serialization of calibration results, matched points,
and radar range polygon data.
"""

import json
from pathlib import Path
from typing import Optional
import numpy as np

from data.data_models import CalibrationResult, MatchedPoint


def save_calibration_json(
    path: Path,
    result: CalibrationResult,
    camera_sensor_id: int = 1,
    radar_sensor_id: int = 2,
):
    """Save calibration result to a JSON file.

    Args:
        path: Output file path.
        result: CalibrationResult to save.
        camera_sensor_id: Camera sensor ID for metadata.
        radar_sensor_id: Radar sensor ID for metadata.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    data = {
        "version": "1.0",
        "calibration_type": "radar_to_camera_homography",
        "camera_sensor_id": camera_sensor_id,
        "radar_sensor_id": radar_sensor_id,
        "coordinate_system": {
            "radar": {
                "x": "lateral distance, unit: m",
                "y": "forward distance, unit: m"
            },
            "image": {
                "u": "horizontal pixel coordinate",
                "v": "vertical pixel coordinate"
            }
        },
        "homography": result.H.tolist(),
        "inverse_homography": result.H_inv.tolist(),
        "matched_points": [p.to_dict() for p in result.matched_points],
        "mean_reprojection_error": round(result.mean_error, 4),
        "max_reprojection_error": round(result.max_error, 4),
    }

    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)

    print(f"[CalibIO] Saved calibration to {path}")


def load_calibration_json(path: Path) -> dict:
    """Load a calibration JSON file.

    Args:
        path: Path to the calibration JSON file.

    Returns:
        Parsed calibration data dictionary.

    Raises:
        FileNotFoundError: If the file does not exist.
        ValueError: If required fields are missing.
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"Calibration file not found: {path}")

    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    if "homography" not in data:
        raise ValueError("Missing 'homography' field in calibration file")

    return data


def save_range_json(
    path: Path,
    range_data: dict,
):
    """Save radar range estimation result to a JSON file.

    Args:
        path: Output file path.
        range_data: Dictionary with range polygon data and method parameters.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    with open(path, "w", encoding="utf-8") as f:
        json.dump(range_data, f, indent=2, ensure_ascii=False)

    print(f"[CalibIO] Saved radar range to {path}")


def save_matched_points_json(path: Path, points: list):
    """Save matched points to a standalone JSON file.

    Args:
        path: Output file path.
        points: List of MatchedPoint objects or dicts.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    data = {
        "matched_points": [
            p.to_dict() if isinstance(p, MatchedPoint) else p
            for p in points
        ]
    }

    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)

    print(f"[CalibIO] Saved {len(points)} matched points to {path}")


def load_matched_points_json(path: Path) -> list:
    """Load matched points from a JSON file.

    Args:
        path: Path to the matched points JSON file.

    Returns:
        List of MatchedPoint objects.
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"Matched points file not found: {path}")

    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    points = []
    for mp in data.get("matched_points", []):
        points.append(MatchedPoint.from_dict(mp))

    return points
