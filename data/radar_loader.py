"""
radar_loader.py - Load and parse radar JSON files.

Handles the radar detection format with coord normalization,
class field defaults, and props extraction.
"""

import json
from pathlib import Path
from typing import List, Tuple
import numpy as np

from .data_models import RadarObject, RadarFrame


# Chinese to English class name mapping for legacy data compatibility
CLASS_MAP = {
    "机动车": "vehicle",
    "汽车": "vehicle",
    "小汽车": "car",
    "货车": "truck",
    "公交车": "bus",
    "非机动车": "cyclist",
    "行人": "pedestrian",
    "unknown": "unknown",
}


def normalize_class(raw_class: str) -> str:
    """Normalize object class name to English enum.

    Maps Chinese class names to their English equivalents.
    Unknown/unmapped classes are returned as-is.

    Args:
        raw_class: Raw class string from radar JSON.

    Returns:
        Normalized English class name.
    """
    return CLASS_MAP.get(raw_class, raw_class)


def normalize_coord(coord) -> Tuple[float, float]:
    """Normalize coordinate format to (x, y) tuple.

    Handles both formats:
        - coord = [x, y]        -> (x, y)
        - coord = [[x, y]]     -> (x, y)

    Args:
        coord: Coordinate value from radar JSON.

    Returns:
        Tuple of (x, y) floats.

    Raises:
        ValueError: If coord format is unrecognized.
    """
    if isinstance(coord, (list, tuple)):
        if len(coord) == 2 and isinstance(coord[0], (int, float)):
            return (float(coord[0]), float(coord[1]))
        elif len(coord) >= 1 and isinstance(coord[0], (list, tuple)):
            inner = coord[0]
            if len(inner) >= 2:
                return (float(inner[0]), float(inner[1]))
    raise ValueError(f"Unrecognized coord format: {coord}")


def parse_radar_objects(objects_list: list) -> List[RadarObject]:
    """Parse a list of raw radar object dicts into RadarObject instances.

    Args:
        objects_list: List of object dictionaries from radar JSON.

    Returns:
        List of parsed RadarObject instances.
    """
    result = []
    for obj in objects_list:
        try:
            # Normalize coordinate
            coord = obj.get("coord", [0.0, 0.0])
            x, y = normalize_coord(coord)

            # Get class with default "unknown", normalize Chinese to English
            obj_class = normalize_class(obj.get("class", "unknown"))

            # Get props with safe access
            props = obj.get("props", {})

            radar_obj = RadarObject(
                track_id=obj.get("id", 0),
                obj_class=obj_class,
                x=x,
                y=y,
                speed=props.get("speed"),
                speed_x=props.get("speedX"),
                speed_y=props.get("speedY"),
                acc_x=props.get("accX"),
                acc_y=props.get("accY"),
                lane_id=props.get("vehicleLaneID"),
            )
            result.append(radar_obj)
        except Exception as e:
            print(f"[WARN] Failed to parse radar object: {obj}, error: {e}")

    return result


def load_radar_json(path: Path) -> RadarFrame:
    """Load a radar JSON file and parse all objects.

    Args:
        path: Path to the radar JSON file.

    Returns:
        Parsed RadarFrame with all detected objects.

    Raises:
        FileNotFoundError: If the file does not exist.
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"Radar file not found: {path}")

    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    objects = parse_radar_objects(data.get("objects", []))

    return RadarFrame(
        base_id=data.get("baseId", 0),
        filename=data.get("filename", path.name),
        objects=objects,
    )


def extract_radar_points(objects: List[RadarObject]) -> np.ndarray:
    """Extract (x, y) coordinate array from radar objects.

    Args:
        objects: List of RadarObject instances.

    Returns:
        Nx2 numpy array of [x, y] coordinates. Empty (0,2) array if no objects.
    """
    if not objects:
        return np.empty((0, 2), dtype=np.float64)

    points = np.array([[obj.x, obj.y] for obj in objects], dtype=np.float64)
    return points
