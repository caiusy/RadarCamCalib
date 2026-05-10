"""
vision_loader.py - Load and parse vision detection JSON files.

Handles the vision detection format with bounding boxes, class labels,
and detection scores.
"""

import json
from pathlib import Path
from typing import List

from .vision_models import VisionObject, VisionFrame


def load_vision_json(path: Path) -> VisionFrame:
    """Load a vision detection JSON file and parse all objects.

    Expected JSON format:
    {
        "sensor_id": 1,
        "frame_index": 0,
        "time_stamp": 1767954310202,
        "image_path": "image/000000-1767954310202.jpg",
        "objects": [
            {
                "id": 101,
                "track_id": 21,
                "class": "vehicle",
                "bbox": [720.5, 310.2, 810.8, 390.6],
                "score": 0.92
            }
        ]
    }

    Args:
        path: Path to the vision JSON file.

    Returns:
        Parsed VisionFrame with all detected objects.

    Raises:
        FileNotFoundError: If the file does not exist.
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"Vision file not found: {path}")

    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    objects = parse_vision_objects(data.get("objects", []))

    return VisionFrame(
        sensor_id=data.get("sensor_id", 0),
        frame_index=data.get("frame_index", 0),
        time_stamp=data.get("time_stamp", 0),
        image_path=data.get("image_path", ""),
        objects=objects,
    )


def parse_vision_objects(objects_list: list) -> List[VisionObject]:
    """Parse a list of raw vision object dicts into VisionObject instances.

    Args:
        objects_list: List of object dictionaries from vision JSON.

    Returns:
        List of parsed VisionObject instances.
    """
    result = []
    for obj in objects_list:
        try:
            bbox = obj.get("bbox", [0, 0, 0, 0])
            if len(bbox) != 4:
                print(f"[WARN] Invalid bbox length: {bbox}")
                continue

            vision_obj = VisionObject(
                id=obj.get("id", 0),
                track_id=obj.get("track_id", 0),
                obj_class=obj.get("class", "unknown"),
                bbox=[float(b) for b in bbox],
                score=float(obj.get("score", 0.0)),
            )
            result.append(vision_obj)
        except Exception as e:
            print(f"[WARN] Failed to parse vision object: {obj}, error: {e}")

    return result
