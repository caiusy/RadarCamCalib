"""
vision_models.py - Data models for vision detection data.

Defines VisionObject and VisionFrame dataclasses for representing
camera-based object detection results, plus bbox utility functions.
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple
import numpy as np


@dataclass
class VisionObject:
    """A single vision-detected object (bounding box).

    Attributes:
        id: Unique object ID within the frame.
        track_id: Tracking ID across frames.
        obj_class: Object class string, e.g. "vehicle", "pedestrian".
        bbox: Bounding box as [x1, y1, x2, y2] (top-left, bottom-right).
        score: Detection confidence score (0-1).
    """
    id: int
    track_id: int
    obj_class: str
    bbox: List[float]
    score: float = 0.0

    @property
    def x1(self) -> float:
        return self.bbox[0]

    @property
    def y1(self) -> float:
        return self.bbox[1]

    @property
    def x2(self) -> float:
        return self.bbox[2]

    @property
    def y2(self) -> float:
        return self.bbox[3]

    @property
    def width(self) -> float:
        return self.x2 - self.x1

    @property
    def height(self) -> float:
        return self.y2 - self.y1

    @property
    def center(self) -> Tuple[float, float]:
        """Center of the bounding box (u, v)."""
        return ((self.x1 + self.x2) / 2.0, (self.y1 + self.y2) / 2.0)

    @property
    def bottom_center(self) -> Tuple[float, float]:
        """Bottom center of the bounding box (u, v).

        This is typically where the object contacts the ground plane,
        making it the best point for back-projection to BEV.
        """
        return ((self.x1 + self.x2) / 2.0, self.y2)

    def to_dict(self) -> dict:
        """Serialize to dictionary."""
        return {
            "id": self.id,
            "track_id": self.track_id,
            "class": self.obj_class,
            "bbox": self.bbox,
            "score": self.score,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "VisionObject":
        """Deserialize from dictionary."""
        return cls(
            id=d.get("id", 0),
            track_id=d.get("track_id", 0),
            obj_class=d.get("class", "unknown"),
            bbox=d.get("bbox", [0, 0, 0, 0]),
            score=d.get("score", 0.0),
        )


@dataclass
class VisionFrame:
    """A complete vision detection frame.

    Attributes:
        sensor_id: Vision sensor ID (typically same as camera).
        frame_index: Frame sequence number.
        time_stamp: Timestamp in milliseconds.
        image_path: Relative path to the corresponding image file.
        objects: List of detected vision objects.
    """
    sensor_id: int
    frame_index: int
    time_stamp: int
    image_path: str
    objects: List[VisionObject]

    def to_dict(self) -> dict:
        """Serialize to dictionary."""
        return {
            "sensor_id": self.sensor_id,
            "frame_index": self.frame_index,
            "time_stamp": self.time_stamp,
            "image_path": self.image_path,
            "objects": [obj.to_dict() for obj in self.objects],
        }


def bbox_bottom_center(bbox: List[float]) -> Tuple[float, float]:
    """Compute the bottom center point of a bounding box.

    The bottom center is where the object meets the ground plane,
    which is the most meaningful point for BEV back-projection.

    Args:
        bbox: [x1, y1, x2, y2] bounding box coordinates.

    Returns:
        Tuple (u, v) of the bottom center pixel coordinates.
    """
    x1, y1, x2, y2 = bbox
    return ((x1 + x2) / 2.0, y2)


def bbox_area(bbox: List[float]) -> float:
    """Compute the area of a bounding box.

    Args:
        bbox: [x1, y1, x2, y2] bounding box coordinates.

    Returns:
        Area in square pixels.
    """
    x1, y1, x2, y2 = bbox
    return max(0.0, x2 - x1) * max(0.0, y2 - y1)


def expand_bbox(
    bbox: List[float],
    margin: float = 20.0,
    image_width: Optional[int] = None,
    image_height: Optional[int] = None,
) -> List[float]:
    """Expand a bounding box by a uniform margin.

    Optionally clips to image boundaries.

    Args:
        bbox: [x1, y1, x2, y2] bounding box coordinates.
        margin: Expansion margin in pixels.
        image_width: If given, clip x coordinates to [0, width].
        image_height: If given, clip y coordinates to [0, height].

    Returns:
        Expanded [x1, y1, x2, y2] bounding box.
    """
    x1, y1, x2, y2 = bbox
    x1 -= margin
    y1 -= margin
    x2 += margin
    y2 += margin

    if image_width is not None:
        x1 = max(0.0, x1)
        x2 = min(float(image_width), x2)
    if image_height is not None:
        y1 = max(0.0, y1)
        y2 = min(float(image_height), y2)

    return [x1, y1, x2, y2]


def extract_vision_bottom_centers(objects: List[VisionObject]) -> np.ndarray:
    """Extract bottom center points from vision objects.

    Args:
        objects: List of VisionObject instances.

    Returns:
        Nx2 array of bottom center [u, v] coordinates.
    """
    if not objects:
        return np.empty((0, 2), dtype=np.float64)

    centers = np.array(
        [bbox_bottom_center(obj.bbox) for obj in objects],
        dtype=np.float64,
    )
    return centers
