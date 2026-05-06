"""
data_models.py - Core data structures for radar-camera calibration system.

All data structures are defined as dataclasses for clarity and immutability.
These models are shared across all layers (data, calibration, trajectory, etc.).
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, List
import numpy as np


@dataclass
class SensorMeta:
    """Metadata for a single sensor within a sync batch.

    Attributes:
        sensor_id: Unique identifier for the sensor.
        frame_index: Frame sequence number for this sensor.
        time_stamp: Timestamp in milliseconds.
    """
    sensor_id: int
    frame_index: int
    time_stamp: int


@dataclass
class SensorDataItem:
    """Data file reference for a single sensor channel.

    Attributes:
        channel_id: Channel identifier.
        sensor_id: Sensor identifier (links to SensorMeta).
        data_type: Type of data, e.g. "image" or "radar".
        data_path: Relative path to the data file (relative to dataset root).
    """
    channel_id: int
    sensor_id: int
    data_type: str
    data_path: Path


@dataclass
class RadarObject:
    """A single radar-detected object.

    Attributes:
        track_id: Radar tracking ID.
        obj_class: Object class in English, e.g. "vehicle", "pedestrian", "unknown".
        x: Lateral distance in meters.
        y: Forward distance in meters.
        speed: Total speed magnitude (m/s), optional.
        speed_x: Lateral speed component (m/s), optional.
        speed_y: Forward speed component (m/s), optional.
        acc_x: Lateral acceleration (m/s^2), optional.
        acc_y: Forward acceleration (m/s^2), optional.
        lane_id: Lane identifier string, optional.
    """
    track_id: int
    obj_class: str
    x: float
    y: float
    speed: Optional[float] = None
    speed_x: Optional[float] = None
    speed_y: Optional[float] = None
    acc_x: Optional[float] = None
    acc_y: Optional[float] = None
    lane_id: Optional[str] = None


@dataclass
class RadarFrame:
    """A complete radar data frame.

    Attributes:
        base_id: Radar sensor base ID.
        filename: Original filename of the radar JSON.
        objects: List of detected radar objects.
        sensor_id: Sensor ID (from sync metadata).
        frame_index: Frame index (from sync metadata).
        time_stamp: Timestamp in milliseconds (from sync metadata).
    """
    base_id: int
    filename: str
    objects: List[RadarObject]
    sensor_id: int = 0
    frame_index: int = 0
    time_stamp: int = 0


@dataclass
class ImageFrame:
    """A loaded image frame.

    Attributes:
        image: The image as a numpy array (BGR format from OpenCV).
        file_path: Absolute path to the image file.
        width: Image width in pixels.
        height: Image height in pixels.
        sensor_id: Camera sensor ID.
        frame_index: Frame index.
        time_stamp: Timestamp in milliseconds.
    """
    image: np.ndarray
    file_path: Path
    width: int
    height: int
    sensor_id: int = 0
    frame_index: int = 0
    time_stamp: int = 0


@dataclass
class SyncFrame:
    """A single synchronization batch containing image and radar data references.

    Attributes:
        sync_batch: Batch sequence number.
        batch_size: Number of sensor data items in this batch.
        sensors: List of sensor metadata entries.
        sensor_data: List of sensor data file references.
    """
    sync_batch: int
    batch_size: int
    sensors: List[SensorMeta]
    sensor_data: List[SensorDataItem]


@dataclass
class MatchedPoint:
    """A matched pair of radar point and image point.

    Used for homography computation. Stores the radar BEV coordinates
    and the corresponding image pixel coordinates.

    Attributes:
        radar_x: Radar x coordinate (lateral, meters).
        radar_y: Radar y coordinate (forward, meters).
        image_u: Image x coordinate (horizontal, pixels).
        image_v: Image y coordinate (vertical, pixels).
        reprojection_error: Computed reprojection error after homography (pixels).
        inlier: Whether this point was classified as inlier by RANSAC.
    """
    radar_x: float
    radar_y: float
    image_u: float
    image_v: float
    reprojection_error: Optional[float] = None
    inlier: Optional[bool] = None

    def to_dict(self) -> dict:
        """Convert to dictionary for JSON serialization."""
        d = {
            "radar_point": {"x": self.radar_x, "y": self.radar_y},
            "image_point": {"u": self.image_u, "v": self.image_v},
        }
        if self.reprojection_error is not None:
            d["reprojection_error"] = round(self.reprojection_error, 4)
        if self.inlier is not None:
            d["inlier"] = self.inlier
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "MatchedPoint":
        """Create from dictionary (JSON deserialization)."""
        rp = d.get("radar_point", {})
        ip = d.get("image_point", {})
        return cls(
            radar_x=rp.get("x", 0.0),
            radar_y=rp.get("y", 0.0),
            image_u=ip.get("u", 0.0),
            image_v=ip.get("v", 0.0),
            reprojection_error=d.get("reprojection_error"),
            inlier=d.get("inlier"),
        )


@dataclass
class CalibrationResult:
    """Result of homography calibration computation.

    Attributes:
        H: 3x3 homography matrix (radar -> image).
        H_inv: 3x3 inverse homography matrix (image -> radar).
        matched_points: List of matched points used for computation.
        mean_error: Mean reprojection error across all inlier points (pixels).
        max_error: Maximum reprojection error across all points (pixels).
        inlier_mask: Boolean mask indicating inlier points.
    """
    H: np.ndarray
    H_inv: np.ndarray
    matched_points: List[MatchedPoint]
    mean_error: float
    max_error: float
    inlier_mask: Optional[np.ndarray] = None
