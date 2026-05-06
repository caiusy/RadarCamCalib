"""
trajectory_models.py - Data structures for radar trajectories.

A trajectory is a sequence of radar detections with the same track_id
across multiple frames.
"""

from dataclasses import dataclass, field
from typing import Optional, List


@dataclass
class TrajectoryPoint:
    """A single point in a radar trajectory.

    Attributes:
        time_stamp: Timestamp in milliseconds.
        frame_index: Frame sequence number.
        x: Lateral position (meters).
        y: Forward position (meters).
        speed: Speed magnitude (m/s), optional.
        lane_id: Lane identifier, optional.
        obj_class: Object class, e.g. "vehicle".
    """
    time_stamp: int
    frame_index: int
    x: float
    y: float
    speed: Optional[float] = None
    lane_id: Optional[str] = None
    obj_class: str = "unknown"


@dataclass
class Trajectory:
    """A radar object trajectory (sequence of detections with same track_id).

    Attributes:
        track_id: Radar tracking ID.
        sensor_id: Radar sensor ID.
        points: Ordered list of trajectory points.
        obj_class: Dominant object class for this trajectory.
    """
    track_id: int
    sensor_id: int
    points: List[TrajectoryPoint] = field(default_factory=list)
    obj_class: str = "unknown"

    @property
    def length(self) -> int:
        """Number of points in the trajectory."""
        return len(self.points)

    def add_point(self, point: TrajectoryPoint):
        """Add a point and keep the list sorted by time_stamp."""
        self.points.append(point)
        # Update dominant class (use the most recent non-unknown)
        if point.obj_class != "unknown":
            self.obj_class = point.obj_class
