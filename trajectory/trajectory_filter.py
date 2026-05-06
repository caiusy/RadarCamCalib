"""
trajectory_filter.py - Filter and transform trajectories.

Provides filtering by track length, object class, and utility
functions for flattening trajectories into point arrays.
"""

from typing import Dict, List, Optional
import numpy as np

from .trajectory_models import Trajectory, TrajectoryPoint


def filter_short_tracks(
    trajectories: Dict[int, Trajectory],
    min_track_len: int = 5
) -> Dict[int, Trajectory]:
    """Remove trajectories shorter than a minimum length.

    Args:
        trajectories: Dictionary of track_id -> Trajectory.
        min_track_len: Minimum number of points to keep a trajectory.

    Returns:
        Filtered dictionary with only long-enough trajectories.
    """
    filtered = {
        tid: traj for tid, traj in trajectories.items()
        if traj.length >= min_track_len
    }
    removed = len(trajectories) - len(filtered)
    if removed > 0:
        print(f"[TrajectoryFilter] Removed {removed} short tracks "
              f"(min_len={min_track_len})")
    return filtered


def filter_by_class(
    trajectories: Dict[int, Trajectory],
    allowed_classes: List[str]
) -> Dict[int, Trajectory]:
    """Filter trajectories by object class.

    Args:
        trajectories: Dictionary of track_id -> Trajectory.
        allowed_classes: List of allowed class names, e.g. ["vehicle"].

    Returns:
        Filtered dictionary with only matching trajectories.
    """
    return {
        tid: traj for tid, traj in trajectories.items()
        if traj.obj_class in allowed_classes
    }


def flatten_trajectory_points(
    trajectories: Dict[int, Trajectory]
) -> np.ndarray:
    """Flatten all trajectory points into a single Nx2 array of (x, y).

    Args:
        trajectories: Dictionary of track_id -> Trajectory.

    Returns:
        Nx2 numpy array of all (x, y) points.
    """
    points = []
    for traj in trajectories.values():
        for pt in traj.points:
            points.append([pt.x, pt.y])

    if not points:
        return np.empty((0, 2), dtype=np.float64)

    return np.array(points, dtype=np.float64)


def flatten_with_metadata(
    trajectories: Dict[int, Trajectory]
) -> List[dict]:
    """Flatten trajectories with full metadata.

    Args:
        trajectories: Dictionary of track_id -> Trajectory.

    Returns:
        List of dicts with keys: track_id, x, y, speed, obj_class, etc.
    """
    result = []
    for tid, traj in trajectories.items():
        for pt in traj.points:
            result.append({
                "track_id": traj.track_id,
                "sensor_id": traj.sensor_id,
                "x": pt.x,
                "y": pt.y,
                "speed": pt.speed,
                "obj_class": pt.obj_class,
                "time_stamp": pt.time_stamp,
                "frame_index": pt.frame_index,
            })
    return result
