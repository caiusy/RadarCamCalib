"""
trajectory - Multi-frame radar trajectory building, filtering, and storage.

This package provides:
- Trajectory data models
- Trajectory building from multiple radar frames
- Trajectory filtering (short tracks, class filter)
- SQLite-based trajectory database for real-time queries
"""

from .trajectory_models import TrajectoryPoint, Trajectory
from .trajectory_builder import TrajectoryBuilder
from .trajectory_filter import filter_short_tracks, filter_by_class, flatten_trajectory_points
from .trajectory_db import TrajectoryDB

