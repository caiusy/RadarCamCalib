"""
trajectory_builder.py - Build trajectories from multiple sync frames.

Scans all sync frames, loads radar data, and groups detections
by track_id to form trajectories.
"""

from pathlib import Path
from typing import Dict, List, Optional

from data.data_models import SyncFrame, RadarFrame
from data.sync_loader import load_sync_folder, get_radar_items, get_sensor_meta
from data.radar_loader import load_radar_json

from .trajectory_models import TrajectoryPoint, Trajectory


class TrajectoryBuilder:
    """Build trajectories from radar data across multiple sync frames.

    Groups radar detections by (sensor_id, track_id) to form trajectories.

    Usage:
        builder = TrajectoryBuilder()
        builder.build_from_sync_folder(dataset_root, "sync")
        trajectories = builder.get_trajectories()
    """

    def __init__(self):
        # {sensor_id: {track_id: Trajectory}}
        self._trajectories: Dict[int, Dict[int, Trajectory]] = {}
        self._total_points: int = 0

    @property
    def total_points(self) -> int:
        """Total number of trajectory points collected."""
        return self._total_points

    def build_from_sync_frames(
        self,
        sync_frames: List[SyncFrame],
        dataset_root: Path,
        radar_sensor_id: Optional[int] = None,
    ):
        """Build trajectories from a list of sync frames.

        Args:
            sync_frames: List of sync frames to process.
            dataset_root: Root directory for resolving relative data paths.
            radar_sensor_id: If specified, only process this radar sensor.
                             If None, process all radar sensors.
        """
        dataset_root = Path(dataset_root)

        for sync_frame in sync_frames:
            radar_items = get_radar_items(sync_frame)

            for item in radar_items:
                # Filter by sensor_id if specified
                if radar_sensor_id is not None and item.sensor_id != radar_sensor_id:
                    continue

                # Resolve data path
                data_path = item.data_path
                if not data_path.is_absolute():
                    data_path = dataset_root / data_path

                if not data_path.exists():
                    continue

                try:
                    radar_frame = load_radar_json(data_path)
                except Exception as e:
                    print(f"[WARN] Failed to load radar file {data_path}: {e}")
                    continue

                # Get sensor metadata for timestamps
                meta = get_sensor_meta(sync_frame, item.sensor_id)
                time_stamp = meta.time_stamp if meta else 0
                frame_index = meta.frame_index if meta else sync_frame.sync_batch

                sid = item.sensor_id

                # Ensure sensor dict exists
                if sid not in self._trajectories:
                    self._trajectories[sid] = {}

                # Add each object to its trajectory
                for obj in radar_frame.objects:
                    tid = obj.track_id
                    if tid not in self._trajectories[sid]:
                        self._trajectories[sid][tid] = Trajectory(
                            track_id=tid,
                            sensor_id=sid,
                        )

                    point = TrajectoryPoint(
                        time_stamp=time_stamp,
                        frame_index=frame_index,
                        x=obj.x,
                        y=obj.y,
                        speed=obj.speed,
                        lane_id=obj.lane_id,
                        obj_class=obj.obj_class,
                    )
                    self._trajectories[sid][tid].add_point(point)
                    self._total_points += 1

    def build_from_sync_folder(
        self,
        dataset_root: Path,
        sync_folder: str = "sync",
        radar_sensor_id: Optional[int] = None,
    ):
        """Build trajectories by scanning all sync files in a folder.

        Args:
            dataset_root: Root directory of the dataset.
            sync_folder: Name of the sync subfolder.
            radar_sensor_id: Optional radar sensor ID filter.
        """
        dataset_root = Path(dataset_root)
        sync_dir = dataset_root / sync_folder

        sync_frames = load_sync_folder(sync_dir)
        print(f"[TrajectoryBuilder] Loaded {len(sync_frames)} sync frames")

        self.build_from_sync_frames(sync_frames, dataset_root, radar_sensor_id)

        # Count stats
        total_tracks = sum(len(tracks) for tracks in self._trajectories.values())
        print(f"[TrajectoryBuilder] Built {total_tracks} trajectories, "
              f"{self._total_points} total points")

    def get_trajectories(
        self, sensor_id: Optional[int] = None
    ) -> Dict[int, Trajectory]:
        """Get all trajectories for a given sensor.

        Args:
            sensor_id: Sensor ID to filter. If None, returns merged dict
                       from all sensors.

        Returns:
            Dictionary mapping track_id -> Trajectory.
        """
        if sensor_id is not None:
            return dict(self._trajectories.get(sensor_id, {}))

        # Merge all sensors
        merged = {}
        for sid, tracks in self._trajectories.items():
            for tid, traj in tracks.items():
                # Use composite key to avoid collisions
                key = tid if len(self._trajectories) == 1 else (sid * 10000 + tid)
                merged[key] = traj
        return merged

    def get_all_trajectories_list(self) -> List[Trajectory]:
        """Get a flat list of all trajectories across all sensors.

        Returns:
            List of all Trajectory objects.
        """
        result = []
        for tracks in self._trajectories.values():
            result.extend(tracks.values())
        return result

    def clear(self):
        """Clear all built trajectories."""
        self._trajectories.clear()
        self._total_points = 0
