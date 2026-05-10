"""
sync_dataset.py - High-level dataset abstraction.

Wraps sync loading, image loading, and radar loading into a single
unified interface for navigating synchronized sensor data.
"""

from pathlib import Path
from typing import Optional, List, Tuple

from .data_models import SyncFrame, ImageFrame, RadarFrame, SensorDataItem
from .sync_loader import load_sync_folder, get_image_items, get_radar_items, get_vision_items, get_sensor_meta
from .radar_loader import load_radar_json
from .image_loader import load_image


class SyncDataset:
    """High-level dataset manager for synchronized radar-camera data.

    Provides frame navigation, data loading, and sensor filtering.
    Designed to be used by both UI and command-line tools.

    Attributes:
        dataset_root: Root directory of the dataset.
        sync_frames: List of loaded sync frames, sorted by batch number.
        current_index: Index of the currently active frame.
    """

    def __init__(self):
        self.dataset_root: Optional[Path] = None
        self.sync_frames: List[SyncFrame] = []
        self.current_index: int = -1
        self._camera_sensor_id: int = 1
        self._radar_sensor_id: int = 2
        self._vision_sensor_id: int = 1

    @property
    def num_frames(self) -> int:
        """Number of loaded sync frames."""
        return len(self.sync_frames)

    @property
    def current_frame(self) -> Optional[SyncFrame]:
        """Currently active sync frame, or None."""
        if 0 <= self.current_index < self.num_frames:
            return self.sync_frames[self.current_index]
        return None

    def set_sensor_ids(self, camera_id: int, radar_id: int):
        """Configure which sensor IDs correspond to camera and radar.

        Args:
            camera_id: Sensor ID for the camera.
            radar_id: Sensor ID for the radar.
        """
        self._camera_sensor_id = camera_id
        self._radar_sensor_id = radar_id

    def load(self, dataset_root: Path, sync_folder: str = "sync"):
        """Load all sync frames from the dataset.

        Args:
            dataset_root: Root directory containing sync/, image/, radar/ folders.
            sync_folder: Name of the subdirectory with sync JSON files.

        Raises:
            FileNotFoundError: If dataset_root or sync folder doesn't exist.
        """
        self.dataset_root = Path(dataset_root)
        sync_dir = self.dataset_root / sync_folder
        if not sync_dir.exists():
            raise FileNotFoundError(f"Sync folder not found: {sync_dir}")

        self.sync_frames = load_sync_folder(sync_dir)
        self.current_index = 0 if self.sync_frames else -1
        print(f"[SyncDataset] Loaded {self.num_frames} sync frames from {sync_dir}")

    def go_to(self, index: int) -> bool:
        """Navigate to a specific frame index.

        Args:
            index: Target frame index.

        Returns:
            True if navigation succeeded, False if index out of range.
        """
        if 0 <= index < self.num_frames:
            self.current_index = index
            return True
        return False

    def next_frame(self) -> bool:
        """Go to next frame. Returns True if successful."""
        return self.go_to(self.current_index + 1)

    def prev_frame(self) -> bool:
        """Go to previous frame. Returns True if successful."""
        return self.go_to(self.current_index - 1)

    def _resolve_data_path(self, data_item: SensorDataItem) -> Path:
        """Resolve a relative data path to an absolute path.

        Args:
            data_item: Sensor data item with a relative data_path.

        Returns:
            Absolute path to the data file.
        """
        data_path = data_item.data_path
        if not data_path.is_absolute():
            return self.dataset_root / data_path
        return data_path

    def get_image(self, sync_frame: Optional[SyncFrame] = None,
                  camera_sensor_id: Optional[int] = None) -> Optional[ImageFrame]:
        """Load the image for a sync frame.

        Args:
            sync_frame: Sync frame to load from. Uses current frame if None.
            camera_sensor_id: Camera sensor ID. Uses default if None.

        Returns:
            Loaded ImageFrame, or None if not available.
        """
        frame = sync_frame or self.current_frame
        if frame is None:
            return None

        cam_id = camera_sensor_id or self._camera_sensor_id
        image_items = get_image_items(frame)

        # Find image item matching camera sensor ID
        for item in image_items:
            if item.sensor_id == cam_id:
                try:
                    abs_path = self._resolve_data_path(item)
                    img_frame = load_image(abs_path)
                    # Attach sensor metadata
                    meta = get_sensor_meta(frame, cam_id)
                    if meta:
                        img_frame.sensor_id = meta.sensor_id
                        img_frame.frame_index = meta.frame_index
                        img_frame.time_stamp = meta.time_stamp
                    return img_frame
                except Exception as e:
                    print(f"[WARN] Failed to load image: {e}")
                    return None

        # Fallback: use first available image item
        if image_items:
            try:
                abs_path = self._resolve_data_path(image_items[0])
                return load_image(abs_path)
            except Exception as e:
                print(f"[WARN] Failed to load fallback image: {e}")

        return None

    def get_radar(self, sync_frame: Optional[SyncFrame] = None,
                  radar_sensor_id: Optional[int] = None) -> Optional[RadarFrame]:
        """Load the radar data for a sync frame.

        Args:
            sync_frame: Sync frame to load from. Uses current frame if None.
            radar_sensor_id: Radar sensor ID. Uses default if None.

        Returns:
            Loaded RadarFrame, or None if not available.
        """
        frame = sync_frame or self.current_frame
        if frame is None:
            return None

        rad_id = radar_sensor_id or self._radar_sensor_id
        radar_items = get_radar_items(frame)

        # Find radar item matching sensor ID
        for item in radar_items:
            if item.sensor_id == rad_id:
                try:
                    abs_path = self._resolve_data_path(item)
                    radar_frame = load_radar_json(abs_path)
                    # Attach sensor metadata
                    meta = get_sensor_meta(frame, rad_id)
                    if meta:
                        radar_frame.sensor_id = meta.sensor_id
                        radar_frame.frame_index = meta.frame_index
                        radar_frame.time_stamp = meta.time_stamp
                    return radar_frame
                except Exception as e:
                    print(f"[WARN] Failed to load radar: {e}")
                    return None

        # Fallback: use first available radar item
        if radar_items:
            try:
                abs_path = self._resolve_data_path(radar_items[0])
                return load_radar_json(abs_path)
            except Exception as e:
                print(f"[WARN] Failed to load fallback radar: {e}")

        return None

    def get_vision(self, sync_frame=None, vision_sensor_id=None):
        """Load the vision detection data for a sync frame.

        Args:
            sync_frame: Sync frame to load from. Uses current frame if None.
            vision_sensor_id: Vision sensor ID. Uses default if None.

        Returns:
            Loaded VisionFrame, or None if not available.
        """
        frame = sync_frame or self.current_frame
        if frame is None:
            return None

        vis_id = vision_sensor_id or self._vision_sensor_id
        vision_items = get_vision_items(frame)

        # Find vision item matching sensor ID
        for item in vision_items:
            if item.sensor_id == vis_id:
                try:
                    abs_path = self._resolve_data_path(item)
                    from vision.vision_loader import load_vision_json
                    return load_vision_json(abs_path)
                except Exception as e:
                    print(f"[WARN] Failed to load vision: {e}")
                    return None

        # Fallback: use first available vision item
        if vision_items:
            try:
                abs_path = self._resolve_data_path(vision_items[0])
                from vision.vision_loader import load_vision_json
                return load_vision_json(abs_path)
            except Exception as e:
                print(f"[WARN] Failed to load fallback vision: {e}")

        return None

    def get_current_data(self) -> Tuple[Optional[ImageFrame], Optional[RadarFrame]]:
        """Load both image and radar data for the current frame.

        Returns:
            Tuple of (ImageFrame, RadarFrame), either may be None.
        """
        return self.get_image(), self.get_radar()
