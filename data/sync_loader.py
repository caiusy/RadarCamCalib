"""
sync_loader.py - Load and parse sync JSON files.

Each sync JSON describes a synchronized batch of sensor data,
mapping images and radar files together with timestamps.
"""

import json
from pathlib import Path
from typing import List, Optional

from .data_models import SyncFrame, SensorMeta, SensorDataItem


def load_sync_file(path: Path) -> SyncFrame:
    """Load a single sync JSON file and parse it into a SyncFrame.

    Args:
        path: Path to the sync JSON file.

    Returns:
        Parsed SyncFrame object.

    Raises:
        FileNotFoundError: If the sync file does not exist.
        ValueError: If the JSON structure is invalid.
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"Sync file not found: {path}")

    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    # Parse sensor metadata
    sensors = []
    for s in data.get("sensors", []):
        sensors.append(SensorMeta(
            sensor_id=s.get("sensor_id", 0),
            frame_index=s.get("frame_index", 0),
            time_stamp=s.get("time_stamp", 0),
        ))

    # Parse sensor data items
    sensor_data = []
    for sd in data.get("sensor_data", []):
        sensor_data.append(SensorDataItem(
            channel_id=sd.get("channel_id", 0),
            sensor_id=sd.get("sensor_id", 0),
            data_type=sd.get("data_type", ""),
            data_path=Path(sd.get("data_path", "")),
        ))

    return SyncFrame(
        sync_batch=data.get("sync_batch", 0),
        batch_size=data.get("batch_size", len(sensor_data)),
        sensors=sensors,
        sensor_data=sensor_data,
    )


def load_sync_folder(folder: Path) -> List[SyncFrame]:
    """Load all sync JSON files from a folder, sorted by sync_batch or filename.

    Args:
        folder: Path to the folder containing sync JSON files.

    Returns:
        Sorted list of SyncFrame objects.

    Raises:
        FileNotFoundError: If the folder does not exist.
        ValueError: If no sync JSON files are found.
    """
    folder = Path(folder)
    if not folder.exists():
        raise FileNotFoundError(f"Sync folder not found: {folder}")

    # Find all sync JSON files (*.sync.json or *.json)
    sync_files = sorted(folder.glob("*.sync.json"))
    if not sync_files:
        sync_files = sorted(folder.glob("*.json"))
    if not sync_files:
        raise ValueError(f"No sync JSON files found in: {folder}")

    frames = []
    for sf in sync_files:
        try:
            frame = load_sync_file(sf)
            frames.append(frame)
        except Exception as e:
            print(f"[WARN] Failed to load sync file {sf}: {e}")

    # Sort by sync_batch number
    frames.sort(key=lambda f: f.sync_batch)
    return frames


def get_image_items(sync_frame: SyncFrame) -> List[SensorDataItem]:
    """Extract image data items from a sync frame.

    Args:
        sync_frame: The sync frame to extract from.

    Returns:
        List of SensorDataItem with data_type == "image".
    """
    return [sd for sd in sync_frame.sensor_data if sd.data_type == "image"]


def get_radar_items(sync_frame: SyncFrame) -> List[SensorDataItem]:
    """Extract radar data items from a sync frame.

    Args:
        sync_frame: The sync frame to extract from.

    Returns:
        List of SensorDataItem with data_type == "radar".
    """
    return [sd for sd in sync_frame.sensor_data if sd.data_type == "radar"]


def get_vision_items(sync_frame: SyncFrame) -> List[SensorDataItem]:
    """Extract vision detection data items from a sync frame.

    Args:
        sync_frame: The sync frame to extract from.

    Returns:
        List of SensorDataItem with data_type == "vision_detection".
    """
    return [sd for sd in sync_frame.sensor_data if sd.data_type == "vision_detection"]


def get_sensor_meta(sync_frame: SyncFrame, sensor_id: int) -> Optional[SensorMeta]:
    """Get sensor metadata by sensor_id.

    Args:
        sync_frame: The sync frame to search.
        sensor_id: The sensor ID to look for.

    Returns:
        SensorMeta if found, None otherwise.
    """
    for s in sync_frame.sensors:
        if s.sensor_id == sensor_id:
            return s
    return None
