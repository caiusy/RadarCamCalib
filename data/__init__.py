"""
data - Data loading and model definitions for radar-camera calibration system.

This package handles:
- Data structure definitions (dataclasses)
- Sync JSON loading and parsing
- Radar JSON loading and parsing
- Image loading
- Dataset abstraction
"""

from .data_models import (
    SensorMeta, SensorDataItem, RadarObject, RadarFrame,
    ImageFrame, SyncFrame, MatchedPoint, CalibrationResult
)
from .sync_loader import load_sync_file, load_sync_folder
from .radar_loader import load_radar_json, parse_radar_objects, normalize_class
from .image_loader import load_image
from .sync_dataset import SyncDataset
