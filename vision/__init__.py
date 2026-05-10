"""
vision - Vision detection data loading and processing.

This package provides:
- VisionObject and VisionFrame data models
- Vision JSON file loading and parsing
- Bounding box utility functions
"""

from .vision_models import (
    VisionObject,
    VisionFrame,
    bbox_bottom_center,
    bbox_area,
    expand_bbox,
)
from .vision_loader import load_vision_json
