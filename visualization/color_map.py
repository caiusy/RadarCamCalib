"""
color_map.py - Color definitions and utilities for visualization.

Provides consistent colors for different object classes, point pairs,
and visualization elements.
"""

from typing import Tuple

# 10 distinct colors for point pairs (BGR for OpenCV, CSS hex for the web client)
PAIR_COLORS = [
    '#FF4444',  # Red
    '#44FF44',  # Green
    '#4444FF',  # Blue
    '#FFFF44',  # Yellow
    '#FF44FF',  # Magenta
    '#44FFFF',  # Cyan
    '#FF8844',  # Orange
    '#FF44AA',  # Pink
    '#88FF44',  # Lime
    '#AA44FF',  # Purple
]

# Object class colors (hex)
CLASS_COLORS = {
    'vehicle': '#FF00FF',     # Magenta
    'car': '#FF4488',         # Pink-red
    'truck': '#FF8800',       # Orange
    'bus': '#FFAA00',         # Gold
    'cyclist': '#00FF88',     # Green-cyan
    'pedestrian': '#00AAFF',  # Light blue
    'unknown': '#888888',     # Gray
}

# UI element colors
UI_COLORS = {
    'bg': '#1E1E1E',
    'panel': '#252526',
    'border': '#3E3E42',
    'text': '#D4D4D4',
    'text_dim': '#888888',
    'accent': '#007ACC',
    'success': '#4CAF50',
    'error': '#D16D6D',
    'warning': '#FFA500',
    'radar': '#FF00FF',
    'radar_pending': '#FFFF00',
    'image_preview': '#00FFFF',
    'lane': '#FF6600',
}


def hex_to_bgr(hex_color: str) -> Tuple[int, int, int]:
    """Convert hex color string to BGR tuple for OpenCV.

    Args:
        hex_color: Hex color string like '#FF00FF'.

    Returns:
        Tuple of (B, G, R) values.
    """
    h = hex_color.lstrip('#')
    r, g, b = int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)
    return (b, g, r)


def get_class_color(obj_class: str) -> str:
    """Get hex color for an object class.

    Args:
        obj_class: Object class name (e.g. "vehicle").

    Returns:
        Hex color string.
    """
    return CLASS_COLORS.get(obj_class, CLASS_COLORS['unknown'])


def get_pair_color(index: int) -> str:
    """Get hex color for a point pair by index.

    Args:
        index: Pair index (wraps around).

    Returns:
        Hex color string.
    """
    return PAIR_COLORS[index % len(PAIR_COLORS)]
