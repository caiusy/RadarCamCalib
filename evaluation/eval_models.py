"""
eval_models.py - Data models for evaluation metrics.

Defines dataclasses for distance-binned, lane-based, and grid-level
metrics used in radar-vision association quality assessment.
"""

from dataclasses import dataclass, field
from typing import List, Dict, Optional
import numpy as np


@dataclass
class DistanceBinMetrics:
    """Metrics for a specific distance range bin.

    Attributes:
        range_str: Human-readable range string, e.g. "0-30m".
        range_min: Minimum distance of this bin (meters).
        range_max: Maximum distance of this bin (meters).
        total_vision: Total vision detections in this range.
        total_radar: Total radar detections in this range.
        matched_count: Number of matched pairs in this range.
        recall: matched_count / total_vision (or 0 if no vision).
        fp_rate: unmatched_radar_count / total_radar (or 0 if no radar).
        mean_bev_error: Mean BEV error for matched pairs (meters).
        mean_lateral_error: Mean lateral (x) error (meters).
        mean_longitudinal_error: Mean longitudinal (y) error (meters).
    """
    range_str: str
    range_min: float
    range_max: float
    total_vision: int = 0
    total_radar: int = 0
    matched_count: int = 0
    recall: float = 0.0
    fp_rate: float = 0.0
    mean_bev_error: float = 0.0
    mean_lateral_error: float = 0.0
    mean_longitudinal_error: float = 0.0

    def to_dict(self) -> dict:
        return {
            "range": self.range_str,
            "range_min": self.range_min,
            "range_max": self.range_max,
            "total_vision": self.total_vision,
            "total_radar": self.total_radar,
            "matched_count": self.matched_count,
            "recall": round(self.recall, 4),
            "fp_rate": round(self.fp_rate, 4),
            "mean_bev_error": round(self.mean_bev_error, 4),
            "mean_lateral_error": round(self.mean_lateral_error, 4),
            "mean_longitudinal_error": round(self.mean_longitudinal_error, 4),
        }


@dataclass
class LaneMetrics:
    """Metrics segmented by lane position.

    Attributes:
        lane_id: Lane identifier string.
        x_range: Lateral range [x_min, x_max] for this lane (meters).
        total_vision: Total vision detections in this lane.
        matched_count: Number of matched pairs in this lane.
        recall: matched_count / total_vision.
        mean_bev_error: Mean BEV error for matches in this lane.
    """
    lane_id: str
    x_range: List[float]
    total_vision: int = 0
    matched_count: int = 0
    recall: float = 0.0
    mean_bev_error: float = 0.0

    def to_dict(self) -> dict:
        return {
            "lane_id": self.lane_id,
            "x_range": self.x_range,
            "total_vision": self.total_vision,
            "matched_count": self.matched_count,
            "recall": round(self.recall, 4),
            "mean_bev_error": round(self.mean_bev_error, 4),
        }


@dataclass
class GridMetrics:
    """Metrics on a 2D BEV grid.

    Attributes:
        grid_size: Grid cell size in meters (e.g. 5.0).
        x_range: X range of the grid [x_min, x_max].
        y_range: Y range of the grid [y_min, y_max].
        cells: List of cell dicts with (x, y, recall, vision_count, matched_count).
    """
    grid_size: float
    x_range: List[float]
    y_range: List[float]
    cells: List[dict] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "grid_size": self.grid_size,
            "x_range": self.x_range,
            "y_range": self.y_range,
            "num_cells": len(self.cells),
            "cells": self.cells,
        }


@dataclass
class EvaluationResult:
    """Complete evaluation metrics for radar-vision association.

    Attributes:
        radar_recall: matched_vision / total_vision.
        radar_fp_rate: unmatched_radar / total_radar.
        mean_bev_error: Mean BEV distance error (meters).
        mean_lateral_error: Mean lateral (x) error (meters).
        mean_longitudinal_error: Mean longitudinal (y) error (meters).
        p50_bev_error: Median BEV error (meters).
        p90_bev_error: 90th percentile BEV error (meters).
        p95_bev_error: 95th percentile BEV error (meters).
        total_radar: Total radar detections evaluated.
        total_vision: Total vision detections evaluated.
        total_matched: Total matched pairs.
        distance_bins: Per-distance-range metrics.
        lane_metrics: Per-lane metrics.
        grid_metrics: Grid-level metrics.
    """
    radar_recall: float = 0.0
    radar_fp_rate: float = 0.0
    mean_bev_error: float = 0.0
    mean_lateral_error: float = 0.0
    mean_longitudinal_error: float = 0.0
    p50_bev_error: float = 0.0
    p90_bev_error: float = 0.0
    p95_bev_error: float = 0.0
    total_radar: int = 0
    total_vision: int = 0
    total_matched: int = 0
    distance_bins: List[DistanceBinMetrics] = field(default_factory=list)
    lane_metrics: List[LaneMetrics] = field(default_factory=list)
    grid_metrics: Optional[GridMetrics] = None

    def to_dict(self) -> dict:
        d = {
            "radar_recall": round(self.radar_recall, 4),
            "radar_fp_rate": round(self.radar_fp_rate, 4),
            "mean_bev_error": round(self.mean_bev_error, 4),
            "mean_lateral_error": round(self.mean_lateral_error, 4),
            "mean_longitudinal_error": round(self.mean_longitudinal_error, 4),
            "p50_bev_error": round(self.p50_bev_error, 4),
            "p90_bev_error": round(self.p90_bev_error, 4),
            "p95_bev_error": round(self.p95_bev_error, 4),
            "total_radar": self.total_radar,
            "total_vision": self.total_vision,
            "total_matched": self.total_matched,
            "distance_bins": [b.to_dict() for b in self.distance_bins],
            "lane_metrics": [l.to_dict() for l in self.lane_metrics],
        }
        if self.grid_metrics is not None:
            d["grid_metrics"] = self.grid_metrics.to_dict()
        return d
