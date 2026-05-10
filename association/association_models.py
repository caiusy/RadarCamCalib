"""
association_models.py - Data models for radar-vision association results.

Defines Match and AssociationResult dataclasses that hold
the output of the Hungarian matching pipeline.
"""

from dataclasses import dataclass, field
from typing import List, Optional
import numpy as np


@dataclass
class Match:
    """A single radar-vision matched pair.

    Attributes:
        radar_idx: Index into the radar object list.
        vision_idx: Index into the vision object list.
        cost: Total matching cost (0-1, lower is better).
        bev_distance: BEV distance between matched pair (meters).
        pixel_distance: Image pixel distance between matched pair (pixels).
    """
    radar_idx: int
    vision_idx: int
    cost: float
    bev_distance: float = 0.0
    pixel_distance: float = 0.0

    def to_dict(self) -> dict:
        """Serialize to dictionary."""
        return {
            "radar_idx": self.radar_idx,
            "vision_idx": self.vision_idx,
            "cost": round(self.cost, 4),
            "bev_distance": round(self.bev_distance, 4),
            "pixel_distance": round(self.pixel_distance, 4),
        }


@dataclass
class AssociationResult:
    """Result of radar-vision association.

    Attributes:
        matches: List of matched radar-vision pairs.
        unmatched_radar: List of radar indices with no vision match.
        unmatched_vision: List of vision indices with no radar match.
        cost_matrix: Full cost matrix (N_radar × N_vision), optional.
        gate_matrix: Boolean gate matrix (N_radar × N_vision), optional.
    """
    matches: List[Match]
    unmatched_radar: List[int]
    unmatched_vision: List[int]
    cost_matrix: Optional[np.ndarray] = None
    gate_matrix: Optional[np.ndarray] = None

    @property
    def num_matches(self) -> int:
        return len(self.matches)

    @property
    def num_unmatched_radar(self) -> int:
        return len(self.unmatched_radar)

    @property
    def num_unmatched_vision(self) -> int:
        return len(self.unmatched_vision)

    def to_dict(self) -> dict:
        """Serialize to dictionary (without matrices)."""
        return {
            "matches": [m.to_dict() for m in self.matches],
            "unmatched_radar": self.unmatched_radar,
            "unmatched_vision": self.unmatched_vision,
            "num_matches": self.num_matches,
            "num_unmatched_radar": self.num_unmatched_radar,
            "num_unmatched_vision": self.num_unmatched_vision,
        }
