"""
operations.py - Business logic for point pair selection, lane drawing, and undo/redo
operations.py - 点对选择、车道线绘制和撤销/重做业务逻辑
"""

from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass, field
from enum import Enum, auto


class AppMode(Enum):
    NORMAL = auto()
    SELECT_RADAR = auto()      # Waiting for radar point selection
    SELECT_IMAGE = auto()      # Waiting for image point selection
    LANE_START = auto()        # Waiting for lane start point
    LANE_END = auto()          # Waiting for lane end point


@dataclass
class PointPair:
    """A completed radar-image point pair."""
    batch: int
    radar_id: int
    radar_x: float
    radar_y: float
    radar_u: float  # Projected image position of radar
    radar_v: float
    pixel_u: float  # User-selected image position
    pixel_v: float
    radar_range: float = 0.0
    radar_velocity: float = 0.0
    radar_rcs: float = 0.0
    
    def to_dict(self) -> dict:
        return {
            'batch': self.batch,
            'radar_id': self.radar_id,
            'radar_x': self.radar_x,
            'radar_y': self.radar_y,
            'radar_u': self.radar_u,
            'radar_v': self.radar_v,
            'pixel_u': self.pixel_u,
            'pixel_v': self.pixel_v,
            'radar_range': self.radar_range,
            'radar_velocity': self.radar_velocity,
            'radar_rcs': self.radar_rcs,
        }


@dataclass
class Lane:
    """A lane line defined by start and end points."""
    start: Tuple[float, float]
    end: Tuple[float, float]
    batch: int
    
    def to_list(self) -> List[Tuple[float, float]]:
        return [self.start, self.end]


@dataclass
class PendingRadarSelection:
    """Temporary state when a radar point is selected but image point not yet."""
    target: dict
    projected_u: float
    projected_v: float


class OperationsController:
    """
    Controls point pair selection and lane drawing logic.
    控制点对选择和车道线绘制逻辑。
    Manages undo stack and mode transitions.
    管理撤销栈和模式转换。
    """
    
    MAX_PAIRS = 10
    
    def __init__(self):
        self.mode = AppMode.NORMAL
        self.pairs: List[PointPair] = []
        self.lanes: List[Lane] = []
        self.current_batch: int = 0
        
        # Pending selections
        self.pending_radar: Optional[PendingRadarSelection] = None
        self.pending_lane_start: Optional[Tuple[float, float]] = None
        
        # Undo stacks (simple - just track what to remove)
        self._pair_undo_stack: List[PointPair] = []
        self._lane_undo_stack: List[Lane] = []
    
    # -------------------------------------------------------------------------
    # Mode Management
    # -------------------------------------------------------------------------
    def start_pair_selection(self):
        """Enter point pair selection mode."""
        self.mode = AppMode.SELECT_RADAR
        self.pending_radar = None
    
    def start_lane_drawing(self):
        """Enter lane drawing mode."""
        self.mode = AppMode.LANE_START
        self.pending_lane_start = None
    
    def cancel(self):
        """Cancel current operation and return to normal."""
        self.mode = AppMode.NORMAL
        self.pending_radar = None
        self.pending_lane_start = None
    
    # -------------------------------------------------------------------------
    # Point Pair Selection
    # -------------------------------------------------------------------------
    def select_radar_point(self, target: dict, proj_u: float, proj_v: float) -> bool:
        """
        First step: select a radar point.
        Returns True if successful, False if max pairs reached.
        """
        if len(self.pairs) >= self.MAX_PAIRS:
            return False
        
        self.pending_radar = PendingRadarSelection(
            target=target,
            projected_u=proj_u,
            projected_v=proj_v
        )
        self.mode = AppMode.SELECT_IMAGE
        return True
    
    def select_image_point(self, pixel_u: float, pixel_v: float) -> Optional[PointPair]:
        """
        Second step: select corresponding image point.
        Returns the completed PointPair if successful.
        """
        if not self.pending_radar:
            return None
        
        t = self.pending_radar.target
        pair = PointPair(
            batch=self.current_batch,
            radar_id=t.get('id', -1),
            radar_x=t.get('x', 0),
            radar_y=t.get('y', 0),
            radar_u=self.pending_radar.projected_u,
            radar_v=self.pending_radar.projected_v,
            pixel_u=pixel_u,
            pixel_v=pixel_v,
            radar_range=t.get('range', 0),
            radar_velocity=t.get('velocity', 0),
            radar_rcs=t.get('rcs', 0),
        )
        
        self.pairs.append(pair)
        self._pair_undo_stack.append(pair)
        self.pending_radar = None
        
        # Return to radar selection mode for next pair
        if len(self.pairs) < self.MAX_PAIRS:
            self.mode = AppMode.SELECT_RADAR
        else:
            self.mode = AppMode.NORMAL
        
        return pair
    
    def get_pairs_for_batch(self, batch: int) -> List[Tuple[int, PointPair]]:
        """Get all pairs for a specific batch with their indices."""
        return [(i, p) for i, p in enumerate(self.pairs) if p.batch == batch]
    
    # -------------------------------------------------------------------------
    # Lane Drawing
    # -------------------------------------------------------------------------
    def set_lane_start(self, x: float, y: float):
        """Set the start point of a new lane."""
        self.pending_lane_start = (x, y)
        self.mode = AppMode.LANE_END
    
    def set_lane_end(self, x: float, y: float) -> Optional[Lane]:
        """Set the end point and complete the lane."""
        if not self.pending_lane_start:
            return None
        
        lane = Lane(
            start=self.pending_lane_start,
            end=(x, y),
            batch=self.current_batch
        )
        
        self.lanes.append(lane)
        self._lane_undo_stack.append(lane)
        self.pending_lane_start = None
        
        # Ready for next lane
        self.mode = AppMode.LANE_START
        return lane
    
    def get_lanes_for_batch(self, batch: int) -> List[Tuple[int, Lane]]:
        """Get all lanes for a specific batch with their indices."""
        return [(i, l) for i, l in enumerate(self.lanes) if l.batch == batch]
    
    # -------------------------------------------------------------------------
    # Undo
    # -------------------------------------------------------------------------
    def undo_last_pair(self) -> Optional[PointPair]:
        """Undo the last point pair. Returns removed pair or None."""
        if self._pair_undo_stack:
            pair = self._pair_undo_stack.pop()
            if pair in self.pairs:
                self.pairs.remove(pair)
            return pair
        return None
    
    def undo_last_lane(self) -> Optional[Lane]:
        """Undo the last lane. Returns removed lane or None."""
        if self._lane_undo_stack:
            lane = self._lane_undo_stack.pop()
            if lane in self.lanes:
                self.lanes.remove(lane)
            return lane
        return None
    
    def undo_pending(self) -> bool:
        """Undo pending selection (radar or lane start)."""
        if self.pending_radar:
            self.pending_radar = None
            self.mode = AppMode.SELECT_RADAR
            return True
        if self.pending_lane_start:
            self.pending_lane_start = None
            self.mode = AppMode.LANE_START
            return True
        return False
    
    # -------------------------------------------------------------------------
    # Clear
    # -------------------------------------------------------------------------
    def clear_all(self):
        """Clear all pairs and lanes."""
        self.pairs.clear()
        self.lanes.clear()
        self._pair_undo_stack.clear()
        self._lane_undo_stack.clear()
        self.pending_radar = None
        self.pending_lane_start = None
        self.mode = AppMode.NORMAL
    
    def clear_batch(self, batch: int):
        """Clear pairs and lanes for a specific batch."""
        self.pairs = [p for p in self.pairs if p.batch != batch]
        self.lanes = [l for l in self.lanes if l.batch != batch]
    
    # -------------------------------------------------------------------------
    # Stats
    # -------------------------------------------------------------------------
    @property
    def num_pairs(self) -> int:
        return len(self.pairs)
    
    @property
    def num_lanes(self) -> int:
        return len(self.lanes)
    
    @property
    def can_add_pair(self) -> bool:
        return len(self.pairs) < self.MAX_PAIRS
