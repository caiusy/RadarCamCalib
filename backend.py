"""
backend.py - Data loading, calibration computation, and file operations
"""

import os
import json
import numpy as np
from typing import List, Tuple, Optional, Dict
from datetime import datetime


class DataManager:
    """Manages data loading and file operations."""
    
    def __init__(self):
        self.data_root: str = ""
        self.sync_data: List[dict] = []
        self.current_batch: int = 0
        self.current_image_path: str = ""
        self.current_radar_data: dict = {}
    
    def load_sync_json(self, path: str) -> int:
        """Load sync JSON file. Returns number of batches."""
        with open(path, 'r', encoding='utf-8') as f:
            self.sync_data = json.load(f)
        self.data_root = os.path.dirname(path)
        return len(self.sync_data)
    
    def get_batch(self, idx: int) -> Tuple[str, dict]:
        """Get image path and radar data for a batch."""
        if idx >= len(self.sync_data):
            return "", {}
        
        batch = self.sync_data[idx]
        
        # Image
        img_rel = batch.get('image_path', batch.get('image', ''))
        img_path = os.path.join(self.data_root, img_rel) if img_rel else ""
        
        # Radar
        radar_rel = batch.get('radar_json', batch.get('radar', ''))
        radar_data = {}
        if radar_rel:
            radar_path = os.path.join(self.data_root, radar_rel)
            try:
                with open(radar_path, 'r', encoding='utf-8') as f:
                    radar_data = json.load(f)
            except:
                pass
        
        self.current_batch = idx
        self.current_image_path = img_path
        self.current_radar_data = radar_data
        return img_path, radar_data
    
    @property
    def num_batches(self) -> int:
        return len(self.sync_data)


class Calibration:
    """Handles calibration matrix computation and projection."""
    
    def __init__(self):
        self.H: Optional[np.ndarray] = None  # Radar -> Image homography
        self.H_inv: Optional[np.ndarray] = None
        self.loaded: bool = False
        self.num_points: int = 0
    
    def load_from_file(self, path: str) -> int:
        """Load calibration points from file. Returns number of points."""
        points = []
        with open(path, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                vals = list(map(float, line.split()[:4]))
                if len(vals) == 4:
                    points.append(vals)
        
        if len(points) < 4:
            raise ValueError(f"Need at least 4 points, got {len(points)}")
        
        self._compute_homography(points)
        self.loaded = True
        self.num_points = len(points)
        return len(points)
    
    def _compute_homography(self, points: List[List[float]]):
        """Compute homography from point correspondences."""
        import cv2
        src = np.array([[p[0], p[1]] for p in points], dtype=np.float32)
        dst = np.array([[p[2], p[3]] for p in points], dtype=np.float32)
        self.H, _ = cv2.findHomography(src, dst)
        if self.H is not None:
            self.H_inv = np.linalg.inv(self.H)
    
    def project_radar_to_image(self, rx: float, ry: float) -> Tuple[float, float]:
        """Project radar point to image coordinates."""
        if self.H is None:
            return 0, 0
        pt = np.array([rx, ry, 1.0])
        proj = self.H @ pt
        return proj[0]/proj[2], proj[1]/proj[2]
    
    def project_image_to_radar(self, u: float, v: float) -> Tuple[float, float]:
        """Inverse project image point to radar coordinates."""
        if self.H_inv is None:
            return 0, 0
        pt = np.array([u, v, 1.0])
        proj = self.H_inv @ pt
        return proj[0]/proj[2], proj[1]/proj[2]


# Import calibration manager for BEV projection
try:
    from calibration import CalibrationManager as _CalibrationManager
    CalibrationManager = _CalibrationManager
except ImportError:
    # Fallback if calibration.py not available
    CalibrationManager = None


class DataExporter:
    """Handles exporting data to files."""
    
    @staticmethod
    def save_point_pairs(pairs: List[dict], directory: str) -> str:
        """Save point pairs to file. Returns file path."""
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(directory, f"point_pairs_{ts}.txt")
        
        with open(path, 'w', encoding='utf-8') as f:
            f.write("# pixel_u, pixel_v, radar_id, radar_x, radar_y, range, velocity, rcs, batch\n")
            for p in pairs:
                f.write(f"{p['pixel_u']:.2f}, {p['pixel_v']:.2f}, "
                        f"{p['radar_id']}, {p['radar_x']:.2f}, {p['radar_y']:.2f}, "
                        f"{p.get('radar_range', 0):.2f}, {p.get('radar_velocity', 0):.2f}, "
                        f"{p.get('radar_rcs', 0):.2f}, {p['batch']}\n")
        return path
    
    @staticmethod
    def save_lane(lane: List[Tuple[float, float]], lane_id: int, directory: str) -> str:
        """Save a single lane (2 points) to file. Returns file path."""
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(directory, f"lane_{lane_id}_{ts}.txt")
        
        with open(path, 'w', encoding='utf-8') as f:
            f.write("# Lane line: start_u, start_v, end_u, end_v\n")
            if len(lane) >= 2:
                f.write(f"{lane[0][0]:.2f}, {lane[0][1]:.2f}, {lane[1][0]:.2f}, {lane[1][1]:.2f}\n")
        return path
    
    @staticmethod
    def save_all_lanes(lanes: List[List[Tuple[float, float]]], directory: str) -> str:
        """Save all lanes to a single file."""
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(directory, f"all_lanes_{ts}.txt")
        
        with open(path, 'w', encoding='utf-8') as f:
            f.write("# Lane lines: lane_id, start_u, start_v, end_u, end_v\n")
            for i, lane in enumerate(lanes):
                if len(lane) >= 2:
                    f.write(f"{i+1}, {lane[0][0]:.2f}, {lane[0][1]:.2f}, "
                            f"{lane[1][0]:.2f}, {lane[1][1]:.2f}\n")
        return path
