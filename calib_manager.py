"""
CalibrationManager - Wrapper for calibration functionality

Combines VanishingPointCalibrator and CoordinateTransformer.
"""

from calibration import CameraParams, RadarParams, VanishingPointCalibrator, CoordinateTransformer
from typing import Optional, Tuple, List


class CalibrationManager:
    """Manages camera and radar calibration for BEV projection."""
    
    def __init__(self):
        self.camera = CameraParams()
        self.radar = RadarParams()
        self.v_calibrator = VanishingPointCalibrator()
        self.transformer = CoordinateTransformer(self.camera, self.radar)
        self.vanishing_lines = []
    
    @property
    def is_calibrated(self) -> bool:
        return len(self.vanishing_lines) >= 2 or self.camera.pitch != 0
    
    @property
    def num_vanishing_lines(self) -> int:
        return len(self.vanishing_lines)
    
    def update_camera_params(self, **kwargs):
        """Update camera parameters."""
        # Only update provided params, preserve others (especially pitch)
        for key, val in kwargs.items():
            if hasattr(self.camera, key):
                setattr(self.camera, key, val)
        # Recreate transformer with updated params
        self.transformer = CoordinateTransformer(self.camera, self.radar)
    
    def update_radar_params(self, **kwargs):
        """Update radar parameters."""
        for key, val in kwargs.items():
            if hasattr(self.radar, key):
                setattr(self.radar, key, val)
        self.transformer = CoordinateTransformer(self.camera, self.radar)
    
    def add_vanishing_line(self, x1, y1, x2, y2):
        """Add a vanishing line."""
        self.v_calibrator.add_line(x1, y1, x2, y2)
        self.vanishing_lines.append((x1, y1, x2, y2))
    
    def undo_vanishing_line(self) -> bool:
        """Remove last vanishing line."""
        if self.vanishing_lines:
            self.vanishing_lines.pop()
            self.v_calibrator.remove_last_line()
            return True
        return False
    
    def clear_vanishing_lines(self):
        """Clear all vanishing lines."""
        self.vanishing_lines.clear()
        self.v_calibrator.clear()
    
    def compute_pitch_from_vanishing_point(self) -> Optional[float]:
        """Compute pitch from vanishing point."""
        pitch = self.v_calibrator.compute_pitch(self.camera.cy, self.camera.fy)
        if pitch is not None:
            self.camera.pitch = pitch
            self.transformer = CoordinateTransformer(self.camera, self.radar)
        return pitch
    
    def compute_pitch_from_lanes(self, lanes: List) -> Optional[float]:
        """
        Compute pitch from lane lines (parallel lines).
        
        Args:
            lanes: List of lane objects with start/end attributes, 
                   or list of tuples ((x1,y1), (x2,y2))
        
        Returns:
            pitch in radians or None if computation fails
        """
        self.v_calibrator.clear()
        
        for lane in lanes:
            # Handle different lane formats
            if hasattr(lane, 'start') and hasattr(lane, 'end'):
                # Lane object from operations.py
                x1, y1 = lane.start
                x2, y2 = lane.end
            elif isinstance(lane, (list, tuple)) and len(lane) == 2:
                # Tuple of points ((x1,y1), (x2,y2))
                (x1, y1), (x2, y2) = lane
            else:
                continue
            
            self.v_calibrator.add_line(x1, y1, x2, y2)
        
        # Compute pitch
        pitch = self.v_calibrator.compute_pitch(self.camera.cy, self.camera.fy)
        if pitch is not None:
            self.camera.pitch = pitch
            self.transformer = CoordinateTransformer(self.camera, self.radar)
        
        return pitch
    
    def get_vanishing_point(self) -> Optional[Tuple[float, float]]:
        """Get vanishing point."""
        return self.v_calibrator.compute_vanishing_point()
    
    def radar_to_bev(self, x_radar, y_radar):
        """Transform radar to BEV."""
        return self.transformer.radar_to_bev(x_radar, y_radar)
    
    def image_to_bev(self, u, v):
        """Project image pixel to BEV."""
        return self.transformer.image_to_bev(u, v)
    
    def bev_to_image(self, x_bev, y_bev):
        """Project BEV point to image."""
        return self.transformer.bev_to_image(x_bev, y_bev)
    
    def load_ground_truth(self, gt_data: dict):
        """Load ground truth parameters."""
        if 'camera' in gt_data:
            self.update_camera_params(**gt_data['camera'])
        if 'radar' in gt_data:
            self.update_radar_params(**gt_data['radar'])
