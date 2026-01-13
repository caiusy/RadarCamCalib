"""
calibration.py - Camera and Radar Calibration Module

This module handles:
1. Vanishing point detection from parallel lines
2. Camera pitch angle calculation
3. Radar-to-BEV coordinate transformation
4. Image-to-BEV coordinate transformation (ground plane projection)

Author: caiusy
Date: 2026-01-13
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional


@dataclass
class CameraParams:
    """Camera intrinsic and extrinsic parameters."""
    height: float = 1.5      # Camera height above ground (m)
    pitch: float = 0.0       # Camera pitch angle (rad), positive = looking down
    fx: float = 1000.0       # Focal length x (pixels)
    fy: float = 1000.0       # Focal length y (pixels)
    cx: float = 640.0        # Principal point x
    cy: float = 480.0        # Principal point y
    
    @property
    def K(self) -> np.ndarray:
        """Camera intrinsic matrix."""
        return np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ], dtype=np.float64)


@dataclass  
class RadarParams:
    """Radar mounting parameters."""
    yaw: float = 0.0         # Radar yaw offset (rad), positive = rotated left
    elev: float = 0.0        # Radar elevation offset (rad)
    x_offset: float = 3.5    # Radar X position relative to rear axle (m)
    y_offset: float = 0.0    # Radar Y position relative to rear axle (m)


class VanishingPointCalibrator:
    """
    Calculate camera pitch from vanishing point of parallel lines.
    
    Usage:
        calibrator = VanishingPointCalibrator()
        calibrator.add_line(x1, y1, x2, y2)  # Add parallel line
        calibrator.add_line(x1, y1, x2, y2)  # Add another
        vp = calibrator.compute_vanishing_point()
        pitch = calibrator.compute_pitch(cy, fy)
    """
    
    def __init__(self):
        self.lines: List[Tuple[float, float, float, float]] = []
        self._vanishing_point: Optional[Tuple[float, float]] = None
    
    def add_line(self, x1: float, y1: float, x2: float, y2: float):
        """Add a line segment defined by two endpoints."""
        self.lines.append((x1, y1, x2, y2))
        self._vanishing_point = None  # Invalidate cache
    
    def clear(self):
        """Clear all lines."""
        self.lines.clear()
        self._vanishing_point = None
    
    def remove_last_line(self) -> bool:
        """Remove the last added line."""
        if self.lines:
            self.lines.pop()
            self._vanishing_point = None
            return True
        return False
    
    def compute_vanishing_point(self) -> Optional[Tuple[float, float]]:
        """
        Compute vanishing point from at least 2 lines.
        Uses least squares intersection.
        
        Returns:
            (vp_x, vp_y) or None if not enough lines
        """
        if len(self.lines) < 2:
            return None
        
        if self._vanishing_point is not None:
            return self._vanishing_point
        
        # Convert lines to homogeneous representation (ax + by + c = 0)
        homo_lines = []
        for x1, y1, x2, y2 in self.lines:
            # Line through two points: (y1-y2)x + (x2-x1)y + (x1*y2-x2*y1) = 0
            a = y1 - y2
            b = x2 - x1
            c = x1 * y2 - x2 * y1
            # Normalize
            norm = np.sqrt(a*a + b*b)
            if norm > 1e-6:
                homo_lines.append([a/norm, b/norm, c/norm])
        
        if len(homo_lines) < 2:
            return None
        
        # Least squares: find point minimizing sum of squared distances to lines
        # For 2 lines, just find intersection
        if len(homo_lines) == 2:
            l1, l2 = homo_lines
            # Cross product gives intersection in homogeneous coordinates
            vp_homo = np.cross(l1, l2)
            if abs(vp_homo[2]) < 1e-9:
                return None  # Parallel lines
            vp_x = vp_homo[0] / vp_homo[2]
            vp_y = vp_homo[1] / vp_homo[2]
        else:
            # For more lines, use least squares
            A = np.array([[l[0], l[1]] for l in homo_lines])
            b = np.array([-l[2] for l in homo_lines])
            try:
                result = np.linalg.lstsq(A, b, rcond=None)
                vp_x, vp_y = result[0]
            except:
                return None
        
        self._vanishing_point = (vp_x, vp_y)
        return self._vanishing_point
    
    def compute_pitch(self, cy: float, fy: float) -> Optional[float]:
        """
        Compute camera pitch angle from vanishing point.
        
        Args:
            cy: Principal point y coordinate
            fy: Focal length y
            
        Returns:
            Pitch angle in radians (positive = looking down)
        """
        vp = self.compute_vanishing_point()
        if vp is None:
            return None
        
        vp_y = vp[1]
        # Vanishing point below principal point means camera looking down
        pitch = np.arctan((cy - vp_y) / fy)
        return pitch


class CoordinateTransformer:
    """
    Transform coordinates between radar, camera, and BEV systems.
    
    BEV Coordinate System (Vehicle Frame):
        - X: forward (positive ahead)
        - Y: left (positive left)
        - Origin: rear axle center
    
    Radar Coordinate System:
        - Radar measures (x_radar, y_radar) in its own frame
        - Needs yaw rotation and position offset to convert to BEV
    
    Camera Coordinate System:
        - Image pixels (u, v) with origin at top-left
        - Needs intrinsics, height, and pitch to project to ground (BEV)
    """
    
    def __init__(self, camera: CameraParams = None, radar: RadarParams = None):
        self.camera = camera or CameraParams()
        self.radar = radar or RadarParams()
    
    def radar_to_bev(self, x_radar: float, y_radar: float) -> Tuple[float, float]:
        """
        Transform radar coordinates to BEV (vehicle) coordinates.
        
        Args:
            x_radar: Forward distance in radar frame (m)
            y_radar: Lateral distance in radar frame (m), positive = left
            
        Returns:
            (x_bev, y_bev) in vehicle frame
        """
        # Apply yaw rotation (rotate radar frame to align with vehicle frame)
        cos_yaw = np.cos(self.radar.yaw)
        sin_yaw = np.sin(self.radar.yaw)
        
        x_rot = x_radar * cos_yaw - y_radar * sin_yaw
        y_rot = x_radar * sin_yaw + y_radar * cos_yaw
        
        # Add radar mounting offset
        x_bev = x_rot + self.radar.x_offset
        y_bev = y_rot + self.radar.y_offset
        
        return (x_bev, y_bev)
    
    def bev_to_radar(self, x_bev: float, y_bev: float) -> Tuple[float, float]:
        """
        Transform BEV coordinates to radar coordinates (inverse of radar_to_bev).
        """
        # Remove offset
        x_rot = x_bev - self.radar.x_offset
        y_rot = y_bev - self.radar.y_offset
        
        # Inverse yaw rotation
        cos_yaw = np.cos(-self.radar.yaw)
        sin_yaw = np.sin(-self.radar.yaw)
        
        x_radar = x_rot * cos_yaw - y_rot * sin_yaw
        y_radar = x_rot * sin_yaw + y_rot * cos_yaw
        
        return (x_radar, y_radar)
    
    def image_to_bev(self, u: float, v: float) -> Optional[Tuple[float, float]]:
        """
        Project image pixel to BEV assuming the point is on the ground plane.
        
        Uses pinhole camera model with ground plane intersection.
        
        Args:
            u: Pixel x coordinate
            v: Pixel y coordinate
            
        Returns:
            (x_bev, y_bev) or None if projection fails (point above horizon)
        """
        # Normalize pixel to camera coordinates
        x_norm = (u - self.camera.cx) / self.camera.fx
        y_norm = (v - self.camera.cy) / self.camera.fy
        
        # Ray direction in camera frame (looking along Z axis)
        # With pitch rotation, Y points down initially
        ray_cam = np.array([x_norm, y_norm, 1.0])
        ray_cam = ray_cam / np.linalg.norm(ray_cam)
        
        # Map camera frame to vehicle frame with pitch
        # Camera: X=right, Y=down, Z=forward
        # Vehicle: X=forward, Y=left, Z=up
        # pitch > 0 means camera tilted down
        
        # Simple mapping considering pitch
        # Forward direction (vehicle X) = camera Z rotated by pitch
        cos_p = np.cos(self.camera.pitch)
        sin_p = np.sin(self.camera.pitch)
        
        # Ray in vehicle frame
        # X_veh (forward) = Z_cam * cos - Y_cam * sin
        # Y_veh (left) = -X_cam
        # Z_veh (up) = -Z_cam * sin - Y_cam * cos
        ray_vehicle = np.array([
            ray_cam[2] * cos_p - ray_cam[1] * sin_p,  # forward
            -ray_cam[0],                                 # left
            -ray_cam[2] * sin_p - ray_cam[1] * cos_p   # up
        ])
        
        # Camera position in vehicle frame
        # Assume camera is at (cam_x, 0, cam_height) - camera at front, centered
        cam_x = 3.5  # Camera X offset from rear axle
        cam_z = self.camera.height
        
        # Find intersection with ground plane (z = 0)
        # Camera at (cam_x, 0, cam_z), ray direction ray_vehicle
        # Point on ray: P = cam_pos + t * ray
        # Ground plane: z = 0
        # cam_z + t * ray_z = 0 => t = -cam_z / ray_z
        
        if ray_vehicle[2] >= 0:
            # Ray pointing up or horizontal, no ground intersection
            return None
        
        t = -cam_z / ray_vehicle[2]
        if t < 0:
            return None
        
        x_bev = cam_x + t * ray_vehicle[0]
        y_bev = t * ray_vehicle[1]  # Note: left is positive in BEV
        
        return (x_bev, y_bev)
    
    def bev_to_image(self, x_bev: float, y_bev: float) -> Optional[Tuple[float, float]]:
        """
        Project BEV point to image (inverse of image_to_bev).
        
        Args:
            x_bev: Forward distance (m)
            y_bev: Lateral distance (m), positive = left
            
        Returns:
            (u, v) pixel coordinates or None if behind camera
        """
        # Camera position in vehicle frame
        cam_x = 3.5
        cam_z = self.camera.height
        
        # Vector from camera to point (point is on ground, z=0)
        dx = x_bev - cam_x
        dy = y_bev
        dz = -cam_z
        
        # Transform to camera frame (inverse pitch rotation)
        cos_p = np.cos(-self.camera.pitch)
        sin_p = np.sin(-self.camera.pitch)
        
        x_cam = dx
        y_cam = dy * cos_p - dz * sin_p
        z_cam = dy * sin_p + dz * cos_p
        
        if z_cam <= 0:
            return None  # Behind camera
        
        # Project to image
        u = self.camera.fx * (x_cam / z_cam) + self.camera.cx
        v = self.camera.fy * (y_cam / z_cam) + self.camera.cy
        
        return (u, v)


# Utility functions
def fit_homography(src_pts: np.ndarray, dst_pts: np.ndarray) -> Optional[np.ndarray]:
    """
    Fit a homography matrix from source to destination points.
    
    Args:
        src_pts: (N, 2) source points
        dst_pts: (N, 2) destination points
        
    Returns:
        3x3 homography matrix or None if fitting fails
    """
    if len(src_pts) < 4 or len(dst_pts) < 4:
        return None
    
    try:
        import cv2
        H, _ = cv2.findHomography(src_pts, dst_pts)
        return H
    except:
        return None
