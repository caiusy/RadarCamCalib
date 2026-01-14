"""
calibration.py - Camera and Radar Calibration Module
calibration.py - 相机和雷达标定模块

This module handles / 本模块处理:
1. Vanishing point detection from parallel lines (从平行线检测消失点)
2. Camera pitch angle calculation (相机俯仰角计算)
3. Radar-to-BEV coordinate transformation (雷达坐标系到鸟瞰图坐标系的转换)
4. Image-to-BEV coordinate transformation (ground plane projection) (图像到鸟瞰图的坐标转换 - 地面平面投影)

Coordinate Systems / 坐标系定义:
- BEV / Vehicle (鸟瞰图/车辆): X=Forward(前), Y=Left(左), Z=Up(上)
- Camera (相机): X=Right(右), Y=Down(下), Z=Forward(前)

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
    在雷达、相机和鸟瞰图（BEV）坐标系之间进行坐标转换。
    
    BEV Coordinate System (Vehicle Frame) / BEV坐标系（车辆坐标系）:
        - X: forward (positive ahead) / 前向（正向向前）
        - Y: left (positive left) / 横向（正向向左）
        - Origin: rear axle center / 原点：后轴中心
    
    Radar Coordinate System / 雷达坐标系:
        - Radar measures (x_radar, y_radar) in its own frame / 雷达在其自身坐标系中测量
        - Needs yaw rotation and position offset to convert to BEV / 需要偏航旋转和位置偏移以转换为BEV
    
    Camera Coordinate System / 相机坐标系:
        - Image pixels (u, v) with origin at top-left / 图像像素(u, v)，原点在左上角
        - Needs intrinsics, height, and pitch to project to ground (BEV) / 需要内参、高度和俯仰角以投影到地面(BEV)
    """
    
    def __init__(self, camera: CameraParams = None, radar: RadarParams = None):
        self.camera = camera or CameraParams()
        self.radar = radar or RadarParams()
    
    def radar_to_bev(self, x_radar: float, y_radar: float) -> Tuple[float, float]:
        """
        Transform radar coordinates to BEV (vehicle) coordinates.
        NEW SYSTEM: 
          - BEV Y = Forward (0-160m)
          - BEV X = Lateral (Right)
        
        Args:
            x_radar: Forward distance in radar frame (m)
            y_radar: Lateral distance in radar frame (m), positive = left
            
        Returns:
            (x_bev, y_bev) in vehicle frame (X=Right, Y=Forward)
        """
        # 1. Rotate Radar Frame
        # Radar standard: x=Forward, y=Left.
        # Yaw is rotation around Z (Up). Positive = Left.
        cos_yaw = np.cos(self.radar.yaw)
        sin_yaw = np.sin(self.radar.yaw)
        
        # Rotated coordinates (still Forward/Left convention)
        x_rot_fwd = x_radar * cos_yaw - y_radar * sin_yaw
        y_rot_left = x_radar * sin_yaw + y_radar * cos_yaw
        
        # 2. Map to BEV (Y=Forward, X=Right)
        # BEV Y = Forward + Offset Y (User Expectation)
        # BEV X = Right = -Left + Offset X (User Expectation)
        
        y_bev = x_rot_fwd + self.radar.y_offset
        x_bev = -y_rot_left + self.radar.x_offset
        
        return (x_bev, y_bev)
    
    def bev_to_radar(self, x_bev: float, y_bev: float) -> Tuple[float, float]:
        """
        Transform BEV (Y=Forward) to Radar (x=Forward, y=Left).
        """
        # Inverse mapping:
        # y_bev = x_rot_fwd + off_y  => x_rot_fwd = y_bev - off_y
        # x_bev = -y_rot_left + off_x => y_rot_left = -(x_bev - off_x) = off_x - x_bev
        
        x_rot_fwd = y_bev - self.radar.y_offset
        y_rot_left = self.radar.x_offset - x_bev
        
        # Inverse Rotation
        # x_r = x_rot * cos + y_rot * sin (Wait, inverse matrix is transpose)
        # [x_rot] = [c -s] [x]
        # [y_rot] = [s  c] [y]
        #
        # [x] = [ c  s] [x_rot]
        # [y] = [-s  c] [y_rot]
        
        cos_yaw = np.cos(self.radar.yaw)
        sin_yaw = np.sin(self.radar.yaw)
        
        x_radar = x_rot_fwd * cos_yaw + y_rot_left * sin_yaw
        y_radar = -x_rot_fwd * sin_yaw + y_rot_left * cos_yaw
        
        return (x_radar, y_radar)
    
    def image_to_bev(self, u: float, v: float) -> Optional[Tuple[float, float]]:
        """
        Project image pixel to BEV (Y=Forward, X=Right).
        """
        # Normalize pixel to camera coordinates
        x_norm = (u - self.camera.cx) / self.camera.fx
        y_norm = (v - self.camera.cy) / self.camera.fy
        
        # Ray in Camera Frame (Unrotated)
        # Cam X = Right
        # Cam Y = Down
        # Cam Z = Forward
        ray_cam = np.array([x_norm, y_norm, 1.0])
        ray_cam = ray_cam / np.linalg.norm(ray_cam)
        
        # Apply Pitch Rotation (Inverse from Cam -> World lookdown)
        # We want Ray in Vehicle Frame.
        # Vehicle Frame: X=Right, Y=Forward, Z=Up
        # Camera Frame at Pitch=0: X->X, Y->-Z, Z->Y (Wait, CamZ is Forward(Y), CamY is Down(-Z))
        
        # Let's do it step by step.
        # Camera Frame (rotated by pitch p):
        # Ray_c.
        
        # Un-pitch to align with vehicle axes directions:
        # Rotated around C_X (Right) by pitch.
        # Inverse: Rotate by -pitch.
        pitch = self.camera.pitch
        cos_p = np.cos(-pitch)
        sin_p = np.sin(-pitch)
        
        # ray_unpitched = [x, y*c - z*s, y*s + z*c]
        xc = ray_cam[0]
        yc = ray_cam[1]
        zc = ray_cam[2]
        
        rx = xc
        ry = yc * cos_p - zc * sin_p
        rz = yc * sin_p + zc * cos_p
        
        # Now we have ray in "Unrotated Camera Frame".
        # Axes: X_c=Right, Y_c=Down, Z_c=Forward.
        
        # Map to Vehicle Frame: X_v=Right, Y_v=Forward, Z_v=Up.
        # X_v = X_c
        # Y_v = Z_c
        # Z_v = -Y_c
        
        ray_veh = np.array([
            rx,      # X (Right) = Cam X (Right)
            rz,      # Y (Forward) = Cam Z (Forward)
            -ry      # Z (Up) = -Cam Y (Down)
        ])
        
        # Camera Position in Vehicle Frame
        # Cam is at Forward = cam_fwd (offset), Up = cam_h.
        cam_y_pos = 3.5  # Forward offset
        cam_x_pos = 0.0  # Lateral offset (center)
        cam_z_pos = self.camera.height
        
        # Intersection with Ground (Z=0)
        # P = C + t*dir
        # C_z + t*dir_z = 0 => t = -C_z / dir_z
        
        if ray_veh[2] >= 0:
            return None # pointing up
            
        t = -cam_z_pos / ray_veh[2]
        if t < 0:
            return None
            
        x_bev = cam_x_pos + t * ray_veh[0]
        y_bev = cam_y_pos + t * ray_veh[1]
        
        return (x_bev, y_bev)
    
    def bev_to_image(self, x_bev: float, y_bev: float) -> Optional[Tuple[float, float]]:
        """
        Project BEV (X=Right, Y=Forward) to Image.
        """
        cam_y_pos = 3.5
        cam_x_pos = 0.0
        cam_z_pos = self.camera.height
        
        # 1. Relative Vector in Vehicle Frame
        dx_v = x_bev - cam_x_pos
        dy_v = y_bev - cam_y_pos
        dz_v = 0.0 - cam_z_pos
        
        # 2. Map to Camera Frame (Unrotated)
        # Veh: X=Right, Y=Forward, Z=Up
        # Cam: X=Right, Y=Down, Z=Forward
        
        # Cam X = Veh X
        # Cam Y = -Veh Z
        # Cam Z = Veh Y
        
        cx0 = dx_v
        cy0 = -dz_v
        cz0 = dy_v
        
        # 3. Apply Pitch (Rotate coordinates, not point)
        # Point is fixed, Frame rotates.
        # Matrix R_pitch * P_unrot? 
        # Or P_rot = R_pitch * P_unrot.
        # Pitch is positive down (around X).
        # [1 0 0]
        # [0 c -s]
        # [0 s c]
        
        pitch = self.camera.pitch
        cos_p = np.cos(pitch)
        sin_p = np.sin(pitch)
        
        x_cam = cx0
        y_cam = cy0 * cos_p - cz0 * sin_p
        z_cam = cy0 * sin_p + cz0 * cos_p
        
        if z_cam <= 0.1:
            return None
            
        # 4. Project
        u = self.camera.fx * (x_cam / z_cam) + self.camera.cx
        v = self.camera.fy * (y_cam / z_cam) + self.camera.cy
        
        return (u, v)
    
    def optimize_pitch(self, pairs: List[dict], search_range: int = 50) -> float:
        """
        Optimize pitch by searching Vanishing Point Y within range.
        pairs: list of dicts with 'radar_x', 'radar_y', 'pixel_u', 'pixel_v'.
        """
        if not pairs:
            return self.camera.pitch
        
        # Current VP
        current_pitch = self.camera.pitch
        cy = self.camera.cy
        fy = self.camera.fy
        
        # vp_y = cy - fy * tan(pitch)
        current_vp_y = cy - fy * np.tan(current_pitch)
        
        best_vp_y = current_vp_y
        min_error = float('inf')
        
        # Grid search +/- search_range pixels
        # 101 steps = 1 pixel per step roughly (if range is 50, strictly 1 pixel)
        search_vals = np.linspace(current_vp_y - search_range, current_vp_y + search_range, 101)
        
        original_pitch = self.camera.pitch
        
        for vp_y in search_vals:
            # pitch = atan((cy - vp_y)/fy)
            pitch = np.arctan((cy - vp_y) / fy)
            self.camera.pitch = pitch
            
            error = 0.0
            valid_count = 0
            for p in pairs:
                rx, ry = p['radar_x'], p['radar_y']
                u_meas, v_meas = p['pixel_u'], p['pixel_v']
                
                # Radar -> BEV
                x_bev, y_bev = self.radar_to_bev(rx, ry)
                
                # BEV -> Image
                res = self.bev_to_image(x_bev, y_bev)
                if res:
                    u_proj, v_proj = res
                    # Squared Euclidean distance
                    dist = (u_proj - u_meas)**2 + (v_proj - v_meas)**2
                    error += dist
                    valid_count += 1
                else:
                    error += 100000.0 # Penalty
            
            # Average error (RMSE-like metric)
            if valid_count > 0:
                if error < min_error:
                    min_error = error
                    best_vp_y = vp_y
        
        # Set best pitch
        final_pitch = np.arctan((cy - best_vp_y) / fy)
        self.camera.pitch = final_pitch
        print(f"[Optimize] Best VP_y: {best_vp_y:.2f} (Delta: {best_vp_y - current_vp_y:.2f}), Pitch: {final_pitch:.4f}, Min Error: {min_error:.2f}")
        return final_pitch
        return final_pitch

    def get_radar_bev_homography(self) -> List[List[float]]:
        """
        Get Radar -> BEV Homography (Transformation Matrix).
        H_rb such that [x_b, y_b, 1] = H * [x_r, y_r, 1]
        
        Logic:
        x_bev = -y_rot_left + off_x
        y_bev = x_rot_fwd + off_y
        
        x_rot_fwd = x*c - y*s
        y_rot_left = x*s + y*c
        
        x_bev = -(x*s + y*c) + off_x = -s*x - c*y + off_x
        y_bev = (x*c - y*s) + off_y = c*x - s*y + off_y
        
        H = [
          [-s, -c, off_x],
          [ c, -s, off_y],
          [ 0,  0,     1]
        ]
        """
        yaw = self.radar.yaw
        cos_y = np.cos(yaw)
        sin_y = np.sin(yaw)
        
        off_x = self.radar.x_offset
        off_y = self.radar.y_offset
        
        # Check sign conventions carefully:
        # x_bev = -sin*x - cos*y + off_x
        # y_bev = cos*x - sin*y + off_y
        
        H = [
            [-sin_y, -cos_y, off_x],
            [cos_y, -sin_y, off_y],
            [0.0, 0.0, 1.0]
        ]
        return H

    def get_camera_bev_homography(self) -> Optional[List[List[float]]]:
        """
        Get Camera Image -> BEV Homography (H_ib).
        Project 4 points from BEV (Z=0 plane) to Image, then fit H.
        This H maps pixel (u,v) to BEV (x,y) assuming Z=0.
        """
        # Select 4 points in BEV
        # 0,0 (Ego)
        # 10, 50 (Right Forward)
        # -10, 50 (Left Forward)
        # 0, 100 (Far Forward)
        
        bev_pts = np.array([
            [0.0, 20.0],
            [10.0, 50.0],
            [-10.0, 50.0],
            [0.0, 100.0]
        ], dtype=np.float32)
        
        img_pts_list = []
        valid_bev = []
        
        for pt in bev_pts:
            res = self.bev_to_image(pt[0], pt[1])
            if res:
                img_pts_list.append(res)
                valid_bev.append(pt)
        
        if len(img_pts_list) < 4:
            return None
            
        img_pts = np.array(img_pts_list, dtype=np.float32)
        dst_bev = np.array(valid_bev, dtype=np.float32)
        
        # We want Image -> BEV, so src=Image, dst=BEV
        import cv2
        H, _ = cv2.findHomography(img_pts, dst_bev)
        
        if H is not None:
            return H.tolist()
        return None


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
