"""
Calibration module for Radar-Camera fusion system.

Coordinate System Definition:
- Y-axis: Forward direction (0-180m)
- X-axis: Lateral direction (-15 to 15m)
- Z-axis: Vertical direction
"""

import numpy as np
import cv2
from typing import Tuple, Optional


class CoordinateTransformer:
    """Handles coordinate system transformations between radar, camera, and world frames."""
    
    def __init__(self, 
                 focal_length: float,
                 principal_point: Tuple[float, float],
                 camera_to_body_rotation: np.ndarray,
                 camera_to_body_translation: np.ndarray):
        """
        Initialize the coordinate transformer.
        
        Args:
            focal_length: Camera focal length in pixels
            principal_point: Camera principal point (cx, cy)
            camera_to_body_rotation: 3x3 rotation matrix from camera to body frame
            camera_to_body_translation: 3x1 translation vector from camera to body frame
        """
        self.focal_length = focal_length
        self.principal_point = principal_point
        self.R_camera_to_body = camera_to_body_rotation
        self.t_camera_to_body = camera_to_body_translation
        
        # Precompute camera intrinsic matrix
        self.K = np.array([
            [focal_length, 0, principal_point[0]],
            [0, focal_length, principal_point[1]],
            [0, 0, 1]
        ], dtype=np.float32)
    
    def camera_to_world_point(self, 
                             image_point: np.ndarray,
                             distance: float) -> np.ndarray:
        """
        Convert image point to world coordinates given distance from camera.
        
        Args:
            image_point: 2D point in image [u, v]
            distance: Distance from camera to point in body frame
            
        Returns:
            World coordinates [X, Y, Z] where:
            - Y: Forward direction (0-180m)
            - X: Lateral direction (-15 to 15m)
            - Z: Vertical direction
        """
        # Normalize image coordinates
        u, v = image_point
        x_norm = (u - self.principal_point[0]) / self.focal_length
        y_norm = (v - self.principal_point[1]) / self.focal_length
        
        # Create normalized 3D ray in camera frame
        ray_camera = np.array([x_norm, y_norm, 1.0], dtype=np.float32)
        ray_camera = ray_camera / np.linalg.norm(ray_camera)
        
        # Scale by distance to get 3D point in camera frame
        point_camera = ray_camera * distance
        
        # Transform from camera frame to body/world frame
        point_world = self.R_camera_to_body @ point_camera + self.t_camera_to_body.flatten()
        
        return point_world
    
    def world_to_image_point(self, 
                            world_point: np.ndarray) -> Tuple[np.ndarray, float]:
        """
        Project world point to image coordinates.
        
        Args:
            world_point: 3D world point [X, Y, Z] where:
            - Y: Forward direction (0-180m)
            - X: Lateral direction (-15 to 15m)
            - Z: Vertical direction
            
        Returns:
            Tuple of (image_point [u, v], distance)
        """
        # Transform from world frame to camera frame
        point_camera = self.R_camera_to_body.T @ (world_point - self.t_camera_to_body.flatten())
        
        # Calculate distance
        distance = np.linalg.norm(point_camera)
        
        # Project to image using intrinsic matrix
        point_image_homogeneous = self.K @ point_camera
        image_point = point_image_homogeneous[:2] / point_image_homogeneous[2]
        
        return image_point, distance
    
    def radar_to_world(self, 
                      radar_x: float,
                      radar_y: float,
                      radar_z: float = 0.0) -> np.ndarray:
        """
        Convert radar coordinates to world coordinates.
        
        Radar typically provides measurements in the body-aligned frame.
        X: Lateral (-15 to 15m)
        Y: Forward (0-180m)
        Z: Vertical
        
        Args:
            radar_x: Lateral position (-15 to 15m)
            radar_y: Forward position (0-180m)
            radar_z: Vertical position
            
        Returns:
            World coordinates [X, Y, Z]
        """
        # Radar is already in body-aligned coordinates which match world coordinates
        radar_point = np.array([radar_x, radar_y, radar_z], dtype=np.float32)
        return radar_point
    
    def world_to_radar(self, 
                      world_point: np.ndarray) -> Tuple[float, float, float]:
        """
        Convert world coordinates to radar coordinates.
        
        Args:
            world_point: 3D world point [X, Y, Z]
            
        Returns:
            Tuple of (radar_x, radar_y, radar_z)
        """
        # World coordinates match radar body-aligned frame
        radar_x = world_point[0]  # Lateral
        radar_y = world_point[1]  # Forward
        radar_z = world_point[2]  # Vertical
        
        return radar_x, radar_y, radar_z


class CalibrationValidator:
    """Validates calibration quality and coordinate system transformations."""
    
    def __init__(self, transformer: CoordinateTransformer):
        """
        Initialize validator.
        
        Args:
            transformer: CoordinateTransformer instance
        """
        self.transformer = transformer
    
    def validate_point_cloud(self, 
                            radar_points: np.ndarray,
                            image_points: np.ndarray,
                            distances: np.ndarray) -> dict:
        """
        Validate alignment between radar points and image projections.
        
        Args:
            radar_points: Nx3 array of radar points [X, Y, Z]
            image_points: Nx2 array of image points [u, v]
            distances: N array of distances from camera
            
        Returns:
            Dictionary with validation metrics
        """
        errors = []
        
        for i, (radar_pt, img_pt, dist) in enumerate(zip(radar_points, image_points, distances)):
            # Project radar point to image
            projected, _ = self.transformer.world_to_image_point(radar_pt)
            
            # Calculate reprojection error
            error = np.linalg.norm(projected - img_pt)
            errors.append(error)
        
        errors = np.array(errors)
        
        return {
            'mean_reprojection_error': float(np.mean(errors)),
            'std_reprojection_error': float(np.std(errors)),
            'max_reprojection_error': float(np.max(errors)),
            'min_reprojection_error': float(np.min(errors))
        }
    
    def validate_coordinate_ranges(self, points: np.ndarray) -> dict:
        """
        Validate that points fall within expected coordinate ranges.
        
        Args:
            points: Nx3 array of world points [X, Y, Z]
            
        Returns:
            Dictionary with range validation
        """
        x_values = points[:, 0]
        y_values = points[:, 1]
        
        return {
            'x_range': (float(np.min(x_values)), float(np.max(x_values))),
            'y_range': (float(np.min(y_values)), float(np.max(y_values))),
            'x_valid': float(np.min(x_values)) >= -15 and float(np.max(x_values)) <= 15,
            'y_valid': float(np.min(y_values)) >= 0 and float(np.max(y_values)) <= 180
        }


class CalibrationManager:
    """High-level manager for calibration operations."""
    
    def __init__(self):
        """Initialize calibration manager."""
        self.transformer: Optional[CoordinateTransformer] = None
        self.validator: Optional[CalibrationValidator] = None
    
    def load_calibration(self,
                        focal_length: float,
                        principal_point: Tuple[float, float],
                        rotation_matrix: np.ndarray,
                        translation_vector: np.ndarray) -> None:
        """
        Load calibration parameters.
        
        Args:
            focal_length: Camera focal length in pixels
            principal_point: Camera principal point (cx, cy)
            rotation_matrix: 3x3 camera to body rotation matrix
            translation_vector: 3x1 camera to body translation vector
        """
        self.transformer = CoordinateTransformer(
            focal_length=focal_length,
            principal_point=principal_point,
            camera_to_body_rotation=rotation_matrix,
            camera_to_body_translation=translation_vector
        )
        self.validator = CalibrationValidator(self.transformer)
    
    def project_radar_to_image(self, 
                               radar_points: np.ndarray) -> np.ndarray:
        """
        Project radar points to image coordinates.
        
        Args:
            radar_points: Nx3 array of radar detections [X, Y, Z]
            
        Returns:
            Nx2 array of projected image points [u, v]
        """
        if self.transformer is None:
            raise RuntimeError("Calibration not loaded. Call load_calibration first.")
        
        image_points = []
        for radar_pt in radar_points:
            img_pt, _ = self.transformer.world_to_image_point(radar_pt)
            image_points.append(img_pt)
        
        return np.array(image_points, dtype=np.float32)
    
    def triangulate_radar_camera(self,
                                image_point: np.ndarray,
                                radar_distance: float) -> np.ndarray:
        """
        Triangulate world position using radar distance and camera image point.
        
        Args:
            image_point: 2D image point [u, v]
            radar_distance: Distance measurement from radar (forward Y direction)
            
        Returns:
            3D world point [X, Y, Z]
        """
        if self.transformer is None:
            raise RuntimeError("Calibration not loaded. Call load_calibration first.")
        
        # Use radar distance as the Z-distance and camera for lateral position
        world_point = self.transformer.camera_to_world_point(image_point, radar_distance)
        return world_point
    
    def get_calibration_report(self,
                              radar_points: np.ndarray,
                              image_points: np.ndarray,
                              distances: np.ndarray) -> str:
        """
        Generate a calibration quality report.
        
        Args:
            radar_points: Nx3 array of radar points [X, Y, Z]
            image_points: Nx2 array of image points [u, v]
            distances: N array of distances
            
        Returns:
            Formatted calibration report string
        """
        if self.validator is None:
            raise RuntimeError("Calibration not loaded. Call load_calibration first.")
        
        metrics = self.validator.validate_point_cloud(radar_points, image_points, distances)
        ranges = self.validator.validate_coordinate_ranges(radar_points)
        
        report = """
=== Radar-Camera Calibration Report ===

Coordinate System:
  X-axis: Lateral (-15 to 15m)
  Y-axis: Forward (0-180m)
  Z-axis: Vertical

Reprojection Error Metrics:
  Mean Error: {:.2f} pixels
  Std Dev:    {:.2f} pixels
  Max Error:  {:.2f} pixels
  Min Error:  {:.2f} pixels

Coordinate Ranges:
  X Range: [{:.2f}, {:.2f}] m (Valid: {})
  Y Range: [{:.2f}, {:.2f}] m (Valid: {})

Calibration Status: {}
""".format(
            metrics['mean_reprojection_error'],
            metrics['std_reprojection_error'],
            metrics['max_reprojection_error'],
            metrics['min_reprojection_error'],
            ranges['x_range'][0],
            ranges['x_range'][1],
            ranges['x_valid'],
            ranges['y_range'][0],
            ranges['y_range'][1],
            ranges['y_valid'],
            'VALID' if ranges['x_valid'] and ranges['y_valid'] else 'INVALID'
        )
        
        return report
