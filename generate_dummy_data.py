"""
Generate dummy data for RadarCamCalib with corrected coordinate systems.

Coordinate System Definition:
- Y-axis: Forward direction (0-180m)
- X-axis: Lateral direction (-15 to 15m)
- Z-axis: Height (typically vehicle height)
"""

import numpy as np
import json
from pathlib import Path
from typing import Dict, List, Tuple
import cv2


class DummyDataGenerator:
    """Generate synthetic radar and camera data for calibration."""
    
    def __init__(self, output_dir: str = "./dummy_data"):
        """
        Initialize the dummy data generator.
        
        Args:
            output_dir: Output directory for generated data
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Coordinate system parameters
        self.y_range = (0, 180)  # Forward direction: 0-180m
        self.x_range = (-15, 15)  # Lateral direction: -15 to 15m
        self.z_range = (0, 3)    # Height: 0-3m
        
        # Camera parameters (example intrinsics)
        self.focal_length = 500
        self.image_width = 1280
        self.image_height = 720
        self.cx = self.image_width / 2
        self.cy = self.image_height / 2
        
    def _transform_world_to_camera(
        self, 
        point_world: np.ndarray,
        rotation_matrix: np.ndarray,
        translation_vector: np.ndarray
    ) -> np.ndarray:
        """
        Transform a point from world coordinates to camera coordinates.
        
        Args:
            point_world: Point in world coordinates [x, y, z]
            rotation_matrix: 3x3 rotation matrix (world to camera)
            translation_vector: 3x1 translation vector (world to camera)
            
        Returns:
            Point in camera coordinates [x_cam, y_cam, z_cam]
        """
        point_camera = rotation_matrix @ point_world + translation_vector
        return point_camera
    
    def _transform_camera_to_image(
        self,
        point_camera: np.ndarray
    ) -> Tuple[float, float]:
        """
        Transform a point from camera coordinates to image coordinates.
        
        Args:
            point_camera: Point in camera coordinates [x_cam, y_cam, z_cam]
            
        Returns:
            Tuple of (u, v) image coordinates
        """
        if point_camera[2] <= 0:
            return None  # Point is behind camera
        
        # Project using intrinsic matrix
        u = self.focal_length * point_camera[0] / point_camera[2] + self.cx
        v = self.focal_length * point_camera[1] / point_camera[2] + self.cy
        
        return (u, v)
    
    def _transform_world_to_image(
        self,
        point_world: np.ndarray,
        rotation_matrix: np.ndarray,
        translation_vector: np.ndarray
    ) -> Tuple[float, float]:
        """
        Transform a point from world coordinates directly to image coordinates.
        
        Args:
            point_world: Point in world coordinates [x, y, z]
            rotation_matrix: 3x3 rotation matrix (world to camera)
            translation_vector: 3x1 translation vector (world to camera)
            
        Returns:
            Tuple of (u, v) image coordinates, or None if out of view
        """
        # First transform to camera coordinates
        point_camera = self._transform_world_to_camera(
            point_world, rotation_matrix, translation_vector
        )
        
        # Then transform to image coordinates
        return self._transform_camera_to_image(point_camera)
    
    def generate_radar_detections(
        self, 
        num_detections: int = 50
    ) -> List[Dict]:
        """
        Generate synthetic radar detections in world coordinates.
        
        Coordinate system:
        - Y: Forward direction (0-180m)
        - X: Lateral direction (-15 to 15m)
        - Z: Height (0-3m)
        
        Args:
            num_detections: Number of detections to generate
            
        Returns:
            List of detection dictionaries with world coordinates
        """
        detections = []
        
        for i in range(num_detections):
            # Generate random position in world coordinates
            y = np.random.uniform(self.y_range[0], self.y_range[1])  # Forward
            x = np.random.uniform(self.x_range[0], self.x_range[1])  # Lateral
            z = np.random.uniform(self.z_range[0], self.z_range[1])  # Height
            
            detection = {
                "id": i,
                "position_world": {
                    "x": float(x),  # Lateral: -15 to 15m
                    "y": float(y),  # Forward: 0 to 180m
                    "z": float(z)   # Height: 0 to 3m
                },
                "velocity": {
                    "vx": float(np.random.uniform(-10, 10)),  # Lateral velocity
                    "vy": float(np.random.uniform(-5, 30)),   # Forward velocity
                    "vz": float(np.random.uniform(-1, 1))     # Vertical velocity
                },
                "rcs": float(np.random.uniform(0.1, 10.0))  # Radar cross section
            }
            detections.append(detection)
        
        return detections
    
    def generate_camera_detections(
        self,
        radar_detections: List[Dict],
        rotation_matrix: np.ndarray = None,
        translation_vector: np.ndarray = None
    ) -> List[Dict]:
        """
        Project radar detections to camera image coordinates.
        
        Args:
            radar_detections: List of radar detections in world coordinates
            rotation_matrix: Rotation from world to camera (default: identity)
            translation_vector: Translation from world to camera (default: zero)
            
        Returns:
            List of detections with projected image coordinates
        """
        if rotation_matrix is None:
            rotation_matrix = np.eye(3)
        if translation_vector is None:
            translation_vector = np.zeros((3, 1))
        
        camera_detections = []
        
        for detection in radar_detections:
            pos = detection["position_world"]
            point_world = np.array([[pos["x"]], [pos["y"]], [pos["z"]]])
            
            # Transform to image coordinates
            image_coords = self._transform_world_to_image(
                point_world, rotation_matrix, translation_vector
            )
            
            if image_coords is not None:
                u, v = image_coords
                
                # Check if point is within image bounds
                if 0 <= u < self.image_width and 0 <= v < self.image_height:
                    cam_detection = detection.copy()
                    cam_detection["position_image"] = {
                        "u": float(u),
                        "v": float(v)
                    }
                    camera_detections.append(cam_detection)
        
        return camera_detections
    
    def _generate_calibration_transform(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Generate a realistic calibration transformation (world to camera).
        
        Assumes camera is mounted above the vehicle, looking slightly down and forward.
        
        Returns:
            Tuple of (rotation_matrix, translation_vector)
        """
        # Camera is 1.5m forward, 0m lateral, 1.5m up from vehicle origin
        translation = np.array([[0.0], [1.5], [1.5]])
        
        # Camera looks slightly down and forward
        # Small pitch angle (camera looking down)
        pitch = np.radians(-15)
        rotation_pitch = np.array([
            [1, 0, 0],
            [0, np.cos(pitch), -np.sin(pitch)],
            [0, np.sin(pitch), np.cos(pitch)]
        ])
        
        # Small yaw angle (camera looking slightly right)
        yaw = np.radians(0)
        rotation_yaw = np.array([
            [np.cos(yaw), 0, np.sin(yaw)],
            [0, 1, 0],
            [-np.sin(yaw), 0, np.cos(yaw)]
        ])
        
        rotation = rotation_yaw @ rotation_pitch
        
        return rotation, translation
    
    def save_dummy_data(self, filename: str = "dummy_detections.json"):
        """
        Generate and save complete dummy dataset.
        
        Args:
            filename: Output JSON filename
        """
        # Generate radar detections
        radar_detections = self.generate_radar_detections(num_detections=50)
        
        # Generate calibration transform
        rotation, translation = self._generate_calibration_transform()
        
        # Project to camera
        camera_detections = self.generate_camera_detections(
            radar_detections,
            rotation_matrix=rotation,
            translation_vector=translation
        )
        
        # Prepare output data
        output_data = {
            "coordinate_system": {
                "description": "World coordinates with corrected axes",
                "x_axis": "Lateral direction: -15 to 15m",
                "y_axis": "Forward direction: 0 to 180m",
                "z_axis": "Height: 0 to 3m"
            },
            "calibration": {
                "rotation_matrix": rotation.tolist(),
                "translation_vector": translation.tolist(),
                "camera_intrinsics": {
                    "focal_length": self.focal_length,
                    "cx": self.cx,
                    "cy": self.cy,
                    "image_width": self.image_width,
                    "image_height": self.image_height
                }
            },
            "radar_detections": radar_detections,
            "camera_detections": camera_detections,
            "statistics": {
                "total_radar_detections": len(radar_detections),
                "visible_in_camera": len(camera_detections),
                "camera_coverage": f"{len(camera_detections) / len(radar_detections) * 100:.1f}%"
            }
        }
        
        # Save to JSON
        output_path = self.output_dir / filename
        with open(output_path, 'w') as f:
            json.dump(output_data, f, indent=2)
        
        print(f"Dummy data saved to {output_path}")
        print(f"Total radar detections: {len(radar_detections)}")
        print(f"Visible in camera: {len(camera_detections)}")
        print(f"Camera coverage: {len(camera_detections) / len(radar_detections) * 100:.1f}%")
        
        return output_path


def main():
    """Generate and save dummy calibration data."""
    generator = DummyDataGenerator(output_dir="./dummy_data")
    generator.save_dummy_data()


if __name__ == "__main__":
    main()
