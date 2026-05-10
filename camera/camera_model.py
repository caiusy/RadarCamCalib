"""
camera_model.py - Camera intrinsic/extrinsic model and projection utilities.

Provides a CameraModel dataclass for storing camera parameters,
and functions for pinhole projection and undistortion.
"""

from dataclasses import dataclass, field
from typing import Optional, Tuple
import numpy as np
import cv2


@dataclass
class CameraModel:
    """Camera intrinsic and extrinsic parameters.

    Attributes:
        camera_matrix: 3x3 intrinsic matrix K.
            [[fx, 0, cx],
             [0, fy, cy],
             [0,  0,  1]]
        dist_coeffs: Distortion coefficients [k1, k2, p1, p2, k3].
        image_size: (width, height) in pixels.
        radar_to_camera: 4x4 extrinsic transformation matrix from
            radar coordinate system to camera coordinate system.
            Optional — only needed for full 3D projection.
    """
    camera_matrix: np.ndarray
    dist_coeffs: np.ndarray
    image_size: Tuple[int, int]
    radar_to_camera: Optional[np.ndarray] = None

    def __post_init__(self):
        """Validate shapes after initialization."""
        self.camera_matrix = np.asarray(self.camera_matrix, dtype=np.float64)
        self.dist_coeffs = np.asarray(self.dist_coeffs, dtype=np.float64).ravel()

        if self.camera_matrix.shape != (3, 3):
            raise ValueError(
                f"camera_matrix must be 3x3, got {self.camera_matrix.shape}"
            )
        if len(self.dist_coeffs) < 4:
            # Pad with zeros if fewer than 5 coefficients
            self.dist_coeffs = np.pad(
                self.dist_coeffs, (0, 5 - len(self.dist_coeffs))
            )

        if self.radar_to_camera is not None:
            self.radar_to_camera = np.asarray(
                self.radar_to_camera, dtype=np.float64
            )
            if self.radar_to_camera.shape != (4, 4):
                raise ValueError(
                    f"radar_to_camera must be 4x4, got {self.radar_to_camera.shape}"
                )

    @property
    def fx(self) -> float:
        """Focal length in x direction (pixels)."""
        return float(self.camera_matrix[0, 0])

    @property
    def fy(self) -> float:
        """Focal length in y direction (pixels)."""
        return float(self.camera_matrix[1, 1])

    @property
    def cx(self) -> float:
        """Principal point x coordinate (pixels)."""
        return float(self.camera_matrix[0, 2])

    @property
    def cy(self) -> float:
        """Principal point y coordinate (pixels)."""
        return float(self.camera_matrix[1, 2])

    @property
    def width(self) -> int:
        """Image width in pixels."""
        return self.image_size[0]

    @property
    def height(self) -> int:
        """Image height in pixels."""
        return self.image_size[1]

    def to_dict(self) -> dict:
        """Serialize to dictionary for JSON output."""
        d = {
            "camera_matrix": self.camera_matrix.tolist(),
            "distortion_coefficients": self.dist_coeffs.tolist(),
            "image_size": {
                "width": self.image_size[0],
                "height": self.image_size[1],
            },
        }
        if self.radar_to_camera is not None:
            d["radar_to_camera"] = self.radar_to_camera.tolist()
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "CameraModel":
        """Deserialize from dictionary."""
        K = np.array(d["camera_matrix"], dtype=np.float64)
        D = np.array(d.get("distortion_coefficients", [0, 0, 0, 0, 0]),
                     dtype=np.float64)
        img_size_data = d.get("image_size", {})
        image_size = (
            img_size_data.get("width", 1280),
            img_size_data.get("height", 720),
        )
        r2c = None
        if "radar_to_camera" in d:
            r2c = np.array(d["radar_to_camera"], dtype=np.float64)

        return cls(
            camera_matrix=K,
            dist_coeffs=D,
            image_size=image_size,
            radar_to_camera=r2c,
        )


def get_default_camera_model(
    width: int = 1280,
    height: int = 720,
    fov_deg: float = 60.0,
) -> CameraModel:
    """Generate a reasonable default camera model for a given resolution.

    Uses a symmetric pinhole model with specified field of view.

    Args:
        width: Image width in pixels.
        height: Image height in pixels.
        fov_deg: Horizontal field of view in degrees.

    Returns:
        CameraModel with computed intrinsics and zero distortion.
    """
    fov_rad = np.deg2rad(fov_deg)
    fx = (width / 2.0) / np.tan(fov_rad / 2.0)
    fy = fx  # Square pixels
    cx = width / 2.0
    cy = height / 2.0

    K = np.array([
        [fx, 0.0, cx],
        [0.0, fy, cy],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)

    D = np.zeros(5, dtype=np.float64)

    # Default radar_to_camera extrinsic: identity + small translation
    # Radar is typically mounted on vehicle front bumper, camera above
    R2C = np.eye(4, dtype=np.float64)
    R2C[1, 3] = -1.5   # Camera is 1.5m above radar
    R2C[2, 3] = 0.3     # Camera is 0.3m behind radar

    return CameraModel(
        camera_matrix=K,
        dist_coeffs=D,
        image_size=(width, height),
        radar_to_camera=R2C,
    )


def project_points_pinhole(
    points_3d: np.ndarray,
    K: np.ndarray,
    D: np.ndarray,
    rvec: Optional[np.ndarray] = None,
    tvec: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Project 3D points to 2D pixel coordinates using pinhole camera model.

    Args:
        points_3d: Nx3 array of 3D points in camera frame.
        K: 3x3 camera intrinsic matrix.
        D: Distortion coefficients (at least 4 elements).
        rvec: Rotation vector (3,). If None, identity rotation assumed.
        tvec: Translation vector (3,). If None, zero translation assumed.

    Returns:
        Nx2 array of projected pixel [u, v] coordinates.
    """
    if len(points_3d) == 0:
        return np.empty((0, 2), dtype=np.float64)

    pts = np.asarray(points_3d, dtype=np.float64)
    if pts.ndim == 1:
        pts = pts.reshape(1, 3)

    if rvec is None:
        rvec = np.zeros(3, dtype=np.float64)
    if tvec is None:
        tvec = np.zeros(3, dtype=np.float64)

    rvec = np.asarray(rvec, dtype=np.float64).ravel()
    tvec = np.asarray(tvec, dtype=np.float64).ravel()
    D = np.asarray(D, dtype=np.float64).ravel()

    projected, _ = cv2.projectPoints(pts, rvec, tvec, K, D)
    return projected.reshape(-1, 2)


def undistort_points(
    points_2d: np.ndarray,
    K: np.ndarray,
    D: np.ndarray,
) -> np.ndarray:
    """Undistort 2D pixel coordinates.

    Removes lens distortion from pixel coordinates using the camera model.

    Args:
        points_2d: Nx2 array of distorted pixel coordinates [u, v].
        K: 3x3 camera intrinsic matrix.
        D: Distortion coefficients.

    Returns:
        Nx2 array of undistorted pixel coordinates.
    """
    if len(points_2d) == 0:
        return np.empty((0, 2), dtype=np.float64)

    pts = np.asarray(points_2d, dtype=np.float64).reshape(-1, 1, 2)
    D = np.asarray(D, dtype=np.float64).ravel()

    undistorted = cv2.undistortPoints(pts, K, D, P=K)
    return undistorted.reshape(-1, 2)
