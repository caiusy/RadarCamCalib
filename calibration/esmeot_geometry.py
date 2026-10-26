"""Geometry helpers for the ES-MEOT camera/radar config."""

from __future__ import annotations

import configparser
import json
from pathlib import Path

import cv2
import numpy as np


def homography_from_esmeot_config(config_path: str | Path,
                                  ground_height: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """Build the ground-plane (radar x,z) to image homography.

    The upstream convention is ``camera = R @ (radar + T)``.  On the radar
    ground plane ``y=ground_height``, the plane-to-image matrix is
    ``K [R[:,0], R[:,2], R @ ([0,ground_height,0] + T)]``.
    A plane through the optical center is degenerate and is rejected.
    """
    cfg = configparser.ConfigParser()
    cfg.read(config_path)
    K = np.asarray(json.loads(cfg["cameraIntrinsic"]["cameramatrix"]), dtype=float)
    rt = np.asarray(json.loads(cfg["radar2camera"]["rt"]), dtype=float)
    if rt.shape != (3, 4):
        raise ValueError(f"radar2camera.rt must be 3x4, got {rt.shape}")
    R, T = rt[:, :3], rt[:, 3]
    if not np.isfinite(ground_height):
        raise ValueError("ground_height must be finite")
    H = K @ np.column_stack((R[:, 0], R[:, 2], R @ (T + [0., ground_height, 0.])))
    if np.linalg.matrix_rank(H) < 3:
        raise ValueError("ground plane passes through camera center; measured nonzero ground_height required")
    H /= H[2, 2] if abs(H[2, 2]) > 1e-12 else np.linalg.norm(H)
    return H, np.linalg.inv(H)


def project_radar_ground_points(points_xz: np.ndarray, H: np.ndarray) -> np.ndarray:
    points = np.asarray(points_xz, dtype=float).reshape(-1, 2)
    if len(points) == 0:
        return np.empty((0, 2), dtype=float)
    return cv2.perspectiveTransform(points[None].astype(np.float64), H)[0]
