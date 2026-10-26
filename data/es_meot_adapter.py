"""Trusted ES-MEOT object-array reader for calibration experiments."""

from __future__ import annotations

import configparser
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator

import numpy as np


@dataclass(frozen=True)
class RadarXYZ:
    frame: int
    timestamp: float
    radar_id: int | str
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float

    @property
    def ground_point(self) -> tuple[float, float]:
        """Return the actual ES-MEOT ground-plane axes: lateral x, forward z."""
        return self.x, self.z


class ESMeotDataset:
    """Load only user-confirmed ES-MEOT files; never downloads data."""

    def __init__(self, root: str | Path):
        self.root = Path(root)
        self.radar_path = self.root / "radarTrack.npy"
        self.config_path = self.root / "config"

    def _array(self) -> np.ndarray:
        if not self.radar_path.exists():
            raise FileNotFoundError(self.radar_path)
        return np.load(self.radar_path, allow_pickle=True)

    def timestamps(self) -> np.ndarray:
        return np.asarray([float(frame["time"]) for frame in self._array()], dtype=float)

    def iter_tracks(self) -> Iterator[list[RadarXYZ]]:
        for frame_index, frame in enumerate(self._array()):
            timestamp = float(frame["time"])
            yield [
                RadarXYZ(
                    frame=frame_index,
                    timestamp=timestamp,
                    radar_id=item["id"],
                    x=float(item["x"]),
                    y=float(item["y"]),
                    z=float(item["z"]),
                    vx=float(item.get("vx", 0.0)),
                    vy=float(item.get("vy", 0.0)),
                    vz=float(item.get("vz", 0.0)),
                )
                for item in frame.get("data", [])
            ]

    def camera_parameters(self) -> dict:
        cfg = configparser.ConfigParser()
        if not self.config_path.exists():
            raise FileNotFoundError(self.config_path)
        cfg.read(self.config_path)
        K = json.loads(cfg["cameraIntrinsic"]["cameramatrix"])
        D = json.loads(cfg["cameraIntrinsic"]["distcoeffs"])
        rt = np.asarray(json.loads(cfg["radar2camera"]["rt"]), dtype=float)
        if rt.shape != (3, 4):
            raise ValueError(f"radar2camera.rt must be 3x4, got {rt.shape}")
        return {
            "camera_matrix": K,
            "distortion_coefficients": D,
            "radar_to_camera_rotation": rt[:, :3].tolist(),
            "radar_to_camera_translation": rt[:, 3].tolist(),
            "coordinate_convention": {
                "radar_lateral": "x",
                "radar_vertical": "y",
                "radar_forward": "z",
                "ground_plane": ["x", "z"],
            },
        }

    def project_radar_to_image(self, points_xyz: np.ndarray) -> np.ndarray:
        """Project ES-MEOT radar XYZ points using the dataset K/D/R/T."""
        params = self.camera_parameters()
        points = np.asarray(points_xyz, dtype=float).reshape(-1, 3)
        R = np.asarray(params["radar_to_camera_rotation"], dtype=float)
        T = np.asarray(params["radar_to_camera_translation"], dtype=float)
        # The upstream Space2Plane config stores a translation BEFORE rotation.
        camera_points = (points + T) @ R.T
        positive = camera_points[:, 2] > 1e-9
        result = np.full((len(points), 2), np.nan, dtype=float)
        K = np.asarray(params["camera_matrix"], dtype=float)
        D = np.asarray(params["distortion_coefficients"], dtype=float)
        if np.any(positive):
            import cv2
            projected, _ = cv2.projectPoints(
                camera_points[positive], np.zeros(3), np.zeros(3), K, D
            )
            result[positive] = projected.reshape(-1, 2)
        return result
