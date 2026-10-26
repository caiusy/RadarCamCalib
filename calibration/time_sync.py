"""Frame-clock fitting for datasets that lack camera timestamps."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class TimeSyncFit:
    frame_count: int
    radar_period_seconds: float
    radar_fps_estimated: float
    camera_fps_metadata: float | None
    clock_scale: float
    clock_offset: float
    rmse_seconds: float
    max_abs_error_seconds: float
    valid: bool

    def to_dict(self) -> dict:
        return asdict(self)

    def write_json(self, path: str | Path) -> None:
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(self.to_dict(), indent=2), encoding="utf-8")


def fit_time_sync(radar_timestamps, camera_fps: float | None = None,
                  tolerance_seconds: float | None = None) -> TimeSyncFit:
    times = np.asarray(radar_timestamps, dtype=float).reshape(-1)
    if len(times) < 2 or not np.all(np.isfinite(times)):
        raise ValueError("at least two finite timestamps are required")
    deltas = np.diff(times)
    if np.any(deltas <= 0):
        raise ValueError("timestamps must be strictly increasing")
    median_period = float(np.median(deltas))
    radar_fps = 1.0 / median_period
    if camera_fps is not None and (not np.isfinite(camera_fps) or camera_fps <= 0):
        raise ValueError("camera_fps must be positive")
    fps = float(camera_fps) if camera_fps is not None else radar_fps
    camera_clock = np.arange(len(times), dtype=float) / fps
    scale, offset = np.polyfit(camera_clock, times, 1)
    residuals = times - (offset + scale * camera_clock)
    rmse = float(np.sqrt(np.mean(residuals**2)))
    max_abs = float(np.max(np.abs(residuals)))
    valid = rmse <= (tolerance_seconds or max(0.5 * median_period, 1e-3))
    return TimeSyncFit(
        frame_count=len(times),
        radar_period_seconds=float(scale / fps),
        radar_fps_estimated=radar_fps,
        camera_fps_metadata=float(camera_fps) if camera_fps is not None else None,
        clock_scale=float(scale),
        clock_offset=float(offset),
        rmse_seconds=rmse,
        max_abs_error_seconds=max_abs,
        valid=bool(valid),
    )
