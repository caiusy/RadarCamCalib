#!/usr/bin/env python3
"""Fit ES-MEOT radar-plane to image homography from paired detections.

The ES-MEOT annotation file contains camera track ids and their radar-plane
bottom points.  This tool joins those ids to the trusted ByteTrack boxes,
fits RANSAC on the first part of the sequence, and reports a temporal holdout
error.  It is an operational calibration fit, not an independently measured
hardware extrinsic calibration.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def collect_pairs(annotation_path: Path, camera_tracks_path: Path):
    annotations = np.load(annotation_path, allow_pickle=True)
    camera_tracks = np.load(camera_tracks_path, allow_pickle=True)
    radar, image, frames = [], [], []
    for frame_index, (ann, detections) in enumerate(zip(annotations, camera_tracks)):
        ids = {int(value): index for index, value in enumerate(np.asarray(detections.get("ids", [])).reshape(-1))}
        points = np.asarray(ann.get("pos_bottom_mid", []), dtype=float)
        for point_index, camera_id in enumerate(np.asarray(ann.get("ids", [])).reshape(-1)):
            camera_index = ids.get(int(camera_id))
            if camera_index is None or point_index >= len(points):
                continue
            y1, x1, y2, x2 = (float(value) for value in np.asarray(detections["boxes"])[camera_index])
            radar.append([float(points[point_index, 0]), float(points[point_index, 2])])
            image.append([(x1 + x2) / 2.0, y2])
            frames.append(frame_index)
    if len(radar) < 4:
        raise ValueError(f"at least four paired points are required, got {len(radar)}")
    return np.asarray(radar), np.asarray(image), np.asarray(frames)


def fit_esmeot_pairs(annotation_path: Path, camera_tracks_path: Path, holdout_fraction: float = 0.5,
                     ransac_threshold: float = 12.0) -> dict:
    radar, image, frames = collect_pairs(annotation_path, camera_tracks_path)
    split_frame = int(np.quantile(frames, 1.0 - holdout_fraction))
    train = frames <= split_frame
    if train.sum() < 4 or (~train).sum() == 0:
        raise ValueError("not enough points in train/holdout split")
    H, mask = cv2.findHomography(radar[train].reshape(-1, 1, 2), image[train].reshape(-1, 1, 2),
                                 cv2.RANSAC, ransac_threshold)
    if H is None or mask is None:
        raise ValueError("homography estimation failed")
    projected = cv2.perspectiveTransform(radar.reshape(1, -1, 2).astype(np.float32), H)[0]
    errors = np.linalg.norm(projected - image, axis=1)

    def stats(values: np.ndarray) -> dict:
        return {"count": int(len(values)), "median_px": float(np.median(values)),
                "mean_px": float(np.mean(values)), "p95_px": float(np.percentile(values, 95)),
                "max_px": float(np.max(values))}

    return {
        "version": "2.0",
        "calibration_type": "esmeot_pair_fit_homography",
        "homography": H.tolist(),
        "inverse_homography": np.linalg.inv(H).tolist(),
        "coordinate_system": {"radar_ground_plane": ["x", "z"], "radar_vertical": "y"},
        "pixel_space": "raw_detection_pixels",
        "fit": {"source": "annotation.pos_bottom_mid joined by ByteTrack ids",
                "total_pairs": int(len(radar)), "train_frame_end": split_frame,
                "train_inliers": int(mask.astype(bool).sum()),
                "ransac_threshold_px": ransac_threshold,
                "train_all_point_error": stats(errors[train]),
                "holdout_error": stats(errors[~train])},
        "validation_status": "pair-fit; holdout reported; annotation is derived upstream and is not independent hardware ground truth",
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset_root", type=Path)
    parser.add_argument("--annotation", type=Path, default=None)
    parser.add_argument("--camera-tracks", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--holdout-fraction", type=float, default=0.5)
    parser.add_argument("--ransac-threshold", type=float, default=12.0)
    args = parser.parse_args()
    annotation = args.annotation or args.dataset_root / "annotation.npy"
    payload = fit_esmeot_pairs(annotation, args.camera_tracks, args.holdout_fraction, args.ransac_threshold)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(json.dumps(payload, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
