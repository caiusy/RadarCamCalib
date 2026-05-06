#!/usr/bin/env python3
"""
generate_mock_dataset.py - Generate simulated radar-camera sync dataset.

Creates a complete mock dataset with:
- Sync JSON files
- Simulated camera images (with road, lanes, vehicle boxes)
- Simulated radar JSON files (with trajectories, noise)
- Ground truth calibration JSON
- Ground truth matched points JSON

Usage:
    python tools/generate_mock_dataset.py --output mock_dataset --num_frames 100
"""

import argparse
import json
import sys
import os
from pathlib import Path
from datetime import datetime

import cv2
import numpy as np

# Add project root to path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from data.data_models import MatchedPoint


def generate_ground_truth_homography() -> np.ndarray:
    """Generate a realistic ground-truth homography (radar BEV → image).

    Uses a set of known point correspondences that mimic a typical
    roadside camera-radar setup.

    Returns:
        3x3 homography matrix.
    """
    # Define realistic correspondences:
    # (radar_x, radar_y) in meters → (image_u, image_v) in pixels
    # Road goes forward from sensor, camera captures a 1280x720 view
    radar_pts = np.array([
        [-5.0, 30.0],
        [5.0, 30.0],
        [-3.0, 60.0],
        [3.0, 60.0],
        [0.0, 45.0],
        [-7.0, 80.0],
        [7.0, 80.0],
        [0.0, 100.0],
    ], dtype=np.float64)

    image_pts = np.array([
        [280, 520],
        [1000, 520],
        [380, 400],
        [900, 400],
        [640, 450],
        [320, 340],
        [960, 340],
        [640, 300],
    ], dtype=np.float64)

    H, _ = cv2.findHomography(radar_pts, image_pts)
    return H


def draw_road_image(
    width: int = 1280,
    height: int = 720,
    num_lanes: int = 4,
) -> np.ndarray:
    """Draw a simple road scene background.

    Args:
        width, height: Image dimensions.
        num_lanes: Number of lanes to draw.

    Returns:
        BGR image array.
    """
    img = np.zeros((height, width, 3), dtype=np.uint8)

    # Sky gradient (top portion)
    for y in range(height // 3):
        ratio = y / (height // 3)
        b = int(180 - 80 * ratio)
        g = int(140 - 40 * ratio)
        r = int(80 - 20 * ratio)
        img[y, :] = (b, g, r)

    # Road surface (gray)
    road_top = height // 3
    road_color = (60, 60, 60)
    img[road_top:, :] = road_color

    # Vanishing point
    vp_x, vp_y = width // 2, road_top

    # Lane boundaries
    lane_width_bottom = width // (num_lanes + 1)
    lane_color = (200, 200, 200)

    for i in range(num_lanes + 1):
        # Bottom x position
        x_bottom = int((i + 0.5) * lane_width_bottom)
        # Top converges to vanishing point
        cv2.line(img, (vp_x, vp_y), (x_bottom, height), lane_color, 2, cv2.LINE_AA)

    # Dashed center line
    for y_start in range(road_top + 20, height, 30):
        y_end = min(y_start + 15, height)
        ratio_start = (y_start - road_top) / (height - road_top)
        ratio_end = (y_end - road_top) / (height - road_top)
        x_start = int(vp_x + 0 * ratio_start)
        x_end = int(vp_x + 0 * ratio_end)
        cv2.line(img, (x_start, y_start), (x_end, y_end), (0, 200, 200), 2)

    return img


def draw_vehicle_on_image(
    img: np.ndarray,
    u: float, v: float,
    vehicle_id: int,
    scale: float = 1.0,
) -> np.ndarray:
    """Draw a simple vehicle rectangle on the image.

    Args:
        img: Image to draw on (modified in-place).
        u, v: Center position in pixels.
        vehicle_id: ID to label.
        scale: Size scaling factor (smaller for distant vehicles).

    Returns:
        Modified image.
    """
    w = int(40 * scale)
    h = int(25 * scale)
    u, v = int(u), int(v)

    # Vehicle body
    color = (50, 50, 200)  # Dark red
    cv2.rectangle(img, (u - w // 2, v - h // 2), (u + w // 2, v + h // 2), color, -1)
    cv2.rectangle(img, (u - w // 2, v - h // 2), (u + w // 2, v + h // 2), (100, 100, 255), 2)

    # ID label
    cv2.putText(
        img, f"V{vehicle_id}",
        (u - 10, v - h // 2 - 5),
        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA,
    )

    return img


class TrajectorySimulator:
    """Simulate radar vehicle trajectories with realistic motion."""

    def __init__(self, num_trajectories: int = 7, num_frames: int = 100):
        self.num_frames = num_frames
        self.trajectories = []
        self._init_trajectories(num_trajectories)

    def _init_trajectories(self, n: int):
        """Initialize vehicle trajectories with random parameters."""
        np.random.seed(42)

        lane_x_positions = [-7.0, -3.5, 0.0, 3.5, 7.0]

        for i in range(n):
            lane_x = lane_x_positions[i % len(lane_x_positions)]
            start_y = np.random.uniform(20, 60)
            speed_y = np.random.uniform(8, 20)  # m/s forward
            start_frame = np.random.randint(0, max(1, self.num_frames // 3))
            duration = np.random.randint(
                self.num_frames // 3,
                self.num_frames,
            )

            self.trajectories.append({
                'id': i + 1,
                'lane_x': lane_x,
                'start_y': start_y,
                'speed_y': speed_y,
                'speed_x': np.random.uniform(-0.3, 0.3),
                'start_frame': start_frame,
                'duration': duration,
                'class': 'vehicle',
                'noise_x': np.random.uniform(0.05, 0.2),
                'noise_y': np.random.uniform(0.05, 0.2),
            })

    def get_frame_objects(self, frame_idx: int) -> list:
        """Get radar objects visible at a given frame.

        Args:
            frame_idx: Frame index.

        Returns:
            List of radar object dicts.
        """
        objects = []

        for traj in self.trajectories:
            rel_frame = frame_idx - traj['start_frame']
            if rel_frame < 0 or rel_frame >= traj['duration']:
                continue

            dt = rel_frame * 0.1  # 10 Hz
            x = traj['lane_x'] + traj['speed_x'] * dt + np.random.normal(0, traj['noise_x'])
            y = traj['start_y'] + traj['speed_y'] * dt + np.random.normal(0, traj['noise_y'])

            speed = np.sqrt(traj['speed_x']**2 + traj['speed_y']**2)

            objects.append({
                'id': traj['id'],
                'class': traj['class'],
                'coord': [round(x, 3), round(y, 3)],
                'props': {
                    'speed': round(speed, 2),
                    'speedX': round(traj['speed_x'], 3),
                    'speedY': round(traj['speed_y'], 3),
                    'accX': round(np.random.uniform(-0.1, 0.1), 3),
                    'accY': round(np.random.uniform(-0.2, 0.2), 3),
                    'vehicleLaneID': f"lane_{(traj['id'] - 1) % 4 + 1}",
                },
            })

        # Add noise points (short-lived, random positions)
        if np.random.random() < 0.3:
            noise_x = np.random.uniform(-15, 15)
            noise_y = np.random.uniform(10, 120)
            objects.append({
                'id': 100 + frame_idx % 10,
                'class': 'unknown',
                'coord': [round(noise_x, 3), round(noise_y, 3)],
                'props': {},
            })

        return objects


def generate_mock_dataset(
    output_dir: str,
    num_frames: int = 100,
    image_width: int = 1280,
    image_height: int = 720,
):
    """Generate a complete mock dataset.

    Args:
        output_dir: Output directory path.
        num_frames: Number of frames to generate.
        image_width, image_height: Image dimensions.
    """
    output = Path(output_dir)
    sync_dir = output / "sync"
    image_dir = output / "image"
    radar_dir = output / "radar"

    for d in [sync_dir, image_dir, radar_dir]:
        d.mkdir(parents=True, exist_ok=True)

    # Generate ground truth homography
    H_gt = generate_ground_truth_homography()
    H_gt_inv = np.linalg.inv(H_gt)

    # Save ground truth calibration
    gt_calib = {
        "version": "1.0",
        "calibration_type": "radar_to_camera_homography",
        "camera_sensor_id": 1,
        "radar_sensor_id": 2,
        "homography": H_gt.tolist(),
        "inverse_homography": H_gt_inv.tolist(),
        "matched_points": [],
        "mean_reprojection_error": 0.0,
        "max_reprojection_error": 0.0,
    }

    # Initialize trajectory simulator
    simulator = TrajectorySimulator(num_trajectories=7, num_frames=num_frames)
    base_timestamp = 1767954310202

    # Collect matched points for ground truth
    all_matched_points = []

    print(f"Generating {num_frames} frames in {output_dir}...")

    for frame_idx in range(num_frames):
        timestamp = base_timestamp + frame_idx * 100
        image_frame_idx = 266339 + frame_idx
        radar_frame_idx = 128723 + frame_idx

        # --- Radar JSON ---
        radar_objects = simulator.get_frame_objects(frame_idx)
        radar_filename = f"{radar_frame_idx:06d}-{timestamp}.radar.json"

        radar_json = {
            "baseId": 2,
            "filename": radar_filename,
            "objects": radar_objects,
        }

        with open(radar_dir / radar_filename, 'w') as f:
            json.dump(radar_json, f, indent=2)

        # --- Image ---
        road_img = draw_road_image(image_width, image_height)

        # Project radar objects to image and draw vehicles
        for obj in radar_objects:
            if obj['class'] == 'unknown':
                continue
            x, y = obj['coord']
            pt = np.array([x, y, 1.0])
            proj = H_gt @ pt
            if abs(proj[2]) > 1e-10:
                u = proj[0] / proj[2]
                v = proj[1] / proj[2]
                if 0 <= u < image_width and 0 <= v < image_height:
                    # Scale based on distance (closer = larger)
                    scale = max(0.3, min(1.5, 50.0 / max(y, 1)))
                    draw_vehicle_on_image(road_img, u, v, obj['id'], scale)

                    # Collect matched point for first few frames
                    if frame_idx < 5 and obj['id'] <= 4:
                        all_matched_points.append({
                            "radar_point": {"x": x, "y": y},
                            "image_point": {"u": round(u, 2), "v": round(v, 2)},
                        })

        image_filename = f"{image_frame_idx:06d}-{timestamp}.jpg"
        cv2.imwrite(str(image_dir / image_filename), road_img)

        # --- Sync JSON ---
        sync_data = {
            "sync_batch": frame_idx,
            "batch_size": 2,
            "sensors": [
                {
                    "frame_index": image_frame_idx,
                    "sensor_id": 1,
                    "time_stamp": timestamp,
                },
                {
                    "frame_index": radar_frame_idx,
                    "sensor_id": 2,
                    "time_stamp": timestamp,
                },
            ],
            "sensor_data": [
                {
                    "channel_id": 0,
                    "sensor_id": 1,
                    "data_type": "image",
                    "data_path": f"image/{image_filename}",
                },
                {
                    "channel_id": 1,
                    "sensor_id": 2,
                    "data_type": "radar",
                    "data_path": f"radar/{radar_filename}",
                },
            ],
        }

        sync_filename = f"{frame_idx:06d}.sync.json"
        with open(sync_dir / sync_filename, 'w') as f:
            json.dump(sync_data, f, indent=2)

    # Save ground truth calibration
    # Deduplicate matched points for ground truth
    unique_points = {}
    for mp in all_matched_points:
        key = (mp["radar_point"]["x"], mp["radar_point"]["y"])
        if key not in unique_points:
            unique_points[key] = mp
    gt_matched = list(unique_points.values())

    gt_calib["matched_points"] = gt_matched
    with open(output / "ground_truth_calibration.json", 'w') as f:
        json.dump(gt_calib, f, indent=2)

    # Save mock matched points
    with open(output / "mock_matched_points.json", 'w') as f:
        json.dump({"matched_points": gt_matched}, f, indent=2)

    print(f"✅ Generated {num_frames} frames:")
    print(f"   Sync:  {sync_dir}")
    print(f"   Image: {image_dir}")
    print(f"   Radar: {radar_dir}")
    print(f"   Ground truth calibration: {output / 'ground_truth_calibration.json'}")
    print(f"   Mock matched points ({len(gt_matched)}): {output / 'mock_matched_points.json'}")


def main():
    parser = argparse.ArgumentParser(
        description="Generate mock radar-camera sync dataset"
    )
    parser.add_argument(
        "--output", type=str, default="mock_dataset",
        help="Output directory (default: mock_dataset)",
    )
    parser.add_argument(
        "--num_frames", type=int, default=100,
        help="Number of frames to generate (default: 100)",
    )
    parser.add_argument(
        "--width", type=int, default=1280,
        help="Image width (default: 1280)",
    )
    parser.add_argument(
        "--height", type=int, default=720,
        help="Image height (default: 720)",
    )

    args = parser.parse_args()
    generate_mock_dataset(args.output, args.num_frames, args.width, args.height)


if __name__ == "__main__":
    main()
