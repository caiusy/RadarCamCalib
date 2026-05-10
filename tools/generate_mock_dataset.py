#!/usr/bin/env python3
"""
generate_mock_dataset.py - Generate simulated radar-camera-vision sync dataset.

Creates a complete mock dataset with:
- Sync JSON files (image + radar + vision_detection)
- Simulated camera images (with road, lanes, vehicle boxes)
- Simulated radar JSON files (with trajectories, noise, false alarms)
- Simulated vision detection JSON files (with bboxes, noise, misses)
- Association ground truth JSON files
- Ground truth calibration JSON (with camera K, D)
- Ground truth matched points JSON
- Dataset configuration JSON

Noise model:
- Distance-dependent radar detection: 0-60m:0.95, 60-100m:0.85, 100-150m:0.65, 150m+:0.40
- Distance-dependent vision detection: 0-80m:0.95, 80-150m:0.85, 150m+:0.65
- Radar position noise scales with distance
- Vision bbox jitter and size perturbation
- Radar false alarms (~5%)
- Vision false detections (~3%)
- Short-lived trajectories
- Blind zone region
- Far-range detection degradation

Usage:
    python tools/generate_mock_dataset.py --output mock_dataset --num_frames 200
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


# ============================================================================
# Detection rate models
# ============================================================================

def radar_detection_rate(distance_y: float) -> float:
    """Distance-dependent radar detection probability."""
    if distance_y < 60:
        return 0.95
    elif distance_y < 100:
        return 0.85
    elif distance_y < 150:
        return 0.65
    else:
        return 0.40


def vision_detection_rate(distance_y: float) -> float:
    """Distance-dependent vision detection probability."""
    if distance_y < 80:
        return 0.95
    elif distance_y < 150:
        return 0.85
    else:
        return 0.65


def radar_noise_scale(distance_y: float) -> float:
    """Distance-dependent radar noise scaling factor."""
    return 0.1 + 0.005 * distance_y


# ============================================================================
# Ground truth generation
# ============================================================================

def generate_ground_truth_homography() -> np.ndarray:
    """Generate a realistic ground-truth homography (radar BEV → image).

    Uses a set of known point correspondences that mimic a typical
    roadside camera-radar setup.

    Returns:
        3x3 homography matrix.
    """
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


def generate_camera_params(width: int = 1280, height: int = 720) -> dict:
    """Generate realistic camera intrinsic parameters.

    Returns:
        Dictionary with camera_matrix, distortion_coefficients, image_size.
    """
    fov_deg = 60.0
    fov_rad = np.deg2rad(fov_deg)
    fx = (width / 2.0) / np.tan(fov_rad / 2.0)
    fy = fx
    cx = width / 2.0
    cy = height / 2.0

    K = [
        [fx, 0.0, cx],
        [0.0, fy, cy],
        [0.0, 0.0, 1.0],
    ]

    # Slight distortion
    D = [-0.08, 0.01, 0.0, 0.0, 0.0]

    # Radar to camera extrinsic (identity-ish)
    R2C = np.eye(4).tolist()
    R2C[1][3] = -1.5
    R2C[2][3] = 0.3

    return {
        "camera_matrix": K,
        "distortion_coefficients": D,
        "image_size": {"width": width, "height": height},
        "radar_to_camera": R2C,
    }


# ============================================================================
# Image drawing
# ============================================================================

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
        x_bottom = int((i + 0.5) * lane_width_bottom)
        cv2.line(img, (vp_x, vp_y), (x_bottom, height), lane_color, 2, cv2.LINE_AA)

    # Dashed center line
    for y_start in range(road_top + 20, height, 30):
        y_end = min(y_start + 15, height)
        cv2.line(img, (vp_x, y_start), (vp_x, y_end), (0, 200, 200), 2)

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


# ============================================================================
# Trajectory simulation
# ============================================================================

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
            speed_y = np.random.uniform(8, 20)
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

        # Add short-lived trajectories (< 5 frames)
        for i in range(3):
            tid = n + i + 1
            lane_x = np.random.choice(lane_x_positions)
            self.trajectories.append({
                'id': tid,
                'lane_x': lane_x,
                'start_y': np.random.uniform(30, 80),
                'speed_y': np.random.uniform(10, 15),
                'speed_x': 0.0,
                'start_frame': np.random.randint(0, max(1, self.num_frames - 4)),
                'duration': np.random.randint(2, 5),
                'class': 'vehicle',
                'noise_x': 0.3,
                'noise_y': 0.3,
            })

        # Add far-distance trajectories (degraded detection)
        for i in range(2):
            tid = n + 3 + i + 1
            self.trajectories.append({
                'id': tid,
                'lane_x': np.random.uniform(-5, 5),
                'start_y': np.random.uniform(120, 180),
                'speed_y': np.random.uniform(5, 12),
                'speed_x': np.random.uniform(-0.2, 0.2),
                'start_frame': np.random.randint(0, max(1, self.num_frames // 4)),
                'duration': np.random.randint(self.num_frames // 3, self.num_frames),
                'class': 'vehicle',
                'noise_x': 0.5,
                'noise_y': 0.5,
            })

    def get_ground_truth_objects(self, frame_idx: int) -> list:
        """Get all ground truth object positions at a given frame.

        Returns list of dicts with true positions (no detection model applied).
        """
        objects = []
        for traj in self.trajectories:
            rel_frame = frame_idx - traj['start_frame']
            if rel_frame < 0 or rel_frame >= traj['duration']:
                continue

            dt = rel_frame * 0.1  # 10 Hz
            x = traj['lane_x'] + traj['speed_x'] * dt
            y = traj['start_y'] + traj['speed_y'] * dt

            objects.append({
                'id': traj['id'],
                'class': traj['class'],
                'x': x,
                'y': y,
                'speed_x': traj['speed_x'],
                'speed_y': traj['speed_y'],
            })
        return objects

    def get_radar_objects(self, frame_idx: int, rng: np.random.RandomState) -> list:
        """Get radar-detected objects with noise, misses, and false alarms."""
        gt_objects = self.get_ground_truth_objects(frame_idx)
        radar_objects = []

        # Blind zone: x ∈ [-15, -10], y ∈ [20, 50]
        blind_x_range = (-15, -10)
        blind_y_range = (20, 50)

        for obj in gt_objects:
            x, y = obj['x'], obj['y']

            # Check blind zone — much lower detection rate
            in_blind = (blind_x_range[0] <= x <= blind_x_range[1] and
                        blind_y_range[0] <= y <= blind_y_range[1])

            det_rate = 0.1 if in_blind else radar_detection_rate(y)

            if rng.random() > det_rate:
                continue  # Missed detection

            # Add noise scaled by distance
            noise = radar_noise_scale(y)
            nx = x + rng.normal(0, noise)
            ny = y + rng.normal(0, noise)

            speed = np.sqrt(obj['speed_x']**2 + obj['speed_y']**2)

            radar_objects.append({
                'id': obj['id'],
                'class': obj['class'],
                'coord': [round(nx, 3), round(ny, 3)],
                'props': {
                    'speed': round(speed, 2),
                    'speedX': round(obj['speed_x'], 3),
                    'speedY': round(obj['speed_y'], 3),
                    'accX': round(rng.uniform(-0.1, 0.1), 3),
                    'accY': round(rng.uniform(-0.2, 0.2), 3),
                    'vehicleLaneID': f"lane_{(obj['id'] - 1) % 4 + 1}",
                },
                '_gt_id': obj['id'],
            })

        # False alarms (~5% chance per frame, 1-2 false targets)
        if rng.random() < 0.05:
            n_false = rng.randint(1, 3)
            for _ in range(n_false):
                fx = rng.uniform(-15, 15)
                fy = rng.uniform(10, 150)
                radar_objects.append({
                    'id': 900 + rng.randint(0, 99),
                    'class': 'unknown',
                    'coord': [round(fx, 3), round(fy, 3)],
                    'props': {},
                    '_gt_id': -1,  # No ground truth match
                })

        return radar_objects

    def get_vision_objects(
        self, frame_idx: int, H: np.ndarray,
        img_w: int, img_h: int, rng: np.random.RandomState,
    ) -> list:
        """Get vision-detected bboxes with noise, misses, and false detections."""
        gt_objects = self.get_ground_truth_objects(frame_idx)
        vision_objects = []
        vision_id_counter = 100

        for obj in gt_objects:
            x, y = obj['x'], obj['y']

            det_rate = vision_detection_rate(y)
            if rng.random() > det_rate:
                continue  # Missed detection

            # Project to image
            pt = np.array([x, y, 1.0])
            proj = H @ pt
            if abs(proj[2]) < 1e-10:
                continue
            u = proj[0] / proj[2]
            v = proj[1] / proj[2]

            if u < 0 or u >= img_w or v < 0 or v >= img_h:
                continue

            # Generate bbox with noise
            scale = max(0.3, min(1.5, 50.0 / max(y, 1)))
            half_w = 40 * scale + rng.uniform(-5, 5)
            half_h = 25 * scale + rng.uniform(-3, 3)

            # Bbox jitter
            u_jitter = u + rng.uniform(-3, 3)
            v_jitter = v + rng.uniform(-3, 3)

            x1 = max(0, u_jitter - half_w)
            y1 = max(0, v_jitter - half_h)
            x2 = min(img_w, u_jitter + half_w)
            y2 = min(img_h, v_jitter + half_h)

            if (x2 - x1) < 5 or (y2 - y1) < 5:
                continue

            vision_id_counter += 1
            vision_objects.append({
                'id': vision_id_counter,
                'track_id': obj['id'] + 20,
                'class': obj['class'],
                'bbox': [round(x1, 1), round(y1, 1), round(x2, 1), round(y2, 1)],
                'score': round(min(0.99, max(0.5, 0.95 - 0.002 * y + rng.uniform(-0.05, 0.05))), 2),
                '_gt_id': obj['id'],
            })

        # False detections (~3% chance)
        if rng.random() < 0.03:
            fu = rng.uniform(100, img_w - 100)
            fv = rng.uniform(img_h * 0.4, img_h * 0.9)
            fw = rng.uniform(20, 60)
            fh = rng.uniform(15, 40)
            vision_id_counter += 1
            vision_objects.append({
                'id': vision_id_counter,
                'track_id': 999,
                'class': 'vehicle',
                'bbox': [round(fu, 1), round(fv, 1), round(fu + fw, 1), round(fv + fh, 1)],
                'score': round(rng.uniform(0.3, 0.6), 2),
                '_gt_id': -1,
            })

        return vision_objects


# ============================================================================
# Main generation
# ============================================================================

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
    vision_dir = output / "vision"
    assoc_gt_dir = output / "association_gt"

    for d in [sync_dir, image_dir, radar_dir, vision_dir, assoc_gt_dir]:
        d.mkdir(parents=True, exist_ok=True)

    # Generate ground truth
    H_gt = generate_ground_truth_homography()
    H_gt_inv = np.linalg.inv(H_gt)
    camera_params = generate_camera_params(image_width, image_height)

    # Initialize trajectory simulator
    simulator = TrajectorySimulator(num_trajectories=7, num_frames=num_frames)
    base_timestamp = 1767954310202
    rng = np.random.RandomState(seed=123)

    # Collect matched points for ground truth
    all_matched_points = []

    print(f"Generating {num_frames} frames in {output_dir}...")

    for frame_idx in range(num_frames):
        timestamp = base_timestamp + frame_idx * 100
        image_frame_idx = 266339 + frame_idx
        radar_frame_idx = 128723 + frame_idx

        # --- Get objects ---
        radar_objects = simulator.get_radar_objects(frame_idx, rng)
        vision_objects = simulator.get_vision_objects(
            frame_idx, H_gt, image_width, image_height, rng
        )

        # --- Association ground truth ---
        assoc_gt_matches = []
        for ri, robj in enumerate(radar_objects):
            r_gt_id = robj.get('_gt_id', -1)
            if r_gt_id < 0:
                continue
            for vi, vobj in enumerate(vision_objects):
                v_gt_id = vobj.get('_gt_id', -1)
                if r_gt_id == v_gt_id:
                    assoc_gt_matches.append({
                        'radar_idx': ri,
                        'vision_idx': vi,
                        'gt_id': r_gt_id,
                    })

        assoc_gt = {
            'frame_index': frame_idx,
            'time_stamp': timestamp,
            'matches': assoc_gt_matches,
            'num_radar': len(radar_objects),
            'num_vision': len(vision_objects),
        }

        assoc_gt_filename = f"{frame_idx:06d}-{timestamp}.association_gt.json"
        with open(assoc_gt_dir / assoc_gt_filename, 'w') as f:
            json.dump(assoc_gt, f, indent=2)

        # --- Radar JSON (strip internal _gt_id) ---
        radar_filename = f"{radar_frame_idx:06d}-{timestamp}.radar.json"
        clean_radar = []
        for ro in radar_objects:
            cro = {k: v for k, v in ro.items() if not k.startswith('_')}
            clean_radar.append(cro)

        radar_json = {
            "baseId": 2,
            "filename": radar_filename,
            "objects": clean_radar,
        }

        with open(radar_dir / radar_filename, 'w') as f:
            json.dump(radar_json, f, indent=2)

        # --- Vision JSON (strip internal _gt_id) ---
        image_filename = f"{image_frame_idx:06d}-{timestamp}.jpg"
        vision_filename = f"{frame_idx:06d}-{timestamp}.vision.json"

        clean_vision = []
        for vo in vision_objects:
            cvo = {k: v for k, v in vo.items() if not k.startswith('_')}
            clean_vision.append(cvo)

        vision_json = {
            "sensor_id": 1,
            "frame_index": frame_idx,
            "time_stamp": timestamp,
            "image_path": f"image/{image_filename}",
            "objects": clean_vision,
        }

        with open(vision_dir / vision_filename, 'w') as f:
            json.dump(vision_json, f, indent=2)

        # --- Image ---
        road_img = draw_road_image(image_width, image_height)

        for obj in radar_objects:
            if obj.get('_gt_id', -1) < 0:
                continue
            x, y = obj['coord']
            pt = np.array([x, y, 1.0])
            proj = H_gt @ pt
            if abs(proj[2]) > 1e-10:
                u = proj[0] / proj[2]
                v = proj[1] / proj[2]
                if 0 <= u < image_width and 0 <= v < image_height:
                    scale = max(0.3, min(1.5, 50.0 / max(y, 1)))
                    draw_vehicle_on_image(road_img, u, v, obj['id'], scale)

                    # Collect matched points for first few frames
                    if frame_idx < 5 and obj['id'] <= 4:
                        all_matched_points.append({
                            "radar_point": {"x": x, "y": y},
                            "image_point": {"u": round(u, 2), "v": round(v, 2)},
                        })

        cv2.imwrite(str(image_dir / image_filename), road_img)

        # --- Sync JSON ---
        sync_data = {
            "sync_batch": frame_idx,
            "batch_size": 3,
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
                {
                    "channel_id": 2,
                    "sensor_id": 1,
                    "data_type": "vision_detection",
                    "data_path": f"vision/{vision_filename}",
                },
            ],
        }

        sync_filename = f"{frame_idx:06d}.sync.json"
        with open(sync_dir / sync_filename, 'w') as f:
            json.dump(sync_data, f, indent=2)

    # --- Save ground truth calibration (enhanced with camera params) ---
    unique_points = {}
    for mp in all_matched_points:
        key = (mp["radar_point"]["x"], mp["radar_point"]["y"])
        if key not in unique_points:
            unique_points[key] = mp
    gt_matched = list(unique_points.values())

    gt_calib = {
        "version": "2.0",
        "calibration_type": "radar_to_camera_homography",
        "camera_sensor_id": 1,
        "radar_sensor_id": 2,
        "homography": H_gt.tolist(),
        "inverse_homography": H_gt_inv.tolist(),
        "matched_points": gt_matched,
        "mean_reprojection_error": 0.0,
        "max_reprojection_error": 0.0,
        "camera_parameters": camera_params,
    }

    with open(output / "ground_truth_calibration.json", 'w') as f:
        json.dump(gt_calib, f, indent=2)

    # --- Save mock matched points ---
    with open(output / "mock_matched_points.json", 'w') as f:
        json.dump({"matched_points": gt_matched}, f, indent=2)

    # --- Save dataset config ---
    dataset_config = {
        "version": "1.0",
        "created_at": datetime.now().isoformat(),
        "num_frames": num_frames,
        "image_size": {"width": image_width, "height": image_height},
        "num_trajectories": len(simulator.trajectories),
        "radar_detection_rates": {
            "0-60m": 0.95,
            "60-100m": 0.85,
            "100-150m": 0.65,
            "150m+": 0.40,
        },
        "vision_detection_rates": {
            "0-80m": 0.95,
            "80-150m": 0.85,
            "150m+": 0.65,
        },
        "radar_false_alarm_rate": 0.05,
        "vision_false_detection_rate": 0.03,
        "blind_zone": {
            "x_range": [-15, -10],
            "y_range": [20, 50],
        },
        "sensors": {
            "camera_sensor_id": 1,
            "radar_sensor_id": 2,
        },
    }

    with open(output / "mock_dataset_config.json", 'w') as f:
        json.dump(dataset_config, f, indent=2)

    print(f"✅ Generated {num_frames} frames:")
    print(f"   Sync:    {sync_dir}")
    print(f"   Image:   {image_dir}")
    print(f"   Radar:   {radar_dir}")
    print(f"   Vision:  {vision_dir}")
    print(f"   Assoc GT:{assoc_gt_dir}")
    print(f"   Ground truth calibration: {output / 'ground_truth_calibration.json'}")
    print(f"   Mock matched points ({len(gt_matched)}): {output / 'mock_matched_points.json'}")
    print(f"   Dataset config: {output / 'mock_dataset_config.json'}")


def main():
    parser = argparse.ArgumentParser(
        description="Generate mock radar-camera-vision sync dataset"
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
