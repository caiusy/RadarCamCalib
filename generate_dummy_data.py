#!/usr/bin/env python3
"""
generate_dummy_data.py - Generate Synthetic Dataset for BEV Projection System

This script generates a synchronized dataset with:
1. Simulated camera images with objects
2. Radar JSON files with targets in radar coordinates
3. Data sync file mapping images to radar
4. Ground truth parameters for verification

Coordinate Systems:
- BEV (Vehicle): X=forward, Y=left, origin at rear axle
- Radar: X=forward, Y=left in radar frame, offset by radar_yaw and position
- Camera: Projects BEV points to image using intrinsics + pitch + height

Author: caiusy
Date: 2026-01-13
"""

import os
import json
import numpy as np
import cv2
from typing import List, Tuple

# =============================================================================
# Configuration
# =============================================================================

# Output paths
OUTPUT_DIR = "dataset"
IMAGES_DIR = os.path.join(OUTPUT_DIR, "images")
RADAR_DIR = os.path.join(OUTPUT_DIR, "radar")

# Image parameters
IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 960

# Camera parameters (ground truth)
CAMERA_HEIGHT = 1.5      # meters above ground
CAMERA_PITCH = 0.05      # radians (positive = looking down)
CAMERA_FX = 1000.0       # focal length
CAMERA_FY = 1000.0
CAMERA_CX = 640.0        # principal point
CAMERA_CY = 480.0
CAMERA_X_OFFSET = 3.5    # camera position from rear axle (m)

# Radar parameters (ground truth)
RADAR_YAW = 0.03         # radians (radar azimuth offset, positive = rotated left)
RADAR_X_OFFSET = 3.5     # radar X position from rear axle (m)
RADAR_Y_OFFSET = 0.0     # radar Y position from rear axle (m)

# Object parameters
CAR_WIDTH_PX = 60        # car width in pixels
CAR_HEIGHT_PX = 40       # car height in pixels
CAR_COLOR = (220, 220, 220)  # light gray
CENTER_MARKER_RADIUS = 4
CENTER_MARKER_COLOR = (0, 0, 255)  # red

# Lane parameters
LANE_COLOR = (255, 255, 0)  # cyan
LANE_WIDTH = 2

# Number of frames
NUM_FRAMES = 5

# =============================================================================
# Coordinate Transforms (Y=Forward, X=Right)
# =============================================================================

def bev_to_radar(x_bev: float, y_bev: float) -> Tuple[float, float]:
    """
    Transform BEV (Y=Forward, X=Right) to Radar (X=Forward, Y=Left).
    """
    # Inverse mapping logic from calibration.py
    # x_rot_fwd = y_bev - offset_x
    # y_rot_left = offset_y - x_bev
    
    x_rot_fwd = y_bev - RADAR_X_OFFSET
    y_rot_left = RADAR_Y_OFFSET - x_bev
    
    cos_yaw = np.cos(RADAR_YAW)
    sin_yaw = np.sin(RADAR_YAW)
    
    x_radar = x_rot_fwd * cos_yaw + y_rot_left * sin_yaw
    y_radar = -x_rot_fwd * sin_yaw + y_rot_left * cos_yaw
    
    return (x_radar, y_radar)


def radar_to_bev(x_radar: float, y_radar: float) -> Tuple[float, float]:
    """Transform Radar (X=Fwd, Y=Left) to BEV (Y=Fwd, X=Right)."""
    cos_yaw = np.cos(RADAR_YAW)
    sin_yaw = np.sin(RADAR_YAW)
    
    x_rot_fwd = x_radar * cos_yaw - y_radar * sin_yaw
    y_rot_left = x_radar * sin_yaw + y_radar * cos_yaw
    
    y_bev = x_rot_fwd + RADAR_X_OFFSET
    x_bev = -y_rot_left + RADAR_Y_OFFSET
    
    return (x_bev, y_bev)


def bev_to_image(x_bev: float, y_bev: float) -> Tuple[float, float]:
    """
    Project BEV (X=Right, Y=Forward) to image pixel.
    """
    cam_y_pos = CAMERA_X_OFFSET # Forward offset (3.5m)
    cam_x_pos = 0.0             # Lateral offset
    cam_z_pos = CAMERA_HEIGHT
    
    # 1. Relative Vector in Vehicle Frame
    dx_v = x_bev - cam_x_pos
    dy_v = y_bev - cam_y_pos
    dz_v = 0.0 - cam_z_pos
    
    # 2. Map to Camera Frame (Unrotated)
    # Veh: X=Right, Y=Forward, Z=Up
    # Cam: X=Right, Y=Down, Z=Forward
    
    cx0 = dx_v      # Cam X = Veh X
    cy0 = -dz_v     # Cam Y = -Veh Z
    cz0 = dy_v      # Cam Z = Veh Y
    
    # 3. Apply Pitch Rotation
    cos_p = np.cos(CAMERA_PITCH)
    sin_p = np.sin(CAMERA_PITCH)
    
    x_cam = cx0
    y_cam = cy0 * cos_p - cz0 * sin_p
    z_cam = cy0 * sin_p + cz0 * cos_p
    
    if z_cam <= 0.1:
        return None
    
    # 4. Project
    u = CAMERA_FX * (x_cam / z_cam) + CAMERA_CX
    v = CAMERA_FY * (y_cam / z_cam) + CAMERA_CY
    
    return (u, v)


# =============================================================================
# Data Generation
# =============================================================================

def generate_objects_in_bev(frame_id: int) -> List[dict]:
    """
    Generate random objects in BEV coordinates (Y=Forward).
    """
    np.random.seed(42 + frame_id)
    
    objects = []
    n_objects = np.random.randint(5, 12) # More objects for larger range
    
    for i in range(n_objects):
        # Random position in BEV (Y=Forward 10-150m, X=Lateral -10 to +10m)
        y_bev = np.random.uniform(10, 150)
        x_bev = np.random.uniform(-10, 10)
        
        velocity = np.random.uniform(-5, 25)
        rcs = np.random.uniform(5, 30)
        
        objects.append({
            'id': i,
            'x_bev': x_bev,
            'y_bev': y_bev,
            'velocity': round(velocity, 2),
            'rcs': round(rcs, 2)
        })
    
    return objects


def generate_lane_lines() -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """
    Generate lane line endpoints in BEV coordinates (Y=Forward).
    """
    lanes = []
    
    # Left lane line (X = -3.5m)
    lanes.append(((-3.5, 5), (-3.5, 150)))
    # Right lane line (X = 3.5m)
    lanes.append(((3.5, 5), (3.5, 150)))
    # Center dashed line
    lanes.append(((0, 5), (0, 150)))
    
    return lanes


def create_image(objects: List[dict], lanes: List, filepath: str):
    """Create camera image with projected objects and lanes."""
    img = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH, 3), dtype=np.uint8)
    
    # Draw gradient sky
    for y in range(IMAGE_HEIGHT // 2):
        intensity = int(40 + (IMAGE_HEIGHT//2 - y) * 0.2)
        img[y, :] = (intensity, intensity + 10, intensity + 20)
    
    # Draw ground
    for y in range(IMAGE_HEIGHT // 2, IMAGE_HEIGHT):
        intensity = int(60 - (y - IMAGE_HEIGHT//2) * 0.05)
        img[y, :] = (intensity, intensity, intensity - 10)
    
    # Draw lane lines (DISABLED - user will annotate manually)
    # for (bev_start, bev_end) in lanes:
    #     pt1 = bev_to_image(*bev_start)
    #     pt2 = bev_to_image(*bev_end)
    #     if pt1 and pt2:
    #         pt1 = (int(pt1[0]), int(pt1[1]))
    #         pt2 = (int(pt2[0]), int(pt2[1]))
#             cv2.line(img, pt1, pt2, LANE_COLOR, LANE_WIDTH)

    
    # Draw objects (cars)
    for obj in objects:
        result = bev_to_image(obj['x_bev'], obj['y_bev'])
        if result is None:
            continue
        u, v = result
        
        # Check if in image bounds
        if not (0 <= u < IMAGE_WIDTH and 0 <= v < IMAGE_HEIGHT):
            continue
        
        u, v = int(u), int(v)
        
        # Scale car size based on distance (perspective)
        distance = obj['x_bev']
        scale = max(0.3, min(1.0, 30.0 / distance))
        w = int(CAR_WIDTH_PX * scale)
        h = int(CAR_HEIGHT_PX * scale)
        
        # Draw car rectangle
        top_left = (u - w // 2, v - h // 2)
        bottom_right = (u + w // 2, v + h // 2)
        cv2.rectangle(img, top_left, bottom_right, CAR_COLOR, -1)
        cv2.rectangle(img, top_left, bottom_right, (100, 100, 100), 1)
        
        # Draw center marker
        cv2.circle(img, (u, v), int(CENTER_MARKER_RADIUS * scale + 2), CENTER_MARKER_COLOR, -1)
    
    cv2.imwrite(filepath, img)


def create_radar_json(objects: List[dict], filepath: str):
    """Create radar JSON with targets in radar coordinates."""
    targets = []
    
    for obj in objects:
        # Convert BEV to radar coordinates
        x_radar, y_radar = bev_to_radar(obj['x_bev'], obj['y_bev'])
        
        # Calculate range and azimuth
        range_m = np.sqrt(x_radar**2 + y_radar**2)
        azimuth = np.arctan2(y_radar, x_radar)  # radians
        
        targets.append({
            'id': obj['id'],
            'x': round(x_radar, 3),
            'y': round(y_radar, 3),
            'range': round(range_m, 3),
            'azimuth': round(azimuth, 4),
            'velocity': obj['velocity'],
            'rcs': obj['rcs']
        })
    
    data = {
        'frame_id': os.path.splitext(os.path.basename(filepath))[0],
        'targets': targets,
        'radar_params': {
            'yaw': RADAR_YAW,
            'x_offset': RADAR_X_OFFSET,
            'y_offset': RADAR_Y_OFFSET
        }
    }
    
    with open(filepath, 'w') as f:
        json.dump(data, f, indent=2)


def create_sync_json(num_frames: int, filepath: str):
    """Create data synchronization JSON."""
    entries = []
    for i in range(num_frames):
        entries.append({
            'batch_id': i,
            'image_path': f'images/{i:03d}.jpg',
            'radar_json': f'radar/{i:03d}.json'
        })
    
    with open(filepath, 'w') as f:
        json.dump(entries, f, indent=2)


def create_ground_truth(filepath: str):
    """Save ground truth parameters for verification."""
    params = {
        'camera': {
            'height': CAMERA_HEIGHT,
            'pitch': CAMERA_PITCH,
            'fx': CAMERA_FX,
            'fy': CAMERA_FY,
            'cx': CAMERA_CX,
            'cy': CAMERA_CY,
            'x_offset': CAMERA_X_OFFSET
        },
        'radar': {
            'yaw': RADAR_YAW,
            'x_offset': RADAR_X_OFFSET,
            'y_offset': RADAR_Y_OFFSET
        },
        'image': {
            'width': IMAGE_WIDTH,
            'height': IMAGE_HEIGHT
        }
    }
    
    with open(filepath, 'w') as f:
        json.dump(params, f, indent=2)


def create_vanishing_lines(filepath: str):
    """
    Create a file with pre-defined parallel lines for vanishing point calibration.
    These correspond to the lane lines in the image.
    """
    # Get lane line endpoints in image coordinates
    lanes = generate_lane_lines()
    lines = []
    
    for bev_start, bev_end in lanes:
        pt1 = bev_to_image(*bev_start)
        pt2 = bev_to_image(*bev_end)
        if pt1 and pt2:
            lines.append({
                'x1': round(pt1[0], 1),
                'y1': round(pt1[1], 1),
                'x2': round(pt2[0], 1),
                'y2': round(pt2[1], 1)
            })
    
    data = {
        'description': 'Pre-defined parallel lines for vanishing point calibration',
        'lines': lines
    }
    
    with open(filepath, 'w') as f:
        json.dump(data, f, indent=2)


# =============================================================================
# Main
# =============================================================================

def main():
    print("=" * 60)
    print("BEV Projection System - Dataset Generator")
    print("=" * 60)
    
    # Create directories
    os.makedirs(IMAGES_DIR, exist_ok=True)
    os.makedirs(RADAR_DIR, exist_ok=True)
    
    # Generate frames
    print(f"\nGenerating {NUM_FRAMES} frames...")
    lanes = generate_lane_lines()
    
    for i in range(NUM_FRAMES):
        objects = generate_objects_in_bev(i)
        
        img_path = os.path.join(IMAGES_DIR, f"{i:03d}.jpg")
        radar_path = os.path.join(RADAR_DIR, f"{i:03d}.json")
        
        create_image(objects, lanes, img_path)
        create_radar_json(objects, radar_path)
        
        print(f"  Frame {i}: {len(objects)} objects")
    
    # Create auxiliary files
    create_sync_json(NUM_FRAMES, os.path.join(OUTPUT_DIR, "data_sync.json"))
    create_ground_truth(os.path.join(OUTPUT_DIR, "ground_truth.json"))
    create_vanishing_lines(os.path.join(OUTPUT_DIR, "vanishing_lines.json"))
    
    print("\n" + "-" * 60)
    print("Ground Truth Parameters:")
    print(f"  Camera Height: {CAMERA_HEIGHT} m")
    print(f"  Camera Pitch:  {CAMERA_PITCH:.4f} rad ({np.degrees(CAMERA_PITCH):.2f}°)")
    print(f"  Camera FX:     {CAMERA_FX}")
    print(f"  Radar Yaw:     {RADAR_YAW:.4f} rad ({np.degrees(RADAR_YAW):.2f}°)")
    print("-" * 60)
    
    print(f"\n✅ Dataset generated in '{OUTPUT_DIR}/'")
    print(f"   - {NUM_FRAMES} images")
    print(f"   - {NUM_FRAMES} radar JSON files")
    print(f"   - data_sync.json")
    print(f"   - ground_truth.json")
    print(f"   - vanishing_lines.json")


if __name__ == "__main__":
    main()
