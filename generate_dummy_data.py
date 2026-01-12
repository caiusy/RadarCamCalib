#!/usr/bin/env python3
"""
generate_dummy_data.py

Generates synthetic dataset for testing Radar-Camera Fusion calibration system.
Creates images with simulated cars, corresponding radar JSON data, and calibration files.

Author: AI Assistant
Date: 2026-01-12
"""

import os
import json
import random
import numpy as np
import cv2


# =============================================================================
# Configuration
# =============================================================================

# Output directories
OUTPUT_DIR = "dataset"
IMAGES_DIR = os.path.join(OUTPUT_DIR, "images")
RADAR_DIR = os.path.join(OUTPUT_DIR, "radar")

# Image parameters
IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720

# Radar simulation parameters (in meters)
RADAR_X_RANGE = (10, 50)    # Distance range (front)
RADAR_Y_RANGE = (-10, 10)   # Lateral range (left-right)
OBJECTS_PER_FRAME = (3, 5)  # Random number of objects per frame
NOISE_STD = 0.3             # Std deviation for radar noise (meters)

# Number of frames to generate
NUM_BATCHES = 5

# Car drawing parameters (pixels)
CAR_WIDTH = 80
CAR_HEIGHT = 50
CAR_COLOR = (255, 255, 255)  # White
CENTER_MARKER_COLOR = (0, 0, 255)  # Red (BGR)
CENTER_MARKER_RADIUS = 5


# =============================================================================
# Ground Truth Homography Matrix
# =============================================================================
# This matrix maps Radar coordinates (x, y) in meters to Image coordinates (u, v) in pixels.
# 
# Radar coordinate system:
#   - x: forward distance (positive = ahead)
#   - y: lateral distance (positive = right)
#
# Image coordinate system:
#   - u: horizontal pixel (0 = left, 1280 = right)
#   - v: vertical pixel (0 = top, 720 = bottom)
#
# The homography is designed so that:
#   - Objects further away (large x) appear higher in the image (smaller v)
#   - Objects to the right (positive y) appear on the right side of the image (larger u)
#   - Image center (~640, ~500) corresponds roughly to (30m, 0m) on ground

H_GROUND_TRUTH = np.array([
    [  0.0,   20.0,  640.0],   # u = 20*y + 640 (lateral mapping)
    [-12.0,    0.0,  860.0],   # v = -12*x + 860 (depth mapping, inverted)
    [  0.0,    0.0,    1.0]    # homogeneous coordinate
], dtype=np.float64)


# =============================================================================
# Helper Functions
# =============================================================================

def create_directories():
    """Create output directory structure."""
    os.makedirs(IMAGES_DIR, exist_ok=True)
    os.makedirs(RADAR_DIR, exist_ok=True)
    print(f"[INFO] Created directories:")
    print(f"       - {IMAGES_DIR}")
    print(f"       - {RADAR_DIR}")


def radar_to_image(radar_x: float, radar_y: float, H: np.ndarray) -> tuple:
    """
    Project radar coordinates to image coordinates using homography matrix.
    
    Args:
        radar_x: Forward distance in meters
        radar_y: Lateral distance in meters
        H: 3x3 Homography matrix
        
    Returns:
        (u, v): Pixel coordinates in image
    """
    # Homogeneous coordinates
    radar_point = np.array([radar_x, radar_y, 1.0])
    
    # Apply homography
    img_point = H @ radar_point
    
    # Normalize by homogeneous coordinate
    u = img_point[0] / img_point[2]
    v = img_point[1] / img_point[2]
    
    return (u, v)


def generate_random_objects(num_objects: int) -> list:
    """
    Generate random car positions in radar coordinates.
    
    Args:
        num_objects: Number of objects to generate
        
    Returns:
        List of (x, y) tuples in meters
    """
    objects = []
    for _ in range(num_objects):
        x = random.uniform(*RADAR_X_RANGE)
        y = random.uniform(*RADAR_Y_RANGE)
        objects.append((x, y))
    return objects


def add_radar_noise(objects: list, noise_std: float) -> list:
    """
    Add Gaussian noise to simulate radar sensor noise.
    
    Args:
        objects: List of (x, y) tuples
        noise_std: Standard deviation of noise
        
    Returns:
        List of (x, y) tuples with added noise
    """
    noisy_objects = []
    for x, y in objects:
        noisy_x = x + random.gauss(0, noise_std)
        noisy_y = y + random.gauss(0, noise_std)
        noisy_objects.append((noisy_x, noisy_y))
    return noisy_objects


def create_radar_json(objects: list, filepath: str):
    """
    Save radar target data to JSON file.
    
    Args:
        objects: List of (x, y) tuples (with noise)
        filepath: Output JSON file path
    """
    targets = []
    for i, (x, y) in enumerate(objects):
        target = {
            "id": i,
            "x": round(x, 2),
            "y": round(y, 2),
            "range": round(np.sqrt(x**2 + y**2), 2),
            "velocity": round(random.uniform(-5, 30), 2),  # Simulated velocity
            "rcs": round(random.uniform(5, 25), 2)  # Radar cross section
        }
        targets.append(target)
    
    data = {"targets": targets}
    
    with open(filepath, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def create_image_with_cars(objects: list, H: np.ndarray, filepath: str):
    """
    Create an image with simulated cars drawn at projected positions.
    
    Args:
        objects: List of (x, y) tuples in radar coordinates (ground truth, no noise)
        H: Homography matrix
        filepath: Output image file path
    """
    # Create blank black image
    img = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH, 3), dtype=np.uint8)
    
    # Optional: Add some background texture (road-like gradient)
    for row in range(IMAGE_HEIGHT):
        gray_value = int(30 + (row / IMAGE_HEIGHT) * 40)  # Gradient from dark to slightly lighter
        img[row, :] = (gray_value, gray_value, gray_value)
    
    # Draw horizon line
    horizon_y = 200
    cv2.line(img, (0, horizon_y), (IMAGE_WIDTH, horizon_y), (60, 60, 80), 1)
    
    # Draw each car
    for x, y in objects:
        # Project to image coordinates
        u, v = radar_to_image(x, y, H)
        u, v = int(round(u)), int(round(v))
        
        # Skip if outside image bounds
        if not (0 <= u < IMAGE_WIDTH and 0 <= v < IMAGE_HEIGHT):
            continue
        
        # Calculate car size based on distance (perspective effect)
        scale = max(0.3, min(1.5, 30 / x))  # Closer = larger
        car_w = int(CAR_WIDTH * scale)
        car_h = int(CAR_HEIGHT * scale)
        
        # Draw white rectangle (car)
        top_left = (u - car_w // 2, v - car_h // 2)
        bottom_right = (u + car_w // 2, v + car_h // 2)
        cv2.rectangle(img, top_left, bottom_right, CAR_COLOR, -1)  # Filled
        cv2.rectangle(img, top_left, bottom_right, (180, 180, 180), 2)  # Border
        
        # Draw red center marker for easy clicking
        cv2.circle(img, (u, v), CENTER_MARKER_RADIUS, CENTER_MARKER_COLOR, -1)
        
        # Draw distance label
        label = f"{x:.1f}m"
        cv2.putText(img, label, (u - 20, v - car_h // 2 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    
    # Save image
    cv2.imwrite(filepath, img)


def generate_calibration_points(H: np.ndarray, filepath: str):
    """
    Generate 4 fixed calibration points and save to file.
    These points form a rectangle on the ground plane.
    
    Args:
        H: Homography matrix
        filepath: Output text file path
    """
    # Define 4 calibration points in radar coordinates (forming a rectangle)
    # These are chosen to be within the typical detection range
    calibration_radar_points = [
        (20.0, -5.0),   # Near-left
        (20.0,  5.0),   # Near-right
        (40.0, -5.0),   # Far-left
        (40.0,  5.0),   # Far-right
    ]
    
    with open(filepath, 'w', encoding='utf-8') as f:
        f.write("# Calibration Points: radar_x radar_y img_u img_v\n")
        f.write("# These 4 points can be loaded to compute Homography matrix\n")
        for rx, ry in calibration_radar_points:
            u, v = radar_to_image(rx, ry, H)
            f.write(f"{rx:.2f} {ry:.2f} {u:.2f} {v:.2f}\n")
    
    print(f"[INFO] Saved calibration points to: {filepath}")
    print("       Points (radar_x, radar_y) -> (img_u, img_v):")
    for rx, ry in calibration_radar_points:
        u, v = radar_to_image(rx, ry, H)
        print(f"         ({rx:.1f}, {ry:.1f}) -> ({u:.1f}, {v:.1f})")


def save_homography_matrix(H: np.ndarray, filepath: str):
    """
    Save the ground truth homography matrix for reference.
    
    Args:
        H: Homography matrix
        filepath: Output text file path
    """
    with open(filepath, 'w', encoding='utf-8') as f:
        f.write("# Ground Truth Homography Matrix (3x3)\n")
        f.write("# Maps Radar (x, y) to Image (u, v)\n")
        f.write("# Usage: [u, v, 1]^T = H @ [x, y, 1]^T (then normalize)\n")
        for row in H:
            f.write(" ".join(f"{val:12.4f}" for val in row) + "\n")
    
    print(f"[INFO] Saved homography matrix to: {filepath}")


# =============================================================================
# Main Generation Logic
# =============================================================================

def main():
    print("=" * 60)
    print("Radar-Camera Fusion Test Data Generator")
    print("=" * 60)
    
    # Create directories
    create_directories()
    
    # Data sync list
    sync_data = []
    
    # Generate each batch/frame
    print(f"\n[INFO] Generating {NUM_BATCHES} synthetic frames...")
    
    for batch_id in range(NUM_BATCHES):
        print(f"\n--- Batch {batch_id} ---")
        
        # Generate random object positions (ground truth)
        num_objects = random.randint(*OBJECTS_PER_FRAME)
        gt_objects = generate_random_objects(num_objects)
        print(f"  Objects: {num_objects}")
        
        # Add noise for radar data
        noisy_objects = add_radar_noise(gt_objects, NOISE_STD)
        
        # Create file paths
        img_filename = f"{batch_id:03d}.jpg"
        radar_filename = f"{batch_id:03d}.json"
        img_path = os.path.join(IMAGES_DIR, img_filename)
        radar_path = os.path.join(RADAR_DIR, radar_filename)
        
        # Create radar JSON (with noisy data)
        create_radar_json(noisy_objects, radar_path)
        print(f"  Radar JSON: {radar_path}")
        
        # Create image (using ground truth positions for drawing)
        create_image_with_cars(gt_objects, H_GROUND_TRUTH, img_path)
        print(f"  Image: {img_path}")
        
        # Add to sync data (relative paths)
        sync_data.append({
            "batch_id": batch_id,
            "image_path": f"images/{img_filename}",
            "radar_json": f"radar/{radar_filename}"
        })
    
    # Save data_sync.json
    sync_filepath = os.path.join(OUTPUT_DIR, "data_sync.json")
    with open(sync_filepath, 'w', encoding='utf-8') as f:
        json.dump(sync_data, f, indent=2, ensure_ascii=False)
    print(f"\n[INFO] Saved sync file to: {sync_filepath}")
    
    # Generate calibration points
    calib_filepath = os.path.join(OUTPUT_DIR, "calibration_points.txt")
    generate_calibration_points(H_GROUND_TRUTH, calib_filepath)
    
    # Save ground truth homography for reference
    h_filepath = os.path.join(OUTPUT_DIR, "ground_truth_H.txt")
    save_homography_matrix(H_GROUND_TRUTH, h_filepath)
    
    # Summary
    print("\n" + "=" * 60)
    print("Generation Complete!")
    print("=" * 60)
    print(f"\nOutput structure:")
    print(f"  {OUTPUT_DIR}/")
    print(f"  ├── images/           ({NUM_BATCHES} .jpg files)")
    print(f"  ├── radar/            ({NUM_BATCHES} .json files)")
    print(f"  ├── data_sync.json    (sync mapping)")
    print(f"  ├── calibration_points.txt (4 point pairs for H matrix)")
    print(f"  └── ground_truth_H.txt     (reference H matrix)")
    print(f"\nYou can now use this dataset to test the calibration tool!")


if __name__ == "__main__":
    main()
