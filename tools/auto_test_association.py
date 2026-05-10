#!/usr/bin/env python3
"""
auto_test_association.py - Automated association testing on mock dataset.

Loads a mock dataset, runs radar-vision association on all frames,
compares against association ground truth, and prints statistics.

Usage:
    python tools/auto_test_association.py --dataset mock_dataset
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

# Add project root to path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from data.sync_dataset import SyncDataset
from data.radar_loader import load_radar_json
from vision.vision_loader import load_vision_json
from vision.vision_models import VisionObject
from data.data_models import RadarObject
from association.associator import associate_radar_vision


def load_ground_truth_calibration(dataset_root: Path) -> dict:
    """Load ground truth calibration from dataset."""
    path = dataset_root / "ground_truth_calibration.json"
    with open(path, "r") as f:
        return json.load(f)


def load_association_gt(path: Path) -> dict:
    """Load a single association ground truth file."""
    with open(path, "r") as f:
        return json.load(f)


def run_association_test(dataset_root: str):
    """Run association on all frames and compare with ground truth."""
    root = Path(dataset_root)
    if not root.exists():
        print(f"❌ Dataset not found: {root}")
        return

    # Load calibration
    calib_data = load_ground_truth_calibration(root)
    H = np.array(calib_data["homography"], dtype=np.float64)
    H_inv = np.array(calib_data["inverse_homography"], dtype=np.float64)

    # Load dataset
    dataset = SyncDataset()
    dataset.load(str(root))

    # Find association GT files
    assoc_gt_dir = root / "association_gt"
    gt_files = sorted(assoc_gt_dir.glob("*.association_gt.json"))

    print(f"📊 Running association test on {dataset.num_frames} frames...")
    print(f"   Ground truth files: {len(gt_files)}")
    print()

    total_matches = 0
    total_correct = 0
    total_radar = 0
    total_vision = 0
    total_gt_matches = 0
    frame_results = []

    for frame_idx in range(dataset.num_frames):
        dataset.go_to(frame_idx)

        # Load radar
        radar_frame = dataset.get_radar()
        if not radar_frame:
            continue

        # Load vision
        vision_frame = dataset.get_vision()
        if not vision_frame:
            continue

        # Run association
        result = associate_radar_vision(
            radar_frame.objects,
            vision_frame.objects,
            H, H_inv,
        )

        # Load GT if available
        if frame_idx < len(gt_files):
            gt = load_association_gt(gt_files[frame_idx])
            gt_match_map = {}
            for gm in gt.get("matches", []):
                gt_match_map[gm["radar_idx"]] = gm["vision_idx"]

            # Count correct matches
            correct = 0
            for m in result.matches:
                if gt_match_map.get(m.radar_idx) == m.vision_idx:
                    correct += 1

            total_correct += correct
            total_gt_matches += len(gt_match_map)
        else:
            correct = 0

        total_matches += result.num_matches
        total_radar += len(radar_frame.objects)
        total_vision += len(vision_frame.objects)

        frame_results.append({
            "frame": frame_idx,
            "n_radar": len(radar_frame.objects),
            "n_vision": len(vision_frame.objects),
            "n_matches": result.num_matches,
            "n_unmatched_radar": result.num_unmatched_radar,
            "n_unmatched_vision": result.num_unmatched_vision,
            "correct": correct,
        })

    # Print summary
    print("=" * 70)
    print("ASSOCIATION TEST RESULTS")
    print("=" * 70)
    print(f"Total frames:          {len(frame_results)}")
    print(f"Total radar objects:    {total_radar}")
    print(f"Total vision objects:   {total_vision}")
    print(f"Total matches:         {total_matches}")
    print(f"Total GT matches:      {total_gt_matches}")
    print(f"Correct matches:       {total_correct}")

    if total_gt_matches > 0:
        accuracy = total_correct / total_gt_matches * 100
        print(f"Match accuracy:        {accuracy:.1f}%")

    if total_vision > 0:
        recall = total_matches / total_vision * 100
        print(f"Recall:                {recall:.1f}%")

    if total_radar > 0:
        unmatched_radar = total_radar - total_matches
        fp_rate = unmatched_radar / total_radar * 100
        print(f"FP rate:               {fp_rate:.1f}%")

    print()

    # Per-frame sample (first 10 frames)
    print("Per-frame sample (first 10 frames):")
    print(f"{'Frame':>5} {'Radar':>6} {'Vision':>7} {'Match':>6} {'UR':>4} {'UV':>4} {'Correct':>8}")
    print("-" * 50)
    for fr in frame_results[:10]:
        print(
            f"{fr['frame']:>5} {fr['n_radar']:>6} {fr['n_vision']:>7} "
            f"{fr['n_matches']:>6} {fr['n_unmatched_radar']:>4} "
            f"{fr['n_unmatched_vision']:>4} {fr['correct']:>8}"
        )

    print()
    print("✅ Association test complete.")


def main():
    parser = argparse.ArgumentParser(
        description="Run automated association testing on mock dataset"
    )
    parser.add_argument(
        "--dataset", type=str, required=True,
        help="Path to mock dataset directory",
    )
    args = parser.parse_args()
    run_association_test(args.dataset)


if __name__ == "__main__":
    main()
