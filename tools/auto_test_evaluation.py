#!/usr/bin/env python3
"""
auto_test_evaluation.py - Automated evaluation testing on mock dataset.

Loads a mock dataset, runs association + evaluation on all frames,
computes aggregate metrics and effective detection areas,
saves results to JSON.

Usage:
    python tools/auto_test_evaluation.py --dataset mock_dataset
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

# Add project root to path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from data.sync_dataset import SyncDataset
from association.associator import associate_radar_vision
from association.association_models import AssociationResult
from evaluation.metrics import compute_association_metrics
from evaluation.effective_range import (
    compute_effective_detection_area,
    save_effective_range_json,
)


def run_evaluation_test(dataset_root: str):
    """Run full evaluation pipeline on mock dataset."""
    root = Path(dataset_root)
    if not root.exists():
        print(f"❌ Dataset not found: {root}")
        return

    # Load calibration
    calib_path = root / "ground_truth_calibration.json"
    with open(calib_path, "r") as f:
        calib_data = json.load(f)

    H = np.array(calib_data["homography"], dtype=np.float64)
    H_inv = np.array(calib_data["inverse_homography"], dtype=np.float64)

    # Load dataset
    dataset = SyncDataset()
    dataset.load(str(root))

    print(f"📊 Running evaluation on {dataset.num_frames} frames...")

    # Collect per-frame data for aggregate evaluation
    all_matches = []
    all_vision_objects = []
    all_radar_bev = []

    # Aggregate counters
    agg_radar = 0
    agg_vision = 0
    agg_matched = 0
    agg_bev_errors = []

    for frame_idx in range(dataset.num_frames):
        dataset.go_to(frame_idx)

        radar_frame = dataset.get_radar()
        vision_frame = dataset.get_vision()

        if not radar_frame or not vision_frame:
            all_matches.append([])
            all_vision_objects.append([])
            all_radar_bev.append(np.empty((0, 2)))
            continue

        # Run association
        result = associate_radar_vision(
            radar_frame.objects,
            vision_frame.objects,
            H, H_inv,
        )

        # Compute per-frame metrics
        from evaluation.metrics import compute_association_metrics
        frame_metrics = compute_association_metrics(
            result, radar_frame.objects, vision_frame.objects, H_inv,
        )

        agg_radar += frame_metrics.total_radar
        agg_vision += frame_metrics.total_vision
        agg_matched += frame_metrics.total_matched

        if frame_metrics.total_matched > 0:
            agg_bev_errors.append(frame_metrics.mean_bev_error)

        # Collect for effective range analysis
        all_matches.append(result.matches)
        all_vision_objects.append(vision_frame.objects)
        radar_bev = np.array(
            [[obj.x, obj.y] for obj in radar_frame.objects],
            dtype=np.float64,
        )
        all_radar_bev.append(radar_bev)

    # Aggregate metrics
    overall_recall = agg_matched / agg_vision if agg_vision > 0 else 0.0
    overall_fp_rate = (agg_radar - agg_matched) / agg_radar if agg_radar > 0 else 0.0
    mean_bev = float(np.mean(agg_bev_errors)) if agg_bev_errors else 0.0

    # Effective detection area
    print("Computing effective detection areas...")
    eff_result = compute_effective_detection_area(
        all_matches, all_vision_objects, all_radar_bev, H_inv,
    )

    # Print results
    print()
    print("=" * 70)
    print("EVALUATION RESULTS")
    print("=" * 70)
    print(f"Total frames:              {dataset.num_frames}")
    print(f"Total radar detections:    {agg_radar}")
    print(f"Total vision detections:   {agg_vision}")
    print(f"Total matched pairs:       {agg_matched}")
    print(f"Overall radar recall:      {overall_recall:.4f} ({overall_recall*100:.1f}%)")
    print(f"Overall FP rate:           {overall_fp_rate:.4f} ({overall_fp_rate*100:.1f}%)")
    print(f"Mean BEV error:            {mean_bev:.4f} m")
    print()

    # Effective detection area stats
    eff_stats = eff_result.get("stats", {})
    print("Effective Detection Area:")
    print(f"  Observed cells:          {eff_stats.get('observed_cells', 0)}")
    print(f"  Effective cells:         {eff_stats.get('effective_cells', 0)}")
    print(f"  Blind zone cells:        {eff_stats.get('blind_zone_cells', 0)}")
    print(f"  Overall grid recall:     {eff_stats.get('overall_recall', 0):.4f}")
    print()

    # Save results
    eval_output = {
        "version": "1.0",
        "dataset": str(root),
        "num_frames": dataset.num_frames,
        "aggregate_metrics": {
            "total_radar": agg_radar,
            "total_vision": agg_vision,
            "total_matched": agg_matched,
            "radar_recall": round(overall_recall, 4),
            "radar_fp_rate": round(overall_fp_rate, 4),
            "mean_bev_error": round(mean_bev, 4),
        },
        "effective_detection_area": eff_result,
    }

    output_path = root / "evaluation_results.json"
    with open(output_path, "w") as f:
        json.dump(eval_output, f, indent=2)

    print(f"💾 Results saved to: {output_path}")

    # Save effective range separately
    eff_path = root / "effective_detection_area.json"
    save_effective_range_json(eff_path, eff_result)

    print("✅ Evaluation complete.")


def main():
    parser = argparse.ArgumentParser(
        description="Run automated evaluation testing on mock dataset"
    )
    parser.add_argument(
        "--dataset", type=str, required=True,
        help="Path to mock dataset directory",
    )
    args = parser.parse_args()
    run_evaluation_test(args.dataset)


if __name__ == "__main__":
    main()
