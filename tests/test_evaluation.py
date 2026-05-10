"""
test_evaluation.py - Tests for evaluation metrics module.

Tests metric computation, distance binning, lane/grid metrics.
"""

import unittest
import numpy as np
from pathlib import Path

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from data.data_models import RadarObject
from vision.vision_models import VisionObject
from association.association_models import Match, AssociationResult
from evaluation.metrics import compute_association_metrics
from evaluation.eval_models import EvaluationResult


class TestEvaluationMetrics(unittest.TestCase):
    """Test evaluation metric computation."""

    def _create_test_data(self):
        """Create synthetic test data for evaluation."""
        import cv2

        radar_pts = np.array([
            [-5, 30], [5, 30], [-3, 60], [3, 60],
            [0, 45], [-7, 80], [7, 80], [0, 100],
        ], dtype=np.float64)
        image_pts = np.array([
            [280, 520], [1000, 520], [380, 400], [900, 400],
            [640, 450], [320, 340], [960, 340], [640, 300],
        ], dtype=np.float64)
        H, _ = cv2.findHomography(radar_pts, image_pts)
        H_inv = np.linalg.inv(H)

        radar_objects = [
            RadarObject(track_id=1, obj_class="vehicle", x=-3.0, y=40.0),
            RadarObject(track_id=2, obj_class="vehicle", x=3.0, y=50.0),
            RadarObject(track_id=3, obj_class="vehicle", x=0.0, y=80.0),
            RadarObject(track_id=99, obj_class="unknown", x=10.0, y=90.0),  # FP
        ]

        from calibration.projector import project_single_point
        vision_objects = []
        for i in range(3):
            robj = radar_objects[i]
            u, v = project_single_point(robj.x, robj.y, H)
            bbox = [u - 30, v - 20, u + 30, v + 20]
            vision_objects.append(VisionObject(
                id=100 + i, track_id=20 + i,
                obj_class="vehicle", bbox=bbox, score=0.9,
            ))

        matches = [
            Match(radar_idx=0, vision_idx=0, cost=0.1, bev_distance=1.0, pixel_distance=5.0),
            Match(radar_idx=1, vision_idx=1, cost=0.2, bev_distance=2.0, pixel_distance=8.0),
            Match(radar_idx=2, vision_idx=2, cost=0.15, bev_distance=1.5, pixel_distance=6.0),
        ]

        result = AssociationResult(
            matches=matches,
            unmatched_radar=[3],
            unmatched_vision=[],
        )

        return result, radar_objects, vision_objects, H_inv

    def test_basic_metrics(self):
        result, radar_objs, vision_objs, H_inv = self._create_test_data()
        metrics = compute_association_metrics(
            result, radar_objs, vision_objs, H_inv,
        )
        # 3 matches / 3 vision = 1.0 recall
        self.assertAlmostEqual(metrics.radar_recall, 1.0)
        # 1 unmatched / 4 radar = 0.25 FP rate
        self.assertAlmostEqual(metrics.radar_fp_rate, 0.25)
        self.assertEqual(metrics.total_radar, 4)
        self.assertEqual(metrics.total_vision, 3)
        self.assertEqual(metrics.total_matched, 3)

    def test_bev_error_stats(self):
        result, radar_objs, vision_objs, H_inv = self._create_test_data()
        metrics = compute_association_metrics(
            result, radar_objs, vision_objs, H_inv,
        )
        self.assertGreater(metrics.mean_bev_error, 0)
        self.assertGreaterEqual(metrics.p90_bev_error, metrics.p50_bev_error)
        self.assertGreaterEqual(metrics.p95_bev_error, metrics.p90_bev_error)

    def test_distance_bins(self):
        result, radar_objs, vision_objs, H_inv = self._create_test_data()
        metrics = compute_association_metrics(
            result, radar_objs, vision_objs, H_inv,
        )
        self.assertTrue(len(metrics.distance_bins) > 0)
        for db in metrics.distance_bins:
            self.assertTrue(hasattr(db, 'range_str'))
            self.assertTrue(hasattr(db, 'recall'))

    def test_lane_metrics(self):
        result, radar_objs, vision_objs, H_inv = self._create_test_data()
        metrics = compute_association_metrics(
            result, radar_objs, vision_objs, H_inv,
        )
        self.assertTrue(len(metrics.lane_metrics) > 0)

    def test_grid_metrics(self):
        result, radar_objs, vision_objs, H_inv = self._create_test_data()
        metrics = compute_association_metrics(
            result, radar_objs, vision_objs, H_inv,
        )
        self.assertIsNotNone(metrics.grid_metrics)

    def test_to_dict(self):
        result, radar_objs, vision_objs, H_inv = self._create_test_data()
        metrics = compute_association_metrics(
            result, radar_objs, vision_objs, H_inv,
        )
        d = metrics.to_dict()
        self.assertIn("radar_recall", d)
        self.assertIn("radar_fp_rate", d)
        self.assertIn("mean_bev_error", d)
        self.assertIn("distance_bins", d)

    def test_empty_association(self):
        """Test metrics with no matches."""
        radar_objs = [RadarObject(1, "vehicle", 0, 50)]
        vision_objs = [VisionObject(1, 1, "vehicle", [100, 200, 200, 300], 0.9)]
        result = AssociationResult(
            matches=[], unmatched_radar=[0], unmatched_vision=[0],
        )
        H_inv = np.eye(3)
        metrics = compute_association_metrics(
            result, radar_objs, vision_objs, H_inv,
        )
        self.assertAlmostEqual(metrics.radar_recall, 0.0)
        self.assertAlmostEqual(metrics.radar_fp_rate, 1.0)


if __name__ == "__main__":
    unittest.main()
