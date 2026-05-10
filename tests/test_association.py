"""
test_association.py - Tests for radar-vision association module.

Tests gating functions, cost matrix, Hungarian matching, and full pipeline.
"""

import unittest
import numpy as np
from pathlib import Path

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from data.data_models import RadarObject
from vision.vision_models import VisionObject
from association.gating import (
    image_gating,
    bev_gating,
    class_compatibility,
    combined_gating,
)
from association.cost_function import (
    compute_bev_distances,
    compute_pixel_distances,
    normalize_matrix,
    compute_cost_matrix,
)
from association.hungarian_matcher import hungarian_match
from association.associator import associate_radar_vision


class TestImageGating(unittest.TestCase):
    """Test image gating function."""

    def test_point_inside_bbox(self):
        radar_uv = np.array([[150, 250]])
        bboxes = np.array([[100, 200, 200, 300]])
        gate = image_gating(radar_uv, bboxes, margin=0)
        self.assertTrue(gate[0, 0])

    def test_point_outside_bbox(self):
        radar_uv = np.array([[50, 50]])
        bboxes = np.array([[100, 200, 200, 300]])
        gate = image_gating(radar_uv, bboxes, margin=0)
        self.assertFalse(gate[0, 0])

    def test_point_inside_with_margin(self):
        radar_uv = np.array([[95, 195]])
        bboxes = np.array([[100, 200, 200, 300]])
        gate = image_gating(radar_uv, bboxes, margin=10)
        self.assertTrue(gate[0, 0])

    def test_empty_inputs(self):
        gate = image_gating(np.empty((0, 2)), np.array([[100, 200, 200, 300]]))
        self.assertEqual(gate.shape, (0, 1))

    def test_multi_pairs(self):
        radar_uv = np.array([[150, 250], [50, 50]])
        bboxes = np.array([[100, 200, 200, 300], [40, 40, 60, 60]])
        gate = image_gating(radar_uv, bboxes, margin=0)
        self.assertTrue(gate[0, 0])   # r0 inside b0
        self.assertFalse(gate[0, 1])  # r0 not in b1
        self.assertFalse(gate[1, 0])  # r1 not in b0
        self.assertTrue(gate[1, 1])   # r1 inside b1


class TestBEVGating(unittest.TestCase):
    """Test BEV gating function."""

    def test_close_points(self):
        radar = np.array([[0, 50]])
        vision = np.array([[1, 51]])
        gate = bev_gating(radar, vision, threshold=5.0)
        self.assertTrue(gate[0, 0])

    def test_far_points(self):
        radar = np.array([[0, 50]])
        vision = np.array([[10, 60]])
        gate = bev_gating(radar, vision, threshold=5.0)
        self.assertFalse(gate[0, 0])


class TestClassCompat(unittest.TestCase):
    """Test class compatibility."""

    def test_vehicle_vehicle(self):
        gate = class_compatibility(["vehicle"], ["vehicle"])
        self.assertTrue(gate[0, 0])

    def test_vehicle_car(self):
        gate = class_compatibility(["vehicle"], ["car"])
        self.assertTrue(gate[0, 0])

    def test_vehicle_pedestrian(self):
        gate = class_compatibility(["vehicle"], ["pedestrian"])
        self.assertFalse(gate[0, 0])

    def test_unknown_matches_all(self):
        gate = class_compatibility(["unknown"], ["vehicle"])
        self.assertTrue(gate[0, 0])


class TestCostFunction(unittest.TestCase):
    """Test cost matrix computation."""

    def test_bev_distances(self):
        radar = np.array([[0, 0], [1, 1]])
        vision = np.array([[0, 0], [2, 2]])
        dist = compute_bev_distances(radar, vision)
        self.assertEqual(dist.shape, (2, 2))
        self.assertAlmostEqual(dist[0, 0], 0.0)
        self.assertGreater(dist[0, 1], 0)

    def test_normalize_matrix(self):
        m = np.array([[0, 5], [10, 15]], dtype=float)
        n = normalize_matrix(m)
        self.assertAlmostEqual(n.min(), 0.0)
        self.assertAlmostEqual(n.max(), 1.0)

    def test_normalize_uniform(self):
        m = np.array([[5, 5], [5, 5]], dtype=float)
        n = normalize_matrix(m)
        self.assertTrue(np.allclose(n, 0))

    def test_cost_matrix_gating(self):
        bev = np.array([[1, 2], [3, 4]], dtype=float)
        pix = np.array([[10, 20], [30, 40]], dtype=float)
        gate = np.array([[True, False], [False, True]])
        cost = compute_cost_matrix(bev, pix, gate)
        self.assertTrue(np.isfinite(cost[0, 0]))
        self.assertTrue(np.isinf(cost[0, 1]))
        self.assertTrue(np.isinf(cost[1, 0]))
        self.assertTrue(np.isfinite(cost[1, 1]))


class TestHungarianMatcher(unittest.TestCase):
    """Test Hungarian matching."""

    def test_simple_match(self):
        cost = np.array([[0.1, 0.9], [0.9, 0.2]])
        gate = np.array([[True, True], [True, True]])
        bev = np.array([[1, 5], [5, 1]], dtype=float)
        pix = np.array([[10, 50], [50, 10]], dtype=float)

        result = hungarian_match(cost, gate, bev, pix, max_cost=0.8)
        self.assertEqual(len(result.matches), 2)
        # Optimal: r0→v0, r1→v1
        match_map = {m.radar_idx: m.vision_idx for m in result.matches}
        self.assertEqual(match_map[0], 0)
        self.assertEqual(match_map[1], 1)

    def test_max_cost_filter(self):
        cost = np.array([[0.1, 0.9]])
        gate = np.array([[True, True]])
        bev = np.array([[1, 5]], dtype=float)
        pix = np.array([[10, 50]], dtype=float)

        result = hungarian_match(cost, gate, bev, pix, max_cost=0.5)
        self.assertEqual(len(result.matches), 1)
        self.assertEqual(result.matches[0].vision_idx, 0)


class TestAssociator(unittest.TestCase):
    """Test full association pipeline."""

    def test_basic_association(self):
        """Test with synthetic data and known homography."""
        import cv2

        # Create simple homography
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

        # Create radar objects at known positions
        radar_objects = [
            RadarObject(track_id=1, obj_class="vehicle", x=-3.0, y=50.0),
            RadarObject(track_id=2, obj_class="vehicle", x=3.0, y=50.0),
        ]

        # Project radar to image to create vision bboxes
        from calibration.projector import project_single_point
        vision_objects = []
        for i, robj in enumerate(radar_objects):
            u, v = project_single_point(robj.x, robj.y, H)
            # Create bbox centered on projection with bottom at v
            # The bottom center should back-project close to the radar point
            bbox = [u - 40, v - 40, u + 40, v]
            vision_objects.append(VisionObject(
                id=100 + i, track_id=20 + i,
                obj_class="vehicle", bbox=bbox, score=0.9,
            ))

        # Use relaxed config so synthetic data passes gating
        config = {
            "bev_threshold": 15.0,
            "image_margin": 50.0,
            "max_cost": 0.95,
        }
        result = associate_radar_vision(
            radar_objects, vision_objects, H, H_inv, config
        )
        # Should match all
        self.assertEqual(len(result.matches), 2)
        self.assertEqual(len(result.unmatched_radar), 0)
        self.assertEqual(len(result.unmatched_vision), 0)

    def test_empty_inputs(self):
        H = np.eye(3)
        H_inv = np.eye(3)
        result = associate_radar_vision([], [], H, H_inv)
        self.assertEqual(len(result.matches), 0)


if __name__ == "__main__":
    unittest.main()
