"""
test_effective_range.py - Tests for effective detection area analysis.

Tests grid-based recall computation, zone classification, and polygon extraction.
"""

import unittest
import numpy as np
from pathlib import Path

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from vision.vision_models import VisionObject
from association.association_models import Match
from evaluation.effective_range import (
    compute_effective_detection_area,
    _mask_to_rect_polygons,
)


class TestEffectiveRange(unittest.TestCase):
    """Test effective detection area computation."""

    def _create_test_data(self):
        """Create synthetic multi-frame data."""
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

        from calibration.projector import project_single_point

        # Create 10 frames of data
        all_matches = []
        all_vision_objects = []
        all_radar_bev = []

        for frame in range(10):
            # 3 objects per frame at different positions
            radar_bev = np.array([
                [-3.0, 40.0 + frame],
                [3.0, 50.0 + frame],
                [0.0, 80.0 + frame],
            ], dtype=np.float64)
            all_radar_bev.append(radar_bev)

            vision_objs = []
            for i in range(3):
                u, v = project_single_point(radar_bev[i, 0], radar_bev[i, 1], H)
                bbox = [u - 30, v - 20, u + 30, v + 20]
                vision_objs.append(VisionObject(
                    id=100 + i, track_id=20 + i,
                    obj_class="vehicle", bbox=bbox, score=0.9,
                ))
            all_vision_objects.append(vision_objs)

            # All matched
            matches = [
                Match(radar_idx=0, vision_idx=0, cost=0.1, bev_distance=1.0),
                Match(radar_idx=1, vision_idx=1, cost=0.2, bev_distance=2.0),
                Match(radar_idx=2, vision_idx=2, cost=0.15, bev_distance=1.5),
            ]
            all_matches.append(matches)

        return all_matches, all_vision_objects, all_radar_bev, H_inv

    def test_compute_effective_area(self):
        all_matches, all_vision, all_radar, H_inv = self._create_test_data()
        config = {
            "grid_size": 10.0,
            "x_range": [-20, 20],
            "y_range": [0, 150],
            "min_vision_count": 1,
            "min_recall": 0.5,
            "blind_zone_max_recall": 0.2,
        }

        result = compute_effective_detection_area(
            all_matches, all_vision, all_radar, H_inv, config,
        )

        self.assertIn("radar_observed_coverage_polygon", result)
        self.assertIn("radar_effective_detection_polygon", result)
        self.assertIn("radar_blind_zone_polygon", result)
        self.assertIn("stats", result)

        stats = result["stats"]
        self.assertGreater(stats["observed_cells"], 0)
        self.assertGreater(stats["total_vision_detections"], 0)

    def test_empty_data(self):
        """Test with no data — should return empty polygons."""
        result = compute_effective_detection_area(
            [], [], [], np.eye(3),
        )
        stats = result["stats"]
        self.assertEqual(stats["observed_cells"], 0)

    def test_mask_to_polygons(self):
        mask = np.zeros((5, 5), dtype=bool)
        mask[1:3, 1:3] = True  # 2x2 block

        polys = _mask_to_rect_polygons(mask, 0, 0, 10.0)
        self.assertTrue(len(polys) > 0)

    def test_mask_to_polygons_empty(self):
        mask = np.zeros((5, 5), dtype=bool)
        polys = _mask_to_rect_polygons(mask, 0, 0, 10.0)
        self.assertEqual(len(polys), 0)


if __name__ == "__main__":
    unittest.main()
