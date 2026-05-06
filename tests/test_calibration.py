"""
test_calibration.py - Unit tests for calibration modules.
"""

import sys
import unittest
from pathlib import Path
import numpy as np
import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from calibration.homography_solver import (
    compute_homography, compute_reprojection_errors, invert_homography
)
from calibration.calibration_session import CalibrationSession
from calibration.projector import (
    project_radar_to_image, project_image_to_radar, project_single_point
)
from data.data_models import MatchedPoint


class TestHomographySolver(unittest.TestCase):
    """Test homography computation."""

    def setUp(self):
        """Create known point correspondences."""
        self.radar_pts = np.array([
            [-5, 30], [5, 30], [-3, 60], [3, 60],
            [0, 45], [-7, 80], [7, 80], [0, 100]
        ], dtype=np.float64)
        self.image_pts = np.array([
            [280, 520], [1000, 520], [380, 400], [900, 400],
            [640, 450], [320, 340], [960, 340], [640, 300]
        ], dtype=np.float64)

    def test_compute_homography(self):
        H, mask = compute_homography(self.radar_pts, self.image_pts)
        self.assertIsNotNone(H)
        self.assertEqual(H.shape, (3, 3))
        self.assertIsNotNone(mask)

    def test_reprojection_errors(self):
        H, _ = compute_homography(self.radar_pts, self.image_pts)
        errors = compute_reprojection_errors(self.radar_pts, self.image_pts, H)
        self.assertEqual(len(errors), 8)
        # With 8 points, homography may have some residual error
        self.assertLess(np.mean(errors), 50.0)

    def test_invert_homography(self):
        H, _ = compute_homography(self.radar_pts, self.image_pts)
        H_inv = invert_homography(H)
        identity = H @ H_inv
        np.testing.assert_array_almost_equal(
            identity / identity[2, 2], np.eye(3), decimal=5
        )

    def test_too_few_points(self):
        with self.assertRaises(ValueError):
            compute_homography(self.radar_pts[:3], self.image_pts[:3])


class TestCalibrationSession(unittest.TestCase):
    """Test calibration session workflow."""

    def test_add_and_compute(self):
        session = CalibrationSession()

        pairs = [
            (-5, 30, 280, 520), (5, 30, 1000, 520),
            (-3, 60, 380, 400), (3, 60, 900, 400),
            (0, 45, 640, 450), (-7, 80, 320, 340),
            (7, 80, 960, 340), (0, 100, 640, 300),
        ]

        for rx, ry, iu, iv in pairs:
            session.add_point(rx, ry, iu, iv)

        self.assertEqual(session.num_points, 8)
        self.assertFalse(session.has_homography)

        result = session.compute()
        self.assertTrue(session.has_homography)
        self.assertIsNotNone(result.H)
        self.assertLess(result.mean_error, 5.0)

    def test_too_few_points_raises(self):
        session = CalibrationSession()
        session.add_point(0, 0, 0, 0)
        with self.assertRaises(ValueError):
            session.compute()

    def test_clear(self):
        session = CalibrationSession()
        session.add_point(0, 0, 0, 0)
        session.clear_points()
        self.assertEqual(session.num_points, 0)


class TestProjector(unittest.TestCase):
    """Test point projection functions."""

    def setUp(self):
        radar_pts = np.array([
            [-5, 30], [5, 30], [-3, 60], [3, 60],
            [0, 45], [-7, 80], [7, 80], [0, 100]
        ], dtype=np.float64)
        image_pts = np.array([
            [280, 520], [1000, 520], [380, 400], [900, 400],
            [640, 450], [320, 340], [960, 340], [640, 300]
        ], dtype=np.float64)
        self.H, _ = cv2.findHomography(radar_pts, image_pts)
        self.H_inv = np.linalg.inv(self.H)
        self.radar_pts = radar_pts
        self.image_pts = image_pts

    def test_radar_to_image(self):
        projected = project_radar_to_image(self.radar_pts, self.H)
        self.assertEqual(projected.shape, (8, 2))
        # Check approximately close (homography has inherent error with non-ideal points)
        # Most points should be within 100px
        errors = np.linalg.norm(projected - self.image_pts, axis=1)
        self.assertLess(np.mean(errors), 100.0)

    def test_image_to_radar(self):
        recovered = project_image_to_radar(self.image_pts, self.H_inv)
        self.assertEqual(recovered.shape, (8, 2))
        np.testing.assert_array_almost_equal(recovered, self.radar_pts, decimal=0)

    def test_single_point(self):
        u, v = project_single_point(-5.0, 30.0, self.H)
        self.assertAlmostEqual(u, 280, delta=30)
        self.assertAlmostEqual(v, 520, delta=30)

    def test_empty_input(self):
        result = project_radar_to_image(np.empty((0, 2)), self.H)
        self.assertEqual(result.shape, (0, 2))


if __name__ == "__main__":
    unittest.main()
