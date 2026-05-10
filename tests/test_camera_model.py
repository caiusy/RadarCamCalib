"""
test_camera_model.py - Tests for camera parameter module.

Tests CameraModel creation, serialization, pinhole projection,
and undistortion utilities.
"""

import unittest
import numpy as np
import tempfile
import json
from pathlib import Path

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from camera.camera_model import (
    CameraModel,
    get_default_camera_model,
    project_points_pinhole,
    undistort_points,
)
from camera.camera_io import (
    save_camera_params,
    load_camera_params,
)


class TestCameraModel(unittest.TestCase):
    """Test CameraModel dataclass."""

    def test_create_default_model(self):
        model = get_default_camera_model(1280, 720)
        self.assertEqual(model.width, 1280)
        self.assertEqual(model.height, 720)
        self.assertEqual(model.camera_matrix.shape, (3, 3))
        self.assertEqual(len(model.dist_coeffs), 5)
        self.assertIsNotNone(model.radar_to_camera)
        self.assertEqual(model.radar_to_camera.shape, (4, 4))

    def test_model_properties(self):
        model = get_default_camera_model(1280, 720)
        self.assertGreater(model.fx, 0)
        self.assertGreater(model.fy, 0)
        self.assertAlmostEqual(model.cx, 640.0)
        self.assertAlmostEqual(model.cy, 360.0)

    def test_to_dict_from_dict(self):
        model = get_default_camera_model(1280, 720)
        d = model.to_dict()
        self.assertIn("camera_matrix", d)
        self.assertIn("distortion_coefficients", d)
        self.assertIn("image_size", d)
        self.assertIn("radar_to_camera", d)

        model2 = CameraModel.from_dict(d)
        np.testing.assert_array_almost_equal(model.camera_matrix, model2.camera_matrix)
        np.testing.assert_array_almost_equal(model.dist_coeffs, model2.dist_coeffs)
        self.assertEqual(model.image_size, model2.image_size)

    def test_invalid_camera_matrix_shape(self):
        with self.assertRaises(ValueError):
            CameraModel(
                camera_matrix=np.eye(4),
                dist_coeffs=np.zeros(5),
                image_size=(1280, 720),
            )

    def test_dist_coeffs_padding(self):
        """Short dist_coeffs should be padded to length 5."""
        model = CameraModel(
            camera_matrix=np.eye(3),
            dist_coeffs=np.array([0.1, 0.2]),
            image_size=(1280, 720),
        )
        self.assertEqual(len(model.dist_coeffs), 5)
        self.assertAlmostEqual(model.dist_coeffs[0], 0.1)
        self.assertAlmostEqual(model.dist_coeffs[2], 0.0)


class TestCameraProjection(unittest.TestCase):
    """Test projection functions."""

    def test_project_points_identity(self):
        """Points at z=1 with identity transform should project via K."""
        K = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]], dtype=np.float64)
        D = np.zeros(5)
        pts = np.array([[0, 0, 1], [0.1, 0.2, 1]], dtype=np.float64)
        projected = project_points_pinhole(pts, K, D)
        self.assertEqual(projected.shape, (2, 2))
        # (0,0,1) should project to principal point (320, 240)
        np.testing.assert_array_almost_equal(projected[0], [320, 240], decimal=1)

    def test_project_empty_points(self):
        K = np.eye(3)
        D = np.zeros(5)
        pts = np.empty((0, 3))
        result = project_points_pinhole(pts, K, D)
        self.assertEqual(result.shape, (0, 2))

    def test_undistort_zero_distortion(self):
        """With zero distortion, undistort should return same points."""
        K = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]], dtype=np.float64)
        D = np.zeros(5)
        pts = np.array([[320, 240], [400, 300]], dtype=np.float64)
        result = undistort_points(pts, K, D)
        np.testing.assert_array_almost_equal(result, pts, decimal=1)

    def test_undistort_empty(self):
        K = np.eye(3)
        D = np.zeros(5)
        pts = np.empty((0, 2))
        result = undistort_points(pts, K, D)
        self.assertEqual(result.shape, (0, 2))


class TestCameraIO(unittest.TestCase):
    """Test camera parameter save/load."""

    def test_save_load_roundtrip(self):
        model = get_default_camera_model(1280, 720)
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as f:
            path = Path(f.name)

        try:
            save_camera_params(path, model)
            loaded = load_camera_params(path)
            np.testing.assert_array_almost_equal(
                model.camera_matrix, loaded.camera_matrix
            )
            np.testing.assert_array_almost_equal(
                model.dist_coeffs, loaded.dist_coeffs
            )
            self.assertEqual(model.image_size, loaded.image_size)
        finally:
            path.unlink(missing_ok=True)

    def test_load_nonexistent(self):
        with self.assertRaises(FileNotFoundError):
            load_camera_params(Path("/nonexistent/camera.json"))


if __name__ == "__main__":
    unittest.main()
