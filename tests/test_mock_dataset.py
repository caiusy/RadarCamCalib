"""
test_mock_dataset.py - Tests for mock dataset generation.

Generates a small mock dataset and verifies all output files
exist with correct structure.
"""

import unittest
import json
import tempfile
import shutil
from pathlib import Path

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from tools.generate_mock_dataset import generate_mock_dataset


class TestMockDataset(unittest.TestCase):
    """Test mock dataset generation."""

    @classmethod
    def setUpClass(cls):
        """Generate a small mock dataset for testing."""
        cls.test_dir = Path(tempfile.mkdtemp()) / "test_mock"
        generate_mock_dataset(
            str(cls.test_dir),
            num_frames=10,
            image_width=640,
            image_height=480,
        )

    @classmethod
    def tearDownClass(cls):
        """Clean up test directory."""
        if cls.test_dir.parent.exists():
            shutil.rmtree(cls.test_dir.parent, ignore_errors=True)

    def test_directories_exist(self):
        self.assertTrue((self.test_dir / "sync").exists())
        self.assertTrue((self.test_dir / "image").exists())
        self.assertTrue((self.test_dir / "radar").exists())
        self.assertTrue((self.test_dir / "vision").exists())
        self.assertTrue((self.test_dir / "association_gt").exists())

    def test_sync_files_count(self):
        sync_files = list((self.test_dir / "sync").glob("*.sync.json"))
        self.assertEqual(len(sync_files), 10)

    def test_image_files_exist(self):
        image_files = list((self.test_dir / "image").glob("*.jpg"))
        self.assertEqual(len(image_files), 10)

    def test_radar_files_count(self):
        radar_files = list((self.test_dir / "radar").glob("*.radar.json"))
        self.assertEqual(len(radar_files), 10)

    def test_vision_files_count(self):
        vision_files = list((self.test_dir / "vision").glob("*.vision.json"))
        self.assertEqual(len(vision_files), 10)

    def test_association_gt_files(self):
        gt_files = list((self.test_dir / "association_gt").glob("*.association_gt.json"))
        self.assertEqual(len(gt_files), 10)

    def test_ground_truth_calibration(self):
        path = self.test_dir / "ground_truth_calibration.json"
        self.assertTrue(path.exists())
        with open(path) as f:
            data = json.load(f)
        self.assertIn("homography", data)
        self.assertIn("inverse_homography", data)
        self.assertIn("camera_parameters", data)
        self.assertEqual(data["version"], "2.0")

    def test_camera_params_in_calibration(self):
        with open(self.test_dir / "ground_truth_calibration.json") as f:
            data = json.load(f)
        cam = data["camera_parameters"]
        self.assertIn("camera_matrix", cam)
        self.assertIn("distortion_coefficients", cam)
        self.assertIn("image_size", cam)
        self.assertEqual(cam["image_size"]["width"], 640)
        self.assertEqual(cam["image_size"]["height"], 480)

    def test_mock_matched_points(self):
        path = self.test_dir / "mock_matched_points.json"
        self.assertTrue(path.exists())
        with open(path) as f:
            data = json.load(f)
        self.assertIn("matched_points", data)

    def test_dataset_config(self):
        path = self.test_dir / "mock_dataset_config.json"
        self.assertTrue(path.exists())
        with open(path) as f:
            data = json.load(f)
        self.assertEqual(data["num_frames"], 10)
        self.assertIn("radar_detection_rates", data)
        self.assertIn("vision_detection_rates", data)
        self.assertIn("blind_zone", data)

    def test_sync_json_structure(self):
        """Verify sync JSON has image + radar + vision_detection."""
        sync_files = sorted((self.test_dir / "sync").glob("*.sync.json"))
        with open(sync_files[0]) as f:
            data = json.load(f)
        self.assertIn("sensor_data", data)
        types = [sd["data_type"] for sd in data["sensor_data"]]
        self.assertIn("image", types)
        self.assertIn("radar", types)
        self.assertIn("vision_detection", types)

    def test_vision_json_structure(self):
        """Verify vision JSON has correct format."""
        vision_files = sorted((self.test_dir / "vision").glob("*.vision.json"))
        with open(vision_files[0]) as f:
            data = json.load(f)
        self.assertIn("sensor_id", data)
        self.assertIn("frame_index", data)
        self.assertIn("time_stamp", data)
        self.assertIn("image_path", data)
        self.assertIn("objects", data)

    def test_radar_no_internal_fields(self):
        """Verify radar JSON doesn't contain internal _gt_id fields."""
        radar_files = sorted((self.test_dir / "radar").glob("*.radar.json"))
        with open(radar_files[0]) as f:
            data = json.load(f)
        for obj in data.get("objects", []):
            self.assertNotIn("_gt_id", obj)

    def test_association_gt_structure(self):
        """Verify association GT has correct format."""
        gt_files = sorted(
            (self.test_dir / "association_gt").glob("*.association_gt.json")
        )
        with open(gt_files[0]) as f:
            data = json.load(f)
        self.assertIn("matches", data)
        self.assertIn("num_radar", data)
        self.assertIn("num_vision", data)


if __name__ == "__main__":
    unittest.main()
