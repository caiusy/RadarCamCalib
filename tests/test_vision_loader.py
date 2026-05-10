"""
test_vision_loader.py - Tests for vision detection loading and parsing.

Tests VisionObject, VisionFrame, bbox utilities, and JSON loading.
"""

import unittest
import json
import tempfile
import numpy as np
from pathlib import Path

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from vision.vision_models import (
    VisionObject,
    VisionFrame,
    bbox_bottom_center,
    bbox_area,
    expand_bbox,
    extract_vision_bottom_centers,
)
from vision.vision_loader import load_vision_json, parse_vision_objects


class TestVisionObject(unittest.TestCase):
    """Test VisionObject dataclass."""

    def test_create_object(self):
        obj = VisionObject(
            id=101, track_id=21, obj_class="vehicle",
            bbox=[720.5, 310.2, 810.8, 390.6], score=0.92,
        )
        self.assertEqual(obj.id, 101)
        self.assertEqual(obj.track_id, 21)
        self.assertEqual(obj.obj_class, "vehicle")
        self.assertAlmostEqual(obj.score, 0.92)

    def test_bbox_properties(self):
        obj = VisionObject(
            id=1, track_id=1, obj_class="vehicle",
            bbox=[100, 200, 200, 300], score=0.9,
        )
        self.assertAlmostEqual(obj.x1, 100)
        self.assertAlmostEqual(obj.y1, 200)
        self.assertAlmostEqual(obj.x2, 200)
        self.assertAlmostEqual(obj.y2, 300)
        self.assertAlmostEqual(obj.width, 100)
        self.assertAlmostEqual(obj.height, 100)

    def test_center(self):
        obj = VisionObject(
            id=1, track_id=1, obj_class="vehicle",
            bbox=[100, 200, 200, 300], score=0.9,
        )
        cx, cy = obj.center
        self.assertAlmostEqual(cx, 150)
        self.assertAlmostEqual(cy, 250)

    def test_bottom_center(self):
        obj = VisionObject(
            id=1, track_id=1, obj_class="vehicle",
            bbox=[100, 200, 200, 300], score=0.9,
        )
        bx, by = obj.bottom_center
        self.assertAlmostEqual(bx, 150)
        self.assertAlmostEqual(by, 300)

    def test_to_dict_from_dict(self):
        obj = VisionObject(
            id=101, track_id=21, obj_class="vehicle",
            bbox=[720.5, 310.2, 810.8, 390.6], score=0.92,
        )
        d = obj.to_dict()
        obj2 = VisionObject.from_dict(d)
        self.assertEqual(obj.id, obj2.id)
        self.assertEqual(obj.track_id, obj2.track_id)
        self.assertEqual(obj.obj_class, obj2.obj_class)
        self.assertEqual(obj.bbox, obj2.bbox)
        self.assertAlmostEqual(obj.score, obj2.score)


class TestBboxUtils(unittest.TestCase):
    """Test bbox utility functions."""

    def test_bbox_bottom_center(self):
        u, v = bbox_bottom_center([100, 200, 300, 400])
        self.assertAlmostEqual(u, 200)
        self.assertAlmostEqual(v, 400)

    def test_bbox_area(self):
        area = bbox_area([100, 200, 300, 400])
        self.assertAlmostEqual(area, 40000)

    def test_bbox_area_zero(self):
        area = bbox_area([100, 200, 100, 200])
        self.assertAlmostEqual(area, 0)

    def test_expand_bbox(self):
        expanded = expand_bbox([100, 200, 300, 400], margin=10)
        self.assertEqual(expanded, [90, 190, 310, 410])

    def test_expand_bbox_clipped(self):
        expanded = expand_bbox([5, 5, 100, 100], margin=10,
                               image_width=110, image_height=110)
        self.assertAlmostEqual(expanded[0], 0)  # Clipped at 0
        self.assertAlmostEqual(expanded[1], 0)
        self.assertAlmostEqual(expanded[2], 110)
        self.assertAlmostEqual(expanded[3], 110)

    def test_extract_vision_bottom_centers(self):
        objs = [
            VisionObject(1, 1, "vehicle", [100, 200, 200, 300], 0.9),
            VisionObject(2, 2, "vehicle", [300, 400, 500, 600], 0.8),
        ]
        centers = extract_vision_bottom_centers(objs)
        self.assertEqual(centers.shape, (2, 2))
        np.testing.assert_array_almost_equal(centers[0], [150, 300])
        np.testing.assert_array_almost_equal(centers[1], [400, 600])

    def test_extract_empty(self):
        centers = extract_vision_bottom_centers([])
        self.assertEqual(centers.shape, (0, 2))


class TestVisionLoader(unittest.TestCase):
    """Test vision JSON loading."""

    def test_load_vision_json(self):
        data = {
            "sensor_id": 1,
            "frame_index": 0,
            "time_stamp": 1767954310202,
            "image_path": "image/000000.jpg",
            "objects": [
                {
                    "id": 101,
                    "track_id": 21,
                    "class": "vehicle",
                    "bbox": [720.5, 310.2, 810.8, 390.6],
                    "score": 0.92,
                },
                {
                    "id": 102,
                    "track_id": 22,
                    "class": "pedestrian",
                    "bbox": [100, 200, 130, 280],
                    "score": 0.85,
                },
            ],
        }

        with tempfile.NamedTemporaryFile(
            suffix=".vision.json", mode="w", delete=False
        ) as f:
            json.dump(data, f)
            path = Path(f.name)

        try:
            frame = load_vision_json(path)
            self.assertEqual(frame.sensor_id, 1)
            self.assertEqual(frame.frame_index, 0)
            self.assertEqual(len(frame.objects), 2)
            self.assertEqual(frame.objects[0].obj_class, "vehicle")
            self.assertEqual(frame.objects[1].obj_class, "pedestrian")
        finally:
            path.unlink(missing_ok=True)

    def test_load_nonexistent(self):
        with self.assertRaises(FileNotFoundError):
            load_vision_json(Path("/nonexistent/vision.json"))

    def test_parse_invalid_bbox(self):
        """Invalid bbox should be skipped."""
        objs = [{"id": 1, "track_id": 1, "class": "v", "bbox": [1, 2, 3], "score": 0.5}]
        result = parse_vision_objects(objs)
        self.assertEqual(len(result), 0)


if __name__ == "__main__":
    unittest.main()
