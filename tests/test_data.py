"""
test_data.py - Unit tests for data loading modules.
"""

import json
import tempfile
import sys
import os
import unittest
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from data.data_models import RadarObject, RadarFrame, MatchedPoint
from data.radar_loader import (
    normalize_coord, normalize_class, parse_radar_objects, load_radar_json
)
from data.sync_loader import load_sync_file


class TestNormalizeCoord(unittest.TestCase):
    """Test coordinate normalization."""

    def test_flat_list(self):
        self.assertEqual(normalize_coord([1.5, 2.5]), (1.5, 2.5))

    def test_nested_list(self):
        self.assertEqual(normalize_coord([[3.0, 4.0]]), (3.0, 4.0))

    def test_tuple(self):
        self.assertEqual(normalize_coord((1.0, 2.0)), (1.0, 2.0))

    def test_invalid(self):
        with self.assertRaises(ValueError):
            normalize_coord("invalid")


class TestNormalizeClass(unittest.TestCase):
    """Test Chinese to English class mapping."""

    def test_chinese_vehicle(self):
        self.assertEqual(normalize_class("机动车"), "vehicle")
        self.assertEqual(normalize_class("汽车"), "vehicle")

    def test_chinese_pedestrian(self):
        self.assertEqual(normalize_class("行人"), "pedestrian")

    def test_chinese_cyclist(self):
        self.assertEqual(normalize_class("非机动车"), "cyclist")

    def test_english_passthrough(self):
        self.assertEqual(normalize_class("vehicle"), "vehicle")

    def test_unknown(self):
        self.assertEqual(normalize_class("unknown"), "unknown")

    def test_unmapped(self):
        self.assertEqual(normalize_class("xyz_custom"), "xyz_custom")


class TestParseRadarObjects(unittest.TestCase):
    """Test radar object parsing."""

    def test_basic_object(self):
        data = [
            {
                "id": 1,
                "class": "机动车",
                "coord": [3.5, 42.0],
                "props": {"speed": 15.0, "speedX": 0.1, "speedY": 14.9},
            }
        ]
        objs = parse_radar_objects(data)
        self.assertEqual(len(objs), 1)
        self.assertEqual(objs[0].track_id, 1)
        self.assertEqual(objs[0].obj_class, "vehicle")
        self.assertAlmostEqual(objs[0].x, 3.5)
        self.assertAlmostEqual(objs[0].y, 42.0)
        self.assertAlmostEqual(objs[0].speed, 15.0)

    def test_nested_coord(self):
        data = [{"id": 2, "class": "unknown", "coord": [[1.0, 2.0]], "props": {}}]
        objs = parse_radar_objects(data)
        self.assertEqual(len(objs), 1)
        self.assertAlmostEqual(objs[0].x, 1.0)
        self.assertAlmostEqual(objs[0].y, 2.0)

    def test_missing_props(self):
        data = [{"id": 3, "coord": [5.0, 10.0]}]
        objs = parse_radar_objects(data)
        self.assertEqual(len(objs), 1)
        self.assertIsNone(objs[0].speed)

    def test_empty_list(self):
        objs = parse_radar_objects([])
        self.assertEqual(len(objs), 0)


class TestMatchedPoint(unittest.TestCase):
    """Test MatchedPoint serialization."""

    def test_to_dict(self):
        mp = MatchedPoint(radar_x=1.0, radar_y=2.0, image_u=100, image_v=200)
        d = mp.to_dict()
        self.assertEqual(d["radar_point"]["x"], 1.0)
        self.assertEqual(d["image_point"]["u"], 100)

    def test_from_dict(self):
        d = {
            "radar_point": {"x": 3.0, "y": 4.0},
            "image_point": {"u": 300, "v": 400},
        }
        mp = MatchedPoint.from_dict(d)
        self.assertEqual(mp.radar_x, 3.0)
        self.assertEqual(mp.image_v, 400)

    def test_roundtrip(self):
        mp = MatchedPoint(1.5, 2.5, 150, 250, reprojection_error=1.23, inlier=True)
        mp2 = MatchedPoint.from_dict(mp.to_dict())
        self.assertAlmostEqual(mp2.radar_x, mp.radar_x)
        self.assertAlmostEqual(mp2.reprojection_error, 1.23, places=3)
        self.assertTrue(mp2.inlier)


class TestLoadRadarJson(unittest.TestCase):
    """Test loading radar JSON files."""

    def test_load_file(self):
        data = {
            "baseId": 2,
            "filename": "test.json",
            "objects": [
                {"id": 1, "class": "vehicle", "coord": [1.0, 2.0], "props": {}},
                {"id": 2, "class": "行人", "coord": [3.0, 4.0], "props": {}},
            ],
        }
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".json", delete=False
        ) as f:
            json.dump(data, f)
            f.flush()
            frame = load_radar_json(f.name)

        self.assertEqual(frame.base_id, 2)
        self.assertEqual(len(frame.objects), 2)
        self.assertEqual(frame.objects[0].obj_class, "vehicle")
        self.assertEqual(frame.objects[1].obj_class, "pedestrian")
        os.unlink(f.name)


class TestLoadSyncFile(unittest.TestCase):
    """Test loading sync JSON files."""

    def test_load_sync(self):
        data = {
            "sync_batch": 0,
            "batch_size": 2,
            "sensors": [
                {"frame_index": 100, "sensor_id": 1, "time_stamp": 123456},
                {"frame_index": 50, "sensor_id": 2, "time_stamp": 123456},
            ],
            "sensor_data": [
                {
                    "channel_id": 0,
                    "sensor_id": 1,
                    "data_type": "image",
                    "data_path": "image/test.jpg",
                },
                {
                    "channel_id": 1,
                    "sensor_id": 2,
                    "data_type": "radar",
                    "data_path": "radar/test.json",
                },
            ],
        }
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".sync.json", delete=False
        ) as f:
            json.dump(data, f)
            f.flush()
            frame = load_sync_file(f.name)

        self.assertEqual(frame.sync_batch, 0)
        self.assertEqual(len(frame.sensors), 2)
        self.assertEqual(len(frame.sensor_data), 2)
        os.unlink(f.name)


if __name__ == "__main__":
    unittest.main()
