"""Adapters used by the web workbench; datasets stay on their host machine."""
from pathlib import Path

import cv2
import numpy as np

from data.data_models import RadarObject
from data.sync_dataset import SyncDataset
from vision.vision_models import VisionObject


class WorkbenchDataset:
    def __init__(self, root, sync_folder="sync", camera_tracks=None, video=None):
        self.root = Path(root).expanduser().resolve()
        self.native = (self.root / "radarTrack.npy").is_file() and not (self.root / sync_folder).is_dir()
        self.video = None
        self.current_index = 0
        self.radar_frames = None
        self.camera_frames = None
        self.camera_parameters = None
        if self.native:
            # Object arrays are only read from the user's trusted dataset paths.
            from data.es_meot_adapter import ESMeotDataset
            native = ESMeotDataset(self.root)
            self.radar_frames = native._array()
            if camera_tracks:
                self.camera_frames = np.load(Path(camera_tracks).expanduser(), allow_pickle=True)
            if video:
                self.video = cv2.VideoCapture(str(Path(video).expanduser()))
                if not self.video.isOpened():
                    self.video.release()
                    raise ValueError("Cannot open video")
            self.num_frames = len(self.radar_frames)
            try:
                self.camera_parameters = native.camera_parameters()
            except FileNotFoundError:
                pass
        else:
            self.sync = SyncDataset()
            self.sync.load(self.root, sync_folder)
            self.num_frames = self.sync.num_frames
        if not self.num_frames:
            self.close()
            raise ValueError("Dataset contains no usable frames")

    def close(self):
        if self.video is not None:
            self.video.release()

    @property
    def current_frame(self):
        """Compatibility marker for the browser state; data is loaded lazily."""
        return self.current_index if self.num_frames else None

    def go_to(self, index):
        if not 0 <= int(index) < self.num_frames:
            return False
        self.current_index = int(index)
        if not self.native:
            self.sync.go_to(self.current_index)
        return True

    def objects(self, index):
        if not 0 <= index < self.num_frames:
            raise ValueError("Frame index is outside dataset")
        if not self.native:
            self.sync.go_to(index)
            radar, vision = self.sync.get_radar(), self.sync.get_vision()
            return radar.objects if radar else [], vision.objects if vision else []
        frame = self.radar_frames[index]
        radar = [RadarObject(
            track_id=obj["id"], obj_class="vehicle", x=float(obj["x"]), y=float(obj["z"]),
            speed_x=float(obj.get("vx", 0)), speed_y=float(obj.get("vz", 0)),
        ) for obj in frame.get("data", [])]
        vision = []
        if self.camera_frames is not None and index < len(self.camera_frames):
            frame = self.camera_frames[index]
            # Official ES-MEOT ByteTrack arrays use a dict with boxes/ids;
            # older exports may contain one dict per detection.
            if isinstance(frame, dict) and "boxes" in frame:
                boxes = np.asarray(frame.get("boxes", []), dtype=float).reshape(-1, 4)
                ids = np.asarray(frame.get("ids", []), dtype=object)
                scores = np.asarray(frame.get("scores", []), dtype=float)
                for i, box in enumerate(boxes):
                    vision.append(VisionObject(i, int(ids[i]) if i < len(ids) else i,
                                               "vehicle", box.tolist(),
                                               float(scores[i]) if i < len(scores) else 0.0))
            else:
                for i, obj in enumerate(frame):
                    bbox = np.asarray(obj["bbox"], dtype=float).reshape(4).tolist()
                    vision.append(VisionObject(i, int(obj.get("id", i)), "vehicle", bbox,
                                               float(obj.get("score", 0.0))))
        return radar, vision

    def image(self, index):
        if not 0 <= index < self.num_frames:
            raise ValueError("Frame index is outside dataset")
        if self.native:
            if self.video is None:
                return None
            self.video.set(cv2.CAP_PROP_POS_FRAMES, index)
            ok, image = self.video.read()
            return image if ok else None
        self.sync.go_to(index)
        frame = self.sync.get_image()
        return frame.image if frame else None

    def timestamp(self, index):
        if self.native:
            return float(self.radar_frames[index]["time"])
        self.sync.go_to(index)
        frame = self.sync.current_frame
        return frame.sensors[0].time_stamp if frame.sensors else None
