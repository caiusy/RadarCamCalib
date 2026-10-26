"""Lightweight local web server for the RadarCamCalib workbench.

The calibration algorithms remain in the existing Python packages.  This module
only provides a small JSON API and serves the dependency-free browser client;
it deliberately uses the Python standard library so the remote WSL machine
does not need a desktop session or a Qt installation.
"""

from __future__ import annotations

import argparse
import base64
import json
import mimetypes
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, quote, unquote, urlparse

import numpy as np
import cv2

from calibration.calibration_session import CalibrationSession
from calibration.projector import project_radar_to_image
from data.workbench_dataset import WorkbenchDataset


WEB_ROOT = Path(__file__).resolve().parent / "web"


def _json_default(value: Any):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.integer, np.floating)):
        return value.item()
    raise TypeError(f"not JSON serializable: {type(value)!r}")


class CalibrationApp:
    """In-memory state for one browser session.

    The server is intentionally local and single-workspace.  A browser can
    load a dataset, select point pairs, compute a fit and export it without
    keeping a second copy of the dataset in the repository.
    """

    def __init__(self):
        self.lock = threading.RLock()
        self.dataset: WorkbenchDataset | None = None
        self.dataset_root: Path | None = None
        self.current_index = -1
        self.session = CalibrationSession()
        self.last_error: str | None = None

    def load(self, root: str | Path, sync_folder: str = "sync", camera_tracks: str | None = None,
             video: str | None = None) -> dict:
        root_path = Path(root).expanduser().resolve()
        dataset = WorkbenchDataset(root_path, sync_folder=sync_folder, camera_tracks=camera_tracks, video=video)
        with self.lock:
            self.dataset = dataset
            self.dataset_root = root_path
            self.current_index = dataset.current_index
            self.session.clear_points()
            self.last_error = None
        return self.snapshot()

    def _frame(self, index: int | None = None) -> dict:
        if self.dataset is None or self.dataset_root is None:
            return {"index": -1, "total": 0, "image": None, "radar": [], "vision": []}
        if index is not None:
            index = max(0, min(int(index), self.dataset.num_frames - 1))
            self.dataset.go_to(index)
            self.current_index = index
        frame = self.dataset.current_frame
        if frame is None:
            return {"index": -1, "total": 0, "image": None, "radar": [], "vision": []}
        radar_objects, vision_objects = self.dataset.objects(self.current_index)
        image_array = self.dataset.image(self.current_index)
        radar_items = []
        points = []
        if radar_objects:
            for obj in radar_objects:
                item = {
                    "id": obj.track_id,
                    "class": obj.obj_class,
                    "x": obj.x,
                    "y": obj.y,
                    "speed": obj.speed,
                    "speed_x": obj.speed_x,
                    "speed_y": obj.speed_y,
                }
                radar_items.append(item)
                points.append([obj.x, obj.y])
        projected = None
        if self.session.H is not None and points:
            projected = project_radar_to_image(np.asarray(points, dtype=float), self.session.H)
        for item, uv in zip(radar_items, projected if projected is not None else []):
            item["projected"] = [float(uv[0]), float(uv[1])]
        image_payload = None
        if image_array is not None:
            ok, encoded = cv2.imencode(".jpg", image_array, [cv2.IMWRITE_JPEG_QUALITY, 84])
            if ok:
                image_payload = {
                    "data": "data:image/jpeg;base64," + base64.b64encode(encoded).decode("ascii"),
                    "width": int(image_array.shape[1]), "height": int(image_array.shape[0]),
                    "timestamp": self.dataset.timestamp(self.current_index),
                }
        vision_items = [obj.to_dict() for obj in vision_objects]
        return {
            "index": self.current_index,
            "total": self.dataset.num_frames,
            "batch": self.current_index,
            "image": image_payload,
            "radar": radar_items,
            "vision": vision_items,
            "points": [p.to_dict() for p in self.session.matched_points],
        }

    def snapshot(self) -> dict:
        with self.lock:
            result = self.session.result
            return {
                "dataset_root": str(self.dataset_root) if self.dataset_root else None,
                "frames": self.dataset.num_frames if self.dataset else 0,
                "current": self.current_index,
                "points": [p.to_dict() for p in self.session.matched_points],
                "calibration": {
                    "ready": result is not None,
                    "mean_error": result.mean_error if result else None,
                    "max_error": result.max_error if result else None,
                    "inliers": int(result.inlier_mask.sum()) if result and result.inlier_mask is not None else 0,
                    "H": result.H.tolist() if result else None,
                },
                "frame": self._frame(),
                "error": self.last_error,
            }

    def add_point(self, payload: dict) -> dict:
        radar = payload.get("radar_point") or payload.get("radar") or {}
        image = payload.get("image_point") or payload.get("image") or {}
        required = (radar.get("x"), radar.get("y"), image.get("u"), image.get("v"))
        if any(value is None for value in required):
            raise ValueError("radar_point.x/y and image_point.u/v are required")
        with self.lock:
            self.session.add_point(float(radar["x"]), float(radar["y"]), float(image["u"]), float(image["v"]))
            self.session.result = None
            return self.snapshot()

    def replace_points(self, payload: dict | list) -> dict:
        raw = payload.get("matched_points", payload) if isinstance(payload, dict) else payload
        if not isinstance(raw, list):
            raise ValueError("matched_points must be a list")
        points = []
        from data.data_models import MatchedPoint
        for item in raw:
            if "radar_point" in item:
                points.append(MatchedPoint.from_dict(item))
            else:
                points.append(MatchedPoint(float(item["radar_x"]), float(item["radar_y"]), float(item["image_u"]), float(item["image_v"])))
        with self.lock:
            self.session.clear_points()
            self.session.set_points(points)
        return self.snapshot()

    def compute(self, threshold: float = 5.0) -> dict:
        with self.lock:
            self.session.ransac_threshold = float(threshold)
            result = self.session.compute()
            return self.snapshot()

    def calibration_json(self) -> dict:
        with self.lock:
            if self.session.result is None:
                raise ValueError("compute a calibration before exporting")
            result = self.session.result
            return {
                "version": "1.0",
                "calibration_type": "radar_to_camera_homography",
                "homography": result.H.tolist(),
                "inverse_homography": result.H_inv.tolist(),
                "matched_points": [p.to_dict() for p in result.matched_points],
                "mean_reprojection_error": round(result.mean_error, 4),
                "max_reprojection_error": round(result.max_error, 4),
            }

    def analyze(self, payload: dict) -> dict:
        with self.lock:
            if self.dataset is None:
                raise ValueError("load a dataset before analysis")
            from web_service import analyze as analyze_dataset
            return analyze_dataset(self.dataset, self.session, payload.get("start", 0),
                                   payload.get("end"), payload.get("step", 1))


class Handler(BaseHTTPRequestHandler):
    server_version = "RadarCamCalib/2.0"

    @property
    def app(self) -> CalibrationApp:
        return self.server.app  # type: ignore[attr-defined]

    def log_message(self, fmt, *args):
        # Keep the remote terminal readable; errors are returned as JSON.
        if self.command != "GET" or self.path.startswith("/api"):
            super().log_message(fmt, *args)

    def _send(self, status: int, body: bytes, content_type: str = "application/json; charset=utf-8"):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, value: Any, status: int = HTTPStatus.OK):
        self._send(status, json.dumps(value, ensure_ascii=False, default=_json_default).encode("utf-8"))

    def _error(self, message: str, status: int = HTTPStatus.BAD_REQUEST):
        self._json({"error": message}, status)

    def _body(self) -> dict | list:
        length = int(self.headers.get("Content-Length", "0"))
        if length > 2_000_000:
            raise ValueError("request body is too large")
        raw = self.rfile.read(length)
        return json.loads(raw or b"{}")

    def do_GET(self):  # noqa: N802
        parsed = urlparse(self.path)
        try:
            if parsed.path == "/" or parsed.path == "/index.html":
                body = (WEB_ROOT / "index.html").read_bytes()
                return self._send(HTTPStatus.OK, body, "text/html; charset=utf-8")
            if parsed.path == "/api/health":
                return self._json({"ok": True, "service": "radarcamcalib-web", "qt": False})
            if parsed.path == "/api/session":
                return self._json(self.app.snapshot())
            if parsed.path == "/api/frame":
                query = parse_qs(parsed.query)
                index = int(query["index"][0]) if query.get("index") else None
                with self.app.lock:
                    return self._json(self.app._frame(index))
            if parsed.path == "/api/export":
                return self._json(self.app.calibration_json())
            if parsed.path == "/media":
                return self._media(parse_qs(parsed.query).get("path", [""])[0])
            self._error("not found", HTTPStatus.NOT_FOUND)
        except (ValueError, FileNotFoundError) as exc:
            self._error(str(exc))
        except Exception as exc:  # pragma: no cover - defensive boundary
            self._error(f"server error: {exc}", HTTPStatus.INTERNAL_SERVER_ERROR)

    def _media(self, raw_path: str):
        if not self.app.dataset_root or not raw_path:
            return self._error("dataset is not loaded", HTTPStatus.NOT_FOUND)
        root = self.app.dataset_root.resolve()
        target = (root / unquote(raw_path)).resolve()
        if root != target and root not in target.parents:
            return self._error("media path is outside dataset", HTTPStatus.FORBIDDEN)
        if not target.is_file():
            return self._error("media file not found", HTTPStatus.NOT_FOUND)
        content_type = mimetypes.guess_type(target.name)[0] or "application/octet-stream"
        return self._send(HTTPStatus.OK, target.read_bytes(), content_type)

    def do_POST(self):  # noqa: N802
        parsed = urlparse(self.path)
        try:
            payload = self._body()
            if parsed.path == "/api/load":
                return self._json(self.app.load(payload["dataset_root"], payload.get("sync_folder", "sync"),
                                                payload.get("camera_tracks"), payload.get("video")))
            if parsed.path == "/api/points":
                return self._json(self.app.add_point(payload))
            if parsed.path == "/api/points/replace":
                return self._json(self.app.replace_points(payload))
            if parsed.path == "/api/points/clear":
                with self.app.lock:
                    self.app.session.clear_points()
                return self._json(self.app.snapshot())
            if parsed.path == "/api/calibrate":
                return self._json(self.app.compute(payload.get("ransac_threshold", 5.0)))
            if parsed.path == "/api/analyze":
                return self._json(self.app.analyze(payload))
            self._error("not found", HTTPStatus.NOT_FOUND)
        except (ValueError, KeyError, FileNotFoundError) as exc:
            self._error(str(exc))
        except Exception as exc:  # pragma: no cover - defensive boundary
            self._error(f"server error: {exc}", HTTPStatus.INTERNAL_SERVER_ERROR)


def create_server(host: str = "127.0.0.1", port: int = 8765) -> ThreadingHTTPServer:
    server = ThreadingHTTPServer((host, int(port)), Handler)
    server.app = CalibrationApp()  # type: ignore[attr-defined]
    return server


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="RadarCamCalib lightweight web workbench")
    parser.add_argument("--host", default="127.0.0.1", help="listen address; use 0.0.0.0 only behind a firewall")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--dataset-root", default=None)
    parser.add_argument("--sync-folder", default="sync")
    parser.add_argument("--camera-tracks", default=None, help="trusted ES-MEOT ByteTrack object-array NPY")
    parser.add_argument("--video", default=None, help="ES-MEOT video used for lazy frame preview")
    args = parser.parse_args(argv)
    server = create_server(args.host, args.port)
    if args.dataset_root:
        try:
            server.app.load(args.dataset_root, args.sync_folder, args.camera_tracks, args.video)  # type: ignore[attr-defined]
        except Exception as exc:
            print(f"[WARN] dataset not loaded: {exc}")
    print(f"RadarCamCalib web workbench: http://{args.host}:{args.port}")
    print("No PyQt desktop dependency; access remotely with an SSH tunnel.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
