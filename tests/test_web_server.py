"""Smoke tests for the dependency-free calibration workbench API."""

import json
import threading
import urllib.request

from server import CalibrationApp, create_server


def test_app_point_pairs_compute_and_export():
    app = CalibrationApp()
    pairs = [
        ((0, 10), (100, 400)), ((1, 10), (110, 400)),
        ((0, 20), (100, 300)), ((1, 20), (110, 300)),
    ]
    for radar, image in pairs:
        app.add_point({"radar_point": {"x": radar[0], "y": radar[1]},
                       "image_point": {"u": image[0], "v": image[1]}})
    snapshot = app.compute()
    assert snapshot["calibration"]["ready"]
    assert snapshot["calibration"]["inliers"] == 4
    assert app.calibration_json()["calibration_type"] == "radar_to_camera_homography"


def test_http_health_endpoint():
    server = create_server("127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{server.server_port}/api/health") as response:
            payload = json.load(response)
        assert payload == {"ok": True, "service": "radarcamcalib-web", "qt": False}
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
