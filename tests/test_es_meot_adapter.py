import json

import numpy as np
import pytest

from calibration.time_sync import fit_time_sync
from data.es_meot_adapter import ESMeotDataset


def test_es_meot_reader_uses_xz_ground_plane(tmp_path):
    np.save(tmp_path / "radarTrack.npy", np.array([
        {"time": 10.0, "data": [{"id": 3, "x": 1, "y": 2, "z": 4, "vx": .5, "vy": .2, "vz": .1}]}
    ], dtype=object), allow_pickle=True)
    (tmp_path / "config").write_text(
        "[cameraIntrinsic]\n"
        "cameramatrix=[[1,0,0],[0,1,0],[0,0,1]]\n"
        "distcoeffs=[0,0,0,0]\n"
        "[radar2camera]\n"
        "rt=[[1,0,0,0],[0,1,0,0],[0,0,1,0]]\n",
        encoding="utf-8",
    )
    frame = next(ESMeotDataset(tmp_path).iter_tracks())[0]
    assert frame.ground_point == (1.0, 4.0)
    assert ESMeotDataset(tmp_path).camera_parameters()["coordinate_convention"]["ground_plane"] == ["x", "z"]


def test_time_sync_fit_is_stable():
    times = 100.0 + np.arange(20) * .1
    fit = fit_time_sync(times, camera_fps=20)
    assert fit.valid
    assert fit.clock_scale == pytest.approx(2.0)
    assert fit.radar_period_seconds == pytest.approx(.1)
