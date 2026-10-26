import numpy as np
import pytest

from calibration.esmeot_geometry import homography_from_esmeot_config, project_radar_ground_points


def test_derive_ground_plane_homography(tmp_path):
    (tmp_path / "config").write_text(
        "[cameraIntrinsic]\n"
        "cameramatrix=[[100,0,50],[0,100,50],[0,0,1]]\n"
        "distcoeffs=[0,0,0,0]\n"
        "[radar2camera]\n"
        "rt=[[1,0,0,0],[0,1,0,0],[0,0,1,0]]\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="ground plane"):
        homography_from_esmeot_config(tmp_path / "config")
    H, H_inv = homography_from_esmeot_config(tmp_path / "config", ground_height=2.)
    projected = project_radar_ground_points(np.array([[1., 10.]]), H)
    assert np.isfinite(H).all() and np.isfinite(H_inv).all()
    assert np.allclose(projected[0], [60., 70.])
