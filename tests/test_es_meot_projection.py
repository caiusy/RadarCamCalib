import numpy as np

from data.es_meot_adapter import ESMeotDataset


def test_project_radar_xyz_uses_rt_and_distortion(tmp_path):
    np.save(tmp_path / "radarTrack.npy", np.array([], dtype=object), allow_pickle=True)
    (tmp_path / "config").write_text(
        "[cameraIntrinsic]\n"
        "cameramatrix=[[100,0,50],[0,100,50],[0,0,1]]\n"
        "distcoeffs=[0,0,0,0]\n"
        "[radar2camera]\n"
        "rt=[[1,0,0,0],[0,1,0,0],[0,0,1,0]]\n",
        encoding="utf-8",
    )
    projected = ESMeotDataset(tmp_path).project_radar_to_image(np.array([[1., 2., 10.], [0., 0., 0.]]))
    assert np.allclose(projected[0], [60., 70.])
    assert np.isnan(projected[1]).all()
