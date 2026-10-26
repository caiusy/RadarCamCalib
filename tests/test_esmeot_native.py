import numpy as np

from calibration.esmeot_native import fit_native_pairs


def test_native_pair_fit_reports_unseen_id_holdout():
    pairs = []
    for frame in range(20):
        for lane in range(2):
            x, z = float(lane * 3 - 1), float(10 + frame)
            pairs.append({"frame": frame, "camera_id": frame * 2 + lane,
                          "radar_id": frame * 2 + lane,
                          "radar": [x, z], "image": [100 + 20 * x, 200 + 5 * z],
                          "seed_cost": 0.01})
    result = fit_native_pairs(pairs, train_end_frame=10, ransac_threshold=1.0)
    assert result["fit"]["holdout_camera_ids"] == 20
    assert result["fit"]["holdout_pixels"]["median"] < 1e-3
    assert result["annotation_used"] is False
