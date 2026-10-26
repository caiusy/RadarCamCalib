#!/usr/bin/env python3
"""Fit the ES-MEOT radar clock to the video frame clock."""

import argparse
from pathlib import Path

from calibration.time_sync import fit_time_sync
from data.es_meot_adapter import ESMeotDataset


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--camera-fps", type=float, default=None)
    parser.add_argument("--output", type=Path, default=Path("outputs/esmeot_time_sync.json"))
    args = parser.parse_args()
    fit = fit_time_sync(ESMeotDataset(args.dataset).timestamps(), args.camera_fps)
    fit.write_json(args.output)
    print(fit.to_dict())
    return 0 if fit.valid else 2


if __name__ == "__main__":
    raise SystemExit(main())
