#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

from calibration.esmeot_geometry import homography_from_esmeot_config


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("config", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ground-height", type=float, required=True,
                        help="measured road-plane radar y coordinate in metres")
    args = parser.parse_args()
    H, H_inv = homography_from_esmeot_config(args.config, args.ground_height)
    payload = {
        "version": "2.0",
        "calibration_type": "esmeot_physical_ground_plane",
        "homography": H.tolist(),
        "inverse_homography": H_inv.tolist(),
        "coordinate_system": {"radar_ground_plane": ["x", "z"], "radar_vertical": "y"},
        "ground_height": args.ground_height,
        "pixel_space": "undistorted",
        "source_config": str(args.config),
        "validation_status": "derived_from_intrinsics_and_extrinsics; requires heldout validation",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(json.dumps(payload, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
