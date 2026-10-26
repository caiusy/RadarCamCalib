#!/usr/bin/env python3
"""Fit native ES-MEOT radar/box correspondences; never consumes annotation."""
import argparse
import json
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from calibration.esmeot_native import fit_dataset


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('dataset_root', type=Path)
    p.add_argument('--camera-tracks', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--train-fraction', type=float, default=.5)
    p.add_argument('--step', type=int, default=3)
    args = p.parse_args()
    if not 0 < args.train_fraction < 1 or args.step < 1:
        p.error('train-fraction must be in (0,1) and step must be positive')
    result = fit_dataset(args.dataset_root, args.camera_tracks, args.output, args.train_fraction, args.step)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
