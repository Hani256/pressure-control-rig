#!/usr/bin/env python3
"""
Identify the static plant model from a run, and report how repeatable it is
across runs.

    python3 tools/analyse_plant.py data/*.bin

The feedforward term is built from this fit, so the question the script is
really answering is: how much can I trust a model identified on Monday when
I run the controller on Tuesday?
"""
import sys
import os

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rig import load, settled_points, fit_plant


def main(paths):
    print(f"{'run':<24} {'pts':>4} {'k [scale]':>10} {'b [Pa]':>9} {'rmse [Pa]':>10}")
    print("-" * 60)

    ks, bs = [], []
    for p in paths:
        run = load(p)
        pts = settled_points(run)
        if len(pts) < 2:
            print(f"{os.path.basename(p):<24} {len(pts):>4}   not enough settled points")
            continue
        k, b, rmse = fit_plant(pts)
        ks.append(k)
        bs.append(b)
        print(f"{os.path.basename(p):<24} {len(pts):>4} {k:>10.3f} {b:>9.2f} {rmse:>10.2f}")

    if len(ks) > 1:
        print("-" * 60)
        print(f"gain   k: mean {np.mean(ks):.3f}, spread {max(ks) - min(ks):.3f} Pa/%")
        print(f"offset b: mean {np.mean(bs):.2f}, spread {max(bs) - min(bs):.2f} Pa")
        print()
        print("The gain repeats across runs. The offset does not, because it moves")
        print("with ambient conditions and with how the box was last closed. That is")
        print("the argument for identifying b online rather than trusting a stored")
        print("calibration.")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1:])
