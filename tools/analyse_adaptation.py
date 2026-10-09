#!/usr/bin/env python3
"""
Does the online identifier actually converge towards the real plant?

    python3 tools/analyse_adaptation.py data/pair1_adaptive.bin ...

Method, and the reason it is written this way:

The firmware starts from a deliberately wrong model and corrects it while
running. To score that honestly you have to compare the model it learned
against the plant *of the same run*, not against a plant measured on a
different day. An earlier version of this analysis compared across runs and
produced a confident, wrong conclusion, because the rig itself drifts between
sessions. Fitting the reference plant from the same log removes that mistake.

For each run:
  1. fit the true static plant from the settled points of that run
  2. take the identifier's (k, b) at the start and at the end of the run
  3. evaluate both models at the duty actually used, and report how far each
     is from the truth in pascals
  4. report the fraction of the initial model error that was removed
"""
import sys
import os

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rig import load, settled_points, fit_plant, ff_table


def model_error_pa(k_hat, b_hat, k_true, b_true, duties):
    """Mean absolute pressure error of a model over the duty range used."""
    x = ff_table(duties)
    return float(np.mean(np.abs((k_hat * x + b_hat) - (k_true * x + b_true))))


def main(paths):
    print(f"{'run':<24} {'k true':>8} {'k learned':>10} "
          f"{'err start':>10} {'err end':>9} {'closed':>8}")
    print("-" * 74)

    closed_all = []
    for p in paths:
        run = load(p)
        pts = settled_points(run)
        if len(pts) < 3:
            print(f"{os.path.basename(p):<24}  not enough settled points")
            continue

        k_true, b_true, _ = fit_plant(pts)

        # duty range the controller actually visited, which is where the model
        # has to be right. Being right at duties the system never reaches is
        # worth nothing.
        duties = np.linspace(run.duty.min(), run.duty.max(), 50)

        n = len(run.t)
        head = slice(0, max(5, n // 50))        # first 2% of the run
        tail = slice(n - max(5, n // 50), n)    # last 2%

        k0, b0 = run.k[head].mean(), run.b[head].mean()
        k1, b1 = run.k[tail].mean(), run.b[tail].mean()

        e0 = model_error_pa(k0, b0, k_true, b_true, duties)
        e1 = model_error_pa(k1, b1, k_true, b_true, duties)
        closed = (e0 - e1) / e0 if e0 > 0 else float("nan")
        closed_all.append(closed)

        print(f"{os.path.basename(p):<24} {k_true:>8.3f} {k1:>10.3f} "
              f"{e0:>9.2f}P {e1:>8.2f}P {closed * 100:>7.0f}%")

    if closed_all:
        print("-" * 74)
        print(f"error removed, mean over {len(closed_all)} runs: "
              f"{np.mean(closed_all) * 100:.0f}%  "
              f"(range {min(closed_all) * 100:.0f} to {max(closed_all) * 100:.0f}%)")
        print()
        print("Reported as a range over independent runs rather than as one number.")
        print("A single before-and-after pair on this rig is not reproducible enough")
        print("to support a claim on its own: two identical runs differed by a factor")
        print("of three and a half on the first metric I tried.")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1:])
