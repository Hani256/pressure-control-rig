#!/usr/bin/env python3
"""
What does it cost to run the loop without a usable sensor?

    python3 tools/analyse_blind.py data/hybrid_run.bin

Over part of its range the pressure measurement on this rig is not usable, so
the controller has to run open loop through that band and reconnect on the far
side. This script measures three things:

  how long each blind episode lasted
  how far the controller's estimate had drifted by the time it could see again
  the size of the step when the loop reconnected

The last one is the one a user would feel. The output includes a drift rate in
pascals per second, which turns the whole question into a design rule: the
error you are willing to accept, divided by the drift rate, is the longest you
can afford to be blind.
"""
import sys
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rig import load, blind_episodes


def main(path, fig_out=None):
    run = load(path)
    eps = blind_episodes(run)

    print(f"{os.path.basename(path)}: {len(run.t)} samples, "
          f"{run.t[-1]:.0f} s at {run.rate_hz:.1f} Hz")
    print(f"sensor unusable for {100 * (run.valid == 0).mean():.0f}% of the run, "
          f"in {len(eps)} separate episodes")
    print()
    print(f"{'episode':>7} {'duration':>9} {'drift':>9} {'step on reconnect':>19}")
    print("-" * 48)

    durs, drifts, steps = [], [], []
    for n, (s, e) in enumerate(eps, 1):
        if e >= len(run.t):
            continue
        dur = run.t[e - 1] - run.t[s]
        if dur < 0.1:
            continue
        # error between estimate and truth at the moment the sensor returns
        drift = abs(run.est[e] - run.dp[e])
        # how much the estimate jumps once the measurement is trusted again
        step = abs(run.est[min(e + 2, len(run.t) - 1)] - run.est[e - 1])
        durs.append(dur)
        drifts.append(drift)
        steps.append(step)
        print(f"{n:>7} {dur:>8.2f}s {drift:>8.2f}P {step:>18.2f}P")

    if not durs:
        print("no blind episodes in this run")
        return

    durs, drifts, steps = map(np.array, (durs, drifts, steps))
    print("-" * 48)
    print(f"longest blind episode : {durs.max():.2f} s")
    print(f"mean drift            : {drifts.mean():.2f} Pa")
    print(f"worst drift           : {drifts.max():.2f} Pa")
    print(f"mean reconnect step   : {steps.mean():.2f} Pa")

    print()
    if len(durs) >= 3 and np.ptp(durs) > 0.05:
        slope = np.polyfit(durs, drifts, 1)[0]
        r = float(np.corrcoef(durs, drifts)[0, 1])
        if abs(r) >= 0.7:
            print(f"drift rate            : {slope:.1f} Pa/s   (r = {r:.2f})")
        else:
            print(f"drift rate            : not fittable here "
                  f"(r = {r:.2f} over {np.ptp(durs):.2f} s of spread)")
    else:
        print("drift rate            : not fittable here, all episodes same length")

    print()
    print("A drift rate is only identifiable when the blind durations in a run")
    print("actually differ. In runs where every crossing takes about the same")
    print("time, the slope is fitting noise and the correlation says so. Quote it")
    print("as a within-run figure from runs where it fits, never as a constant")
    print("of the rig.")

    if fig_out:
        fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True,
                               gridspec_kw={"height_ratios": [3, 1]})
        ax[0].plot(run.t, run.dp, lw=1.0, label="measured pressure")
        ax[0].plot(run.t, run.est, lw=1.2, label="controller estimate")
        ax[0].plot(run.t, run.sp, lw=1.0, ls="--", label="setpoint")
        for s, e in eps:
            ax[0].axvspan(run.t[s], run.t[min(e, len(run.t) - 1)],
                          color="0.85", zorder=0)
        ax[0].set_ylabel("pressure [Pa]")
        ax[0].legend(loc="upper right", fontsize=9)
        ax[0].set_title("Grey bands: sensor unusable, controller running open loop")

        ax[1].plot(run.t, run.duty, lw=1.0, color="0.3")
        ax[1].set_ylabel("duty [%]")
        ax[1].set_xlabel("time [s]")
        fig.tight_layout()
        fig.savefig(fig_out, dpi=130)
        print(f"\nwrote {fig_out}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    out = sys.argv[2] if len(sys.argv) > 2 else None
    main(sys.argv[1], out)
