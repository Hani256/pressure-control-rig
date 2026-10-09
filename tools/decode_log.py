#!/usr/bin/env python3
"""
Decode the RAM log dumped from the debugger.

    python3 decode_log.py log.bin

Handles both runs:
  mode 1, the staircase: time constants, the feedforward table and the gain spread
  mode 2, the hybrid controller: how far the estimate drifted while the loop was blind

Writes log.csv and steps.png (staircase) or hybrid.png (controller run).
Needs numpy and matplotlib.
"""
import struct
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REC = "<IhhhHhhhBB"   # t_ms, sp, dp, est, rpm, temp, k_x1000, b_x10, duty, valid
SIZE = struct.calcsize(REC)
assert SIZE == 20, SIZE

if len(sys.argv) < 2:
    sys.exit("usage: decode_log.py <log.bin>")

raw = open(sys.argv[1], "rb").read()
rows = []
for i in range(0, len(raw) - SIZE + 1, SIZE):
    t_ms, sp10, dp10, est10, rpm, tmp10, k1000, b10, duty, valid = struct.unpack_from(REC, raw, i)
    if duty > 100 or valid > 1:            # never written, we are past the valid part
        break
    if rows and t_ms <= rows[-1][0]:       # time went backwards: end of the real data
        break
    rows.append((t_ms, sp10 / 10.0, dp10 / 10.0, est10 / 10.0, rpm,
                 tmp10 / 10.0, k1000 / 1000.0, b10 / 10.0, duty, valid))

if not rows:
    sys.exit("no valid samples found: check the dump range")

t = np.array([r[0] for r in rows]) / 1000.0
sp = np.array([r[1] for r in rows])
dp = np.array([r[2] for r in rows])
est = np.array([r[3] for r in rows])
rpm = np.array([r[4] for r in rows])
temp = np.array([r[5] for r in rows])
kk = np.array([r[6] for r in rows])
bb = np.array([r[7] for r in rows])
duty = np.array([r[8] for r in rows])
valid = np.array([r[9] for r in rows])

rate = 1 / np.median(np.diff(t))
print(f"{len(rows)} samples, {t[-1]:.1f} s, {rate:.1f} Hz")

with open("log.csv", "w") as f:
    f.write("t_s,sp_pa,dp_pa,est_pa,duty,rpm,temp_c,k_unit,b_unit,valid\n")
    for i in range(len(t)):
        f.write(f"{t[i]:.3f},{sp[i]:.2f},{dp[i]:.2f},{est[i]:.2f},"
                f"{duty[i]},{rpm[i]},{temp[i]:.2f},{kk[i]:.3f},{bb[i]:.2f},{valid[i]}\n")
print("wrote log.csv")

hybrid = bool((valid == 0).any())

# ----------------------------------------------------------------- staircase
if not hybrid:
    edges = [0] + [i for i in range(1, len(duty)) if duty[i] != duty[i-1]] + [len(duty)]

    sigma = None
    for a, b in zip(edges[:-1], edges[1:]):
        if duty[a] == 0 and b - a > 40:
            quiet = dp[a + (b - a) // 2: b]
            sigma = quiet.std()
            print(f"\nnoise with the fan off: sigma = {sigma:.3f} Pa  (over {len(quiet)} samples)")
            print(f"blind threshold at 3 sigma = {3*sigma:.2f} Pa")
            break
    if sigma is None:
        print("\nno quiet fan-off stretch found, cannot measure the noise floor")

    print()
    print(f"{'duty':>5} {'from':>8} {'to (ss)':>9} {'tau s':>7}  dir    note")
    summary = []
    unsettled = 0
    for a, b in zip(edges[:-1], edges[1:]):
        if b - a < 10:
            continue
        d = duty[a]
        y = dp[a:b]
        tt = t[a:b] - t[a]
        y0 = y[:3].mean()
        yss = y[int(len(y) * 0.75):].mean()
        span = yss - y0
        tau = float("nan")
        if abs(span) > 0.5:
            target = y0 + 0.632 * span
            cross = np.where((y - target) * np.sign(span) >= 0)[0]
            if len(cross):
                tau = tt[cross[0]]
        direction = "up" if span > 0 else ("down" if span < 0 else "flat")
        n = len(y)
        tail1 = y[int(n * 0.80):int(n * 0.90)].mean()
        tail2 = y[int(n * 0.90):].mean()
        note = ""
        if abs(span) > 1.0 and abs(tail2 - tail1) > 0.05 * abs(span):
            note = "still moving, step too short"
            unsettled += 1
        print(f"{d:>5} {y0:>8.2f} {yss:>9.2f} {tau:>7.2f}  {direction:<5}  {note}")
        summary.append((d, yss, tau, direction, note == ""))

    if unsettled:
        print(f"\n{unsettled} steps had not settled. Raise STEP_SECONDS and run again,")
        print("otherwise the steady state numbers and the feedforward table are not real.")

    ups = [s for s in summary if s[3] == "up" and not np.isnan(s[2])]
    downs = [s for s in summary if s[3] == "down" and not np.isnan(s[2])]
    if ups and downs:
        tu, td = np.mean([s[2] for s in ups]), np.mean([s[2] for s in downs])
        print(f"\nmean tau rising  {tu:.2f} s")
        print(f"mean tau falling {td:.2f} s")
        print(f"asymmetry factor {td/tu:.2f}x  (falling over rising)")
        if abs(td/tu - 1.0) < 0.4:
            print("close to symmetric, as expected for a fan. Do not oversell this as SMA behaviour.")
        else:
            print("a real asymmetry between the two directions, worth explaining")

    table = {}
    for d, yss, _, _, settled in summary:
        if settled:
            table.setdefault(int(d), []).append(yss)
    means = {d: float(np.mean(v)) for d, v in table.items()}
    offset = means.get(0, 0.0)
    print("\nfeedforward table")
    print(f"{'duty':>5} {'dP raw':>9} {'dP - rest':>10} {'gain Pa/%':>10}")
    prev_d = prev_y = None
    for d in sorted(means):
        y = means[d] - offset
        gain = ""
        if prev_d is not None and d > prev_d:
            gain = f"{(y - prev_y) / (d - prev_d):.3f}"
        print(f"{d:>5} {means[d]:>9.2f} {y:>10.2f} {gain:>10}")
        prev_d, prev_y = d, y
    if len(means) >= 3:
        ds = sorted(means)
        gains = [(means[ds[i]] - means[ds[i-1]]) / (ds[i] - ds[i-1]) for i in range(1, len(ds))]
        pos = [g for g in gains if g > 0.01]
        if len(pos) >= 2:
            print(f"\ngain varies by {max(pos)/min(pos):.1f}x across the range")
            print("one fixed set of PID gains cannot cover that: this is the case for gain scheduling")

    fig, ax = plt.subplots(3, 1, figsize=(11, 7.5), sharex=True,
                           gridspec_kw={"height_ratios": [3, 1, 1]})
    ax[0].plot(t, dp, lw=1.1, color="#111")
    ax[0].set_ylabel("dP [Pa]"); ax[0].grid(alpha=.3); ax[0].set_title("staircase response")
    ax[1].step(t, duty, where="post", color="#b45309"); ax[1].set_ylabel("duty [%]"); ax[1].grid(alpha=.3)
    ax[2].plot(t, rpm, lw=1, color="#1d4ed8"); ax[2].set_ylabel("rpm")
    ax[2].set_xlabel("time [s]"); ax[2].grid(alpha=.3)
    fig.tight_layout(); fig.savefig("steps.png", dpi=130)
    print("wrote steps.png")

# ------------------------------------------------------------------- hybrid
else:
    err_est = est - dp                       # what the controller believed minus the truth

    # find each stretch where the controller was blind
    runs = []
    i = 0
    while i < len(valid):
        if valid[i] == 0:
            j = i
            while j < len(valid) and valid[j] == 0:
                j += 1
            if j - i > 2:
                runs.append((i, j))
            i = j
        else:
            i += 1

    print(f"\nthe controller was blind {len(runs)} times")
    print(f"{'#':>3} {'start s':>8} {'length s':>9} {'peak |err|':>11} {'err at re-entry':>16}")
    peaks, durs, reent = [], [], []
    for k, (a, b) in enumerate(runs, 1):
        d = t[b-1] - t[a]
        pk = np.abs(err_est[a:b]).max()
        re = abs(err_est[b-1])
        peaks.append(pk); durs.append(d); reent.append(re)
        print(f"{k:>3} {t[a]:>8.1f} {d:>9.2f} {pk:>11.2f} {re:>16.2f}")

    if peaks:
        print(f"\nmean blind time      {np.mean(durs):.2f} s   (worst {max(durs):.2f} s)")
        print(f"mean peak estimate error {np.mean(peaks):.2f} Pa   (worst {max(peaks):.2f} Pa)")
        print(f"mean error at re-entry   {np.mean(reent):.2f} Pa   (worst {max(reent):.2f} Pa)")
        span = dp.max() - dp.min()
        print(f"worst error is {100*max(peaks)/span:.1f} percent of the working range ({span:.1f} Pa)")

        # The headline finding: dead reckoning error grows with how long the loop stayed blind.
        if len(durs) >= 4:
            r = np.corrcoef(durs, peaks)[0, 1]
            k = np.polyfit(durs, peaks, 1)
            print(f"\nestimate error against blind time: correlation {r:.2f}, "
                  f"slope {k[0]:.2f} Pa per second blind")
            print("this is what dead reckoning looks like, and it is the argument for capping the blind time")
            fig2, ax2 = plt.subplots(figsize=(6.5, 4.6))
            ax2.scatter(durs, peaks, s=48, color="#b45309", zorder=3)
            xs = np.linspace(0, max(durs) * 1.05, 20)
            ax2.plot(xs, np.polyval(k, xs), color="#1d4ed8", lw=1.4,
                     label=f"{k[0]:.2f} Pa per second blind")
            ax2.set_xlabel("time spent blind [s]")
            ax2.set_ylabel("peak estimate error [Pa]")
            ax2.set_title("estimate error grows with time blind")
            ax2.grid(alpha=.3); ax2.legend(fontsize=9)
            fig2.tight_layout(); fig2.savefig("blind_error.png", dpi=130)
            print("wrote blind_error.png")

    # did the model learn anything?
    if kk.max() - kk.min() > 0.005 or abs(bb.max() - bb.min()) > 0.05:
        print(f"\nmodel learning: k went {kk[0]:.3f} -> {kk[-1]:.3f}, b went {bb[0]:+.2f} -> {bb[-1]:+.2f} Pa")
        half = len(runs) // 2
        if half >= 2:
            early = np.mean([np.abs(err_est[a:b]).max() for a, b in runs[:half]])
            late = np.mean([np.abs(err_est[a:b]).max() for a, b in runs[half:]])
            print(f"peak estimate error, first half of the run {early:.2f} Pa, second half {late:.2f} Pa")
            if late < early:
                print(f"the model learned: {early/max(late, 0.01):.1f}x better once k and b had settled")
    else:
        print("\nthe model did not move: adaptation was off, or the gate never opened")

    # tracking error over the last third of each setpoint hold, where it should have settled
    sp_edges = [0] + [i for i in range(1, len(sp)) if sp[i] != sp[i-1]] + [len(sp)]
    tr = []
    for a, b in zip(sp_edges[:-1], sp_edges[1:]):
        if b - a < 20:
            continue
        seg = dp[a + int((b - a) * 0.7): b]
        tr.append(abs(seg.mean() - sp[a]))
    if tr:
        print(f"\nsteady state tracking error {np.mean(tr):.2f} Pa mean, {max(tr):.2f} Pa worst")

    fig, ax = plt.subplots(3, 1, figsize=(12, 8.5), sharex=True,
                           gridspec_kw={"height_ratios": [3, 1, 1]})
    for a, b in runs:
        for k_ax in range(3):
            ax[k_ax].axvspan(t[a], t[b-1], color="#f3d0cd", alpha=.75, lw=0)
    ax[0].plot(t, sp, lw=1.4, color="#b45309", ls="--", label="setpoint")
    ax[0].plot(t, dp, lw=1.2, color="#111", label="true pressure")
    ax[0].plot(t, est, lw=1.2, color="#1d4ed8", label="controller estimate")
    ax[0].set_ylabel("pressure [Pa]"); ax[0].grid(alpha=.3)
    ax[0].legend(loc="upper right", fontsize=9)
    ax[0].set_title("hybrid control across an artificial blind band (shaded = controller blind)")
    ax[1].step(t, duty, where="post", color="#15803d")
    ax[1].set_ylabel("duty [%]"); ax[1].grid(alpha=.3)
    ax[2].plot(t, kk, lw=1.4, color="#7c3aed", label="k (learned gain)")
    ax[2].axhline(1.0, color="#999", lw=.9, ls=":")
    ax[2].set_ylabel("k"); ax[2].set_xlabel("time [s]"); ax[2].grid(alpha=.3)
    ax[2].legend(fontsize=9, loc="lower right")
    fig.tight_layout(); fig.savefig("hybrid.png", dpi=130)
    print("wrote hybrid.png")
