"""
Shared loading and plant fitting for the rig logs.

Record layout written by the firmware, 20 bytes, little endian:

    uint32  t_ms       milliseconds since the control task started
    int16   sp_x10     setpoint, pascals x10
    int16   dp_x10     measured differential pressure, pascals x10
    int16   est_x10    controller's estimate of pressure, pascals x10
    uint16  rpm        fan speed from the tachometer
    int16   temp_x10   ambient temperature, degrees C x10
    int16   k_x1000    identified plant gain, pascals per percent duty x1000
    int16   b_x10      identified plant offset, pascals x10
    uint8   duty       commanded PWM duty, 0 to 100
    uint8   valid      1 when the pressure measurement is inside the valid band
"""
import struct
from dataclasses import dataclass

import numpy as np

REC = "<IhhhHhhhBB"
SIZE = struct.calcsize(REC)
assert SIZE == 20


@dataclass
class Run:
    t: np.ndarray        # seconds
    sp: np.ndarray       # setpoint, Pa
    dp: np.ndarray       # measured pressure, Pa
    est: np.ndarray      # controller estimate, Pa
    rpm: np.ndarray
    temp: np.ndarray
    k: np.ndarray        # identified gain
    b: np.ndarray        # identified offset
    duty: np.ndarray
    valid: np.ndarray    # 1 = sensor trusted, 0 = blind

    @property
    def rate_hz(self):
        return 1.0 / float(np.median(np.diff(self.t)))


def load(path):
    """Read a .bin dump and stop at the first record the firmware never wrote."""
    raw = open(path, "rb").read()
    rows = []
    for i in range(0, len(raw) - SIZE + 1, SIZE):
        r = struct.unpack_from(REC, raw, i)
        t_ms, sp, dp, est, rpm, temp, k, b, duty, valid = r
        if duty > 100 or valid > 1:
            break                       # past the part that was filled in
        if rows and t_ms <= rows[-1][0]:
            break                       # time went backwards, buffer wrapped
        rows.append(r)

    if not rows:
        raise ValueError(f"no valid samples in {path}")

    a = np.array(rows, dtype=float)
    return Run(
        t=a[:, 0] / 1000.0,
        sp=a[:, 1] / 10.0,
        dp=a[:, 2] / 10.0,
        est=a[:, 3] / 10.0,
        rpm=a[:, 4],
        temp=a[:, 5] / 10.0,
        k=a[:, 6] / 1000.0,
        b=a[:, 7] / 10.0,
        duty=a[:, 8],
        valid=a[:, 9],
    )


# Feedforward characterisation table measured on the rig: commanded duty in
# percent against settled differential pressure in pascals. The plant model is
# a scale and an offset on top of this curve, not a straight line in duty,
# because the fan is not linear.
FF_DUTY = [0, 30, 50, 70, 100]
FF_PA = [0.0, 10.4, 21.8, 38.0, 56.9]


def ff_table(duty):
    """Interpolate the characterisation table. Accepts a scalar or an array."""
    return np.interp(duty, FF_DUTY, FF_PA)


def settled_points(run, hold_s=6.0, duty_lo=0, duty_hi=100):
    """
    Every (duty, pressure) sample from the last `hold_s` seconds of each
    setpoint hold, keeping only samples where the sensor is trusted. Taking
    the end of the hold is what keeps the transient out of the fit.

    duty_lo / duty_hi restrict the fit to a duty window. Two runs can only be
    compared on the window both of them actually visited.
    """
    pts = []
    edges = np.flatnonzero(np.diff(run.sp) != 0) + 1
    starts = np.concatenate(([0], edges))
    ends = np.concatenate((edges, [len(run.sp)]))

    for s, e in zip(starts, ends):
        t_end = run.t[e - 1]
        m = ((run.t[s:e] >= t_end - hold_s) & (run.valid[s:e] == 1)
             & (run.duty[s:e] >= duty_lo) & (run.duty[s:e] <= duty_hi))
        if m.sum() == 0:
            continue
        pts.append(np.column_stack((run.duty[s:e][m], run.dp[s:e][m])))

    return np.vstack(pts) if pts else np.empty((0, 2))


def fit_plant(points):
    """
    Least squares fit of pressure = k * ff_table(duty) + b.

    k is a scale factor on the characterisation curve, so k = 1.0 means the
    plant matches the day it was characterised. b absorbs the zero offset.
    Returns (k, b, rmse).
    """
    if len(points) < 2:
        raise ValueError("need at least two settled points")
    x, press = ff_table(points[:, 0]), points[:, 1]
    k, b = np.polyfit(x, press, 1)
    rmse = float(np.sqrt(np.mean((press - (k * x + b)) ** 2)))
    return float(k), float(b), rmse


def blind_episodes(run):
    """
    Yield (start_index, end_index) for every stretch where valid == 0,
    i.e. every time the controller had to run without its sensor.
    """
    v = run.valid.astype(int)
    d = np.diff(v)
    starts = list(np.flatnonzero(d == -1) + 1)
    ends = list(np.flatnonzero(d == 1) + 1)

    if v[0] == 0:
        starts.insert(0, 0)
    if v[-1] == 0:
        ends.append(len(v))

    return list(zip(starts, ends))
