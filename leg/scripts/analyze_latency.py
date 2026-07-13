#!/usr/bin/env python3
"""Distinguish a communication/servo DELAY from a MAPPER (gain/offset) error in
validate_trajectory output.

Idea:
  - A pure transport delay makes the measured signal a time-shifted copy of the
    reference -> a non-zero cross-correlation lag, and a hysteresis LOOP in the
    measured-vs-reference X-Y plot.
  - A mapper error (wrong gain/offset/coupling) shows ~zero lag but a slope != 1
    and/or offset != 0, i.e. a tilted/shifted straight LINE, no loop.

The CAN+drive delay is measured directly from motor_command -> motor_encoder.
Compare it to the foot expected -> measured lag:
  - foot lag ~= motor lag (both small)         => comms/servo delay is NOT the
    cause of a residual foot mismatch; it's the mapper.
  - foot lag  > motor lag                       => extra lag appears in the
    kinematic chain (unlikely to be CAN comms).

Usage:
    python3 leg/scripts/analyze_latency.py validation_out_joint_swapped.csv \
        [--dt 0.001] [--max-lag-ms 50] [--deg] [--save latency.png]

Requires numpy (and matplotlib for --save).
"""

import argparse
import csv
import math
import sys

try:
    import numpy as np
except ImportError:
    sys.exit("numpy is required: pip install numpy")


def load(path):
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        fields = reader.fieldnames or []
        cols = {k: [] for k in fields}
        for row in reader:
            for k in fields:
                try:
                    cols[k].append(float(row[k]))
                except (TypeError, ValueError):
                    cols[k].append(float("nan"))
    return {k: np.asarray(v, dtype=float) for k, v in cols.items()}, set(fields)


def stationary_split(ref, meas, dt, vel_thresh):
    """Split the raw error (meas - ref) into samples where the reference is
    (near-)stationary vs moving. A transport DELAY contributes ~0 error while
    stationary (both signals equal and constant) and grows with speed; a MAPPER
    error (offset/gain) persists even at rest. Returns a dict of RMS/mean errors
    and the correlation between |error| and |velocity| (delay -> positive)."""
    vel = np.gradient(ref) / dt
    err = meas - ref
    still = np.abs(vel) < vel_thresh
    moving = ~still

    def rms(x):
        return float(np.sqrt(np.mean(x**2))) if x.size else float("nan")

    corr_ev = float("nan")
    if err.size > 2 and np.std(np.abs(vel)) > 0 and np.std(np.abs(err)) > 0:
        corr_ev = float(np.corrcoef(np.abs(vel), np.abs(err))[0, 1])
    return {
        "n_still": int(still.sum()),
        "n_moving": int(moving.sum()),
        "rms_still": rms(err[still]) if still.any() else float("nan"),
        "mean_still": float(np.mean(err[still])) if still.any() else float("nan"),
        "rms_moving": rms(err[moving]) if moving.any() else float("nan"),
        "corr_err_vel": corr_ev,
    }


def best_lag(ref, meas, max_lag, use_velocity=True):
    """Return (lag, corr) maximizing normalized cross-correlation over
    [-max_lag, max_lag]. Positive lag => meas lags behind ref by `lag` samples.

    By default the correlation is computed on the signal VELOCITY (first
    difference). A slow/smooth position signal barely decorrelates under a small
    time shift, so its correlation surface is flat and the lag is unresolved;
    the velocity emphasizes the moving parts and gives a sharp, trustworthy
    peak."""
    if use_velocity:
        ref = np.gradient(ref)
        meas = np.gradient(meas)
    r = ref - ref.mean()
    m = meas - meas.mean()
    rd = np.linalg.norm(r)
    md = np.linalg.norm(m)
    if rd == 0 or md == 0:
        return 0, float("nan")
    best = (0, -2.0)
    n = len(ref)
    for lag in range(-max_lag, max_lag + 1):
        if lag >= 0:
            a = r[lag:]
            b = m[: n - lag] if lag > 0 else m
        else:
            a = r[: n + lag]
            b = m[-lag:]
        L = min(len(a), len(b))
        if L < 16:
            continue
        c = float(np.dot(a[:L], b[:L]) / (rd * md))
        if c > best[1]:
            best = (lag, c)
    return best


def fit(ref, meas, lag):
    """Least-squares meas ~= a*ref + b after aligning meas to ref by `lag`
    samples. Returns (a, b, residual_rms, r2, corr0)."""
    n = len(ref)
    if lag >= 0:
        x = ref[lag:]
        y = meas[: n - lag] if lag > 0 else meas
    else:
        x = ref[: n + lag]
        y = meas[-lag:]
    L = min(len(x), len(y))
    x, y = x[:L], y[:L]
    A = np.vstack([x, np.ones_like(x)]).T
    (a, b), *_ = np.linalg.lstsq(A, y, rcond=None)
    resid = y - (a * x + b)
    rms = float(np.sqrt(np.mean(resid**2)))
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    r2 = 1.0 - float(np.sum(resid**2)) / ss_tot if ss_tot > 0 else float("nan")
    # correlation at zero lag for reference
    r0 = ref - ref.mean()
    m0 = meas - meas.mean()
    corr0 = float(np.dot(r0, m0) / (np.linalg.norm(r0) * np.linalg.norm(m0)))
    return a, b, rms, r2, corr0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv")
    parser.add_argument("--dt", type=float, default=0.001, help="sample period (s) for ms readout")
    parser.add_argument("--max-lag-ms", type=float, default=50.0)
    parser.add_argument("--deg", action="store_true")
    parser.add_argument("--save", help="save X-Y hysteresis + cross-correlation figure")
    args = parser.parse_args()

    data, fields = load(args.csv)
    scale = 180.0 / math.pi if args.deg else 1.0
    unit = "deg" if args.deg else "rad"
    max_lag = max(1, int(round(args.max_lag_ms / 1000.0 / args.dt)))

    # (label, reference_col, measured_col)
    pairs = []
    if {"motor_1_rad", "motor_1_measured"}.issubset(fields):
        pairs.append(("motor_1 (CAN+drive)", "motor_1_rad", "motor_1_measured"))
    if {"motor_2_rad", "motor_2_measured"}.issubset(fields):
        pairs.append(("motor_2 (CAN+drive)", "motor_2_rad", "motor_2_measured"))
    pairs.append(("foot pitch (chain)", "pitch_expected", "pitch_measured"))
    pairs.append(("foot roll (chain)", "roll_expected", "roll_measured"))

    print(f"samples: {len(data['time'])}   dt(assumed)={args.dt*1000:.3f} ms   "
          f"max lag search: +/-{max_lag} samples (+/-{max_lag*args.dt*1000:.1f} ms)\n")
    header = f"{'signal':22s} {'lag(smp)':>8s} {'lag(ms)':>8s} {'corr@lag':>9s} " \
             f"{'corr@0':>8s} {'gain':>7s} {'offset':>9s} {'resid_rms':>10s} {'R2':>7s}"
    print(header)
    print("-" * len(header))
    results = {}
    for label, ref_c, meas_c in pairs:
        ref, meas = data[ref_c], data[meas_c]
        lag, corr = best_lag(ref, meas, max_lag)
        a, b, rms, r2, corr0 = fit(ref, meas, lag)
        results[label] = (lag, corr, a, b, rms, r2, ref_c, meas_c)
        print(f"{label:22s} {lag:8d} {lag*args.dt*1000:8.2f} {corr:9.4f} "
              f"{corr0:8.4f} {a:7.3f} {b*scale:9.4f} {rms*scale:10.5f} {r2:7.4f}")

    # Stationary vs moving error breakdown -- the quantization-proof delay test.
    vel_thresh = 0.02  # rad/s (~1.1 deg/s): "nearly still"
    print("\nstationary-vs-moving raw error (meas - ref)  [delay => ~0 when still, "
          "grows with speed]:")
    sh = f"{'signal':22s} {'n_still':>8s} {'rms_still':>10s} {'mean_still':>11s} " \
         f"{'rms_moving':>11s} {'corr(|e|,|v|)':>13s}"
    print(sh)
    print("-" * len(sh))
    for label, ref_c, meas_c in pairs:
        s = stationary_split(data[ref_c], data[meas_c], args.dt, vel_thresh)
        print(f"{label:22s} {s['n_still']:8d} {s['rms_still']*scale:10.5f} "
              f"{s['mean_still']*scale:11.5f} {s['rms_moving']*scale:11.5f} "
              f"{s['corr_err_vel']:13.3f}")
    print(f"(errors in {unit}; 'still' = |ref velocity| < {vel_thresh} rad/s)")

    # Interpretation.
    motor_lags = [results[k][0] for k in results if k.startswith("motor")]
    foot_lags = [results[k][0] for k in results if k.startswith("foot")]
    print()
    if motor_lags:
        ml = np.mean(motor_lags)
        print(f"mean CAN+drive lag (motor cmd->encoder): {ml:.1f} samples "
              f"= {ml*args.dt*1000:.2f} ms")
    if foot_lags:
        fl = np.mean(foot_lags)
        print(f"mean foot lag (expected->measured):      {fl:.1f} samples "
              f"= {fl*args.dt*1000:.2f} ms")
    print("\ninterpretation:")
    print("  * If foot lag ~= motor lag and both small, the residual foot mismatch")
    print("    is NOT CAN delay -- look at the mapper (see gain != 1 / offset != 0).")
    print("  * A large 'gain' deviation from 1.0 or non-zero 'offset' at ~zero lag")
    print("    is a mapping (kinematic) error, not a timing error.")
    print("  * A clear positive lag with gain~1, offset~0 would indicate a delay.")

    if not args.save:
        return
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available; skipping figure")
        return

    foot = [p for p in pairs if p[0].startswith("foot")]
    fig, axes = plt.subplots(1, len(foot), figsize=(6 * len(foot), 6))
    if len(foot) == 1:
        axes = [axes]
    for ax, (label, ref_c, meas_c) in zip(axes, foot):
        x = data[ref_c] * scale
        y = data[meas_c] * scale
        ax.plot(x, y, lw=0.4, alpha=0.5, color="tab:blue")
        lo = min(x.min(), y.min())
        hi = max(x.max(), y.max())
        ax.plot([lo, hi], [lo, hi], "k--", lw=1.0, label="ideal (y=x)")
        lag = results[label][0]
        ax.set_title(f"{label}\nlag={lag} smp ({lag*args.dt*1000:.1f} ms) "
                     f"-> loop=delay, tilt/shift=mapper")
        ax.set_xlabel(f"expected [{unit}]")
        ax.set_ylabel(f"measured [{unit}]")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(args.save, dpi=130)
    print(f"saved {args.save}")


if __name__ == "__main__":
    main()
