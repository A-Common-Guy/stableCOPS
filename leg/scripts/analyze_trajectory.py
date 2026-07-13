#!/usr/bin/env python3
"""Analyze the smoothness of a command trajectory CSV.

Checks whether the motor reference columns are quantized/staircased (which makes
CSP motion jerky) and quantifies velocity / acceleration / jerk.

Usage:
    python3 leg/scripts/analyze_trajectory.py <trajectory.csv> [--counts-per-rev 524288] [--save plot.png]

CSV columns: time, motor_1_reference, motor_2_reference, pitch_foot_expected, roll_foot_expected
"""

import argparse
import csv
import math
import statistics as st


def load(path):
    t, m1, m2 = [], [], []
    with open(path, newline="") as f:
        reader = csv.reader(f)
        next(reader, None)  # header
        for row in reader:
            if len(row) < 3 or not row[0].strip():
                continue
            t.append(float(row[0]))
            m1.append(float(row[1]))
            m2.append(float(row[2]))
    return t, m1, m2


def smallest_nonzero_step(x):
    steps = sorted({round(abs(b - a), 12) for a, b in zip(x, x[1:]) if b != a})
    return steps[0] if steps else 0.0


def dwell_run_lengths(x):
    """Lengths of consecutive-identical runs (how long the reference sits still)."""
    runs = []
    run = 1
    for a, b in zip(x, x[1:]):
        if b == a:
            run += 1
        else:
            runs.append(run)
            run = 1
    runs.append(run)
    return runs


def analyze_axis(name, x, dt, counts_per_rev):
    n = len(x)
    changes = sum(1 for a, b in zip(x, x[1:]) if b != a)
    frac_changing = changes / (n - 1) if n > 1 else 0.0
    step = smallest_nonzero_step(x)
    step_counts = step / (2 * math.pi) * counts_per_rev
    runs = dwell_run_lengths(x)
    # Derivatives (finite difference on the nominal 1 kHz grid).
    vel = [(b - a) / dt for a, b in zip(x, x[1:])]
    acc = [(b - a) / dt for a, b in zip(vel, vel[1:])]
    jerk = [(b - a) / dt for a, b in zip(acc, acc[1:])]

    print(f"\n=== {name} ===")
    print(f"  samples: {n}, range: [{min(x):.4f}, {max(x):.4f}] rad")
    print(f"  reference changes on {frac_changing*100:.1f}% of steps "
          f"({changes}/{n-1}); it sits still the rest of the time")
    print(f"  smallest non-zero step: {step:.6f} rad = {step_counts:.1f} counts "
          f"(quantization)")
    print(f"  dwell run length (samples held constant): "
          f"mean={st.mean(runs):.2f} max={max(runs)} "
          f"(=> updates ~every {st.mean(runs):.1f} ms)")
    if vel:
        print(f"  |velocity|: max={max(abs(v) for v in vel):.3f} rad/s")
    if acc:
        print(f"  |accel|:    max={max(abs(a) for a in acc):.1f} rad/s^2")
    if jerk:
        print(f"  |jerk|:     max={max(abs(j) for j in jerk):.0f} rad/s^3 "
              f"(spikes at each staircase step)")
    return vel, acc


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv")
    parser.add_argument("--counts-per-rev", type=int, default=524288)
    parser.add_argument("--save", help="save a zoomed plot to this PNG")
    parser.add_argument("--zoom-start", type=int, default=30000)
    parser.add_argument("--zoom-len", type=int, default=500)
    args = parser.parse_args()

    t, m1, m2 = load(args.csv)
    print(f"loaded {len(t)} samples from {args.csv}")

    # Time-column resolution check.
    dts = [round(b - a, 9) for a, b in zip(t, t[1:])]
    uniq = sorted(set(dts))
    zero_dt = sum(1 for d in dts if d <= 0)
    print(f"time column: {len(uniq)} distinct step value(s); "
          f"{zero_dt} step(s) are <= 0 "
          f"({zero_dt/len(dts)*100:.1f}% -> timestamps lack ms resolution)")
    print(f"  example dt values: {uniq[:5]}")

    dt = 0.001  # nominal 1 kHz grid for derivative estimates
    v1, a1 = analyze_axis("motor_1 (top)", m1, dt, args.counts_per_rev)
    v2, a2 = analyze_axis("motor_2 (bottom)", m2, dt, args.counts_per_rev)

    if args.save:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            print("matplotlib not available; skipping plot")
            return
        s = args.zoom_start
        e = min(s + args.zoom_len, len(m1))
        idx = list(range(s, e))
        fig, (axp, axv) = plt.subplots(2, 1, sharex=True, figsize=(11, 7))
        axp.plot(idx, m1[s:e], label="motor_1 ref", marker=".", ms=3, lw=0.8)
        axp.plot(idx, m2[s:e], label="motor_2 ref", marker=".", ms=3, lw=0.8)
        axp.set_ylabel("position [rad]")
        axp.set_title(f"Command reference (zoom {s}..{e}) — staircase => quantized")
        axp.grid(True, alpha=0.3)
        axp.legend()
        axv.plot(idx[:-1], v1[s:e - 1], label="motor_1 vel", lw=0.8)
        axv.plot(idx[:-1], v2[s:e - 1], label="motor_2 vel", lw=0.8)
        axv.set_ylabel("velocity [rad/s]")
        axv.set_xlabel("sample index")
        axv.set_title("Implied velocity — spikes at each step are the jerk source")
        axv.grid(True, alpha=0.3)
        axv.legend()
        fig.tight_layout()
        fig.savefig(args.save, dpi=130)
        print(f"saved {args.save}")


if __name__ == "__main__":
    main()
