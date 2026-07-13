#!/usr/bin/env python3
"""Plot the reached range-of-motion area from capture_rom output and print/save
numerical results.

Usage:
    python3 leg/scripts/plot_rom.py rom_data.csv [--save rom.png] [--deg] \
        [--summary rom_summary.csv]

Input CSV (from build/examples/capture_rom): time, roll, pitch   (radians).
The plot shows roll (x) vs pitch (y) as the swept area, with its convex hull and
axis-aligned bounding box. Numerical results (min/max/range per axis, hull area)
are printed and optionally written to a summary CSV.
"""

import argparse
import csv
import math
import sys


def load(path):
    roll, pitch = [], []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            roll.append(float(row["roll"]))
            pitch.append(float(row["pitch"]))
    return roll, pitch


def median(x):
    s = sorted(x)
    n = len(s)
    if n == 0:
        return 0.0
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def reject_outliers(roll, pitch, warmup, mad_thresh):
    """Drop leading warmup samples and points far from the median (robust MAD
    test, per axis). Returns (roll, pitch, n_dropped). A point is dropped if
    either axis is an outlier. Isolated encoder-warmup/wrap glitches sit far from
    the median cluster, so they are removed while genuine ROM extremes (part of a
    continuous distribution) are kept."""
    roll = roll[warmup:]
    pitch = pitch[warmup:]
    n = len(roll)
    if n == 0 or mad_thresh <= 0:
        return roll, pitch, 0

    def bounds(x):
        med = median(x)
        mad = median([abs(v - med) for v in x])
        scaled = 1.4826 * mad
        if scaled <= 0:  # degenerate spread; fall back to a wide window
            return -float("inf"), float("inf")
        return med - mad_thresh * scaled, med + mad_thresh * scaled

    rlo, rhi = bounds(roll)
    plo, phi = bounds(pitch)
    fr, fp = [], []
    for r, p in zip(roll, pitch):
        if rlo <= r <= rhi and plo <= p <= phi:
            fr.append(r)
            fp.append(p)
    return fr, fp, n - len(fr)


def convex_hull(points):
    """Andrew's monotone chain convex hull. points: list of (x, y). Returns hull
    vertices in counter-clockwise order (no external deps)."""
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def polygon_area(poly):
    """Shoelace area of a polygon (absolute)."""
    n = len(poly)
    if n < 3:
        return 0.0
    s = 0.0
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        s += x1 * y2 - x2 * y1
    return abs(s) / 2.0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv")
    parser.add_argument("--save", help="write the figure to this PNG instead of showing it")
    parser.add_argument("--summary", help="write numerical results to this CSV")
    parser.add_argument("--deg", action="store_true", help="report/plot in degrees")
    parser.add_argument("--warmup", type=int, default=5,
                        help="drop this many leading samples (pre-valid EtherCAT feedback)")
    parser.add_argument("--outlier-mad", type=float, default=8.0,
                        help="reject points more than N robust MADs from the median (0 disables)")
    args = parser.parse_args()

    roll, pitch = load(args.csv)
    if not roll:
        sys.exit("no samples in input CSV")

    total = len(roll)
    roll, pitch, dropped = reject_outliers(roll, pitch, args.warmup, args.outlier_mad)
    if not roll:
        sys.exit("all samples were rejected as outliers; relax --outlier-mad / --warmup")
    if dropped or args.warmup:
        print(f"filtered {args.warmup} warmup + {dropped} outlier sample(s) "
              f"of {total} (kept {len(roll)})")

    scale = 180.0 / math.pi if args.deg else 1.0
    unit = "deg" if args.deg else "rad"

    roll_s = [r * scale for r in roll]
    pitch_s = [p * scale for p in pitch]

    roll_min, roll_max = min(roll_s), max(roll_s)
    pitch_min, pitch_max = min(pitch_s), max(pitch_s)
    roll_range = roll_max - roll_min
    pitch_range = pitch_max - pitch_min

    hull = convex_hull(list(zip(roll_s, pitch_s)))
    hull_area = polygon_area(hull)
    bbox_area = roll_range * pitch_range

    # Numerical results.
    results = [
        ("samples", len(roll_s), ""),
        ("roll_min", roll_min, unit),
        ("roll_max", roll_max, unit),
        ("roll_range", roll_range, unit),
        ("pitch_min", pitch_min, unit),
        ("pitch_max", pitch_max, unit),
        ("pitch_range", pitch_range, unit),
        ("bbox_area", bbox_area, f"{unit}^2"),
        ("hull_area", hull_area, f"{unit}^2"),
        ("hull_vertices", len(hull), ""),
    ]
    print("range-of-motion results:")
    for name, value, u in results:
        if isinstance(value, int):
            print(f"  {name:14s} {value}")
        else:
            print(f"  {name:14s} {value:10.4f} {u}")

    if args.summary:
        with open(args.summary, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["metric", "value", "unit"])
            for name, value, u in results:
                w.writerow([name, value, u])
        print(f"wrote summary to {args.summary}")

    try:
        import matplotlib
        if args.save:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        if args.save:
            sys.exit("matplotlib is required for plotting: pip install matplotlib")
        print("matplotlib not available; skipping plot")
        return

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(roll_s, pitch_s, "-", color="tab:blue", lw=0.5, alpha=0.35, label="path")
    ax.scatter(roll_s, pitch_s, s=2, color="tab:blue", alpha=0.5)

    if len(hull) >= 3:
        hx = [p[0] for p in hull] + [hull[0][0]]
        hy = [p[1] for p in hull] + [hull[0][1]]
        ax.plot(hx, hy, "-", color="tab:red", lw=1.8,
                label=f"convex hull (area {hull_area:.3f} {unit}^2)")

    # Bounding box.
    ax.plot([roll_min, roll_max, roll_max, roll_min, roll_min],
            [pitch_min, pitch_min, pitch_max, pitch_max, pitch_min],
            "--", color="gray", lw=1.0,
            label=f"bbox {roll_range:.2f}x{pitch_range:.2f} {unit}")

    ax.axhline(0, color="k", lw=0.5, alpha=0.4)
    ax.axvline(0, color="k", lw=0.5, alpha=0.4)
    ax.set_xlabel(f"roll [{unit}]")
    ax.set_ylabel(f"pitch [{unit}]")
    ax.set_title("Reached range of motion (foot roll vs pitch)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()

    if args.save:
        fig.savefig(args.save, dpi=130)
        print(f"saved {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
