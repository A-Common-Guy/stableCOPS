#!/usr/bin/env python3
"""Plot validate_trajectory output: foot pitch/roll (measured vs expected) and,
when present, motor tracking (encoder vs command).

Usage:
    python3 leg/scripts/plot_validation.py validation_out.csv [--save plot.png] [--deg]

Input CSV columns (radians). Foot columns are always required:
    time, pitch_measured, pitch_expected, roll_measured, roll_expected
Motor tracking columns are optional (added by newer validate_trajectory):
    motor_1_rad, motor_2_rad, motor_1_measured, motor_2_measured
"""

import argparse
import csv
import math
import sys


def load(path):
    """Load all present numeric columns into lists keyed by name."""
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
    return cols, set(fields)


def rms(meas, exp, scale):
    errs = [(m - e) for m, e in zip(meas, exp)]
    return math.sqrt(sum(e * e for e in errs) / len(errs)) * scale if errs else float("nan")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", help="validate_trajectory output CSV")
    parser.add_argument("--save", help="write the figure to this PNG instead of showing it")
    parser.add_argument("--deg", action="store_true", help="plot in degrees instead of radians")
    args = parser.parse_args()

    try:
        import matplotlib
        if args.save:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        sys.exit("matplotlib is required: pip install matplotlib")

    data, fields = load(args.csv)
    scale = 180.0 / math.pi if args.deg else 1.0
    unit = "deg" if args.deg else "rad"
    t = data["time"]

    def conv(key):
        return [v * scale for v in data[key]]

    has_motor = {"motor_1_rad", "motor_1_measured", "motor_2_rad",
                 "motor_2_measured"}.issubset(fields)

    # Build the panel list: foot pitch/roll always, plus motor tracking if present.
    panels = [
        ("pitch", "pitch_measured", "pitch_expected", "measured (ecat)", "expected",
         "Foot pitch: measured (EtherCAT encoder) vs expected (CSV)"),
        ("roll", "roll_measured", "roll_expected", "measured (ecat)", "expected",
         "Foot roll: measured (EtherCAT encoder) vs expected (CSV)"),
    ]
    if has_motor:
        panels += [
            ("motor_1", "motor_1_measured", "motor_1_rad", "measured (encoder)", "command",
             "Motor 1 (top): encoder vs command"),
            ("motor_2", "motor_2_measured", "motor_2_rad", "measured (encoder)", "command",
             "Motor 2 (bottom): encoder vs command"),
        ]
    else:
        print("note: no motor_*_measured columns in this CSV; "
              "re-run validate_trajectory to log motor tracking.")

    n = len(panels)
    fig, axes = plt.subplots(n, 1, sharex=True, figsize=(11, 3.2 * n))
    if n == 1:
        axes = [axes]

    print("tracking RMS error:")
    for ax, (name, meas_key, ref_key, meas_lbl, ref_lbl, title) in zip(axes, panels):
        ax.plot(t, conv(ref_key), label=ref_lbl, linewidth=1.2)
        ax.plot(t, conv(meas_key), label=meas_lbl, linewidth=1.0, alpha=0.8)
        ax.set_ylabel(f"{name} [{unit}]")
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")
        print(f"  {name:8s} {rms(data[meas_key], data[ref_key], scale):.6f} {unit}")
    axes[-1].set_xlabel("time [s]")

    fig.tight_layout()
    if args.save:
        fig.savefig(args.save, dpi=130)
        print(f"saved {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
