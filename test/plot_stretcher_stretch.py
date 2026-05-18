#!/usr/bin/env python3
"""Plot spring stretcher target, command, and actual position from CSV."""

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


def load_csv(path):
    rows = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            rows.append(row)
    return rows


def column(rows, name):
    return [float(row[name]) for row in rows]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", nargs="?", default="/tmp/stretcher_stretch_log.csv")
    parser.add_argument("-o", "--output", default=None)
    args = parser.parse_args()

    rows = load_csv(args.csv)
    if not rows:
        raise SystemExit(f"no rows in {args.csv}")

    t = column(rows, "time_s")
    output = args.output or str(Path(args.csv).with_suffix(".png"))

    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(11, 7))
    for ax, side in zip(axes, ("left", "right")):
        ax.plot(t, column(rows, f"{side}_target"), "--", label="final target")
        ax.plot(t, column(rows, f"{side}_cmd"), label="moving target")
        ax.plot(t, column(rows, f"{side}_pos"), label="actual position")
        ax.set_ylabel(f"{side} rad")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")

    axes[-1].set_xlabel("time [s]")
    fig.suptitle("Spring stretcher position tracking")
    fig.tight_layout()
    fig.savefig(output, dpi=160)
    print(output)


if __name__ == "__main__":
    main()
