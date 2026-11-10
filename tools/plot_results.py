#!/usr/bin/env python3
"""Plots a results CSV of planning_incrementally / run_campaign.sh.

Usage: tools/plot_results.py <setting.csv> [output.png|.pdf] [--methods A,B,...]

Three panels against the number of agents n:
  success rate  - solved instances / all instances (pruned rows count as failures)
  runtime       - mean runtime, on the instances solved by every plotted method
  SoC           - mean sum of costs, on the same instances
"""
import argparse
import csv
import math
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# fixed color and marker per method, so a method looks the same in every plot
STYLE = {
    "LazyCPBS":             ("#2a78d6", "o"),
    "CPBS":                 ("#eb6834", "s"),
    "PSIPP":                ("#1baf7a", "^"),
    "CCBS_CTC":             ("#eda100", "D"),
    "CCBS":                 ("#e87ba4", "v"),
    "Eager_ReplanList_EGT": ("#008300", "P"),
    "Eager_ReplanList":     ("#6250d6", "X"),
    "Eager_Fixed_EGT":      ("#e34948", "*"),
}
INK, MUTED, GRID = "#1f1f1e", "#6b6a64", "#e6e5e0"


def load(path):
    rows = list(csv.DictReader(open(path)))
    if not rows:
        raise SystemExit(f"{path}: no rows")
    return rows


def aggregate(rows, methods):
    instances = sorted({int(r["instance"]) for r in rows})
    ns = sorted({int(r["n"]) for r in rows})
    ok = {}  # (method, n, instance) -> (runtime, soc)
    for r in rows:
        if r["status"] == "ok" and r["method"] in methods:
            ok[(r["method"], int(r["n"]), int(r["instance"]))] = (
                float(r["runtime_s"]), float(r["soc"]))
    out = defaultdict(lambda: defaultdict(list))
    for n in ns:
        common = [i for i in instances if all((m, n, i) in ok for m in methods)]
        for m in methods:
            solved = sum((m, n, i) in ok for i in instances)
            out[m]["n"].append(n)
            out[m]["success"].append(solved / len(instances))
            rt = [ok[(m, n, i)][0] for i in common]
            soc = [ok[(m, n, i)][1] for i in common]
            out[m]["runtime"].append(sum(rt) / len(rt) if rt else math.nan)
            out[m]["soc"].append(sum(soc) / len(soc) if soc else math.nan)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("output", nargs="?")
    ap.add_argument("--methods", help="comma separated, default: all in the CSV")
    args = ap.parse_args()

    rows = load(args.csv)
    present = list(dict.fromkeys(r["method"] for r in rows))
    methods = args.methods.split(",") if args.methods else \
        [m for m in STYLE if m in present] + [m for m in present if m not in STYLE]
    data = aggregate(rows, methods)
    title = f'{rows[0]["map"]} {rows[0]["setting"]}'

    plt.rcParams.update({"font.size": 10, "axes.edgecolor": MUTED,
                         "axes.labelcolor": INK, "xtick.color": MUTED,
                         "ytick.color": MUTED, "pdf.fonttype": 42})
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.2))
    panels = [("success", "success rate"), ("runtime", "runtime [s]"),
              ("soc", "sum of costs")]
    for ax, (key, label) in zip(axes, panels):
        for k, m in enumerate(methods):
            color, marker = STYLE.get(m, (f"C{k}", "o"))
            d = data[m]
            ax.plot(d["n"], d[key], color=color, lw=2, marker=marker, ms=4,
                    markevery=max(1, len(d["n"]) // 10), label=m)
        ax.set_xlabel("agents")
        ax.set_ylabel(label)
        ax.grid(True, color=GRID, lw=0.8)
        ax.spines[["top", "right"]].set_visible(False)
        if key == "runtime":
            ax.set_yscale("log")
    axes[0].set_ylim(-0.02, 1.02)
    axes[1].set_title("instances solved by all methods", color=MUTED, fontsize=9)
    axes[2].set_title("instances solved by all methods", color=MUTED, fontsize=9)
    fig.suptitle(title, color=INK, y=0.98)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, 0.93),
               ncol=min(len(methods), 8), frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.88))
    out = args.output or args.csv.rsplit(".", 1)[0] + ".png"
    fig.savefig(out, dpi=150)
    print(out)


if __name__ == "__main__":
    main()
