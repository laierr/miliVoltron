#!/usr/bin/env python3
"""
Battery tap-fault analysis figures (standalone post-processing tool).

Usage:
    python3 make_figures.py FAULT_SESSION.csv [HEALTHY_SESSION.csv] [-o OUTDIR]

Reads a miliVoltron battery CSV and produces report figures:
    fig1_overview.png     - full fault session (spread / SOC / current)
    fig2_schematic.png    - tap measurement topology explainer
    fig3_detail.png       - fault window detail, affected cell pair vs baseline
    fig4_evidence.png     - mirror scatter (r) + sum-cancellation bars
    fig5_soc.png          - reported vs BMS voltage-/coulomb-derived SOC
    fig6_healthy.png      - healthy reference profile        (needs 2nd csv)
    fig7_sidebyside.png   - healthy vs faulty, identical axes (needs 2nd csv)

The affected cell pair, rest spans, percussive-maintenance events, and the
resolution time are DETECTED from the data, not hardcoded. Healthy-session
rows from the first CHARGING event onward are dropped (faulty bench charger).

SOC estimates and cell extremes come from the CSV itself. This script does
not recompute voltage-implied SOC from a generic OCV curve.

Deps (not part of the stdlib collector): pandas, numpy, matplotlib
"""

from __future__ import annotations

import argparse
import os

import matplotlib
import numpy as np
import pandas as pd

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ------------------------------------------------------------------ styling
plt.rcParams.update({
    "font.size": 10.5,
    "font.family": "DejaVu Sans",
    "axes.edgecolor": "#444441",
    "axes.labelcolor": "#2c2c2a",
    "text.color": "#2c2c2a",
    "xtick.color": "#5f5e5a",
    "ytick.color": "#5f5e5a",
    "axes.grid": True,
    "grid.color": "#e1e0d9",
    "grid.linewidth": 0.6,
    "axes.axisbelow": True,
    "axes.spines.top": False,
    "axes.spines.right": False,
})
RED = "#c62b2b"
GREEN = "#1a8f5e"
BLUE = "#1f6fb2"
ORANGE = "#d9711a"
PURPLE = "#5b3fa0"
GRAY = "#8a8a86"
AMBER = "#b8860b"

CELLS = [f"cell_{index}_reported_mv" for index in range(1, 11)]
MODE = "mode_derived"
ELAPSED = "elapsed_s"
CURRENT = "bms_current_reported_a"
CELL_DELTA = "cell_delta_derived_mv"
CELL_MIN = "cell_min_derived_mv"
SOC_REPORTED = "bms_soc_reported_percent"
SOC_VOLTAGE = "bms_voltage_soc_derived_percent"
SOC_COULOMB = "bms_coulomb_soc_derived_percent"
SERIAL = "bms_serial_reported"


# ------------------------------------------------------------------ helpers
def load(path: str, drop_from_first_charge: bool = False) -> pd.DataFrame:
    df = pd.read_csv(path)
    required = {
        MODE,
        ELAPSED,
        CURRENT,
        CELL_DELTA,
        CELL_MIN,
        SOC_REPORTED,
        SOC_VOLTAGE,
        SOC_COULOMB,
        SERIAL,
        *CELLS,
    }
    missing = sorted(required - set(df.columns))
    if missing:
        raise SystemExit(
            f"{os.path.basename(path)} is missing expected columns:\n  "
            + "\n  ".join(missing)
            + "\nUse a miliVoltron battery CSV with reported/derived field names."
        )
    if drop_from_first_charge:
        charging = df.index[df[MODE] == "CHARGING"]
        if len(charging):
            df = df.iloc[: charging[0]].copy()
            print(
                f"  [{os.path.basename(path)}] trimmed at first CHARGING row "
                f"(t={df[ELAPSED].iloc[-1]:.1f}s end)"
            )
    return df


def rest_spans(df: pd.DataFrame) -> list[tuple[float, float]]:
    """Contiguous REST intervals as (t_start, t_end)."""
    spans: list[tuple[float, float]] = []
    start = None
    previous = None
    for _, row in df.iterrows():
        if row[MODE] == "REST" and start is None:
            start = row[ELAPSED]
        elif row[MODE] != "REST" and start is not None:
            spans.append((start, previous))
            start = None
        previous = row[ELAPSED]
    if start is not None and previous is not None:
        spans.append((start, previous))
    return spans


def detect_pair(df: pd.DataFrame):
    """Affected adjacent cell pair = the two cells whose deviation from the
    pack mean has the highest variance; sanity-checked for anticorrelation."""
    mean_all = df[CELLS].mean(axis=1)
    deviation = df[CELLS].sub(mean_all, axis=0)
    top = deviation.std().sort_values(ascending=False).index[:2].tolist()
    indexes = sorted(int(column.split("_")[1]) for column in top)
    cell_a = f"cell_{indexes[0]}_reported_mv"
    cell_b = f"cell_{indexes[1]}_reported_mv"
    others = [column for column in CELLS if column not in (cell_a, cell_b)]
    baseline = df[others].mean(axis=1)
    correlation = np.corrcoef(df[cell_a] - baseline, df[cell_b] - baseline)[0, 1]
    adjacent = indexes[1] - indexes[0] == 1
    print(
        f"  affected pair: cells {indexes[0]}/{indexes[1]} "
        f"(adjacent={adjacent}, dev correlation r={correlation:+.3f})"
    )
    if not adjacent or correlation > -0.8:
        print(
            "  WARNING: pair is not adjacent/anticorrelated - "
            "signature differs from the tap-fault model, inspect manually."
        )
    return indexes, cell_a, cell_b, baseline, correlation


def detect_events(df: pd.DataFrame, jump_mv: float = 150, group_s: float = 3.0):
    """Percussive-maintenance candidates = step-changes in cell spread.
    Resolution = last jump after which delta stays low."""
    delta = df[CELL_DELTA].astype(float)
    jumps = df[ELAPSED][delta.diff().abs() > jump_mv].tolist()
    if not jumps:
        return [], None

    groups: list[list[float]] = []
    current = [jumps[0]]
    for timestamp in jumps[1:]:
        if timestamp - current[-1] <= group_s:
            current.append(timestamp)
        else:
            groups.append(current)
            current = [timestamp]
    groups.append(current)

    representatives = [group[len(group) // 2] for group in groups]
    fix_t = None
    for timestamp in reversed(jumps):
        if (delta[df[ELAPSED] > timestamp] < jump_mv).all():
            fix_t = timestamp
        else:
            break
    if fix_t in representatives:
        representatives.remove(fix_t)
    print(
        f"  pm-event candidates: {[round(value, 1) for value in representatives]}  "
        f"resolution t={fix_t}"
    )
    return representatives, fix_t


def cell_label(column: str) -> str:
    return column.replace("_reported_mv", "").replace("_", " ")


def fault_shade(ax, fix_t, t_end, spans) -> None:
    if fix_t:
        ax.axvspan(0, fix_t, color=RED, alpha=0.05, lw=0)
        ax.axvspan(fix_t, t_end, color=GREEN, alpha=0.05, lw=0)
    for start, end in spans:
        ax.axvspan(start, end, color=GRAY, alpha=0.14, lw=0)


# ------------------------------------------------------------------ figures
def fig_overview(df, pm, fix_t, spans, out) -> None:
    timestamps = df[ELAPSED].values
    fig, (ax1, ax2, ax3) = plt.subplots(
        3, 1, figsize=(10, 7.6), sharex=True,
        gridspec_kw={"height_ratios": [2, 1.3, 1]},
    )
    for axis in (ax1, ax2, ax3):
        fault_shade(axis, fix_t, timestamps[-1], spans)

    ax1.plot(timestamps, df[CELL_DELTA], color=RED, lw=1.1)
    ax1.set_ylabel("Cell voltage spread\n(max − min), mV")
    ax1.set_ylim(-50, 2050)
    for timestamp in pm:
        ax1.axvline(timestamp, color=AMBER, lw=0.9, ls=":", alpha=0.8)
    if fix_t:
        ax1.axvline(fix_t, color=GREEN, lw=1.6)
        ax1.annotate(
            "resolution event:\nreadings normalize, remain stable",
            xy=(fix_t, 60),
            xytext=(fix_t + 13, 900),
            fontsize=8.5,
            color=GREEN,
            arrowprops=dict(arrowstyle="->", color=GREEN, lw=1.2),
        )
    if pm:
        ax1.annotate(
            "percussive-maintenance events (amber):\nstep-changes in fault magnitude",
            xy=(pm[len(pm) // 2], 1080),
            xytext=(18, 1830),
            va="top",
            fontsize=8.5,
            color=AMBER,
            arrowprops=dict(arrowstyle="->", color=AMBER, lw=1),
        )

    ax2.plot(timestamps, df[SOC_REPORTED], color=BLUE, lw=1.2, label="BMS reported")
    ax2.plot(timestamps, df[SOC_VOLTAGE], color=PURPLE, lw=1.0, label="voltage-derived")
    ax2.plot(timestamps, df[SOC_COULOMB], color=ORANGE, lw=1.0, label="coulomb-derived")
    ax2.set_ylabel("SOC, %")
    ax2.set_ylim(
        -2,
        max(
            34,
            float(df[[SOC_REPORTED, SOC_VOLTAGE, SOC_COULOMB]].max(axis=None)) + 6,
        ),
    )
    ax2.legend(loc="upper right", frameon=False, fontsize=8)

    ax3.plot(timestamps, df[CURRENT], color="#5a5a56", lw=0.8)
    ax3.set_ylabel("Pack current, A")
    ax3.set_xlabel("Elapsed time, s")
    for axis in (ax1, ax2, ax3):
        axis.set_xlim(0, timestamps[-1] + 2)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def fig_schematic(idx, out) -> None:
    from matplotlib.patches import Rectangle

    lo, hi = idx
    fig, ax = plt.subplots(figsize=(10, 4.4))
    ax.axis("off")
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 4.4)
    cell_width, cell_height, y0 = 1.5, 1.0, 2.6
    xs = [0.8, 2.9, 5.0, 7.1]
    first = max(1, lo - 1)
    names = [f"cell {first + offset}" for offset in range(4)]
    for x, name in zip(xs, names):
        number = int(name.split()[1])
        bad = number in (lo, hi)
        ax.add_patch(
            Rectangle(
                (x, y0),
                cell_width,
                cell_height,
                facecolor="#fdeaea" if bad else "#f4f3ef",
                edgecolor=RED if bad else "#8a8a86",
                lw=1.4,
            )
        )
        ax.text(x + cell_width / 2, y0 + cell_height / 2, name, ha="center", va="center", fontsize=11)
        if x != xs[-1]:
            ax.plot([x + cell_width, x + cell_width + 0.6], [y0 + cell_height / 2] * 2, color="#444441", lw=1.6)

    tap_xs = [xs[0] - 0.35] + [x + cell_width + 0.3 for x in xs]
    for offset, tap_x in enumerate(tap_xs):
        tap_number = first - 1 + offset
        bad = tap_number == lo
        color = RED if bad else "#444441"
        ax.plot(
            [tap_x, tap_x],
            [y0 + cell_height / 2, y0 + cell_height + 0.55],
            color=color,
            lw=2.2 if bad else 1.3,
            ls="--" if bad else "-",
        )
        ax.plot(tap_x, y0 + cell_height / 2, "o", color=color, ms=7 if bad else 5)
        ax.text(
            tap_x,
            y0 + cell_height + 0.68,
            f"tap {tap_number}",
            ha="center",
            fontsize=9.5,
            color=color,
            fontweight="bold" if bad else "normal",
        )
        if bad:
            ax.text(
                tap_x,
                y0 + cell_height + 1.05,
                "faulty connection\n(+ error ε)",
                ha="center",
                fontsize=9.5,
                color=RED,
                fontweight="bold",
            )
    ax.text(
        0.8,
        1.7,
        "Each cell voltage is computed as the difference between two adjacent tap readings:",
        fontsize=10,
    )
    ax.text(
        1.3,
        1.15,
        f"cell {lo}  =  tap {lo} − tap {lo - 1}  =  true value + ε",
        fontsize=11,
        color=BLUE,
    )
    ax.text(
        1.3,
        0.62,
        f"cell {hi}  =  tap {hi} − tap {lo}  =  true value − ε",
        fontsize=11,
        color=ORANGE,
    )
    ax.text(
        1.3,
        0.1,
        "One faulty tap therefore corrupts exactly two adjacent cell readings by equal and opposite\n"
        "amounts. Their sum remains correct — the observed signature.",
        fontsize=9.5,
    )
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def fig_detail(df, cell_a, cell_b, baseline, pm, fix_t, out, pad: float = 25.0) -> None:
    timestamps = df[ELAPSED].values
    events = ([min(pm), max(pm)] if pm else []) + ([fix_t] if fix_t else [])
    if not events:
        lo_t, hi_t = timestamps[0], timestamps[-1]
    else:
        lo_t, hi_t = max(timestamps[0], min(events) - 10), min(timestamps[-1], max(events) + 15)
    mask = (timestamps >= lo_t - pad / 3) & (timestamps <= hi_t + pad / 2)
    fig, (ax_a, ax_b) = plt.subplots(
        2, 1, figsize=(10, 6.8), sharex=True, gridspec_kw={"height_ratios": [1, 1.3]}
    )
    ax_a.plot(timestamps[mask], df[CELL_DELTA][mask], color=RED, lw=1.3)
    ax_a.set_ylabel("Voltage spread, mV")
    ax_a.set_ylim(-60, 2050)
    for timestamp in pm:
        ax_a.axvline(timestamp, color=AMBER, lw=1, ls=":")
    if fix_t:
        ax_a.axvline(fix_t, color=GREEN, lw=1.8)

    ax_b.plot(timestamps[mask], df[cell_a][mask], color=BLUE, lw=1.3, label=cell_label(cell_a))
    ax_b.plot(timestamps[mask], df[cell_b][mask], color=ORANGE, lw=1.3, label=cell_label(cell_b))
    ax_b.plot(
        timestamps[mask],
        baseline[mask],
        color=GRAY,
        lw=1,
        ls="--",
        label="remaining 8 cells (mean)",
    )
    ax_b.set_ylim(baseline[mask].min() - 900, baseline[mask].max() + 1300)
    ax_b.set_ylabel("Cell voltage, mV")
    ax_b.set_xlabel("Elapsed time, s")
    ax_b.legend(loc="lower left", frameon=False, fontsize=9)
    if fix_t:
        ax_b.axvline(fix_t, color=GREEN, lw=1.8)
    ax_b.text(
        0.01,
        0.985,
        "affected cells deviate in mirror image around the true value —\n"
        "one faulty shared tap, not two degraded cells",
        fontsize=8.7,
        color=GRAY,
        style="italic",
        va="top",
        transform=ax_b.transAxes,
    )
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def fig_evidence(df, cell_a, cell_b, baseline, out) -> None:
    timestamps = df[ELAPSED].values
    deviation_a = (df[cell_a] - baseline).values
    deviation_b = (df[cell_b] - baseline).values
    correlation = np.corrcoef(deviation_a, deviation_b)[0, 1]
    fig, (ax_a, ax_b) = plt.subplots(
        1, 2, figsize=(10, 4.6), gridspec_kw={"width_ratios": [1.15, 1]}
    )
    scatter = ax_a.scatter(deviation_a, deviation_b, c=timestamps, cmap="viridis", s=10, alpha=0.85)
    limit = max(50, np.abs(np.r_[deviation_a, deviation_b]).max() * 1.1)
    ax_a.plot(
        [-limit, limit],
        [limit, -limit],
        color=RED,
        lw=1,
        ls="--",
        alpha=0.6,
        label="exact mirror (slope −1)",
    )
    ax_a.set_xlim(-limit, limit)
    ax_a.set_ylim(-limit, limit)
    ax_a.set_aspect("equal")
    ax_a.set_xlabel(f"{cell_label(cell_a)} deviation from baseline, mV")
    ax_a.set_ylabel(f"{cell_label(cell_b)} deviation from baseline, mV")
    ax_a.legend(loc="lower left", frameon=False, fontsize=8.5)
    ax_a.text(
        0.03,
        0.92,
        f"r = {correlation:+.3f}",
        transform=ax_a.transAxes,
        fontsize=12,
        fontweight="bold",
        color=RED,
    )
    fig.colorbar(scatter, ax=ax_a, fraction=0.046, pad=0.04).set_label("elapsed time, s", fontsize=8.5)

    stds = [np.std(deviation_a), np.std(deviation_b), np.std((df[cell_a] + df[cell_b]).values)]
    bars = ax_b.bar(["alone", "alone", "sum"], stds, color=[BLUE, ORANGE, GREEN], width=0.6)
    ax_b.set_xticks(range(3))
    ax_b.set_xticklabels([cell_label(cell_a), cell_label(cell_b), "sum"])
    ax_b.set_ylabel("Std. deviation, mV")
    ax_b.set_title("Error cancels in the sum", fontsize=10.5)
    for bar, value in zip(bars, stds):
        ax_b.text(
            bar.get_x() + bar.get_width() / 2,
            value + max(stds) * 0.03,
            f"{value:.0f}",
            ha="center",
            fontsize=10,
        )
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def fig_soc(df, fix_t, spans, out) -> None:
    """Use toolkit-reported SOC and BMS voltage/coulomb derived estimates."""
    timestamps = df[ELAPSED].values
    reported = df[SOC_REPORTED].astype(float).values
    voltage = df[SOC_VOLTAGE].astype(float).values
    coulomb = df[SOC_COULOMB].astype(float).values
    r_voltage = np.corrcoef(reported, voltage)[0, 1]
    r_coulomb = np.corrcoef(reported, coulomb)[0, 1]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6.8), sharex=True)
    for axis in (ax1, ax2):
        fault_shade(axis, fix_t, timestamps[-1], spans)

    ax1.plot(timestamps, voltage, color=PURPLE, lw=1.2, label="voltage-derived SOC (toolkit)")
    ax1.plot(timestamps, coulomb, color=ORANGE, lw=1.2, label="coulomb-derived SOC (toolkit)")
    ax1.plot(timestamps, reported, color=BLUE, lw=1.4, label="BMS reported SOC")
    ax1.set_ylabel("SOC, %")
    ax1.set_ylim(-5, 112)
    ax1.legend(loc="upper right", frameon=False, fontsize=8.5)
    ax1.text(
        4,
        101,
        f"reported vs voltage-derived r = {r_voltage:+.2f}; "
        f"vs coulomb-derived r = {r_coulomb:+.2f}",
        fontsize=8.5,
        color="#444441",
    )

    gap = reported - voltage
    ax2.plot(timestamps, gap, color="#333330", lw=1.1)
    ax2.fill_between(timestamps, gap, 0, color=RED, alpha=0.08)
    ax2.axhline(0, color=GRAY, lw=0.8)
    if fix_t:
        mean_correction = gap[timestamps > fix_t].mean()
        ax2.plot([fix_t, timestamps[-1]], [mean_correction] * 2, color=GREEN, lw=1.5, ls="--")
        ax2.annotate(
            f"post-resolution offset {mean_correction:.0f} pts:\n"
            "SOC latched low; recalibration takes hours",
            xy=(timestamps[-1] * 0.88, mean_correction + 2),
            xytext=(timestamps[0] + 50, -18),
            fontsize=8.5,
            va="top",
            color=GREEN,
            arrowprops=dict(arrowstyle="->", color=GREEN, lw=1),
        )
    ax2.set_ylabel("Reported − voltage-derived\nSOC, pts")
    ax2.set_xlabel("Elapsed time, s")
    ax2.set_ylim(min(-108, gap.min() - 8), max(0, gap.max() + 5))
    for axis in (ax1, ax2):
        axis.set_xlim(0, timestamps[-1] + 2)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def fig_healthy(hdf, out) -> None:
    timestamps = hdf[ELAPSED].values
    spans = rest_spans(hdf)
    fig, (ax1, ax2, ax3) = plt.subplots(
        3, 1, figsize=(10, 7), sharex=True, gridspec_kw={"height_ratios": [1, 1.3, 1]}
    )
    for axis in (ax1, ax2, ax3):
        for start, end in spans:
            axis.axvspan(start, end, color=GRAY, alpha=0.14, lw=0)

    ax1.plot(timestamps, hdf[CURRENT], color="#5a5a56", lw=0.9)
    ax1.set_ylabel("Pack current, A")

    ax2.plot(timestamps, hdf[CELL_DELTA], color=GREEN, lw=1.2)
    ax2.set_ylabel("Cell voltage spread\n(max − min), mV")
    ax2.set_ylim(0, 2050)
    delta_max = hdf[CELL_DELTA].max()
    ax2.annotate(
        f"spread stays within {hdf[CELL_DELTA].min()}–{delta_max} mV under full load\n"
        "(same axis scale as the faulty unit)",
        xy=(timestamps[len(timestamps) // 2], delta_max),
        xytext=(20, 1500),
        fontsize=8.5,
        color=GREEN,
        arrowprops=dict(arrowstyle="->", color=GREEN, lw=1),
    )

    ax3.plot(timestamps, hdf[SOC_REPORTED], color=BLUE, lw=1.3)
    ax3.set_ylabel("Reported SOC, %")
    ax3.set_ylim(-5, 112)
    ax3.set_xlabel("Elapsed time, s")
    for axis in (ax1, ax2, ax3):
        axis.set_xlim(0, timestamps[-1] + 2)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def fig_sidebyside(df, hdf, fix_t, out) -> None:
    timestamps = df[ELAPSED].values
    healthy_timestamps = hdf[ELAPSED].values
    fig, ((healthy_a, fault_a), (healthy_b, fault_b)) = plt.subplots(
        2, 2, figsize=(10, 6.4), sharex="col", gridspec_kw={"height_ratios": [1.3, 1]}
    )
    healthy_a.set_title(f"Healthy unit ({hdf[SERIAL].iloc[0]})", fontsize=10.5, color=GREEN)
    fault_a.set_title(f"Faulty unit ({df[SERIAL].iloc[0]})", fontsize=10.5, color=RED)
    healthy_a.plot(healthy_timestamps, hdf[CELL_DELTA], color=GREEN, lw=1.2)
    fault_a.plot(timestamps, df[CELL_DELTA], color=RED, lw=1.0)
    for axis in (healthy_a, fault_a):
        axis.set_ylim(-50, 1850)
    healthy_a.set_ylabel("Cell voltage spread, mV")
    healthy_b.plot(healthy_timestamps, hdf[SOC_REPORTED], color=BLUE, lw=1.3)
    fault_b.plot(timestamps, df[SOC_REPORTED], color=BLUE, lw=1.1)
    for axis in (healthy_b, fault_b):
        axis.set_ylim(-5, 112)
    healthy_b.set_ylabel("Reported SOC, %")
    healthy_b.set_xlabel("Elapsed time, s")
    fault_b.set_xlabel("Elapsed time, s")
    for axis in (healthy_a, healthy_b):
        axis.set_xlim(0, healthy_timestamps[-1] + 2)
    for axis in (fault_a, fault_b):
        axis.set_xlim(0, timestamps[-1] + 2)
    if fix_t:
        for axis in (fault_a, fault_b):
            axis.axvline(fix_t, color=GREEN, lw=1.4)
        fault_a.text(fix_t + 5, 1650, "resolution", fontsize=8, color=GREEN)
    fig.suptitle(
        "Same toolkit, same day, comparable load — identical axes",
        fontsize=10,
        color="#444441",
        y=0.99,
    )
    fig.tight_layout(rect=[0, 0, 1, 0.965])
    fig.savefig(out, dpi=150)
    plt.close(fig)


# ------------------------------------------------------------------ main
def main() -> None:
    parser = argparse.ArgumentParser(
        description="Standalone miliVoltron battery CSV figure generator"
    )
    parser.add_argument("fault_csv")
    parser.add_argument("healthy_csv", nargs="?")
    parser.add_argument("-o", "--outdir", default=".")
    args = parser.parse_args()
    os.makedirs(args.outdir, exist_ok=True)
    output = lambda name: os.path.join(args.outdir, name)

    print("fault session:")
    fault = load(args.fault_csv)
    spans = rest_spans(fault)
    indexes, cell_a, cell_b, baseline, _correlation = detect_pair(fault)
    pm_events, fix_t = detect_events(fault)

    fig_overview(fault, pm_events, fix_t, spans, output("fig1_overview.png"))
    fig_schematic(indexes, output("fig2_schematic.png"))
    fig_detail(fault, cell_a, cell_b, baseline, pm_events, fix_t, output("fig3_detail.png"))
    fig_evidence(fault, cell_a, cell_b, baseline, output("fig4_evidence.png"))
    fig_soc(fault, fix_t, spans, output("fig5_soc.png"))

    if args.healthy_csv:
        print("healthy session:")
        healthy = load(args.healthy_csv, drop_from_first_charge=True)
        fig_healthy(healthy, output("fig6_healthy.png"))
        fig_sidebyside(fault, healthy, fix_t, output("fig7_sidebyside.png"))
    print("done ->", args.outdir)


if __name__ == "__main__":
    main()
