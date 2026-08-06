#!/usr/bin/env python3
"""
Plots what the SPRT/sequential-evidence signal (p_fault_motion, p_fault_tl,
p_fault_combined in st_gat/results/h30_30/traces/*.csv) is actually
measuring, and whether it behaves sensibly -- step 2 of the 2026-08-06
reframe (see docs/research_notes/ and the memory this session was briefed
against). Deliberately NOT framed as "does it cross a threshold" -- there is
no threshold anywhere in this script. The question is whether the continuous
trace is interpretable: does it stay near its floor under real nominal
noise, does it rise under a real fault, and does its SHAPE (which
sub-signal moves, how fast, whether it recovers) say something useful.

One thing to keep in mind reading these plots: p_fault_* = sigmoid(S_t)
where S_t is a Page/Lorden reset-at-zero SPRT accumulator (S_t >= 0
always -- see st_gat/residuals.py's _sprt() docstring). That means the
FLOOR of this signal is sigmoid(0) = 0.5, not 0.0 -- "quiet" looks like
~0.5, not ~0.0. Every y-axis in this script is scaled [0.45, 1.0] for
that reason, not [0, 1].

Produces two figures in --output-dir (default
experiments/analysis/sprt_signal_behavior/):

1. nominal_floor.png -- every nom_v11 trial's p_fault_motion / p_fault_tl /
   p_fault_combined trace overlaid against wall-clock time since trial
   start. Answers: does the signal stay near the floor for real nominal
   driving, or does it drift/false-alarm over a long nominal trial?
2. fault_reaction_grid.png -- one panel per fault campaign (all 8:
   imu_fault_s1/s3/scale/stuck, tl_fault_s2/s3/s4/ramp), x-axis = seconds
   relative to fault onset (t_rel_fault, so every trial's onset aligns at
   t=0 regardless of when the fault actually armed in that trial) --
   p_fault_motion (blue) and p_fault_tl (orange) plotted per trial, against
   a shaded nominal reference band (pooled 5th-95th percentile from panel
   1, not time-aligned -- nominal trials have no fault onset to align to,
   so this is a static reference range, not a matched baseline). Fault
   onset, permanent-stop time, and first MRM trigger (converted from
   trial-relative to fault-relative using each trial's own onset offset)
   are marked as vertical lines where present. The point of plotting
   motion and tl separately (not just combined) is the selectivity check:
   the discriminability test already established TL/camera faults should
   selectively elevate the tl branch and IMU faults the motion branch --
   this is what that looks like as a continuous trace instead of a
   single number.

Usage (no ROS needed -- pure pandas over the already-computed trace CSVs):
  source .venv/bin/activate   # only for pandas/matplotlib if not on system python
  python3 experiments/scripts/plot_sprt_signal_behavior.py
"""

import argparse
import glob
import math
import os
import sys

import numpy as np
import pandas as pd

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))


def mark_event_time(ax, t, color='black', linestyle=':', linewidth=1.5, zorder=1, label=None):
    """Same primitive as experiments/lib/plotting.py's mark_event_time --
    inlined instead of imported so this script stays free of plotting.py's
    module-level `import lanelet2` (needed there for map loading, not for
    this one function) and can run without ROS/Autoware sourced at all."""
    if t is None or (isinstance(t, float) and math.isnan(t)):
        return
    ax.axvline(t, color=color, linestyle=linestyle, linewidth=linewidth, zorder=zorder, label=label)


DEFAULT_TRACES_DIR = os.path.join(REPO_DIR, 'st_gat', 'results', 'h30_30', 'traces')
DEFAULT_OUTPUT_DIR = os.path.join(REPO_DIR, 'experiments', 'analysis', 'sprt_signal_behavior')

FAULT_CAMPAIGNS = [
    'imu_fault_s1', 'imu_fault_s3', 'imu_fault_scale', 'imu_fault_stuck',
    'tl_fault_s2', 'tl_fault_s3', 'tl_fault_s4', 'tl_fault_ramp',
]

_SIGNAL_COLS = ['p_fault_motion', 'p_fault_tl', 'p_fault_combined']
_YLIM = (0.45, 1.0)
_FLOOR = 0.5


def _load_campaign(traces_dir, campaign):
    paths = sorted(glob.glob(os.path.join(traces_dir, campaign, '*.csv')))
    trials = []
    for p in paths:
        df = pd.read_csv(p)
        if df.empty or 't_bag_rel' not in df.columns:
            continue
        name = os.path.splitext(os.path.basename(p))[0]
        trials.append((name, df))
    return trials


def plot_nominal_floor(nominal_trials, out_path):
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=False)
    colors = plt.cm.viridis(np.linspace(0, 0.9, len(nominal_trials)))
    for ax, col, title in zip(axes, _SIGNAL_COLS,
                               ['p_fault_motion', 'p_fault_tl', 'p_fault_combined']):
        for (name, df), c in zip(nominal_trials, colors):
            ax.plot(df['t_bag_rel'], df[col], color=c, linewidth=0.9, alpha=0.55)
        ax.axhline(_FLOOR, color='#333333', linestyle=':', linewidth=1, label='SPRT floor (0.5)')
        ax.set_ylabel(title, fontsize=9)
        ax.set_ylim(*_YLIM)
        ax.tick_params(labelsize=8)
    axes[0].set_title(f'nom_v11: all {len(nominal_trials)} trials overlaid -- what "quiet" looks like', fontsize=11)
    axes[0].legend(loc='upper right', fontsize=8)
    axes[-1].set_xlabel('seconds since trial start', fontsize=9)
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print(f'Saved {out_path}  ({len(nominal_trials)} nominal trials)')


def _nominal_band(nominal_trials, col, lo=5, hi=95):
    vals = np.concatenate([df[col].dropna().values for _, df in nominal_trials])
    return np.percentile(vals, lo), np.percentile(vals, hi)


def _onset_offset(df):
    """Seconds from trial start to fault onset (t_bag_rel - t_rel_fault at
    any row where t_rel_fault is defined) -- lets permanent_stop_rel_s /
    mrm_first_trigger_rel_s (both trial-start-relative) be converted onto
    the same fault-onset-relative x-axis the trace itself uses."""
    valid = df['t_rel_fault'].notna()
    if not valid.any():
        return None
    row = df.loc[valid].iloc[0]
    return float(row['t_bag_rel'] - row['t_rel_fault'])


def plot_fault_reaction_grid(nominal_trials, campaigns_data, out_path):
    band_motion = _nominal_band(nominal_trials, 'p_fault_motion')
    band_tl     = _nominal_band(nominal_trials, 'p_fault_tl')

    n = len(campaigns_data)
    ncols = 2
    nrows = (n + ncols - 1) // ncols
    fig, axes = plt.subplots(nrows, ncols, figsize=(13, 3.6 * nrows), sharey=True)
    axes = np.atleast_1d(axes).flatten()

    for ax, (campaign, trials) in zip(axes, campaigns_data.items()):
        ax.axhspan(*band_motion, color='#1f77b4', alpha=0.10, zorder=0)
        ax.axhspan(*band_tl,     color='#ff7f0e', alpha=0.10, zorder=0)
        ax.axhline(_FLOOR, color='#333333', linestyle=':', linewidth=0.8, zorder=1)

        if not trials:
            ax.set_title(f'{campaign}  (no trace data)', fontsize=10)
            continue

        for name, df in trials:
            offset = _onset_offset(df)
            x = df['t_rel_fault']
            ax.plot(x, df['p_fault_motion'], color='#1f77b4', linewidth=1.3, alpha=0.85, zorder=3)
            ax.plot(x, df['p_fault_tl'],     color='#ff7f0e', linewidth=1.3, alpha=0.85, zorder=3)

            if offset is not None:
                stop_col = 'permanent_stop_rel_s'
                mrm_col  = 'mrm_first_trigger_rel_s'
                stop_s = df[stop_col].dropna().iloc[0] if stop_col in df.columns and df[stop_col].notna().any() else None
                mrm_s  = df[mrm_col].dropna().iloc[0] if mrm_col in df.columns and df[mrm_col].notna().any() else None
                if stop_s is not None:
                    mark_event_time(ax, stop_s - offset, color='black', linestyle='--', linewidth=1.2, zorder=4)
                if mrm_s is not None:
                    mark_event_time(ax, mrm_s - offset, color='#9467bd', linestyle='--', linewidth=1.2, zorder=4)

        mark_event_time(ax, 0.0, color='#d62728', linestyle='-', linewidth=1.3, zorder=5)
        ax.set_title(f'{campaign}  (n={len(trials)} trials)', fontsize=10)
        ax.set_ylim(*_YLIM)
        ax.tick_params(labelsize=8)

    for ax in axes[n:]:
        ax.axis('off')

    # Shared legend, built by hand (colors are consistent across every panel).
    from matplotlib.lines import Line2D
    handles = [
        Line2D([0], [0], color='#1f77b4', lw=1.5, label='p_fault_motion'),
        Line2D([0], [0], color='#ff7f0e', lw=1.5, label='p_fault_tl'),
        Line2D([0], [0], color='#d62728', lw=1.5, label='fault onset (t=0)'),
        Line2D([0], [0], color='black', lw=1.5, linestyle='--', label='permanent stop'),
        Line2D([0], [0], color='#9467bd', lw=1.5, linestyle='--', label='MRM trigger'),
        plt.Rectangle((0, 0), 1, 1, color='#1f77b4', alpha=0.10, label='nominal motion band (5-95%ile)'),
        plt.Rectangle((0, 0), 1, 1, color='#ff7f0e', alpha=0.10, label='nominal tl band (5-95%ile)'),
    ]
    fig.legend(handles=handles, loc='lower center', ncol=4, fontsize=8.5, bbox_to_anchor=(0.5, -0.02))
    fig.suptitle('SPRT signal reaction to each fault campaign, aligned to fault onset (t=0)', fontsize=13)
    fig.text(0.02, 0.5, 'p_fault (sigmoid of accumulated log-likelihood-ratio; floor = 0.5)',
              va='center', rotation='vertical', fontsize=9)
    fig.tight_layout(rect=[0.03, 0.03, 1, 0.96])
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print(f'Saved {out_path}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--traces-dir', default=DEFAULT_TRACES_DIR)
    ap.add_argument('--output-dir', default=DEFAULT_OUTPUT_DIR)
    args = ap.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    nominal_trials = _load_campaign(args.traces_dir, 'nom_v11')
    if not nominal_trials:
        print('No nom_v11 traces found -- nothing to plot.', file=sys.stderr)
        sys.exit(1)

    campaigns_data = {c: _load_campaign(args.traces_dir, c) for c in FAULT_CAMPAIGNS}
    missing = [c for c, t in campaigns_data.items() if not t]
    if missing:
        print(f'WARNING: no trace data found for: {", ".join(missing)} '
              f'(will still appear in the grid, empty)', file=sys.stderr)

    plot_nominal_floor(nominal_trials, os.path.join(args.output_dir, 'nominal_floor.png'))
    plot_fault_reaction_grid(nominal_trials, campaigns_data,
                              os.path.join(args.output_dir, 'fault_reaction_grid.png'))


if __name__ == '__main__':
    main()
