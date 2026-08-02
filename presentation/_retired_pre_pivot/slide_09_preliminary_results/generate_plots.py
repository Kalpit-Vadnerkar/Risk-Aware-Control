"""
Generate preliminary result plots for advisor presentation.
Run from the Risk-Aware-Control root:
    python3 presentation/slide_09_preliminary_results/generate_plots.py

Plots are saved into their respective slide directories:
  slide_09a_timeseries/nll_timeseries.png
  slide_09b_boxplots/boxplot_metrics.png
  slide_09c_single_run/single_run_comparison.png
"""

import os
import glob
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from scipy.stats import mannwhitneyu
from scipy.ndimage import uniform_filter1d

PRES_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR  = os.path.dirname(os.path.dirname(PRES_DIR))
RES_DIR   = os.path.join(ROOT_DIR, 'st_gat', 'results', 'residuals')

OUT_09A = os.path.join(PRES_DIR, '..', 'slide_09a_timeseries')
OUT_09B = os.path.join(PRES_DIR, '..', 'slide_09b_boxplots')
OUT_09C = os.path.join(PRES_DIR, '..', 'slide_09c_single_run')

BLUE         = '#1565C0'
ORANGE       = '#E65100'
LIGHT_BLUE   = '#90CAF9'
LIGHT_ORANGE = '#FFCC80'
GRAY         = '#757575'
RED          = '#B71C1C'
GREEN        = '#2E7D32'

NLL_THRESHOLD = -6.37   # 3σ anomaly threshold from nominal runs

plt.rcParams.update({
    'font.family': 'DejaVu Sans',
    'font.size': 13,
    'axes.titlesize': 14,
    'axes.labelsize': 13,
    'legend.fontsize': 12,
    'figure.dpi': 150,
})


def load_runs(dataset):
    paths = sorted(glob.glob(os.path.join(RES_DIR, dataset, '*.csv')))
    return [pd.read_csv(p) for p in paths]


def interpolate_to_grid(series, n=200):
    x_old = np.linspace(0, 1, len(series))
    x_new = np.linspace(0, 1, n)
    return np.interp(x_new, x_old, series)


def _find_event_windows(df):
    """Return (obstacle_detected_window, avoidance_start_window, avoidance_peak_window)."""
    dist = df['obj_dist_actual'].values
    nll  = df['combined_nll'].values

    # Obstacle detected: first window where obj_dist drops below 0.45 and stays for 5 consecutive
    det_w = None
    for i in range(len(dist) - 5):
        if all(dist[i:i+5] < 0.45):
            det_w = i
            break

    # Avoidance start: first NLL crossing above threshold (building toward spike)
    avoid_start = None
    for i in range(len(nll)):
        if nll[i] > NLL_THRESHOLD:
            avoid_start = i
            break

    # Avoidance peak: argmax NLL
    avoid_peak = int(np.argmax(nll))

    return det_w, avoid_start, avoid_peak


# ── Plot 1: NLL time-series (normalized run time) ───────────────────────────

def plot_timeseries():
    fig, ax = plt.subplots(figsize=(9, 4.5))

    N = 200
    for label, dataset, color, lcolor in [
        ('obs_recovery (avoidance enabled)',  'obs_recovery',  BLUE,   LIGHT_BLUE),
        ('obs_noescape (avoidance disabled)', 'obs_noescape', ORANGE, LIGHT_ORANGE),
    ]:
        runs = load_runs(dataset)
        grids = []
        for df in runs:
            nll      = df['combined_nll'].values.astype(float)
            smoothed = uniform_filter1d(nll, size=30, mode='nearest')
            grids.append(interpolate_to_grid(smoothed, N))
        mat  = np.stack(grids)
        mean = mat.mean(axis=0)
        std  = mat.std(axis=0)
        x    = np.linspace(0, 1, N)
        ax.plot(x, mean, color=color, lw=2.0, label=label)
        ax.fill_between(x, mean - std, mean + std, color=color, alpha=0.18)

    ax.axhline(NLL_THRESHOLD, color='gray', lw=1.2, ls='--', alpha=0.7,
               label=f'3σ anomaly threshold ({NLL_THRESHOLD})')
    ax.set_xlabel('Normalized Run Time  (0 = start, 1 = end / stuck)')
    ax.set_ylabel('Combined NLL  (higher = more anomalous)')
    ax.set_title('ST-GAT Residual Over Run Time\nMean ± 1 SD across 18 runs per condition')
    ax.legend(loc='upper left')
    ax.set_xlim(0, 1)
    ax.grid(True, alpha=0.3)

    ax.annotate('Avoidance maneuver\n(lane change spike)',
                xy=(0.28, -9.8), xytext=(0.48, -8.5),
                arrowprops=dict(arrowstyle='->', color=BLUE, lw=1.5),
                color=BLUE, fontsize=11)

    fig.tight_layout()
    out = os.path.join(OUT_09A, 'nll_timeseries.png')
    fig.savefig(out, bbox_inches='tight')
    print(f'Saved: {out}')
    plt.close(fig)


# ── Plot 2: Box plots of key run-level statistics ───────────────────────────

def plot_boxplots():
    summary = pd.read_csv(os.path.join(RES_DIR, 'summary.csv'))
    rec = summary[summary.dataset == 'obs_recovery']
    noe = summary[summary.dataset == 'obs_noescape']

    metrics = [
        ('max_combined_nll',  'Max NLL\n(per run)',           True),
        ('cvar95_combined',   'CVaR₉₅ NLL\n(full run)',       False),
        ('frac_adjacent_lane','Frac. windows\nadjacent lane', True),
    ]

    fig, axes = plt.subplots(1, 3, figsize=(11, 4.5))

    for ax, (col, ylabel, rec_greater) in zip(axes, metrics):
        r = rec[col].dropna().values
        n = noe[col].dropna().values

        bp = ax.boxplot(
            [r, n],
            labels=['Recovery\n(n=18)', 'Noescape\n(n=18)'],
            patch_artist=True,
            medianprops=dict(color='white', lw=2.5),
            whiskerprops=dict(lw=1.5),
            capprops=dict(lw=1.5),
            flierprops=dict(marker='o', markersize=5),
        )
        bp['boxes'][0].set_facecolor(BLUE)
        bp['boxes'][1].set_facecolor(ORANGE)

        _, p = mannwhitneyu(r, n, alternative='greater' if rec_greater else 'less')
        sig   = '**' if p < 0.05 else ('*' if p < 0.10 else 'n.s.')
        y_max = max(r.max(), n.max())
        y_min = min(r.min(), n.min())
        rng   = y_max - y_min
        bar_y = y_max + 0.05 * rng
        ax.plot([1, 2], [bar_y, bar_y], color='black', lw=1.2)
        ax.text(1.5, bar_y + 0.01 * rng, f'p={p:.3f} {sig}',
                ha='center', va='bottom', fontsize=11)

        ax.set_ylabel(ylabel)
        ax.grid(True, axis='y', alpha=0.3)
        ax.set_title(col.replace('_', ' ').title())

    fig.suptitle('Per-Run Statistics: obs_recovery vs. obs_noescape', fontsize=13, y=1.02)
    fig.tight_layout()
    out = os.path.join(OUT_09B, 'boxplot_metrics.png')
    fig.savefig(out, bbox_inches='tight')
    print(f'Saved: {out}')
    plt.close(fig)


# ── Plot 3: Single representative run — NLL + event markers ─────────────────

def plot_single_run():
    rec_paths = sorted(glob.glob(os.path.join(RES_DIR, 'obs_recovery', '*.csv')))
    noe_paths = sorted(glob.glob(os.path.join(RES_DIR, 'obs_noescape', '*.csv')))

    rec_df = max((pd.read_csv(p) for p in rec_paths), key=len)
    noe_df = sorted([pd.read_csv(p) for p in noe_paths], key=len)[len(noe_paths) // 2]

    rec_det, rec_avoid_start, rec_avoid_peak = _find_event_windows(rec_df)
    noe_det, noe_avoid_start, noe_avoid_peak = _find_event_windows(noe_df)

    fig, axes = plt.subplots(2, 2, figsize=(14, 6.5), sharex='col')

    for col_idx, (df, color, title, det_w, avoid_start, avoid_peak, is_recovery) in enumerate([
        (rec_df, BLUE,   'obs_recovery — avoidance enabled (vehicle reached goal)',
         rec_det, rec_avoid_start, rec_avoid_peak, True),
        (noe_df, ORANGE, 'obs_noescape — avoidance disabled (vehicle stuck)',
         noe_det, noe_avoid_start, noe_avoid_peak, False),
    ]):
        nll = uniform_filter1d(df['combined_nll'].values, size=20, mode='nearest')
        adj = df['has_adjacent_lane'].values.astype(float)
        x   = np.arange(len(nll))

        # --- Top panel: NLL ---
        ax0 = axes[0, col_idx]
        ax0.plot(x, nll, color=color, lw=1.2, alpha=0.9, zorder=3)
        ax0.axhline(NLL_THRESHOLD, color='gray', ls='--', lw=1.4, alpha=0.8,
                    label=f'3σ threshold ({NLL_THRESHOLD})', zorder=2)
        ax0.set_ylabel('Combined NLL\n(higher = more anomalous)')
        ax0.set_title(title, fontsize=11, pad=6)
        ax0.grid(True, alpha=0.3)

        if is_recovery:
            # Obstacle detected: vertical dashed line
            if det_w is not None:
                ax0.axvline(det_w, color=RED, ls=':', lw=1.8, zorder=4,
                            label=f'Obstacle detected (w={det_w})')
                ax0.text(det_w + 5, ax0.get_ylim()[1] if ax0.get_ylim()[1] != 1.0 else -3.5,
                         'Obstacle\ndetected', color=RED, fontsize=9, va='top')

            # Avoidance maneuver: shaded region from avoid_start to avoid_peak+10
            if avoid_start is not None:
                shade_end = avoid_peak + 10 if avoid_peak is not None else avoid_start + 30
                ax0.axvspan(avoid_start, shade_end, color=BLUE, alpha=0.15, zorder=1)
                ax0.axvline(avoid_start, color=BLUE, ls='-', lw=1.8, alpha=0.7, zorder=4)
                ax0.text(avoid_start + 3, -4.5, 'Avoidance\nmaneuver', color=BLUE,
                         fontsize=9, va='top', fontweight='bold')

                # NLL peak annotation
                if avoid_peak is not None:
                    ax0.annotate(
                        f'Peak NLL = {df["combined_nll"].iloc[avoid_peak]:.1f}',
                        xy=(avoid_peak, nll[avoid_peak]),
                        xytext=(avoid_peak + 60, nll[avoid_peak] - 1.5),
                        arrowprops=dict(arrowstyle='->', color=BLUE, lw=1.4),
                        color=BLUE, fontsize=10,
                    )

            ax0.legend(loc='upper right', fontsize=9)

        else:
            # Noescape: annotate that avoidance is policy-disabled
            ax0.text(0.98, 0.95, 'Avoidance policy: OFF\n(no lane change possible)',
                     transform=ax0.transAxes, ha='right', va='top',
                     fontsize=9, color=ORANGE,
                     bbox=dict(boxstyle='round,pad=0.3', facecolor='#FFF3E0', edgecolor=ORANGE, alpha=0.9))
            if det_w is not None:
                ax0.axvline(det_w, color=RED, ls=':', lw=1.8, alpha=0.6,
                            label=f'Obstacle in view: w≥{det_w}')
                ax0.text(det_w + 2, -5.5, 'Obstacle in\nview', color=RED, fontsize=9)
            ax0.legend(loc='upper right', fontsize=9)

        # --- Bottom panel: has_adjacent_lane ---
        ax1 = axes[1, col_idx]
        ax1.fill_between(x, 0, adj, color=color, alpha=0.35, step='post')
        ax1.step(x, adj, color=color, lw=1.5, where='post')

        if is_recovery and det_w is not None:
            ax1.axvline(det_w, color=RED, ls=':', lw=1.8, alpha=0.6)
        if is_recovery and avoid_start is not None:
            shade_end = avoid_peak + 10 if avoid_peak is not None else avoid_start + 30
            ax1.axvspan(avoid_start, shade_end, color=BLUE, alpha=0.12)
            ax1.axvline(avoid_start, color=BLUE, ls='-', lw=1.8, alpha=0.5)

        ax1.set_ylim(-0.05, 1.3)
        ax1.set_yticks([0, 1])
        ax1.set_yticklabels(['No adj.', 'Adj. lane\nexists'])
        ax1.set_xlabel('Window index  (1 window = 0.1 s at 10 Hz)')
        ax1.set_ylabel('has_adjacent_lane')
        ax1.grid(True, alpha=0.3)

    fig.suptitle('Single-Run Deep Dive: NLL Residual Timeline with Event Markers', fontsize=13)
    fig.tight_layout()
    out = os.path.join(OUT_09C, 'single_run_comparison.png')
    fig.savefig(out, bbox_inches='tight')
    print(f'Saved: {out}')
    plt.close(fig)


if __name__ == '__main__':
    plot_timeseries()
    plot_boxplots()
    plot_single_run()
    print('All plots generated.')
