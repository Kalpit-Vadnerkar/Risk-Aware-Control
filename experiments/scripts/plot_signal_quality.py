#!/usr/bin/env python3
"""
Signal-quality artifacts for the dissertation/paper and the slide deck —
NOT another detection-accuracy comparison. Packages three things built this
session into presentation-ready tables/plots:

  1. Calibration tightening: n_calib/threshold/achievable-alpha before vs.
     after extending turn_zones.json/tl_zones.json from 3 goals to all 26
     nominal goals (experiments/scripts/compute_turn_zones.py /
     compute_tl_zones.py, analysis-only — no new experiments run).
  2. Per-campaign detection rate + lead time (experiments/scripts/
     conformal_lead_time.py's campaign_summary.csv), annotated with the two
     things that make the raw numbers legible: imu_fault_s1 is a deliberate
     negative control (low detection rate there is a PASS, not a miss), and
     no TL fault trial has reached a real fatal moment yet (lead time is
     undefined for TL faults in the current dataset — a data gap, not a
     detector failure).
  3. Discriminability rank table (experiments/scripts/
     analyze_fault_discriminability.py's per_campaign_rank.csv), annotated
     with the negative-evidence-subtype finding: traffic_light_discrepancy_
     residual only ranks #1 for tl_fault_s3/s4 (perception reports
     UNKNOWN/nothing — a real map-vs-perception categorical mismatch), not
     for tl_fault_s2/ramp (perception reports a present-but-wrong value —
     not a negative-evidence case by construction).
  4. One representative CUSUM trace (imu_fault_s3/goal_012) showing the
     calibrated threshold crossing well before the fatal moment — the
     single clearest "this is what calibrated lead time looks like" visual.

Usage (no ROS needed — reads already-computed CSVs only):
  source .venv/bin/activate
  python3 experiments/scripts/plot_signal_quality.py
"""

import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pandas as pd

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
sys.path.insert(0, REPO_DIR)

from st_gat.pipeline import config as cfg  # noqa: E402

ANALYSIS_DIR = os.path.join(REPO_DIR, 'experiments', 'analysis')
OUT_DIR = os.path.join(ANALYSIS_DIR, 'signal_quality')
os.makedirs(OUT_DIR, exist_ok=True)

HEADER_BG = '#263238'
GREEN  = '#A5D6A7'
RED    = '#FFCDD2'
YELLOW = '#FFF59D'
GRAY   = '#F5F5F5'


def _style_table(t, header_row_colors=None):
    t.auto_set_font_size(False)
    t.set_fontsize(9.5)
    t.scale(1.0, 2.2)
    for (row, _col), cell in t.get_celld().items():
        cell.set_edgecolor('#333333')
        cell.set_linewidth(0.7)
        if row == 0:
            cell.set_facecolor(HEADER_BG)
            cell.set_text_props(color='white', fontweight='bold', fontsize=9)


# ── 1. Calibration tightening, before vs. after ────────────────────────────

def plot_calibration_tightening():
    # "Before" numbers are from the first run (turn/TL zones only covering
    # goal_007/012/026 — the 3 goals that needed fault campaigns). Recorded
    # here as literals since that run's output isn't cached to disk anywhere
    # else; the "after" numbers are read live from the current calibration
    # file so this plot can't silently go stale relative to the real result.
    before = {'imu_leadin': (6, 552.7786, 0.1429), 'imu_turn': (6, 552.7786, 0.1429),
              'tl': (6, 116.3190, 0.1429)}
    after_df = pd.read_csv(os.path.join(ANALYSIS_DIR, 'conformal_lead_time', 'calibration_by_zone_kind.csv'),
                            index_col=0)

    cols = ['Zone kind', 'n_calib (before)', 'n_calib (after)',
            'Achievable α (before)', 'Achievable α (after)', 'Threshold (after)']
    rows = []
    for zone_kind in ['imu_leadin', 'imu_turn', 'tl']:
        n_before, _thr_before, alpha_before = before[zone_kind]
        row_after = after_df.loc[zone_kind]
        rows.append([
            zone_kind, str(n_before), str(int(row_after['n_calib'])),
            f'{alpha_before:.3f}', f'{row_after["min_achievable_alpha"]:.3f}',
            f'{row_after["threshold"]:.1f}',
        ])

    fig, ax = plt.subplots(figsize=(13, 3.2))
    ax.axis('off')
    ax.set_title('Conformal calibration tightened by extending zone coverage\n'
                  '(analysis-only — reused already-collected nom_v11 bags, no new experiments)',
                  fontsize=11, fontweight='bold', pad=20, color='#263238')
    t = ax.table(cellText=rows, colLabels=cols, loc='center', cellLoc='center',
                 colWidths=[0.14, 0.17, 0.17, 0.19, 0.19, 0.17])
    _style_table(t)
    for (row, col), cell in t.get_celld().items():
        if row == 0:
            continue
        if col in (3,):
            cell.set_facecolor(RED)
        elif col in (4,):
            cell.set_facecolor(GREEN)
    fig.text(0.5, -0.02,
              'Achievable α = 1/(n_calib+1), the tightest one-sided false-alarm rate split-conformal can\n'
              'guarantee from that many calibration points — lower is a stronger guarantee.',
              ha='center', fontsize=8.5, style='italic', color='#555555')
    out = os.path.join(OUT_DIR, 'calibration_tightening_table.png')
    fig.savefig(out, bbox_inches='tight', dpi=150)
    plt.close(fig)
    print(f'Saved {out}')


# ── 2. Per-campaign detection / lead time, annotated ───────────────────────

def plot_campaign_summary():
    import textwrap
    df = pd.read_csv(os.path.join(ANALYSIS_DIR, 'conformal_lead_time', 'campaign_summary.csv'))
    cols = ['Campaign', 'Trials', 'Detection\nrate', 'Mean lead\ntime (s)',
            'Median lead\ntime (s)', 'Trials w/\nfatal moment', 'Read']
    notes = {
        'imu_fault_s1':    'Negative control (deliberately harmless) — low rate is a PASS',
        'imu_fault_s3':    'Clean positive: every dangerous trial flagged, ~100s ahead',
        'imu_fault_scale': 'Positive signal, 1 miss (goal_012)',
        'imu_fault_stuck': 'Clean positive: every dangerous trial flagged',
        'tl_fault_ramp':   'No fatal moment yet — lead time undefined (data gap)',
        'tl_fault_s2':     'No fatal moment yet — lead time undefined (data gap)',
        'tl_fault_s3':     'No fatal moment yet — lead time undefined (data gap)',
        'tl_fault_s4':     'No fatal moment yet — lead time undefined (data gap)',
    }
    rows, row_kind = [], []
    for _, r in df.iterrows():
        c = r['campaign']
        lead_mean = f'{r["mean_lead_time_s"]:.0f}' if pd.notna(r['mean_lead_time_s']) else '—'
        lead_med  = f'{r["median_lead_time_s"]:.0f}' if pd.notna(r['median_lead_time_s']) else '—'
        note_wrapped = '\n'.join(textwrap.wrap(notes.get(c, ''), width=30))
        rows.append([c, f'{int(r["n_trials"])}', f'{r["detection_rate"]:.2f}',
                     lead_mean, lead_med, f'{int(r["n_with_fatal_moment"])}', note_wrapped])
        if c == 'imu_fault_s1':
            row_kind.append('control')
        elif r['n_with_fatal_moment'] > 0:
            row_kind.append('positive')
        else:
            row_kind.append('gap')

    fig, ax = plt.subplots(figsize=(15, 7))
    ax.axis('off')
    fig.suptitle('Per-campaign detection rate and lead time\n(α=0.1, zone-synchronized, all-26-goal calibration)',
                 fontsize=13, fontweight='bold', y=0.98, color='#263238')
    t = ax.table(cellText=rows, colLabels=cols, loc='center', cellLoc='center',
                 colWidths=[0.14, 0.07, 0.09, 0.11, 0.11, 0.11, 0.37])
    t.auto_set_font_size(False)
    t.set_fontsize(9.5)
    t.scale(1.0, 2.6)
    for (row, col), cell in t.get_celld().items():
        cell.set_edgecolor('#333333')
        cell.set_linewidth(0.7)
        if row == 0:
            cell.set_facecolor(HEADER_BG)
            cell.set_text_props(color='white', fontweight='bold', fontsize=9)
    kind_color = {'control': YELLOW, 'positive': GREEN, 'gap': GRAY}
    for (row, col), cell in t.get_celld().items():
        if row == 0:
            continue
        cell.set_facecolor(kind_color[row_kind[row - 1]])
        if col == 6:
            cell.set_text_props(style='italic', fontsize=8, ha='left')
            cell.PAD = 0.02
    fig.subplots_adjust(top=0.86, bottom=0.03)
    out = os.path.join(OUT_DIR, 'campaign_lead_time_table.png')
    fig.savefig(out, bbox_inches='tight', dpi=150)
    plt.close(fig)
    print(f'Saved {out}')


# ── 3. Discriminability rank table, with negative-evidence-subtype note ────

def plot_discriminability_rank():
    df = pd.read_csv(os.path.join(ANALYSIS_DIR, 'fault_discriminability', 'per_campaign_rank.csv'), index_col=0)
    feature_labels = {
        'position_nll_delta': 'position', 'velocity_nll_delta': 'velocity',
        'steering_nll_delta': 'steering', 'acceleration_nll_delta': 'accel.',
        'traffic_light_color_nll_delta': 'tl_color', 'traffic_light_confidence_nll_delta': 'tl_confidence',
        'traffic_light_discrepancy_residual_delta': 'tl_discrepancy',
    }
    negative_evidence_campaigns = {'tl_fault_s3', 'tl_fault_s4'}  # tl_unknown, tl_blackout — see fault_scenario_table.md
    cols = ['Campaign'] + [feature_labels[c] for c in df.columns]
    rows = list(df.reset_index().itertuples(index=False, name=None))

    fig, ax = plt.subplots(figsize=(12, 4.4))
    ax.axis('off')
    fig.suptitle('Discriminability rank per campaign (1 = most discriminative)\n'
                 'tl_discrepancy ranks #1 only for the negative-evidence fault subtype (tl_unknown / tl_blackout)',
                 fontsize=12, fontweight='bold', y=0.98, color='#263238')
    t = ax.table(cellText=rows, colLabels=cols, loc='center', cellLoc='center')
    _style_table(t)
    fig.subplots_adjust(top=0.8)
    tl_discrepancy_col = cols.index('tl_discrepancy')
    for (row, col), cell in t.get_celld().items():
        if row == 0:
            continue
        campaign = rows[row - 1][0]
        if col == tl_discrepancy_col and campaign in negative_evidence_campaigns and rows[row - 1][col] == 1:
            cell.set_facecolor(GREEN)
        elif campaign in negative_evidence_campaigns:
            cell.set_facecolor('#E8F5E9')
    out = os.path.join(OUT_DIR, 'discriminability_rank_table.png')
    fig.savefig(out, bbox_inches='tight', dpi=150)
    plt.close(fig)
    print(f'Saved {out}')


# ── 4. One representative CUSUM trace ──────────────────────────────────────

def plot_representative_trace(campaign='imu_fault_s3', goal_id='goal_012', trial='t1_20260728_195502'):
    trace_csv = os.path.join(REPO_DIR, 'st_gat', 'results', cfg.HORIZON_TAG, 'traces', campaign,
                              f'{goal_id}_{trial}.csv')
    per_trial = pd.read_csv(os.path.join(ANALYSIS_DIR, 'conformal_lead_time', 'per_trial.csv'))
    calib = pd.read_csv(os.path.join(ANALYSIS_DIR, 'conformal_lead_time', 'calibration_by_zone_kind.csv'),
                         index_col=0)
    row = per_trial[(per_trial.campaign == campaign) & (per_trial.goal_id == goal_id)
                     & (per_trial.trial == trial)].iloc[0]
    threshold = calib.loc[row['zone_kind'], 'threshold']

    df = pd.read_csv(trace_csv)
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(df['t_bag_rel'], df['cusum_combined'], color='#1565C0', lw=1.6, label='cusum_combined (model residual signal)')
    ax.axhline(threshold, color='#B71C1C', ls='--', lw=1.4,
               label=f'conformal threshold ({threshold:.0f}, α=0.1)')
    ax.axvline(row['zone_entry_rel_s'], color='#6A1B9A', ls=':', lw=1.4, label='turn-zone entry (t=0 for this analysis)')
    if pd.notna(row['crossing_rel_s']):
        ax.axvline(row['crossing_rel_s'], color='#2E7D32', ls='-', lw=1.8, label='threshold crossing (alarm)')
    if pd.notna(row['permanent_stop_rel_s']):
        ax.axvline(row['permanent_stop_rel_s'], color='#000000', ls='-', lw=2.0, label='fatal moment (permanent stop, Arm A)')
        if pd.notna(row['crossing_rel_s']):
            lead = row['permanent_stop_rel_s'] - row['crossing_rel_s']
            ax.annotate(f'{lead:.0f}s calibrated lead time', xy=((row['crossing_rel_s'] + row['permanent_stop_rel_s']) / 2,
                        ax.get_ylim()[1] * 0.85), ha='center', fontsize=10, fontweight='bold', color='#2E7D32')

    ax.set_xlabel('seconds (bag-relative)')
    ax.set_ylabel('cusum_combined')
    ax.set_title(f'{campaign} / {goal_id} / {trial}\ncalibrated alarm fires well before the fatal moment',
                 fontsize=12, fontweight='bold')
    ax.legend(loc='upper left', fontsize=9)
    ax.grid(alpha=0.3)
    out = os.path.join(OUT_DIR, 'representative_cusum_trace.png')
    fig.savefig(out, bbox_inches='tight', dpi=150)
    plt.close(fig)
    print(f'Saved {out}')


if __name__ == '__main__':
    plot_calibration_tightening()
    plot_campaign_summary()
    plot_discriminability_rank()
    plot_representative_trace()
