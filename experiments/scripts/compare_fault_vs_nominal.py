#!/usr/bin/env python3
"""
Compares a fault campaign's trial(s) against the matching nominal (nom_v11)
baseline for the same goal — two purposes:

  1. VERIFY the fault actually took effect at the message level (not just that
     fault_injector logged a cycle, but that the signal it touched actually
     changed: TL confidence/color for tl_* faults, EKF-vs-ground-truth
     divergence for imu_* faults).
  2. RANK candidate ST-GAT state features (velocity, acceleration, steering,
     object distance, TL status, EKF/GT divergence) by how strongly each
     responds to the fault — guidance for what the state representation
     should include, not just whether the injector works.

Usage (must source ROS/Autoware, then the repo venv):
  source /opt/ros/humble/setup.bash
  source /home/kvadner/Desktop/Dissertation/autoware/install/setup.bash
  source .venv/bin/activate
  python3 experiments/scripts/compare_fault_vs_nominal.py --campaign tl_fault_s1 --goal goal_007
  python3 experiments/scripts/compare_fault_vs_nominal.py --campaign imu_fault_s2 --goal goal_012 --trial 1
"""

import argparse
import glob
import json
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
sys.path.insert(0, os.path.join(REPO_DIR, 'experiments', 'lib'))

from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from nav_msgs.msg import Odometry
from autoware_perception_msgs.msg import TrafficLightGroupArray, TrafficLightElement

from metrics import MetricsCollector  # noqa: E402 — needs sys.path insert above

DATA_DIR = os.path.join(REPO_DIR, 'experiments', 'data')
OUTPUT_DIR = os.path.join(REPO_DIR, 'experiments', 'analysis', 'fault_comparison')
GT_TOPIC = '/awsim/ground_truth/localization/kinematic_state'
TL_TOPIC = '/perception/traffic_light_recognition/traffic_signals'           # Autoware's real, unmodified output
TL_TOPIC_FAULTED = '/perception/traffic_light_recognition/traffic_signals_faulted'  # what planning actually consumes

# Okabe-Ito colorblind-safe pair
COLOR_NOMINAL = '#0072B2'
COLOR_FAULT   = '#D55E00'
COLOR_WINDOW  = '#999999'

FAULT_WINDOW_PAD = 5.0  # seconds of slack when matching fault_log events to a trial's bag span

COLOR_NAME = {0: 'UNKNOWN', 1: 'RED', 2: 'AMBER', 3: 'GREEN'}


# ── Bag readers ───────────────────────────────────────────────────────────────

def read_gt_and_tl(bag_dir):
    """Single pass: ground-truth position + TL status, anchored to the same
    'first message in bag' epoch MetricsCollector uses, so both are directly
    comparable on the same relative time axis."""
    storage_options = StorageOptions(uri=bag_dir, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr',
                                          output_serialization_format='cdr')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    topics = {info.name: info.type for info in reader.get_all_topics_and_types()}

    # Prefer the faulted topic actually consumed by behavior_planning (recorded
    # from 2026-07-22 onward, once the topic-wiring bug was fixed). Trials
    # collected before that fix only have the unmodified topic recorded — fall
    # back to it so old bags still parse, though they'll correctly show no
    # fault effect (that bag never had a working fault path in the first place).
    tl_topic = TL_TOPIC_FAULTED if TL_TOPIC_FAULTED in topics else TL_TOPIC
    if tl_topic != TL_TOPIC_FAULTED:
        print(f'  NOTE: {TL_TOPIC_FAULTED} not in this bag — falling back to {TL_TOPIC} '
              f'(pre-fix trial; this topic was never actually faulted).')

    start_ns = None
    gt_positions = []   # (t_rel_sec, x, y)
    tl_events = []      # (t_rel_sec, dominant_color_code, mean_confidence, any_detected)

    while reader.has_next():
        topic, data, ts = reader.read_next()
        if start_ns is None:
            start_ns = ts
        t_rel = (ts - start_ns) / 1e9

        if topic == GT_TOPIC:
            msg = deserialize_message(data, Odometry)
            gt_positions.append((t_rel, msg.pose.pose.position.x, msg.pose.pose.position.y))
        elif topic == tl_topic:
            msg = deserialize_message(data, TrafficLightGroupArray)
            elems = [e for g in msg.traffic_light_groups for e in g.elements]
            if elems:
                mean_conf = sum(e.confidence for e in elems) / len(elems)
                dominant = max(elems, key=lambda e: e.confidence).color
                tl_events.append((t_rel, dominant, mean_conf, True))
            else:
                tl_events.append((t_rel, TrafficLightElement.UNKNOWN, 0.0, False))

    bag_start_abs_sec = start_ns / 1e9 if start_ns is not None else None
    bag_duration = t_rel if start_ns is not None else 0.0
    return {
        'bag_start_abs_sec': bag_start_abs_sec,
        'bag_duration': bag_duration,
        'gt_positions': gt_positions,
        'tl_events': tl_events,
        'topics': topics,
    }


def ekf_gt_divergence(mc: MetricsCollector, gt_positions):
    """|| EKF position - ground-truth position || at each EKF sample time,
    via nearest-neighbor match against ground truth (both already relative
    to the same bag-start epoch)."""
    if not gt_positions or not mc.positions:
        return []
    gt_t = np.array([p[0] for p in gt_positions])
    gt_xy = np.array([(p[1], p[2]) for p in gt_positions])
    out = []
    for t, x, y, _h in mc.positions:
        idx = int(np.searchsorted(gt_t, t))
        idx = min(max(idx, 0), len(gt_t) - 1)
        gx, gy = gt_xy[idx]
        out.append((t, math.hypot(x - gx, y - gy)))
    return out


def load_fault_log(trial_dir):
    path = os.path.join(trial_dir, 'fault_log.jsonl')
    if not os.path.exists(path):
        return []
    events = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return events


def extract_fault_windows(events, bag_start_abs_sec, bag_duration, kind):
    """Filter a (possibly multi-trial, cumulative) fault_log to just the
    windows whose time falls inside THIS trial's bag span, and convert to the
    same bag-relative time axis as everything else. `kind` is 'tl' or 'imu'.

    Uses each event's `wall_time` (real unix time, from time.time()), NOT
    `sim_time_sec` (AWSIM's own simulation clock, small numbers counting up
    from Autoware/AWSIM boot — a completely different, unrelated clock).
    rosbag2's recorded message timestamps are real wall-clock time (confirmed:
    bag_start_abs_sec lands in the same ~1.78e9 unix-epoch range as fault_log's
    wall_time fields, while sim_time_sec values are only in the hundreds/
    thousands). Mixing the two silently produces zero matches every time,
    which looks identical to "the fault never fired" — caught by cross-checking
    an IMU trial where fault_log's own event stream proved imu_bias_on/off
    cycles genuinely ran, yet this function still reported 0 windows before
    this fix.
    """
    if bag_start_abs_sec is None:
        return []
    lo, hi = -FAULT_WINDOW_PAD, bag_duration + FAULT_WINDOW_PAD
    windows = []
    if kind == 'tl':
        starts = {}
        for e in events:
            if 'wall_time' not in e:
                continue
            rel = e['wall_time'] - bag_start_abs_sec
            if e['event'] == 'tl_fault_start':
                starts[e.get('cycle')] = (rel, e)
            elif e['event'] == 'tl_fault_end':
                cyc = e.get('cycle')
                if cyc in starts:
                    srel, se = starts.pop(cyc)
                    erel = rel
                    if lo <= srel <= hi or lo <= erel <= hi:
                        windows.append({'start': srel, 'end': erel,
                                        'reason': e.get('reason'), 'cycle': cyc})
    else:  # imu
        on_t = None
        for e in events:
            if 'wall_time' not in e:
                continue
            rel = e['wall_time'] - bag_start_abs_sec
            if e['event'] == 'imu_bias_on':
                on_t = rel
            elif e['event'] == 'imu_bias_off' and on_t is not None:
                if lo <= on_t <= hi or lo <= rel <= hi:
                    windows.append({'start': on_t, 'end': rel, 'reason': 'bias_on', 'cycle': None})
                on_t = None
        if on_t is not None and (lo <= on_t <= hi):
            # sustained (S4-style) fault still active at trial end
            windows.append({'start': on_t, 'end': bag_duration, 'reason': 'bias_on_sustained', 'cycle': None})
    return windows


def in_any_window(t, windows):
    return any(w['start'] <= t <= w['end'] for w in windows)


# ── Feature series (all as (t, value) lists, relative to bag start) ──────────

def build_feature_series(bag_dir):
    mc = MetricsCollector(bag_dir)
    mc.read_bag()
    gt_tl = read_gt_and_tl(bag_dir)

    divergence = ekf_gt_divergence(mc, gt_tl['gt_positions'])
    # Only pool/compare confidence over messages where a TL was actually
    # detected — otherwise "no TL in view" (open road, confidence=0 by
    # convention) gets conflated with "TL visible but fault-degraded to
    # zero confidence", which would corrupt both the nominal baseline and
    # the in-fault/out-of-fault comparison for this feature.
    tl_confidence = [(t, c) for t, _col, c, d in gt_tl['tl_events'] if d]
    tl_color = [(t, col) for t, col, _c, _d in gt_tl['tl_events']]

    features = {
        'velocity_mps':        mc.velocities,
        'acceleration_mps2':   mc.accelerations,
        'steering_rad':        mc.steerings,
        'obj_distance_m':      mc.object_distances,
        'ekf_gt_divergence_m': divergence,
        'tl_confidence':       tl_confidence,
    }
    meta = {
        'bag_start_abs_sec': gt_tl['bag_start_abs_sec'],
        'bag_duration':      gt_tl['bag_duration'],
        'tl_color':          tl_color,
        'tl_any_detected':   [(t, d) for t, _c, _cf, d in gt_tl['tl_events']],
    }
    return features, meta


def resample(series, grid):
    """Nearest-neighbor resample of a (t, value) list onto a fixed time grid."""
    if not series:
        return np.full(len(grid), np.nan)
    t = np.array([p[0] for p in series])
    v = np.array([p[1] for p in series], dtype=float)
    idx = np.searchsorted(t, grid)
    idx = np.clip(idx, 0, len(t) - 1)
    # Prefer the closer of idx-1/idx
    idx_prev = np.clip(idx - 1, 0, len(t) - 1)
    use_prev = np.abs(grid - t[idx_prev]) < np.abs(t[idx] - grid)
    idx_final = np.where(use_prev, idx_prev, idx)
    return v[idx_final]


# ── Main comparison ────────────────────────────────────────────────────────────

def find_trial_dirs(campaign, goal, trial=None):
    base = os.path.join(DATA_DIR, campaign, goal)
    pattern = f't{trial}_*' if trial else 't*'
    return sorted(glob.glob(os.path.join(base, pattern)))


def find_nominal_dirs(goal, nominal_campaign):
    base = os.path.join(DATA_DIR, nominal_campaign, goal)
    return sorted(glob.glob(os.path.join(base, 't*')))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--campaign', required=True, help='e.g. tl_fault_s1, imu_fault_s2')
    ap.add_argument('--goal', required=True, help='e.g. goal_007')
    ap.add_argument('--trial', type=int, default=None, help='specific trial number; default = all trials for this goal/campaign')
    ap.add_argument('--nominal-campaign', default='nom_v11')
    ap.add_argument('--output-dir', default=OUTPUT_DIR)
    args = ap.parse_args()

    kind = 'tl' if args.campaign.startswith('tl_fault') else 'imu'

    fault_dirs = find_trial_dirs(args.campaign, args.goal, args.trial)
    if not fault_dirs:
        print(f'No trials found for {args.campaign}/{args.goal} under {DATA_DIR}/{args.campaign}/', file=sys.stderr)
        sys.exit(1)
    nominal_dirs = find_nominal_dirs(args.goal, args.nominal_campaign)
    if not nominal_dirs:
        print(f'No nominal trials found for {args.goal} under {DATA_DIR}/{args.nominal_campaign}/', file=sys.stderr)
        sys.exit(1)

    print(f'Fault trials ({args.campaign}/{args.goal}): {[os.path.basename(d) for d in fault_dirs]}')
    print(f'Nominal trials ({args.nominal_campaign}/{args.goal}): {[os.path.basename(d) for d in nominal_dirs]}')

    # ── Nominal baseline: pool marginal values per feature across all nominal trials ──
    nominal_pool = {}
    for nd in nominal_dirs:
        feats, _meta = build_feature_series(os.path.join(nd, 'rosbag'))
        for name, series in feats.items():
            nominal_pool.setdefault(name, []).extend(v for _t, v in series)

    nominal_stats = {}
    for name, values in nominal_pool.items():
        arr = np.array(values, dtype=float)
        arr = arr[np.isfinite(arr)]
        if arr.size == 0:
            nominal_stats[name] = (0.0, 1.0)
        else:
            std = arr.std()
            nominal_stats[name] = (arr.mean(), std if std > 1e-6 else 1e-6)
    print('\nNominal baseline (mean ± std), pooled across nominal trials:')
    for name, (m, s) in nominal_stats.items():
        print(f'  {name:22s} {m:8.3f} ± {s:6.3f}')

    os.makedirs(args.output_dir, exist_ok=True)

    for fd in fault_dirs:
        trial_name = os.path.basename(fd)
        print(f'\n{"=" * 70}\n{trial_name}\n{"=" * 70}')

        feats, meta = build_feature_series(os.path.join(fd, 'rosbag'))
        events = load_fault_log(fd)
        windows = extract_fault_windows(events, meta['bag_start_abs_sec'], meta['bag_duration'], kind)

        print(f'  bag_start_abs_sec={meta["bag_start_abs_sec"]}  duration={meta["bag_duration"]:.1f}s')
        if events:
            wall_times = [e['wall_time'] for e in events if 'wall_time' in e]
            if wall_times:
                print(f'  fault_log wall_time range: [{min(wall_times):.1f}, {max(wall_times):.1f}]  '
                      f'(bag span: [{meta["bag_start_abs_sec"]:.1f}, {meta["bag_start_abs_sec"] + meta["bag_duration"]:.1f}])')
        print(f'  {kind.upper()} fault windows matched to this trial: {len(windows)}')
        if not windows:
            print('  WARNING: no fault windows matched — either the fault never fired this '
                  'trial, or the wall_time ranges printed above don\'t overlap (compare them '
                  'by hand).')
        for w in windows:
            print(f'    [{w["start"]:7.1f}, {w["end"]:7.1f}]  dur={w["end"]-w["start"]:5.1f}s  '
                  f'reason={w.get("reason")}  cycle={w.get("cycle")}')

        # ── Direct, message-level fault-effect verification ──────────────────
        print('\n  Direct verification (message-level, not z-score):')
        if kind == 'tl' and meta['tl_color']:
            _verify_tl(args.campaign, meta, windows)
        elif kind == 'imu':
            _verify_imu(feats.get('ekf_gt_divergence_m', []), windows)

        # ── Feature discriminability (z-score vs nominal, in-fault vs out) ────
        grid = np.arange(0, meta['bag_duration'], 0.5)
        in_mask = np.array([in_any_window(t, windows) for t in grid])
        print(f'\n  Feature discriminability (z = (value - nominal_mean)/nominal_std, n_in={in_mask.sum()}, n_out={(~in_mask).sum()}):')
        print(f'  {"feature":22s} {"mean|z| in-fault":>17s} {"mean|z| out":>13s} {"delta":>8s}  detectable?')
        rows = []
        for name, series in feats.items():
            mean, std = nominal_stats.get(name, (0.0, 1.0))
            vals = resample(series, grid)
            z = np.abs((vals - mean) / std)
            valid = np.isfinite(z)
            if not valid.any():
                continue
            in_z = z[valid & in_mask]
            out_z = z[valid & ~in_mask]
            m_in = in_z.mean() if in_z.size else float('nan')
            m_out = out_z.mean() if out_z.size else float('nan')
            delta = m_in - m_out if np.isfinite(m_in) and np.isfinite(m_out) else float('nan')
            detectable = np.isfinite(delta) and delta > 0.5 and m_in > 1.0
            rows.append((name, m_in, m_out, delta, detectable))
            print(f'  {name:22s} {m_in:17.2f} {m_out:13.2f} {delta:8.2f}  {"YES" if detectable else "no"}')
        rows.sort(key=lambda r: (-r[3] if np.isfinite(r[3]) else 0))

        # ── Plot ───────────────────────────────────────────────────────────────
        _plot_trial(trial_name, args.campaign, feats, grid, windows,
                    nominal_stats, os.path.join(args.output_dir, f'{args.campaign}_{trial_name}.png'))

        # ── JSON summary ────────────────────────────────────────────────────────
        summary = {
            'campaign': args.campaign, 'goal': args.goal, 'trial': trial_name,
            'fault_windows': windows,
            'discriminability': [
                {'feature': n, 'mean_abs_z_in_fault': m_in, 'mean_abs_z_out_of_fault': m_out,
                 'delta': delta, 'detectable': bool(det)}
                for n, m_in, m_out, delta, det in rows
            ],
        }
        json_path = os.path.join(args.output_dir, f'{args.campaign}_{trial_name}.json')
        with open(json_path, 'w') as f:
            json.dump(summary, f, indent=2, default=lambda o: None if isinstance(o, float) and not math.isfinite(o) else o)
        print(f'\n  Wrote {json_path}')


def _verify_tl(campaign, meta, windows):
    tl_color = meta['tl_color']
    tl_detected = dict(meta['tl_any_detected'])
    if not windows:
        print('    (skipped — no matched fault windows)')
        return
    in_conf, out_conf = [], []
    in_green_frac_num, in_green_frac_den = 0, 0
    in_unknown_num, in_unknown_den = 0, 0
    in_detected_num, in_detected_den = 0, 0
    for t, col in tl_color:
        is_in = in_any_window(t, windows)
        detected = tl_detected.get(t, True)
        if is_in:
            in_detected_den += 1
            in_detected_num += int(detected)
            if detected:
                in_green_frac_den += 1
                in_green_frac_num += int(col == TrafficLightElement.GREEN)
                in_unknown_den += 1
                in_unknown_num += int(col == TrafficLightElement.UNKNOWN)
    if 'confidence' in campaign or 'tl_fault_s1' in campaign:
        pass  # confidence checked via discriminability table (tl_confidence feature)
    detect_rate = in_detected_num / in_detected_den if in_detected_den else float('nan')
    green_rate = in_green_frac_num / in_green_frac_den if in_green_frac_den else float('nan')
    unknown_rate = in_unknown_num / in_unknown_den if in_unknown_den else float('nan')
    print(f'    In-fault message detection rate (non-empty msgs): {detect_rate:.2%}'
          + ('  <- expect near 0% for tl_blackout' if 'blackout' in campaign else ''))
    print(f'    In-fault GREEN rate (of detected msgs):            {green_rate:.2%}'
          + ('  <- expect high for tl_oscillate GREEN half-cycles' if 'oscillate' in campaign else ''))
    print(f'    In-fault UNKNOWN rate (of detected msgs):          {unknown_rate:.2%}'
          + ('  <- expect ~100% for tl_unknown' if 'unknown' in campaign else ''))


def _verify_imu(divergence_series, windows):
    if not divergence_series or not windows:
        print('    (skipped — no EKF/ground-truth divergence data or no matched windows)')
        return
    in_vals = [v for t, v in divergence_series if in_any_window(t, windows)]
    out_vals = [v for t, v in divergence_series if not in_any_window(t, windows)]
    if in_vals and out_vals:
        print(f'    EKF-vs-ground-truth divergence: in-fault mean={np.mean(in_vals):.3f}m, '
              f'out-of-fault mean={np.mean(out_vals):.3f}m  '
              f'(expect in-fault notably higher — gyro bias corrupts EKF twist)')
    else:
        print('    (insufficient divergence samples in one of the two segments)')


def _plot_trial(trial_name, campaign, feats, grid, windows, nominal_stats, out_path):
    names = list(feats.keys())
    fig, axes = plt.subplots(len(names), 1, figsize=(11, 2.2 * len(names)), sharex=True)
    if len(names) == 1:
        axes = [axes]
    for ax, name in zip(axes, names):
        series = feats[name]
        vals = resample(series, grid)
        ax.plot(grid, vals, color=COLOR_FAULT, linewidth=1.4, label=trial_name)
        mean, std = nominal_stats.get(name, (0.0, 1.0))
        ax.axhline(mean, color=COLOR_NOMINAL, linewidth=1.2, linestyle='--', label='nominal mean')
        ax.fill_between(grid, mean - std, mean + std, color=COLOR_NOMINAL, alpha=0.12, label='nominal ±1σ')
        for w in windows:
            ax.axvspan(w['start'], w['end'], color=COLOR_WINDOW, alpha=0.25)
        ax.set_ylabel(name, fontsize=8)
        ax.tick_params(labelsize=8)
    axes[0].legend(loc='upper right', fontsize=7, framealpha=0.9)
    axes[-1].set_xlabel('time since trial start (s)')
    fig.suptitle(f'{campaign} — {trial_name}\n(gray = fault-active window)', fontsize=10)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f'  Wrote {out_path}')


if __name__ == '__main__':
    main()
