#!/usr/bin/env python3
"""
Inspect Layer 1's calibrated predictions under REAL injected faults
(2026-08-25) -- before going deeper into Layer 2, per Kalpit: does the
calibrated conformal band actually behave differently under a real fault
than it does under nominal driving? This is the first time this session's
Layer 1 (point prediction + conformal calibration, now the v2 turn/TL-
weighted retrain) has been run against fault data at all -- everything so
far was nominal-only (by design: run_pipeline.py refuses fault datasets
for TRAINING, correctly, but nobody had checked what the trained model's
predictions actually look like DURING a fault).

Processes every trial across all 8 fault campaigns on disk
(experiments/data/{imu,tl}_fault_*), regardless of whether the trial
reached its goal (many fault trials end 'stuck' -- that's an interesting
OUTCOME to inspect, not a reason to skip the trial the way
run_pipeline.py's nominal-only extraction correctly does). For each
window: run the model, compute the position residual, and check it
against BOTH the vanilla global calibrated band and the Mondrian
group-conditional band, split by whether the window's "now" timestamp
falls inside a real fault-active window (from fault_log.jsonl, matched by
wall-clock time -- see compare_fault_vs_nominal.py's extract_fault_windows
docstring for the wall_time-vs-sim_time_sec bug already found and fixed
once; reused here, not reimplemented) or not.

Usage (ROS + repo venv sourced):
  source /opt/ros/humble/setup.bash
  source /home/kvadner/Desktop/Dissertation/autoware/install/setup.bash
  source .venv/bin/activate
  python3 experiments/scripts/inspect_fault_predictions.py
"""

import argparse
import glob
import json
import math
import os
import sys

import numpy as np
import torch

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
sys.path.insert(0, REPO_DIR)
sys.path.insert(0, SCRIPT_DIR)
sys.path.insert(0, os.path.join(REPO_DIR, 'experiments', 'lib'))

from st_gat.pipeline import config as cfg  # noqa: E402
from st_gat.pipeline.bag_reader import read_bag  # noqa: E402
from st_gat.pipeline.sequence_builder import SequenceBuilder, extract_route_from_bag, TrafficLightExpectationChecker  # noqa: E402
from st_gat.pipeline.State_Estimator.GraphBuilder import GraphBuilder  # noqa: E402
from st_gat.pipeline.State_Estimator.MapProcessor import MapProcessor  # noqa: E402
from st_gat.model import STGAT, TrajectoryDataset  # noqa: E402
from compare_fault_vs_nominal import load_fault_log, extract_fault_windows, in_any_window  # noqa: E402
import scenario_zones  # noqa: E402
from conformal_mondrian_calibration import assign_groups  # noqa: E402
from conformal_horizon_calibration import _SERIES  # noqa: E402

_SERIES_KEYS = [s[0] for s in _SERIES]

DATA_DIR = os.path.join(REPO_DIR, 'experiments', 'data')
OUTPUT_DIR = os.path.join(REPO_DIR, 'experiments', 'analysis', 'fault_prediction_inspection')
VANILLA_REPORT = os.path.join(REPO_DIR, 'experiments', 'analysis',
                               'conformal_horizon_calibration', 'conformal_report.json')
MONDRIAN_REPORT = os.path.join(REPO_DIR, 'experiments', 'analysis',
                                'conformal_mondrian_calibration', 'conformal_mondrian_report.json')

IMU_CAMPAIGNS = ['imu_fault_s1', 'imu_fault_s3', 'imu_fault_scale', 'imu_fault_stuck']
TL_CAMPAIGNS  = ['tl_fault_s2', 'tl_fault_s3', 'tl_fault_s4', 'tl_fault_ramp']


def compute_window_times(frames, max_frame_gap_sec):
    """Mirrors SequenceBuilder.build()'s window-selection predicate (same
    stride, same intra-window gap-rejection check) WITHOUT the expensive
    graph-building work, purely to recover each accepted window's real
    wall-clock 'now' time (frames[i + INPUT_SEQ_LEN - 1]['t_sim']) --
    sequence_builder.py's returned sequence dicts don't carry a timestamp
    (checked directly), so this is reconstructed in parallel rather than
    modifying that shared, heavily-audited file for one analysis script.
    Positionally aligned with build()'s output as long as the same frames
    list and filter_mrm=False are used for both calls."""
    n = len(frames)
    total = cfg.INPUT_SEQ_LEN + cfg.OUTPUT_SEQ_LEN
    if n < total:
        return []
    frame_gaps = [frames[j + 1]['t_sim'] - frames[j]['t_sim'] for j in range(n - 1)]
    times = []
    for i in range(0, n - total + 1, cfg.STRIDE):
        if max(frame_gaps[i:i + total - 1]) > max_frame_gap_sec:
            continue
        times.append(frames[i + cfg.INPUT_SEQ_LEN - 1]['t_sim'])
    return times


def _load_model(model_path, device):
    model_cfg = cfg.build_inference_model_cfg(device)
    model = STGAT(model_cfg).to(device)
    model.load_state_dict(torch.load(model_path, map_location=device, weights_only=True))
    model.eval()
    return model


def process_trial(run_dir, campaign, shared_builder, kind):
    """Returns a dict with per-window t_rel (seconds since bag start),
    position (x,y) real-metre for Mondrian grouping, sequences, and the
    fault windows (start,end) in the same relative time, or None if the
    trial couldn't be processed (too few frames / build error)."""
    bag_dir = os.path.join(run_dir, 'rosbag')
    goal_id = os.path.basename(os.path.dirname(run_dir))
    try:
        frames = read_bag(bag_dir, goal_id=goal_id, verbose=False)
    except Exception as e:
        print(f"    ERROR reading bag: {e}")
        return None
    total = cfg.INPUT_SEQ_LEN + cfg.OUTPUT_SEQ_LEN
    if len(frames) < total:
        print(f"    WARNING: only {len(frames)} frames, skipping")
        return None

    route = extract_route_from_bag(bag_dir)
    shared_builder.route = route
    shared_builder.graph_builder = GraphBuilder(
        map_data=shared_builder.map_data, route=route,
        min_dist_between_node=cfg.MIN_DIST_BETWEEN_NODES,
        connection_threshold=cfg.CONNECTION_THRESHOLD,
        max_nodes=cfg.MAX_GRAPH_NODES, radius_m=cfg.GRAPH_RADIUS_M,
        routing_graph=shared_builder.routing_graph,
    )
    shared_builder._tl_checker = TrafficLightExpectationChecker(shared_builder.map_data, route)

    try:
        sequences = shared_builder.build(frames, verbose=False, filter_mrm=False)
    except Exception as e:
        print(f"    ERROR building sequences: {e}")
        return None
    if not sequences:
        print("    WARNING: zero sequences built, skipping")
        return None

    window_times = compute_window_times(frames, SequenceBuilder._MAX_FRAME_GAP_SEC)
    if len(window_times) != len(sequences):
        print(f"    WARNING: window_times ({len(window_times)}) != sequences ({len(sequences)}), skipping "
              f"(build()'s internal selection must have diverged from compute_window_times' replica)")
        return None

    t0 = frames[0]['t_sim']
    t_rel = np.array(window_times) - t0
    bag_duration = frames[-1]['t_sim'] - t0

    events = load_fault_log(run_dir)
    fault_windows = extract_fault_windows(events, bag_start_abs_sec=t0, bag_duration=bag_duration, kind=kind)

    return {'sequences': sequences, 't_rel': t_rel, 'fault_windows': fault_windows, 'bag_duration': bag_duration}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--model', default=cfg.MODEL_CONFIG['model_path'])
    ap.add_argument('--campaigns', nargs='+', default=IMU_CAMPAIGNS + TL_CAMPAIGNS)
    ap.add_argument('--output-dir', default=OUTPUT_DIR)
    ap.add_argument('--max-trace-plots', type=int, default=6,
                     help='save a per-trial residual-vs-time trace plot for at most this many trials')
    args = ap.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = _load_model(args.model, device)
    print(f"Loaded model: {args.model}")

    with open(VANILLA_REPORT) as f:
        vanilla = json.load(f)
    vanilla_q = {row['feature']: np.array(row['mean_fold_quantile_by_step']) for row in vanilla['features']}
    with open(MONDRIAN_REPORT) as f:
        mondrian = json.load(f)

    print("Loading map (one-time)...")
    map_processor = MapProcessor(cfg.MAP_FILE)
    shared_builder = SequenceBuilder(map_processor.map_data, route=[])

    rows = []   # per-window records across every trial, for the aggregate summary
    trace_plots_saved = 0

    for campaign in args.campaigns:
        kind = 'imu' if campaign.startswith('imu_') else 'tl'
        campaign_dir = os.path.join(DATA_DIR, campaign)
        if not os.path.isdir(campaign_dir):
            print(f"[skip] {campaign}: not found on disk")
            continue
        goal_dirs = sorted(d for d in glob.glob(os.path.join(campaign_dir, 'goal_*')) if os.path.isdir(d))
        for goal_dir in goal_dirs:
            run_dirs = sorted(d for d in glob.glob(os.path.join(goal_dir, 't*')) if os.path.isdir(d))
            for run_dir in run_dirs:
                run_name = os.path.basename(run_dir)
                print(f"\n[{campaign}/{os.path.basename(goal_dir)}/{run_name}]")
                result_path = os.path.join(run_dir, 'result.json')
                status = 'unknown'
                if os.path.exists(result_path):
                    with open(result_path) as f:
                        status = json.load(f).get('status', 'unknown')
                print(f"  outcome: {status}")

                out = process_trial(run_dir, campaign, shared_builder, kind)
                if out is None:
                    continue
                sequences, t_rel, fault_windows = out['sequences'], out['t_rel'], out['fault_windows']
                print(f"  {len(sequences)} windows, {len(fault_windows)} fault-active window(s), "
                      f"duration {out['bag_duration']:.1f}s")

                # Per (report_key, source_feature, mode) -- SAME reduction spec as
                # conformal_horizon_calibration.py's _SERIES, so these residuals are
                # directly comparable to the calibrated widths (2026-08-25, per
                # Kalpit: "is there a reason we are only looking at position residuals?
                # ... fault signatures can manifest in all of them differently" --
                # TL faults in particular should show up most directly in the TL
                # features themselves, which the position-only check would completely miss).
                resid = {k: np.zeros(len(sequences)) for k in _SERIES_KEYS}
                xy = np.zeros((len(sequences), 2))
                with torch.no_grad():
                    for i, seq in enumerate(sequences):
                        past_t  = TrajectoryDataset._build_feature_tensors(seq['past'])
                        future_t = TrajectoryDataset._build_feature_tensors(seq['future'])
                        graph_t = TrajectoryDataset._build_graph_tensors(seq['graph'])
                        past  = {k: torch.as_tensor(v, device=device).unsqueeze(0) for k, v in past_t.items()}
                        graph = {k: torch.as_tensor(v, device=device).unsqueeze(0) for k, v in graph_t.items()}
                        preds = model(past, graph)

                        for report_key, source_key, mode in _SERIES:
                            actual = np.asarray(future_t[source_key])
                            pred = preds[f'{source_key}_mean'][0].cpu().numpy()
                            if mode == 'l2':
                                r = np.linalg.norm(pred - actual, axis=-1).mean() * cfg.POSITION_DISPLACEMENT_RANGE_M
                            elif mode == 'scalar':
                                r = np.abs(pred.reshape(-1) - actual.reshape(-1)).mean()
                            else:
                                _, axis = mode
                                r = np.abs(pred[:, axis] - actual[:, axis]).mean()
                            resid[report_key][i] = r

                        ref_x, ref_y = seq['position_ref']
                        last_past = seq['past'][-1]['position']
                        xy[i, 0] = ref_x + last_past[0] * cfg.POSITION_DISPLACEMENT_RANGE_M
                        xy[i, 1] = ref_y + last_past[1] * cfg.POSITION_DISPLACEMENT_RANGE_M
                position_resid = resid['position']

                groups = assign_groups(xy)
                in_fault = np.array([in_any_window(t, fault_windows) for t in t_rel])
                # seconds since the most recent fault window ENDED (inf if none yet) --
                # tracks LINGERING post-fault effects (e.g. IMU-corrupted localization
                # drift that compounds after the injected signal itself turns back off),
                # which the raw in/out-of-fault boolean alone can't distinguish from
                # "back to normal."
                t_since_fault_end = np.full(len(t_rel), np.inf)
                for j, t in enumerate(t_rel):
                    ends = [t - w['end'] for w in fault_windows if t >= w['end']]
                    if ends:
                        t_since_fault_end[j] = min(ends)

                # quantiles are stored NORMALIZED (position/velocity share
                # POSITION_DISPLACEMENT_RANGE_M's space via l2/axis reduction, the
                # rest are already real-unit scalars from _extract_series's 'scalar'
                # mode) -- position needs the same *100 scaling bug-fixed here
                # (2026-08-24 class of bug) before comparing to real-metre residuals.
                width_scale = {k: (cfg.POSITION_DISPLACEMENT_RANGE_M if k in ('position',) else 1.0)
                                for k in _SERIES_KEYS}
                vanilla_width = {k: vanilla_q[k].mean() * width_scale[k] for k in _SERIES_KEYS}
                mondrian_width = {
                    k: np.array([np.mean(mondrian['by_group'][g][k]['mean_fold_quantile_by_step']) for g in groups])
                       * width_scale[k]
                    for k in _SERIES_KEYS
                }

                for i in range(len(sequences)):
                    row = {
                        'campaign': campaign, 'run': run_name, 'status': status,
                        't_rel': float(t_rel[i]), 'in_fault': bool(in_fault[i]),
                        't_since_fault_end': float(t_since_fault_end[i]),
                        'post_fault_30s': bool(t_since_fault_end[i] <= 30.0),
                        'group': groups[i],
                    }
                    for k in _SERIES_KEYS:
                        row[f'resid_{k}'] = float(resid[k][i])
                        row[f'exceeds_vanilla_{k}'] = bool(resid[k][i] > vanilla_width[k])
                        row[f'exceeds_mondrian_{k}'] = bool(resid[k][i] > mondrian_width[k][i])
                    rows.append(row)

                if trace_plots_saved < args.max_trace_plots and len(fault_windows) > 0:
                    fig, ax = plt.subplots(figsize=(11, 4.5))
                    ax.plot(t_rel, position_resid, color='#1f77b4', linewidth=1.2, label='position residual (mean over horizon, m)')
                    ax.axhline(vanilla_width['position'], color='#7f7f7f', linestyle='--', linewidth=1.3, label='vanilla calibrated width')
                    ax.plot(t_rel, mondrian_width['position'], color='#2ca02c', linewidth=1.0, linestyle=':', label='Mondrian calibrated width (per-window group)')
                    for w in fault_windows:
                        ax.axvspan(w['start'], w['end'], color='#d62728', alpha=0.15)
                    ax.axvspan(np.nan, np.nan, color='#d62728', alpha=0.15, label='fault active')
                    ax.set_xlabel('seconds since trial start')
                    ax.set_ylabel('position residual (m)')
                    ax.set_title(f'{campaign} / {os.path.basename(goal_dir)} / {run_name}  (outcome: {status})', fontsize=11)
                    ax.legend(fontsize=8, loc='upper left')
                    fig.tight_layout()
                    out_path = os.path.join(args.output_dir, f'{campaign}_{os.path.basename(goal_dir)}_{run_name}_trace.png')
                    fig.savefig(out_path, dpi=130)
                    plt.close(fig)
                    print(f"  Saved {out_path}")
                    trace_plots_saved += 1

    # ── Aggregate summary ────────────────────────────────────────────────
    if not rows:
        print("\nNo windows processed -- nothing to summarize.")
        return

    in_fault_arr = np.array([r['in_fault'] for r in rows])
    post_fault_arr = np.array([r['post_fault_30s'] for r in rows]) & ~in_fault_arr
    clean_arr = ~in_fault_arr & ~post_fault_arr
    is_imu = np.array([r['campaign'].startswith('imu_') for r in rows])
    is_tl = ~is_imu

    print(f"\n{'='*70}\nPER-FEATURE SUMMARY ({len(rows)} windows across "
          f"{len(set((r['campaign'], r['run']) for r in rows))} trials)\n{'='*70}")
    print("Kalpit's question: does the fault signature show up in features OTHER than "
          "position? (TL faults in particular should be clearest in the TL features "
          "themselves, which the position-only check missed entirely.)\n")
    for kind_name, kind_mask in [('IMU campaigns', is_imu), ('TL campaigns', is_tl)]:
        print(f"--- {kind_name} ---")
        print(f"{'feature':<26}{'clean exceed':>14}{'active exceed':>15}{'post-fault exceed':>19}{'active/clean ratio':>20}")
        for k in _SERIES_KEYS:
            exc = np.array([r[f'exceeds_vanilla_{k}'] for r in rows])
            c = exc[kind_mask & clean_arr].mean() if (kind_mask & clean_arr).any() else float('nan')
            a = exc[kind_mask & in_fault_arr].mean() if (kind_mask & in_fault_arr).any() else float('nan')
            p = exc[kind_mask & post_fault_arr].mean() if (kind_mask & post_fault_arr).any() else float('nan')
            ratio = a / c if c and c > 0 else float('nan')
            print(f"{k:<26}{c:14.3f}{a:15.3f}{p:19.3f}{ratio:20.2f}")
        print()

    with open(os.path.join(args.output_dir, 'fault_prediction_summary.json'), 'w') as f:
        json.dump(rows, f, indent=2)
    print(f"\nSaved {os.path.join(args.output_dir, 'fault_prediction_summary.json')}")


if __name__ == '__main__':
    main()
