#!/usr/bin/env python3
"""
TL fault severity dose-response analysis (2026-08-25) -- checks whether
data ALREADY ON DISK can produce a real severity sweep before scoping any
new AWSIM collection, per Kalpit's request to scope the TL severity-sweep
experiment.

Key discovery this script exploits: `tl_fault_ramp` (already collected,
3 trials, goal_007/012/026) uses `tl_confidence_ramp`
(confidence_ramp_rate_per_s=0.1, min_confidence_scale=0.0, fault_duration
cap 15s) -- confidence_scale decays CONTINUOUSLY from 1.0 to 0.0 within
EVERY fault-active window (every real intersection the route passes), not
just once per trial. Each trial has ~7-11 such cycles (see
fault_prediction_inspection's earlier run), so 3 existing trials already
contain ~26 independent continuous severity sweeps -- this is very
possibly already most of the "TL severity sweep" experiment, not
something that needs new data collection to exist at all.

Method: for every window in a tl_fault_ramp trial, reconstruct the
INSTANTANEOUS confidence_scale at that window's "now" time directly from
the known ramp formula (rate, floor, and the fault-cycle's own start time
from fault_log.jsonl) -- exact, not estimated, since the ramp is
deterministic. Bin windows by reconstructed confidence_scale and compare
residual/exceed-rate against matched NOMINAL windows from nom_v11 at the
SAME goal (confidence_scale defined as 1.0 for nominal windows -- the
true no-fault reference point on the same severity axis).

IMPORTANT CAVEAT, checked and handled explicitly (do not silently ignore):
nom_v11's goal_007/012 trials are in the TRAINING set (the model has seen
them), NOT the held-out calibration set -- only goal_026's two nom_v11
trials are properly held out. Comparisons against goal_007/012's nominal
baseline will read artificially LOW residual (optimistic) and are NOT a
fair severity-response comparison; only goal_026 gives a clean matched
result with the current data. Both are reported, clearly labeled.

Usage (ROS + repo venv sourced):
  source /opt/ros/humble/setup.bash
  source /home/kvadner/Desktop/Dissertation/autoware/install/setup.bash
  source .venv/bin/activate
  python3 experiments/scripts/tl_severity_sweep_analysis.py
"""

import argparse
import glob
import json
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
from st_gat.pipeline.State_Estimator.MapProcessor import MapProcessor  # noqa: E402
from st_gat.pipeline.sequence_builder import SequenceBuilder  # noqa: E402
from st_gat.model import STGAT, TrajectoryDataset  # noqa: E402
from compare_fault_vs_nominal import load_fault_log  # noqa: E402
from conformal_horizon_calibration import _SERIES  # noqa: E402
from inspect_fault_predictions import process_trial  # noqa: E402

DATA_DIR = os.path.join(REPO_DIR, 'experiments', 'data')
CAL_DIR_TRIALS = {os.path.basename(p)[:-4] for p in glob.glob(os.path.join(cfg.CAL_DIR, '*.pkl'))}
OUTPUT_DIR = os.path.join(REPO_DIR, 'experiments', 'analysis', 'tl_severity_sweep')
_SERIES_KEYS = [s[0] for s in _SERIES]

GOALS = ['goal_007', 'goal_012', 'goal_026']
RAMP_CAMPAIGN = 'tl_fault_ramp'
NOMINAL_CAMPAIGN = 'nom_v11'


def _load_model(model_path):
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model_cfg = cfg.build_inference_model_cfg(device)
    model = STGAT(model_cfg).to(device)
    model.load_state_dict(torch.load(model_path, map_location=device, weights_only=True))
    model.eval()
    print(f"Loaded model: {model_path}")
    return model, device


def reconstruct_confidence_scale(t_rel, fault_windows, events, rate, floor):
    """confidence_scale(t) for every window, exact (deterministic ramp) --
    1.0 outside any fault window, decaying at `rate` per second since that
    cycle's own tl_fault_start, floored at `floor`, inside one."""
    starts = sorted(e['wall_time'] for e in events if e['event'] == 'tl_fault_start')
    conf = np.ones(len(t_rel))
    for i, t in enumerate(t_rel):
        # which fault window (if any) contains t
        active = [w for w in fault_windows if w['start'] <= t <= w['end']]
        if not active:
            continue
        w = active[0]
        elapsed = t - w['start']
        conf[i] = max(floor, 1.0 - rate * elapsed)
    return conf


def residuals_for_sequences(model, device, sequences):
    resid = {k: np.zeros(len(sequences)) for k in _SERIES_KEYS}
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
    return resid


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--model', default=cfg.MODEL_CONFIG['model_path'])
    ap.add_argument('--output-dir', default=OUTPUT_DIR)
    args = ap.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    model, device = _load_model(args.model)
    map_processor = MapProcessor(cfg.MAP_FILE)
    shared_builder = SequenceBuilder(map_processor.map_data, route=[])

    all_conf = []
    all_resid = {k: [] for k in _SERIES_KEYS}
    all_goal = []
    all_source = []   # 'ramp' or 'nominal'
    all_clean = []    # bool: is the nominal reference for this goal held-out (True) or train-contaminated (False)

    for goal in GOALS:
        goal_clean = None

        # --- ramp trials ---
        ramp_dir = os.path.join(DATA_DIR, RAMP_CAMPAIGN, goal)
        for run_dir in sorted(glob.glob(os.path.join(ramp_dir, 't*'))):
            print(f"\n[ramp] {goal}/{os.path.basename(run_dir)}")
            out = process_trial(run_dir, RAMP_CAMPAIGN, shared_builder, kind='tl')
            if out is None:
                continue
            events = load_fault_log(run_dir)
            rate = 0.1
            floor = 0.0
            for e in events:
                if e['event'] == 'tl_fault_start':
                    rate = e['params'].get('confidence_ramp_rate_per_s', rate)
                    floor = e['params'].get('min_confidence_scale', floor)
                    break
            conf = reconstruct_confidence_scale(out['t_rel'], out['fault_windows'], events, rate, floor)
            resid = residuals_for_sequences(model, device, out['sequences'])
            n = len(out['sequences'])
            all_conf.append(conf)
            for k in _SERIES_KEYS:
                all_resid[k].append(resid[k])
            all_goal.append(np.full(n, goal, dtype=object))
            all_source.append(np.full(n, 'ramp', dtype=object))
            all_clean.append(np.ones(n, dtype=bool))   # ramp trials are never in TRAIN_DIR (fault data)
            print(f"  {n} windows, confidence_scale range experienced: "
                  f"[{conf.min():.2f}, {conf.max():.2f}], {(conf < 0.99).sum()} windows with fault active")

        # --- matched nominal trials ---
        nom_dir = os.path.join(DATA_DIR, NOMINAL_CAMPAIGN, goal)
        for run_dir in sorted(glob.glob(os.path.join(nom_dir, 't*'))):
            run_name = os.path.basename(run_dir)
            is_clean = run_name in CAL_DIR_TRIALS
            print(f"\n[nominal] {goal}/{run_name}  (held-out/clean: {is_clean})")
            out = process_trial(run_dir, NOMINAL_CAMPAIGN, shared_builder, kind='tl')
            if out is None:
                continue
            resid = residuals_for_sequences(model, device, out['sequences'])
            n = len(out['sequences'])
            all_conf.append(np.ones(n))   # nominal = confidence_scale 1.0, the reference point
            for k in _SERIES_KEYS:
                all_resid[k].append(resid[k])
            all_goal.append(np.full(n, goal, dtype=object))
            all_source.append(np.full(n, 'nominal', dtype=object))
            all_clean.append(np.full(n, is_clean, dtype=bool))
            print(f"  {n} windows")

    conf = np.concatenate(all_conf)
    goal_arr = np.concatenate(all_goal)
    source_arr = np.concatenate(all_source)
    clean_arr = np.concatenate(all_clean)
    resid = {k: np.concatenate(v) for k, v in all_resid.items()}

    # ── Dose-response: severity bins, CLEAN (goal_026 only) vs ALL (caveated) ──
    severity = 1.0 - conf   # 0 = no fault, 1 = complete confidence loss
    bins = np.array([0.0, 0.1, 0.3, 0.5, 0.7, 0.9, 1.001])
    bin_labels = [f'[{bins[i]:.1f},{bins[i+1]:.1f})' for i in range(len(bins) - 1)]
    bin_idx = np.digitize(severity, bins) - 1
    bin_idx = np.clip(bin_idx, 0, len(bin_labels) - 1)

    check_keys = ['position', 'velocity_lateral', 'traffic_light_color', 'traffic_light_confidence']
    results = {'clean_only': {}, 'all_including_contaminated': {}}
    for subset_name, mask_extra in [('clean_only', clean_arr), ('all_including_contaminated', np.ones_like(clean_arr))]:
        print(f"\n{'='*70}\n{subset_name}\n{'='*70}")
        print(f"{'severity bin':<16}{'n':>7}" + "".join(f"{k:>22}" for k in check_keys))
        for b in range(len(bin_labels)):
            m = (bin_idx == b) & mask_extra
            if m.sum() == 0:
                continue
            row = [f"{resid[k][m].mean():22.4f}" for k in check_keys]
            print(f"{bin_labels[b]:<16}{int(m.sum()):7d}" + "".join(row))
            results[subset_name][bin_labels[b]] = {'n': int(m.sum()), **{k: float(resid[k][m].mean()) for k in check_keys}}

    with open(os.path.join(args.output_dir, 'tl_severity_sweep_results.json'), 'w') as f:
        json.dump({'goals': GOALS, 'bins': bin_labels, 'results': results}, f, indent=2)

    # ── Plot: dose-response curves, clean (goal_026) vs all ──
    fig, axes = plt.subplots(1, len(check_keys), figsize=(5 * len(check_keys), 4.5))
    bin_centers = [(bins[i] + bins[i + 1]) / 2 for i in range(len(bins) - 1)]
    for ax, k in zip(axes, check_keys):
        for subset_name, mask_extra, style in [('clean (goal_026 only)', clean_arr, dict(color='#1f77b4', marker='o')),
                                                 ('all (goal_007/012 nominal is train-contaminated)', np.ones_like(clean_arr), dict(color='#999999', marker='x', linestyle='--'))]:
            ys = []
            for b in range(len(bin_labels)):
                m = (bin_idx == b) & mask_extra
                ys.append(resid[k][m].mean() if m.sum() > 0 else np.nan)
            ax.plot(bin_centers, ys, label=subset_name, **style)
        ax.set_xlabel('fault severity (1 - confidence_scale)')
        ax.set_ylabel(f'mean residual ({k})')
        ax.set_title(k, fontsize=10)
    axes[0].legend(fontsize=7)
    fig.suptitle('TL fault severity dose-response (reconstructed from tl_confidence_ramp + matched nominal)', fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    out_path = os.path.join(args.output_dir, 'tl_severity_dose_response.png')
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print(f"\nSaved {out_path}")


if __name__ == '__main__':
    main()
