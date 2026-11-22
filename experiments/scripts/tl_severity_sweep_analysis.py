#!/usr/bin/env python3
"""
TL fault severity dose-response: model residuals binned by the confidence
scale the TL fault applied (1.0 = no fault), across the fixed-severity
tl_confidence campaigns (tl_conf_030/050/070) and matched nominal trials of
the same goals. Each trial's scale is read from its own fault_log.jsonl
(confidence_scale_for_trial; tl_confidence_ramp trials are reconstructed
from the deterministic ramp).

Clean vs all: a nominal trial is clean if its goal is in the calibration
split (goal_split_manifest.json), i.e. the model never trained on it; fault
trials are always clean. Both are reported.

Usage (ROS + repo venv sourced):
  source /opt/ros/humble/setup.bash
  source /home/kvadner/Desktop/Dissertation/autoware/install/setup.bash
  source .venv/bin/activate
  python3 experiments/scripts/tl_severity_sweep_analysis.py
"""

import argparse
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
from st_gat.pipeline import goal_split as gs  # noqa: E402
from config import load_campaigns  # noqa: E402
from trials import campaigns, trial_dirs  # noqa: E402

OUTPUT_DIR = os.path.join(REPO_DIR, 'experiments', 'analysis', 'tl_severity_sweep')

def _trial_is_clean(goal: str) -> bool:
    """Nominal trial the model never trained on: its goal is in the cal split."""
    return gs.load_manifest().get(NOMINAL_CAMPAIGN, {}).get(goal) == 'cal'
_SERIES_KEYS = [s[0] for s in _SERIES]

GOALS = ['goal_007', 'goal_012', 'goal_026']
# Every fixed-severity TL confidence campaign
TL_FAULT_CAMPAIGNS = [c for c in campaigns('tl') if load_campaigns()[c].fault['tl_fault'] == 'tl_confidence']
NOMINAL_CAMPAIGN = 'nominal'


def _load_model(model_path):
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model_cfg = cfg.build_inference_model_cfg(device)
    model = STGAT(model_cfg).to(device)
    model.load_state_dict(torch.load(model_path, map_location=device, weights_only=True))
    model.eval()
    print(f"Loaded model: {model_path}")
    return model, device


def confidence_scale_for_trial(t_rel, fault_windows, events):
    """confidence_scale(t) for every window -- dispatches on which TL fault
    mode this trial actually used (2026-08-26, added once the fixed-severity
    campaigns existed to analyze): `tl_confidence` holds one FIXED
    confidence_scale for the whole fault-active window (read straight from
    the trial's own tl_fault_start event -- no reconstruction, no ramp
    confound); `tl_confidence_ramp` decays continuously and must be
    reconstructed from the known rate/floor (see reconstruct_ramp below,
    kept as the original 2026-08-25 pilot's method, unchanged)."""
    fault_type = next((e.get('fault_type') for e in events if e['event'] == 'tl_fault_start'), None)
    if fault_type == 'tl_confidence':
        fixed_scale = next(e['params']['confidence_scale'] for e in events if e['event'] == 'tl_fault_start')
        conf = np.ones(len(t_rel))
        for i, t in enumerate(t_rel):
            if any(w['start'] <= t <= w['end'] for w in fault_windows):
                conf[i] = fixed_scale
        return conf
    elif fault_type == 'tl_confidence_ramp':
        rate = next(e['params'].get('confidence_ramp_rate_per_s', 0.1) for e in events if e['event'] == 'tl_fault_start')
        floor = next(e['params'].get('min_confidence_scale', 0.0) for e in events if e['event'] == 'tl_fault_start')
        return reconstruct_ramp(t_rel, fault_windows, rate, floor)
    else:
        # nominal trial (kind='tl', no fault ever configured) -- always confidence_scale=1.0
        return np.ones(len(t_rel))


def reconstruct_ramp(t_rel, fault_windows, rate, floor):
    """confidence_scale(t) for every window, exact (deterministic ramp) --
    1.0 outside any fault window, decaying at `rate` per second since that
    cycle's own tl_fault_start, floored at `floor`, inside one."""
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
            graph_t = TrajectoryDataset._build_graph_tensors(seq['graph'], seq.get('position_ref'), seq.get('heading_ref'), seq.get('route_ahead'))
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
    all_source = []   # campaign name or 'nominal'
    all_clean = []    # bool: is the nominal reference for this goal held-out (True) or train-contaminated (False)

    for goal in GOALS:
        # --- TL-fault trials ---
        for campaign in TL_FAULT_CAMPAIGNS:
            for run_dir in trial_dirs(campaign, goals=[goal]):
                print(f"\n[{campaign}] {goal}/{os.path.basename(run_dir)}")
                out = process_trial(run_dir, campaign, shared_builder, kind='tl')
                if out is None:
                    continue
                events = load_fault_log(run_dir)
                conf = confidence_scale_for_trial(out['t_rel'], out['fault_windows'], events)
                resid = residuals_for_sequences(model, device, out['sequences'])
                n = len(out['sequences'])
                all_conf.append(conf)
                for k in _SERIES_KEYS:
                    all_resid[k].append(resid[k])
                all_goal.append(np.full(n, goal, dtype=object))
                all_source.append(np.full(n, campaign, dtype=object))
                all_clean.append(np.ones(n, dtype=bool))   # fault trials are never in TRAIN_DIR
                print(f"  {n} windows, confidence_scale range experienced: "
                      f"[{conf.min():.2f}, {conf.max():.2f}], {(conf < 0.99).sum()} windows with fault active")

        # --- matched nominal trials ---
        for run_dir in trial_dirs(NOMINAL_CAMPAIGN, goals=[goal]):
            run_name = os.path.basename(run_dir)
            is_clean = _trial_is_clean(goal)
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

    # ── Dose-response: severity bins, CLEAN (never-trained-on trials) vs ALL (caveated) ──
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
        for subset_name, mask_extra, style in [('clean (goal_026 + post-retrain trials only)', clean_arr, dict(color='#1f77b4', marker='o')),
                                                 ('all (goal_007/012 pre-retrain nominal is train-contaminated)', np.ones_like(clean_arr), dict(color='#999999', marker='x', linestyle='--'))]:
            ys = []
            for b in range(len(bin_labels)):
                m = (bin_idx == b) & mask_extra
                ys.append(resid[k][m].mean() if m.sum() > 0 else np.nan)
            ax.plot(bin_centers, ys, label=subset_name, **style)
        ax.set_xlabel('fault severity (1 - confidence_scale)')
        ax.set_ylabel(f'mean residual ({k})')
        ax.set_title(k, fontsize=10)
    axes[0].legend(fontsize=7)
    fig.suptitle('TL fault severity dose-response (tl_confidence campaigns + matched nominal)', fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    out_path = os.path.join(args.output_dir, 'tl_severity_dose_response.png')
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print(f"\nSaved {out_path}")


if __name__ == '__main__':
    main()
