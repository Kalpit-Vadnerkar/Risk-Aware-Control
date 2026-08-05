"""
Per-timestep divergence-trace computation for STGAT-RISE.

Replaces the retired st_gat/infer.py, which had drifted stale: its own
duplicate copy of feature-tensor building only covered 7 of the 10 keys in
cfg.FEATURE_SIZES (missing traffic_light_state, traffic_light_discrepancy,
closest_object_velocity, has_adjacent_lane) and would KeyError on
STGAT.forward(); it targeted the deprioritized obs_recovery/obs_noescape
scenario campaigns (not even present under experiments/data/ on this
machine); and it only logged one CVaR95 number per run instead of a
per-timestep trace. See docs/stgat_pipeline_plan.md §1.12 / TODO.md Phase 1.3.

Builds the per-timestep divergence trace schema from
docs/theoretical_framework.md §5: for every model output feature, predicted
mean/var (where applicable) + actual + raw residual, kept as SEPARATE columns
(not pre-combined into one score) so CUSUM/calibration can be recomputed
offline without re-running the model. Also joins in, per trial:
  - fault-onset-relative time, from fault_log.jsonl's `wall_time` field
    (NOT `sim_time_sec` — a different, unrelated clock; see
    compare_fault_vs_nominal.py's extract_fault_windows docstring for the
    exact bug this distinction already caused once elsewhere in this repo)
  - the map-expectation channel (traffic_light_detected) kept separate from
    the perception-report channel (traffic_light_state/discrepancy)
  - fatal-moment markers: metrics.json's static_collision heuristic
    (Arm A) and the first frame where mrm_active flips True

Reuses TrajectoryDataset._build_feature_tensors/_build_graph_tensors
(st_gat/model/dataset.py) rather than re-implementing them, specifically to
avoid the duplication bug class above.

Usage (with Autoware workspace + repo venv sourced):
    source /home/kvadner/Desktop/Dissertation/autoware/install/setup.bash
    cd /home/kvadner/Desktop/Dissertation/Risk-Aware-Control
    source .venv/bin/activate
    python3 -m st_gat.residuals [--datasets imu_fault_stuck tl_fault_s4] [--out ...]
    python3 -m st_gat.residuals --rerun   # force re-process all (e.g. after schema changes)

Outputs:
    <out>/<dataset>/<run_name>.csv   — one row per timestep (1-step-ahead
                                       prediction; STRIDE=1 means consecutive
                                       windows' step-0 predictions land on
                                       consecutive real timesteps)
    <out>/summary.csv                — cheap per-run aggregate computed OVER
                                       the per-timestep traces above, not
                                       instead of them
"""

import argparse
import json
import math
import os
import sys

import numpy as np
import pandas as pd
import torch
from torch.utils.data import DataLoader, Dataset

from st_gat.pipeline import config as cfg
from st_gat.model import STGAT
from st_gat.model.dataset import TrajectoryDataset  # reuse _build_* helpers — do not reimplement
from st_gat.pipeline.run_pipeline import _goal_from_run_dir

_REPO = cfg.REPO_ROOT


# ── Inline dataset for a single run's sequences ────────────────────────────

class _RunDataset(Dataset):
    """Wraps a list of sequence dicts in-memory (no pkl I/O)."""
    def __init__(self, sequences: list):
        self.sequences = sequences

    def __len__(self):
        return len(self.sequences)

    def __getitem__(self, idx):
        seq = self.sequences[idx]
        return (
            TrajectoryDataset._build_feature_tensors(seq['past']),
            TrajectoryDataset._build_feature_tensors(seq['future']),
            TrajectoryDataset._build_graph_tensors(seq['graph']),
            seq['graph_bounds'],
        )


# ── Fault-onset / fatal-moment joins ────────────────────────────────────────

# Events fault_injector.py logs exactly once when a fault mechanism actually
# turns on for a trial — covers all TL modes (tl_fault_start, once per cycle;
# earliest = first onset) and all IMU modes (imu_fault_activated fires once
# per trial regardless of bias/bias_ramp/scale_factor/stuck_at).
_FAULT_ONSET_EVENTS = {'tl_fault_start', 'imu_fault_activated'}


def _load_fault_log(run_dir: str) -> list:
    path = os.path.join(run_dir, 'fault_log.jsonl')
    events = []
    if not os.path.exists(path):
        return events   # nominal trials never launch the fault interceptor
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


def _first_fault_onset_wall_time(events: list):
    """Earliest fault-onset event's wall_time (real unix seconds), or None.
    Uses wall_time, NOT sim_time_sec — see module docstring."""
    onsets = [e['wall_time'] for e in events
              if e.get('event') in _FAULT_ONSET_EVENTS and 'wall_time' in e]
    return min(onsets) if onsets else None


def _load_fatal_moment_markers(run_dir: str, frames: list) -> dict:
    """
    Trial-level fatal-moment markers (constant across every row of that
    trial's trace) — docs/theoretical_framework.md §6.

    permanent_stop_rel_s / likely_static_collision: from metrics.json's
    static_collision heuristic (Arm A) — already trial-relative seconds,
    same axis plot_fault_impact.py already overlays these on.
    mrm_first_trigger_rel_s: first frame where bag_reader's per-frame
    mrm_active flips True, converted to bag-relative seconds using frames[0]
    as t=0 (same convention as the fault-onset join below).
    """
    out = {
        'permanent_stop_rel_s':    float('nan'),
        'likely_static_collision': False,
        'mrm_first_trigger_rel_s': float('nan'),
    }
    metrics_path = os.path.join(run_dir, 'metrics.json')
    if os.path.exists(metrics_path):
        try:
            with open(metrics_path) as f:
                m = json.load(f)
            sc = m.get('static_collision', {}) or {}
            # .get(key, default) only applies the default when the KEY is
            # absent — metrics.json legitimately has permanent_stop_time_s
            # present but JSON null (Python None) for trials that never got
            # stuck, which .get() would return as-is, not float('nan').
            # Explicit None-check, not a truthy check (0.0 is a real,
            # valid stop time, not "missing").
            stop_time = sc.get('permanent_stop_time_s')
            out['permanent_stop_rel_s']    = float(stop_time) if stop_time is not None else float('nan')
            out['likely_static_collision'] = bool(sc.get('likely_static_collision', False))
        except Exception:
            pass
    if frames:
        t0 = frames[0]['t_sim']
        for fr in frames:
            if fr.get('mrm_active'):
                out['mrm_first_trigger_rel_s'] = fr['t_sim'] - t0
                break
    return out


# ── Residual computation ───────────────────────────────────────────────────

_LOG2PI = math.log(2 * math.pi)

# Output features with a Gaussian (mean/var) head. dims: number of
# dimensions the feature carries (2 for position/velocity, 1 for scalars).
_GAUSSIAN_FEATURES = {
    'position':            2,
    'velocity':             2,
    'steering':             1,
    'acceleration':         1,
    'traffic_light_state':  1,   # perception-report channel (§1.12)
}

# Output features with a deterministic sigmoid head (no predicted variance).
# object_distance removed 2026-08-02 (collapsed-scalar feature retired,
# replaced by objects_set/objects_mask — input-only, no output head; see
# model.py's ObjectSetEncoder and docs/theoretical_framework.md §3.1).
_SIGMOID_FEATURES = (
    'traffic_light_detected',    # map-expectation channel — kept separate
    'traffic_light_discrepancy', # perception-report channel (§1.12)
)


def _gaussian_residual(key: str, dims: int, preds: dict, future: dict, t: int = 0) -> dict:
    """Predicted mean/var + actual + raw residual + NLL for one Gaussian
    feature at prediction step t, per-dimension (not collapsed — see
    docs/theoretical_framework.md §5's 'log full traces' mantra)."""
    mu  = preds[f'{key}_mean'][:, t]     # (B,) or (B, dims)
    var = preds[f'{key}_var'][:, t]      # (B,) or (B, dims)
    actual = future[key][:, t]           # (B, 1) or (B, dims)

    if dims == 1 and actual.dim() == 2 and actual.size(-1) == 1:
        actual = actual.squeeze(-1)

    cols = {}
    if dims == 1:
        residual = torch.abs(mu - actual)
        nll = 0.5 * (torch.log(var) + _LOG2PI + (actual - mu) ** 2 / var)
        cols[f'{key}_pred_mean'] = mu
        cols[f'{key}_pred_var']  = var
        cols[f'{key}_actual']    = actual
        cols[f'{key}_residual']  = residual
        cols[f'{key}_nll']       = nll
    else:
        residual_l2 = torch.norm(mu - actual, dim=-1)
        nll = (0.5 * (torch.log(var) + _LOG2PI + (actual - mu) ** 2 / var)).sum(dim=-1)
        for d in range(dims):
            cols[f'{key}_pred_mean_{d}'] = mu[:, d]
            cols[f'{key}_pred_var_{d}']  = var[:, d]
            cols[f'{key}_actual_{d}']    = actual[:, d]
        cols[f'{key}_residual_l2'] = residual_l2
        cols[f'{key}_nll']         = nll
    return cols


def _sigmoid_residual(key: str, preds: dict, future: dict, t: int = 0) -> dict:
    """Predicted probability + actual + raw residual for one sigmoid-bounded
    feature at prediction step t (no predicted variance for these)."""
    pred = preds[key][:, t]              # (B,)
    actual = future[key][:, t]           # (B, 1)
    if actual.dim() == 2 and actual.size(-1) == 1:
        actual = actual.squeeze(-1)
    residual = torch.abs(pred - actual)
    return {
        f'{key}_pred':     pred,
        f'{key}_actual':   actual,
        f'{key}_residual': residual,
    }


# Temperature scaling (Guo et al. 2017 — see calibrate_tl_discrepancy.py's
# module docstring for the full reasoning): traffic_light_discrepancy is a
# Bernoulli head with no predicted-variance notion of confidence, so its raw
# residual above isn't calibrated the way the Gaussian features' NLL is.
# Loaded once at import time rather than per-row; falls back to T=1.0
# (no-op) if calibrate_tl_discrepancy.py hasn't been run yet against the
# current model -- this column is then identical to the uncalibrated one
# until it has been, not an error.
_TL_CALIBRATION_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'experiments', 'analysis', 'tl_calibration', 'temperature.json')
_TL_DISCREPANCY_TEMPERATURE = 1.0
if os.path.exists(_TL_CALIBRATION_FILE):
    with open(_TL_CALIBRATION_FILE) as _f:
        _TL_DISCREPANCY_TEMPERATURE = json.load(_f)['temperature']
    print(f"[residuals] traffic_light_discrepancy temperature scaling: T={_TL_DISCREPANCY_TEMPERATURE:.2f}")
else:
    print(f"[residuals] {_TL_CALIBRATION_FILE} not found -- traffic_light_discrepancy_calibrated_* "
          f"columns will be identical to the uncalibrated ones (T=1.0) until calibrate_tl_discrepancy.py is run")


def _calibrated_tl_discrepancy_residual(preds: dict, future: dict, t: int = 0) -> dict:
    """Same as _sigmoid_residual('traffic_light_discrepancy', ...) but using
    the temperature-scaled probability (sigmoid(logit / T)) instead of the
    model's raw, uncalibrated sigmoid output."""
    logit = preds['traffic_light_discrepancy_logit'][:, t]
    pred = torch.sigmoid(logit / _TL_DISCREPANCY_TEMPERATURE)
    actual = future['traffic_light_discrepancy'][:, t]
    if actual.dim() == 2 and actual.size(-1) == 1:
        actual = actual.squeeze(-1)
    return {
        'traffic_light_discrepancy_calibrated_pred':     pred,
        'traffic_light_discrepancy_calibrated_residual': torch.abs(pred - actual),
    }


def _compute_row(preds: dict, future: dict) -> dict:
    """One row of the divergence trace, at 1-step-ahead prediction (t=0)."""
    row = {}
    for key, dims in _GAUSSIAN_FEATURES.items():
        row.update(_gaussian_residual(key, dims, preds, future))
    for key in _SIGMOID_FEATURES:
        row.update(_sigmoid_residual(key, preds, future))
    row.update(_calibrated_tl_discrepancy_residual(preds, future))

    # Scene-context passthrough (ground truth only — no output head for
    # these; see docs/stgat_pipeline_plan.md's Stage 0/2 notes on
    # conditioning-only input features).
    row['has_adjacent_lane_actual'] = future['has_adjacent_lane'][:, 0, 0]
    # Object count, not a residual (objects_set/objects_mask, added
    # 2026-08-02, have no output head — see model.py's ObjectSetEncoder).
    row['n_objects_actual'] = future['objects_mask'][:, 0].sum(dim=-1)

    # Trajectory-tracking anomaly score, kept for continuity with the prior
    # obstacle-avoidance-era usage (position + 0.8*velocity + 0.5*steering) —
    # deliberately NOT combined with the TL negative-evidence signals above,
    # per the "kept as separate columns" schema requirement.
    row['combined_nll'] = row['position_nll'] + 0.8 * row['velocity_nll'] + 0.5 * row['steering_nll']

    return row


def _cusum(series: np.ndarray, delta: float = 0.5) -> np.ndarray:
    """
    Two-sided CUSUM on a 1-D series. Returns max(C+, C-) at each timestep.
    A pure function over a full array per call — stateless across trials by
    construction (each call starts from a fresh mu/C_pos/C_neg derived only
    from the array passed in), so the cross-trial-contamination bug class in
    docs/stgat_pipeline_plan.md §1.3 doesn't apply here.
    """
    mu = series.mean()
    C_pos = np.zeros_like(series)
    C_neg = np.zeros_like(series)
    for i in range(1, len(series)):
        C_pos[i] = max(0.0, C_pos[i-1] + series[i] - mu - delta)
        C_neg[i] = max(0.0, C_neg[i-1] + mu - series[i] - delta)
    return np.maximum(C_pos, C_neg)


def _cvar(values: np.ndarray, alpha: float = 0.95) -> float:
    if len(values) == 0:
        return float('nan')
    var = np.quantile(values, alpha)
    tail = values[values >= var]
    return float(np.mean(tail)) if len(tail) > 0 else float(var)


# ── Per-run trace ────────────────────────────────────────────────────────────

def run_trace(
    run_dir:  str,
    goal_id:  str,
    model:    torch.nn.Module,
    device:   torch.device,
    map_data=None,
    batch:    int = 256,
    verbose:  bool = False,
) -> pd.DataFrame | None:
    """
    Read one run's rosbag, build sequences, run model, return the per-timestep
    divergence trace (one row per window's 1-step-ahead prediction).
    Returns None if the bag has too few frames or sequence building fails.
    """
    bag_dir = os.path.join(run_dir, 'rosbag')
    if not os.path.isdir(bag_dir):
        print(f"  [residuals] no rosbag dir in {run_dir}")
        return None

    from st_gat.pipeline.bag_reader import read_bag
    from st_gat.pipeline.sequence_builder import SequenceBuilder, extract_route_from_bag

    frames = read_bag(bag_dir, goal_id=goal_id, verbose=verbose)
    if len(frames) < cfg.INPUT_SEQ_LEN + cfg.OUTPUT_SEQ_LEN:
        print(f"  [residuals] too few frames ({len(frames)}), skipping")
        return None

    if map_data is not None:
        route   = extract_route_from_bag(bag_dir)
        builder = SequenceBuilder(map_data, route)
    else:
        builder = SequenceBuilder.from_bag(bag_dir)
    sequences = builder.build(frames, verbose=verbose)
    if not sequences:
        print(f"  [residuals] zero sequences built, skipping")
        return None

    ds = _RunDataset(sequences)
    loader = DataLoader(ds, batch_size=batch, shuffle=False,
                        num_workers=0, pin_memory=True)

    all_rows: dict = {}
    model.eval()
    with torch.no_grad():
        for past, future, graph, _bounds in loader:
            past   = {k: v.to(device) for k, v in past.items()}
            future = {k: v.to(device) for k, v in future.items()}
            graph  = {k: v.to(device) for k, v in graph.items()}

            preds = model(past, graph)
            row = _compute_row(preds, future)

            for k, v in row.items():
                all_rows.setdefault(k, []).extend(v.cpu().numpy().tolist())

    df = pd.DataFrame(all_rows)
    df.index.name = 'window'

    # ── Timestamps: sim time + fault-onset-relative (docs/theoretical_framework.md §5) ──
    t0 = frames[0]['t_sim']
    # Window j's 1-step-ahead prediction target is future step 0, i.e.
    # frames[j*STRIDE + INPUT_SEQ_LEN].
    frame_idx = [j * cfg.STRIDE + cfg.INPUT_SEQ_LEN for j in range(len(sequences))]
    df['t_sim']     = [frames[i]['t_sim'] for i in frame_idx]
    df['t_bag_rel'] = df['t_sim'] - t0

    fault_events  = _load_fault_log(run_dir)
    onset_wall    = _first_fault_onset_wall_time(fault_events)
    if onset_wall is not None:
        onset_bag_rel   = onset_wall - t0
        df['t_rel_fault'] = df['t_bag_rel'] - onset_bag_rel
    else:
        df['t_rel_fault'] = float('nan')   # nominal trial, or fault never armed

    # ── CUSUM (per relevant signal, not just one combined score) ────────────
    df['cusum_combined']       = _cusum(df['combined_nll'].values)
    df['cusum_tl_state']       = _cusum(df['traffic_light_state_nll'].values)
    df['cusum_tl_discrepancy'] = _cusum(df['traffic_light_discrepancy_residual'].values)

    # ── Fatal-moment markers (constant per trial) ───────────────────────────
    markers = _load_fatal_moment_markers(run_dir, frames)
    for k, v in markers.items():
        df[k] = v

    return df


# ── Main ───────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="ST-GAT-RISE per-timestep divergence trace computation")
    parser.add_argument('--datasets', nargs='+',
                        default=cfg.NOMINAL_DATASETS + cfg.FAULT_DATASETS,
                        help="Datasets to process (default: nominal + all fault campaigns)")
    parser.add_argument('--model', default=cfg.MODEL_CONFIG['model_path'],
                        help="Path to model weights (.pth)")
    parser.add_argument('--out', default=os.path.join(_REPO, 'st_gat', 'results',
                                                       cfg.HORIZON_TAG, 'traces'),
                        help="Output directory for CSVs")
    parser.add_argument('--batch', type=int, default=256)
    parser.add_argument('--rerun', action='store_true',
                        help="Re-process runs even if CSV cache exists (e.g. after schema changes)")
    parser.add_argument('--verbose', action='store_true')
    args = parser.parse_args()

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"[residuals] device: {device}")
    print(f"[residuals] model:  {args.model}")

    model_cfg = cfg.MODEL_CONFIG.copy()
    model_cfg.update({'d_model': 128, 'd_graph': 128, 'hidden_size': 128,
                      'num_layers': 2, 'nhead': 4, 'dropout_rate': 0.15,
                      'device': device})
    model = STGAT(model_cfg).to(device)
    model.load_state_dict(torch.load(args.model, map_location=device, weights_only=True))
    model.eval()
    print(f"[residuals] loaded {model.count_parameters():,} parameter model")

    print("[residuals] Loading map (one-time, ~10-20s)...")
    from st_gat.pipeline.State_Estimator.MapProcessor import MapProcessor
    _map_processor = MapProcessor(cfg.MAP_FILE)
    _map_data      = _map_processor.map_data
    print("[residuals] Map ready.")

    os.makedirs(args.out, exist_ok=True)
    summary_rows = []

    for dataset in args.datasets:
        dataset_dir = os.path.join(cfg.DATA_ROOT, dataset)
        if not os.path.isdir(dataset_dir):
            print(f"[residuals] WARNING: dataset dir not found: {dataset_dir}")
            continue

        out_dir = os.path.join(args.out, dataset)
        os.makedirs(out_dir, exist_ok=True)

        # Runs are nested by goal: experiments/data/<dataset>/<goal_id>/<trial>/
        run_dirs = []
        for goal_entry in sorted(os.listdir(dataset_dir)):
            goal_dir = os.path.join(dataset_dir, goal_entry)
            if not os.path.isdir(goal_dir) or not goal_entry.startswith('goal_'):
                continue
            for trial_entry in sorted(os.listdir(goal_dir)):
                trial_dir = os.path.join(goal_dir, trial_entry)
                if not (os.path.isdir(trial_dir) and os.path.isdir(os.path.join(trial_dir, 'rosbag'))):
                    continue
                run_dirs.append((goal_entry, trial_dir))

        print(f"\n[residuals] Dataset: {dataset}  ({len(run_dirs)} runs)")

        for goal_id, run_dir in run_dirs:
            run_name = f"{goal_id}_{os.path.basename(run_dir)}"
            out_csv  = os.path.join(out_dir, f"{run_name}.csv")

            if os.path.exists(out_csv) and not args.rerun:
                if args.verbose:
                    print(f"  [residuals] skipping (cached): {run_name}")
                df = pd.read_csv(out_csv)
            else:
                print(f"  [residuals] processing: {run_name}")
                df = run_trace(run_dir, goal_id, model, device,
                               map_data=_map_data, batch=args.batch, verbose=args.verbose)
                if df is None:
                    continue
                df.to_csv(out_csv, index=True)
                print(f"    -> {len(df)} rows saved")

            result_file = os.path.join(run_dir, 'result.json')
            status = 'unknown'
            if os.path.exists(result_file):
                with open(result_file) as f:
                    status = json.load(f).get('status', 'unknown')

            row = {
                'dataset':                 dataset,
                'run_name':                run_name,
                'status':                  status,
                'n_rows':                  len(df),
                'cvar95_combined_nll':     _cvar(df['combined_nll'].values),
                'mean_combined_nll':       float(df['combined_nll'].mean()),
                'cvar95_tl_state_nll':     _cvar(df['traffic_light_state_nll'].values),
                'mean_tl_discrepancy':     float(df['traffic_light_discrepancy_actual'].mean()),
                'mean_tl_discrepancy_residual': float(df['traffic_light_discrepancy_residual'].mean()),
                'permanent_stop_rel_s':    float(df['permanent_stop_rel_s'].iloc[0]) if len(df) else float('nan'),
                'likely_static_collision': bool(df['likely_static_collision'].iloc[0]) if len(df) else False,
                'mrm_first_trigger_rel_s': float(df['mrm_first_trigger_rel_s'].iloc[0]) if len(df) else float('nan'),
            }
            summary_rows.append(row)

    summary_df = pd.DataFrame(summary_rows)
    summary_path = os.path.join(args.out, 'summary.csv')
    summary_df.to_csv(summary_path, index=False)
    print(f"\n[residuals] Summary saved to {summary_path}")

    if not summary_df.empty:
        print("\n-- Per-dataset summary -----------------------------------------")
        show_cols = [c for c in ['cvar95_combined_nll', 'cvar95_tl_state_nll',
                                  'mean_tl_discrepancy_residual',
                                  'mrm_first_trigger_rel_s', 'permanent_stop_rel_s']
                     if c in summary_df.columns]
        grp = summary_df.groupby('dataset')[show_cols].agg(['mean', 'std'])
        print(grp.to_string())


if __name__ == '__main__':
    main()
