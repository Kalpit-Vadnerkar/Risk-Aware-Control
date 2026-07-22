"""
Inference script for STGAT-RISE: compute residuals on test datasets.

For each test run, slides a window through the bag (same as training pipeline)
and records per-window residuals between model predictions and actual future values.
Uses the 1-step-ahead prediction (future[0]) so residuals are time-aligned.

Usage (with Autoware workspace sourced):
    source /home/kvadner/Desktop/Dissertation/autoware/install/setup.bash
    cd /home/kvadner/Desktop/Dissertation/Risk-Aware-Control
    python3 -m st_gat.infer [--datasets obs_recovery obs_noescape] [--out results/residuals]
    python3 -m st_gat.infer --rerun   # force re-process all (e.g. after adding columns)

Outputs:
    <out>/<dataset>/<run_name>.csv     — per-window residuals + scene signals
    <out>/summary.csv                  — CVaR_95 per run per metric

Residual types computed:
    pos_l2           : ||μ_pos - actual_pos||_2  (position prediction error, scaled [0,1])
    pos_nll          : Gaussian NLL for position (sum over x,y dims at step t+1)
    vel_nll          : Gaussian NLL for velocity
    combined_nll     : pos_nll + 0.8 * vel_nll  (matches training loss weighting)

Scene-level signals:
    obj_dist_actual  : actual object_distance at future step 0 (scaled [0,1], 1=far)
    obj_dist_pred    : model's predicted object_distance at step 0
    obj_dist_residual: |obj_dist_pred - obj_dist_actual|  (surprise about obstacle distance)
    obj_dist_slope   : linear slope of object_distance over the 10 most recent past steps
                       (negative = approaching obstacle; scale: per-timestep change in [0,1])
    obj_dist_past_min: minimum object_distance in the past window (proxy for closest approach)
    has_adjacent_lane: 1 if the ego's current lanelet has a driveable neighbor in the HD map,
                       0 otherwise (proactive signal: encodes whether avoidance is topologically
                       possible at this window, independent of vehicle trajectory)

Notes:
    - All test runs are processed regardless of result.status.
    - The model is in eval() mode (no dropout), but BatchNorm uses running stats.
    - CUSUM is computed per-run from the combined_nll time series.
    - Use --rerun to force re-processing even if a cached CSV exists.
    - The lanelet2 map and routing graph are loaded once in main() and reused across
      all runs (previously the map was reloaded from disk for each run — fixed here).
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
from st_gat.model.dataset import TrajectoryDataset   # reuse _build_* helpers

import networkx as nx


# ── Inline dataset for a single run's sequences ────────────────────────────

_MAX_GRAPH_NODES = cfg.MAX_GRAPH_NODES
_NODE_FEATURES   = cfg.NODE_FEATURES


def _build_feature_tensors(steps: list) -> dict:
    buf = {k: [] for k in ('position', 'velocity', 'steering', 'acceleration',
                            'object_distance', 'traffic_light_detected', 'uncertainty')}
    for step in steps:
        buf['position'].append(step['position'])
        buf['velocity'].append(step['velocity'])
        buf['steering'].append([step['steering']])
        buf['acceleration'].append([step['acceleration']])
        buf['object_distance'].append([step['object_distance']])
        buf['traffic_light_detected'].append([float(step['traffic_light_detected'])])
        buf['uncertainty'].append(step.get('uncertainty', [0.0, 0.0]))
    return {k: torch.tensor(v, dtype=torch.float32) for k, v in buf.items()}


def _build_graph_tensors(G) -> dict:
    nodes = sorted(G.nodes())
    n = len(nodes)
    nf = torch.zeros(_MAX_GRAPH_NODES, _NODE_FEATURES, dtype=torch.float32)
    adj = torch.zeros(_MAX_GRAPH_NODES, _MAX_GRAPH_NODES, dtype=torch.float32)
    if n > 0:
        k = min(n, _MAX_GRAPH_NODES)
        for i, node in enumerate(nodes[:k]):
            d = G.nodes[node]
            nf[i] = torch.tensor([
                d.get('x', 0.0), d.get('y', 0.0),
                float(d.get('traffic_light_detection_node', 0)),
                float(d.get('path_node', 0)),
            ], dtype=torch.float32)
        sub = nx.to_numpy_array(G, nodelist=nodes[:k])
        adj[:k, :k] = torch.tensor(sub, dtype=torch.float32)
    return {'node_features': nf, 'adj_matrix': adj}


class _RunDataset(Dataset):
    """Wraps a list of sequence dicts in-memory (no pkl I/O)."""
    def __init__(self, sequences: list):
        self.sequences = sequences

    def __len__(self):
        return len(self.sequences)

    def __getitem__(self, idx):
        seq = self.sequences[idx]
        return (
            _build_feature_tensors(seq['past']),
            _build_feature_tensors(seq['future']),
            _build_graph_tensors(seq['graph']),
            seq['graph_bounds'],
        )


# ── Residual computation ───────────────────────────────────────────────────

_LOG2PI = math.log(2 * math.pi)


def _gaussian_nll(mu: torch.Tensor, var: torch.Tensor, actual: torch.Tensor) -> torch.Tensor:
    """
    Element-wise Gaussian NLL: 0.5 * (log(2π σ²) + (actual - μ)² / σ²).
    Returns sum over last dimension (feature dims).
    """
    nll = 0.5 * (torch.log(var) + _LOG2PI + (actual - mu) ** 2 / var)
    return nll.sum(dim=-1)   # sum over feature dims


def _compute_residuals(preds: dict, future: dict, past: dict) -> dict:
    """
    Compute residuals at prediction step 0 (1-step-ahead) + scene-level signals.

    Returns dict of (B,) tensors:
        pos_l2, pos_nll, vel_nll, steer_nll, combined_nll,
        obj_dist_actual, obj_dist_pred, obj_dist_residual,
        obj_dist_slope, obj_dist_past_min
    """
    # ── Trajectory residuals ────────────────────────────────────────────────
    pos_mu    = preds['position_mean'][:, 0]        # (B, 2)
    pos_var   = preds['position_var'][:, 0]         # (B, 2)
    vel_mu    = preds['velocity_mean'][:, 0]        # (B, 2)
    vel_var   = preds['velocity_var'][:, 0]         # (B, 2)
    steer_mu  = preds['steering_mean'][:, 0]        # (B,)
    steer_var = preds['steering_var'][:, 0]         # (B,)

    actual_pos   = future['position'][:, 0]         # (B, 2)
    actual_vel   = future['velocity'][:, 0]         # (B, 2)
    actual_steer = future['steering'][:, 0, 0]      # (B,)

    pos_l2    = torch.norm(pos_mu - actual_pos, dim=-1)
    pos_nll   = _gaussian_nll(pos_mu, pos_var, actual_pos)
    vel_nll   = _gaussian_nll(vel_mu, vel_var, actual_vel)
    steer_nll = _gaussian_nll(
        steer_mu.unsqueeze(-1), steer_var.unsqueeze(-1),
        actual_steer.unsqueeze(-1),
    ).squeeze(-1)

    combined_nll = pos_nll + 0.8 * vel_nll + 0.5 * steer_nll

    # ── Scene-level signals ─────────────────────────────────────────────────
    # Object distance: model output is sigmoid-bounded, actual from future tensor
    obj_pred   = preds['object_distance'][:, 0]          # (B,)  predicted at t+1
    obj_actual = future['object_distance'][:, 0, 0]      # (B,)  actual at t+1
    obj_resid  = torch.abs(obj_pred - obj_actual)         # (B,)

    # Slope over the last 10 past steps (negative = approaching obstacle)
    past_od = past['object_distance'][:, :, 0]            # (B, T_in)
    recent  = past_od[:, -10:]                            # (B, 10)
    # Simple linear slope: (last - first) / 9
    obj_slope    = (recent[:, -1] - recent[:, 0]) / 9.0  # (B,)  per-timestep change
    obj_past_min = recent.min(dim=1).values               # (B,)  closest in recent window

    return {
        'pos_l2':           pos_l2,
        'pos_nll':          pos_nll,
        'vel_nll':          vel_nll,
        'steer_nll':        steer_nll,
        'combined_nll':     combined_nll,
        'obj_dist_actual':  obj_actual,
        'obj_dist_pred':    obj_pred,
        'obj_dist_residual':obj_resid,
        'obj_dist_slope':   obj_slope,
        'obj_dist_past_min':obj_past_min,
    }


# ── CUSUM ──────────────────────────────────────────────────────────────────

def _cusum(series: np.ndarray, delta: float = 0.5) -> np.ndarray:
    """
    Two-sided CUSUM on a 1-D series.  Returns max(C+, C-) at each timestep.
    delta: allowable slack (fraction of series mean).
    """
    mu = series.mean()
    C_pos = np.zeros_like(series)
    C_neg = np.zeros_like(series)
    for i in range(1, len(series)):
        C_pos[i] = max(0.0, C_pos[i-1] + series[i] - mu - delta)
        C_neg[i] = max(0.0, C_neg[i-1] + mu - series[i] - delta)
    return np.maximum(C_pos, C_neg)


# ── Lanelet2 adjacency helpers ─────────────────────────────────────────────

# Affine transform: Autoware bag frame (local_x/local_y ~81000,49000) →
# lanelet2 LocalCartesian frame (~3500,1800).  Derived from cfg.REFERENCE_POINTS,
# which give two corresponding points in both coordinate systems.
# y_B = A * x_A + B (x-channel),  y_B = C * y_A + D (y-channel)
_RP = cfg.REFERENCE_POINTS
_BAG2LL2_AX = (_RP[1][1][0] - _RP[0][1][0]) / (_RP[1][0][0] - _RP[0][0][0])
_BAG2LL2_BX = _RP[0][1][0] - _BAG2LL2_AX * _RP[0][0][0]
_BAG2LL2_CY = (_RP[1][1][1] - _RP[0][1][1]) / (_RP[1][0][1] - _RP[0][0][1])
_BAG2LL2_DY = _RP[0][1][1] - _BAG2LL2_CY * _RP[0][0][1]


def _build_lanelet_routing_graph(map_data):
    """
    Build a lanelet2 routing graph from an already-loaded map.
    Required for left/right/adjacentLeft/adjacentRight queries.
    Germany traffic rules are correct for the Shinjuku map (lanelet2 standard).
    """
    import lanelet2
    traffic_rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle,
    )
    return lanelet2.routing.RoutingGraph(map_data, traffic_rules)


def _has_adjacent_lane_at(
    routing_graph,
    map_data,
    bag_x: float,
    bag_y: float,
    cache: dict,
) -> bool:
    """
    Return True if the lanelet closest to (bag_x, bag_y) has any driveable
    neighbor (left, right, adjacentLeft, or adjacentRight) in the routing graph.

    bag_x, bag_y: ego position in the Autoware map frame (local_x/local_y OSM
    attributes, ~81000/49000 range). These are converted to the lanelet2
    LocalCartesian frame (~3500/1800 range) using the precomputed affine
    transform (_BAG2LL2_*) before querying the map.

    This is the proactive topological signal: it encodes whether the situation
    is avoidable at all, independent of the vehicle's current trajectory.

    Results are cached by 10m grid cell in the lanelet2 frame — lanelet
    transitions happen every ~20-50m, so 10m resolution is accurate with
    ~10× fewer lanelet2 queries.
    """
    # Convert from Autoware bag frame to lanelet2 LocalCartesian frame
    ll2_x = _BAG2LL2_AX * bag_x + _BAG2LL2_BX
    ll2_y = _BAG2LL2_CY * bag_y + _BAG2LL2_DY

    key = (round(ll2_x / 10.0), round(ll2_y / 10.0))
    if key in cache:
        return cache[key]

    best_ll = None
    best_dist_sq = float('inf')
    for ll in map_data.laneletLayer:
        try:
            if ll.attributes["subtype"] != "road":
                continue
        except Exception:
            continue
        cl = ll.centerline
        mid = len(cl) // 2
        dx = cl[mid].x - ll2_x
        dy = cl[mid].y - ll2_y
        d_sq = dx * dx + dy * dy
        if d_sq < best_dist_sq:
            best_dist_sq = d_sq
            best_ll = ll

    result = False
    if best_ll is not None:
        result = (
            routing_graph.left(best_ll) is not None
            or routing_graph.right(best_ll) is not None
            or routing_graph.adjacentLeft(best_ll) is not None
            or routing_graph.adjacentRight(best_ll) is not None
        )
    cache[key] = result
    return result


# ── CVaR ───────────────────────────────────────────────────────────────────

def _cvar(values: np.ndarray, alpha: float = 0.95) -> float:
    if len(values) == 0:
        return float('nan')
    var = np.quantile(values, alpha)
    tail = values[values >= var]
    return float(np.mean(tail)) if len(tail) > 0 else float(var)


# ── Per-run inference ──────────────────────────────────────────────────────

def run_inference(
    run_dir:      str,
    model:        torch.nn.Module,
    device:       torch.device,
    map_data=None,
    routing_graph=None,
    adj_cache:    dict | None = None,
    batch:        int = 256,
    verbose:      bool = False,
) -> pd.DataFrame | None:
    """
    Read one run's rosbag, build sequences, run model, return per-window DataFrame.
    Returns None if the bag has too few frames or sequence building fails.

    map_data and routing_graph: loaded once in main() and shared across all runs
    (avoids reloading the map from disk per run). If None, has_adjacent_lane is
    not computed.
    adj_cache: shared dict for lanelet adjacency memoization (10m grid).
    """
    bag_dir = os.path.join(run_dir, 'rosbag')
    if not os.path.isdir(bag_dir):
        print(f"  [infer] no rosbag dir in {run_dir}")
        return None

    from st_gat.pipeline.bag_reader import read_bag
    from st_gat.pipeline.sequence_builder import SequenceBuilder, extract_route_from_bag

    frames = read_bag(bag_dir, verbose=verbose)
    if len(frames) < cfg.INPUT_SEQ_LEN + cfg.OUTPUT_SEQ_LEN:
        print(f"  [infer] too few frames ({len(frames)}), skipping")
        return None

    # Build sequences using the pre-loaded map (avoids reloading map per run)
    if map_data is not None:
        route    = extract_route_from_bag(bag_dir)
        builder  = SequenceBuilder(map_data, route)
    else:
        builder = SequenceBuilder.from_bag(bag_dir)
    sequences = builder.build(frames, verbose=verbose)
    if not sequences:
        print(f"  [infer] zero sequences built, skipping")
        return None

    ds = _RunDataset(sequences)
    loader = DataLoader(ds, batch_size=batch, shuffle=False,
                        num_workers=0, pin_memory=True)

    _cols = ('pos_l2', 'pos_nll', 'vel_nll', 'steer_nll', 'combined_nll',
             'obj_dist_actual', 'obj_dist_pred', 'obj_dist_residual',
             'obj_dist_slope', 'obj_dist_past_min')
    all_results = {k: [] for k in _cols}

    model.eval()
    with torch.no_grad():
        for past, future, graph, _bounds in loader:
            past   = {k: v.to(device) for k, v in past.items()}
            future = {k: v.to(device) for k, v in future.items()}
            graph  = {k: v.to(device) for k, v in graph.items()}

            preds = model(past, graph)
            resid = _compute_residuals(preds, future, past)

            for k, v in resid.items():
                all_results[k].extend(v.cpu().numpy().tolist())

    df = pd.DataFrame(all_results)
    df.index.name = 'window'

    # Add CUSUM on combined_nll
    df['cusum_combined'] = _cusum(df['combined_nll'].values)

    # ── HD map adjacency (proactive topological signal) ─────────────────────
    # For window j, the ego's last past frame is frames[j * STRIDE + INPUT_SEQ_LEN - 1].
    # Raw map coordinates in frame['ego']['position'] are in the lanelet2 LocalCartesian
    # frame — queried directly against the routing graph.
    if routing_graph is not None and map_data is not None:
        _cache = adj_cache if adj_cache is not None else {}
        has_adj = []
        for j in range(len(sequences)):
            frame_idx = j * cfg.STRIDE + cfg.INPUT_SEQ_LEN - 1
            ego = frames[frame_idx]['ego']['position']
            has_adj.append(
                float(_has_adjacent_lane_at(routing_graph, map_data,
                                            ego['x'], ego['y'], _cache))
            )
        df['has_adjacent_lane'] = has_adj

    return df


# ── Main ───────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="ST-GAT-RISE inference on test datasets")
    parser.add_argument('--datasets', nargs='+', default=cfg.TEST_DATASETS,
                        help="Test datasets to evaluate (default: all)")
    parser.add_argument('--model', default=cfg.MODEL_CONFIG['model_path'],
                        help="Path to model weights (.pth)")
    parser.add_argument('--out', default=os.path.join(_REPO, 'st_gat', 'results', 'residuals'),
                        help="Output directory for CSVs")
    parser.add_argument('--batch', type=int, default=256)
    parser.add_argument('--rerun', action='store_true',
                        help="Re-process runs even if CSV cache exists (e.g. after adding columns)")
    parser.add_argument('--verbose', action='store_true')
    args = parser.parse_args()

    # Guard: no training data
    bad = set(args.datasets) & set(cfg.NOMINAL_DATASETS)
    if bad:
        print(f"ERROR: nominal datasets should not be used for test inference: {bad}")
        sys.exit(1)

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"[infer] device: {device}")
    print(f"[infer] model:  {args.model}")

    # ── Load model ─────────────────────────────────────────────────────────
    model_cfg = cfg.MODEL_CONFIG.copy()
    model_cfg.update({'d_model': 128, 'd_graph': 128, 'hidden_size': 128,
                      'num_layers': 2, 'nhead': 4, 'dropout_rate': 0.15,
                      'device': device})
    model = STGAT(model_cfg).to(device)
    model.load_state_dict(torch.load(args.model, map_location=device, weights_only=True))
    model.eval()
    print(f"[infer] loaded {model.count_parameters():,} parameter model")

    # ── Pre-load map and build routing graph (once, shared across all runs) ──
    print("[infer] Loading map (one-time, ~10-20s)...")
    from st_gat.pipeline.State_Estimator.MapProcessor import MapProcessor
    _map_processor = MapProcessor(cfg.MAP_FILE)
    _map_data      = _map_processor.map_data

    print("[infer] Building lanelet2 routing graph...")
    _routing_graph = _build_lanelet_routing_graph(_map_data)
    _adj_cache: dict = {}
    print("[infer] Map ready.")

    os.makedirs(args.out, exist_ok=True)
    summary_rows = []

    for dataset in args.datasets:
        dataset_dir = os.path.join(cfg.DATA_ROOT, dataset)
        if not os.path.isdir(dataset_dir):
            print(f"[infer] WARNING: dataset dir not found: {dataset_dir}")
            continue

        out_dir = os.path.join(args.out, dataset)
        os.makedirs(out_dir, exist_ok=True)

        run_names = sorted(
            e for e in os.listdir(dataset_dir)
            if os.path.isdir(os.path.join(dataset_dir, e)) and not e.endswith('.json')
        )
        print(f"\n[infer] Dataset: {dataset}  ({len(run_names)} runs)")

        for run_name in run_names:
            run_dir = os.path.join(dataset_dir, run_name)
            out_csv = os.path.join(out_dir, f"{run_name}.csv")

            if os.path.exists(out_csv) and not args.rerun:
                if args.verbose:
                    print(f"  [infer] skipping (cached): {run_name}")
                df = pd.read_csv(out_csv)
            else:
                print(f"  [infer] processing: {run_name}")
                df = run_inference(run_dir, model, device,
                                   map_data=_map_data,
                                   routing_graph=_routing_graph,
                                   adj_cache=_adj_cache,
                                   batch=args.batch, verbose=args.verbose)
                if df is None:
                    continue
                df.to_csv(out_csv, index=True)
                print(f"    → {len(df)} windows saved")

            # Load result metadata
            result_file = os.path.join(run_dir, 'result.json')
            status = 'unknown'
            if os.path.exists(result_file):
                with open(result_file) as f:
                    status = json.load(f).get('status', 'unknown')

            row = {
                'dataset':              dataset,
                'run_name':             run_name,
                'status':               status,
                'n_windows':            len(df),
                'cvar95_pos_l2':        _cvar(df['pos_l2'].values),
                'cvar95_pos_nll':       _cvar(df['pos_nll'].values),
                'cvar95_combined':      _cvar(df['combined_nll'].values),
                'cvar95_cusum':         _cvar(df['cusum_combined'].values),
                'mean_pos_l2':          float(df['pos_l2'].mean()),
                'mean_combined_nll':    float(df['combined_nll'].mean()),
                'max_combined_nll':     float(df['combined_nll'].max()),
            }
            # Scene-level columns (may not exist in old CSVs)
            for col in ('obj_dist_residual', 'obj_dist_slope', 'obj_dist_past_min',
                        'obj_dist_actual', 'obj_dist_pred'):
                if col in df.columns:
                    row[f'mean_{col}']   = float(df[col].mean())
                    row[f'cvar95_{col}'] = _cvar(df[col].values)
                    row[f'min_{col}']    = float(df[col].min())

            # has_adjacent_lane: fraction of windows where avoidance is topologically possible
            if 'has_adjacent_lane' in df.columns:
                row['frac_adjacent_lane'] = float(df['has_adjacent_lane'].mean())

            summary_rows.append(row)

    summary_df = pd.DataFrame(summary_rows)
    summary_path = os.path.join(args.out, 'summary.csv')
    summary_df.to_csv(summary_path, index=False)
    print(f"\n[infer] Summary saved to {summary_path}")

    if not summary_df.empty:
        from scipy.stats import mannwhitneyu

        print("\n── Per-dataset summary ─────────────────────────────────────")
        show_cols = [c for c in ['cvar95_combined', 'mean_pos_l2', 'mean_obj_dist_residual',
                                  'mean_obj_dist_slope', 'min_obj_dist_past_min',
                                  'frac_adjacent_lane']
                     if c in summary_df.columns]
        grp = summary_df.groupby('dataset')[show_cols].agg(['mean', 'std'])
        print(grp.to_string())

        if 'obs_recovery' in summary_df.dataset.values and \
           'obs_noescape' in summary_df.dataset.values:
            rec_df = summary_df[summary_df.dataset == 'obs_recovery']
            noe_df = summary_df[summary_df.dataset == 'obs_noescape']

            print("\n── Statistical tests (Mann-Whitney one-sided) ──────────────")
            tests = [
                ('max_combined_nll',      'recovery > noescape', True),
                ('cvar95_combined',       'noescape > recovery', False),
                ('mean_obj_dist_residual','recovery > noescape', True),
                ('mean_obj_dist_slope',   'noescape > recovery', False),  # more negative = approaching
                ('frac_adjacent_lane',    'recovery > noescape', True),   # topology: expected perfect sep
            ]
            for col, label, rec_greater in tests:
                if col not in summary_df.columns:
                    continue
                r = rec_df[col].dropna()
                n = noe_df[col].dropna()
                alt = 'greater' if rec_greater else 'less'
                stat, p = mannwhitneyu(r, n, alternative=alt)
                sig = '***' if p < 0.01 else ('**' if p < 0.05 else ('*' if p < 0.10 else ''))
                print(f"  {col:30s}  ({label})  p={p:.4f} {sig}")
                print(f"    recovery: {r.mean():.4f} ± {r.std():.4f}")
                print(f"    noescape: {n.mean():.4f} ± {n.std():.4f}")


if __name__ == '__main__':
    main()
