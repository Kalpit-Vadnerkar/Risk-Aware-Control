#!/usr/bin/env python3
"""
"Good enough for safety" benchmark for the ST-GAT point predictor
(2026-10-04). Layer 2 consumes the model's position forecast as the centre
of its counterfactual tube; if the forecast itself leaves the road on
ordinary nominal driving, Layer 2's P(violation) measures predictor error
instead of risk (found while building Layer 2b v2: ~19-23% of held-out
nominal predictions left the drivable area, nominal P(violation) floor ~0.3).
This script turns "good enough" into numbers, on held-out CAL_DIR windows:

  accuracy      ADE/FDE; lateral (cross-track) and longitudinal error at
                1/2/3s in the ego's heading frame -- lateral is what decides
                road departure, so it's reported separately.
  map-consistency
                off-road rate: fraction of predictions whose ego footprint
                leaves the drivable area at any step (margin.DrivableArea --
                the realized futures score 0% on this, so any nonzero rate
                is the model's).
  maneuver      phantom-turn rate (predicts >10 deg heading change on a
                window that actually stays within 10 deg), wrong-direction
                and under-turn rates on real turns (>= 30 deg).
  map reliance  how much the forecast depends on the map at all: change in
                median FDE when each window's graph is swapped for a random
                other window's, and when the route (path_node) flag is zeroed.
                A model that ignores the map shows ~0 here.

Doubles as the metrics table for the architecture ablation study (TODO.md
§7): every candidate checkpoint gets one row, appended to
experiments/analysis/model_safety_benchmark/benchmark_table.md.

--gate exits non-zero if any GATE threshold is missed (for
promote_model.py). Thresholds are a first proposal, not yet validated --
see GATE below.

Usage (ROS + venv sourced -- needs lanelet2):
  python3 experiments/scripts/model_safety_benchmark.py [--model PATH] [--tag NAME] [--gate]
"""

import argparse
import json
import os
import pickle
import sys
from datetime import datetime

import numpy as np
import torch
from torch.utils.data import DataLoader

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
sys.path.insert(0, REPO_DIR)
sys.path.insert(0, os.path.join(REPO_DIR, 'experiments', 'lib'))
sys.path.insert(0, SCRIPT_DIR)

import margin as margin_lib  # noqa: E402
from st_gat.pipeline import config as cfg  # noqa: E402
from st_gat.model import STGAT, TrajectoryDataset  # noqa: E402
from conformal_mondrian_calibration import assign_groups  # noqa: E402

OUTPUT_DIR = os.path.join(REPO_DIR, 'experiments', 'analysis', 'model_safety_benchmark')   # one shared table across schemas
R_POS = cfg.POSITION_DISPLACEMENT_RANGE_M

# First-proposal thresholds. Rationale: the median realized nominal clearance
# between the ego footprint and the road edge is ~0.7m (safe_set_check.json),
# so a forecast whose 3s lateral error is routinely above that cannot centre a
# risk tube; off-road/phantom/wrong-direction rates are set where Layer 2's
# nominal false-alarm floor would be dominated by risk rather than by model
# error. Revisit once a map-consistent model exists to compare against.
GATE = {
    'offroad_rate':          ('<=', 0.02),
    'lat_err_3s_p50':        ('<=', 0.50),
    'phantom_turn_rate':     ('<=', 0.05),
    'wrong_direction_rate':  ('<=', 0.05),
}


class _SeqList(TrajectoryDataset):
    def __init__(self, sequences):   # noqa: deliberately skips folder loading
        self.sequences = sequences


def _wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def _ang(v):
    return np.arctan2(v[..., 1], v[..., 0])


def predict(model, device, seqs, batch, graph_override=None, zero_route=False):
    """Real-metre absolute forecast (N, T, 2). graph_override: list of graphs
    to substitute (map-reliance test)."""
    if graph_override is not None:
        seqs = [dict(s, graph=g) for s, g in zip(seqs, graph_override)]
    out = []
    with torch.no_grad():
        for past, _f, graph, _b in DataLoader(_SeqList(seqs), batch_size=batch, shuffle=False, num_workers=0):
            past = {k: v.to(device) for k, v in past.items()}
            graph = {k: v.to(device) for k, v in graph.items()}
            if zero_route:
                graph['node_features'][..., 3] = 0
            out.append(model(past, graph)['position_mean'].cpu().numpy())
    ref = np.array([s['position_ref'] for s in seqs], dtype=float)
    return ref[:, None, :] + np.concatenate(out) * R_POS


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--model', default=cfg.MODEL_CONFIG['model_path'])
    ap.add_argument('--tag', default=None, help='row label in the benchmark table (default: checkpoint path)')
    ap.add_argument('--stride', type=int, default=5, help='evaluate every Nth CAL window')
    ap.add_argument('--batch', type=int, default=256)
    ap.add_argument('--gate', action='store_true', help='exit 1 if any GATE threshold is missed')
    ap.add_argument('--seed', type=int, default=20261004)
    ap.add_argument('--output-dir', default=OUTPUT_DIR)
    args = ap.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = STGAT(cfg.build_inference_model_cfg(device)).to(device)
    model.load_state_dict(torch.load(args.model, map_location=device, weights_only=True))
    model.eval()
    drivable = margin_lib.DrivableArea(margin_lib.load_map(cfg.MAP_FILE))

    seqs = []
    for f in sorted(x for x in os.listdir(cfg.CAL_DIR) if x.endswith('.pkl')):
        with open(os.path.join(cfg.CAL_DIR, f), 'rb') as fh:
            seqs.extend(pickle.load(fh)[::args.stride])
    n = len(seqs)
    print(f"{n} held-out CAL windows (stride {args.stride}), model {args.model}")

    ref = np.array([s['position_ref'] for s in seqs], dtype=float)
    past = ref[:, None] + np.array([[f['position'] for f in s['past']] for s in seqs]) * R_POS
    act = ref[:, None] + np.array([[f['position'] for f in s['future']] for s in seqs]) * R_POS
    mu = predict(model, device, seqs, args.batch)

    # ego heading frame (from the last 0.5s of observed motion); stopped windows excluded from frame-based metrics
    dp = past[:, -1] - past[:, -6]
    moving = np.linalg.norm(dp, axis=-1) > 0.5
    u = dp / np.maximum(np.linalg.norm(dp, axis=-1, keepdims=True), 1e-6)
    nrm = np.stack([-u[:, 1], u[:, 0]], -1)
    err = mu - act
    lat = np.abs((err * nrm[:, None]).sum(-1))   # (N, T)
    lon = np.abs((err * u[:, None]).sum(-1))
    dist = np.linalg.norm(err, axis=-1)

    rm = margin_lib.road_margin(drivable.signed_distance(mu[..., 0], mu[..., 1])).min(1)
    rm_act = margin_lib.road_margin(drivable.signed_distance(act[..., 0], act[..., 1])).min(1)
    offroad = rm < 0

    h0 = _ang(dp)
    d_act, d_mu = act[:, -1] - act[:, -6], mu[:, -1] - mu[:, -6]
    dpsi_act = np.where(np.linalg.norm(d_act, axis=-1) > 0.5, _wrap(_ang(d_act) - h0), 0.0)
    dpsi_mu = np.where(np.linalg.norm(d_mu, axis=-1) > 0.5, _wrap(_ang(d_mu) - h0), 0.0)
    straight = moving & (np.abs(dpsi_act) < np.radians(10))
    turn = moving & (np.abs(dpsi_act) >= np.radians(30))
    ratio = dpsi_mu[turn] / dpsi_act[turn]

    rng = np.random.default_rng(args.seed)
    perm = rng.permutation(n)
    fde = dist[:, -1]
    fde_swap = np.linalg.norm(predict(model, device, seqs, args.batch,
                                      graph_override=[seqs[j]['graph'] for j in perm])[:, -1] - act[:, -1], axis=-1)
    fde_noroute = np.linalg.norm(predict(model, device, seqs, args.batch, zero_route=True)[:, -1] - act[:, -1], axis=-1)

    m = {
        'n_windows': n, 'n_moving': int(moving.sum()),
        'ade': float(dist.mean()), 'fde_p50': float(np.median(fde)), 'fde_p90': float(np.percentile(fde, 90)),
        'lat_err_1s_p50': float(np.median(lat[moving, 9])), 'lat_err_2s_p50': float(np.median(lat[moving, 19])),
        'lat_err_3s_p50': float(np.median(lat[moving, -1])), 'lat_err_3s_p90': float(np.percentile(lat[moving, -1], 90)),
        'lon_err_3s_p50': float(np.median(lon[moving, -1])), 'lon_err_3s_p90': float(np.percentile(lon[moving, -1], 90)),
        'offroad_rate': float(offroad.mean()),
        'offroad_rate_realized_future': float((rm_act < 0).mean()),
        'offroad_rate_moving': float(offroad[moving].mean()),
        'phantom_turn_rate': float((np.abs(dpsi_mu[straight]) > np.radians(10)).mean()),
        'n_turns': int(turn.sum()),
        'wrong_direction_rate': float((ratio < 0).mean()) if turn.any() else float('nan'),
        'under_turn_rate': float((ratio < 0.5).mean()) if turn.any() else float('nan'),
        'map_reliance_fde_delta_graph_swap': float(np.median(fde_swap) - np.median(fde)),
        'route_reliance_fde_delta_zero_path_node': float(np.median(fde_noroute) - np.median(fde)),
        'offroad_rate_by_group': {g: float(offroad[grp == g].mean())
                                  for grp in [assign_groups(ref)] for g in np.unique(grp)},
    }
    gate = {k: (op, thr, m[k], bool(m[k] <= thr)) for k, (op, thr) in GATE.items()}
    m['gate'] = {k: {'threshold': f'{op} {thr}', 'value': v, 'pass': p} for k, (op, thr, v, p) in gate.items()}
    m['gate_pass'] = all(p for *_x, p in gate.values())

    print("\n=== Model safety benchmark ===")
    for k, v in m.items():
        if k not in ('gate', 'offroad_rate_by_group'):
            print(f"  {k:<42} {v:.4f}" if isinstance(v, float) else f"  {k:<42} {v}")
    print("  offroad_rate_by_group:", {k: round(v, 3) for k, v in m['offroad_rate_by_group'].items()})
    print("\n  GATE:")
    for k, g in m['gate'].items():
        print(f"    {'PASS' if g['pass'] else 'FAIL'}  {k:<24} {g['value']:.4f}  ({g['threshold']})")

    tag = args.tag or os.path.relpath(args.model, REPO_DIR)
    m['model'] = args.model
    m['tag'] = tag
    m['date'] = datetime.now().isoformat(timespec='seconds')
    safe_tag = tag.replace('/', '__')
    with open(os.path.join(args.output_dir, f'benchmark_{safe_tag}.json'), 'w') as f:
        json.dump(m, f, indent=2)
    table = os.path.join(args.output_dir, 'benchmark_table.md')
    cols = ['fde_p50', 'lat_err_3s_p50', 'lat_err_3s_p90', 'lon_err_3s_p50', 'offroad_rate', 'phantom_turn_rate',
            'wrong_direction_rate', 'under_turn_rate', 'map_reliance_fde_delta_graph_swap',
            'route_reliance_fde_delta_zero_path_node']
    if not os.path.exists(table):
        with open(table, 'w') as f:
            f.write('| model | date | ' + ' | '.join(cols) + ' | gate |\n')
            f.write('|' + '---|' * (len(cols) + 3) + '\n')
    with open(table, 'a') as f:
        f.write(f"| {tag} | {m['date'][:10]} | " + ' | '.join(f"{m[c]:.3f}" for c in cols)
                + f" | {'PASS' if m['gate_pass'] else 'FAIL'} |\n")
    print(f"\nSaved {table}")
    if args.gate and not m['gate_pass']:
        sys.exit(1)


if __name__ == '__main__':
    main()
