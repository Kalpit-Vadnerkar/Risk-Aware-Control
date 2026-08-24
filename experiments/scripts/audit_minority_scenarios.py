#!/usr/bin/env python3
"""
Systematic minority-scenario audit (2026-08-24) -- Kalpit's point 3: the
turn-anticipation gap was only found because someone happened to look at
the right plot. That's not a repeatable strategy. This script instead
checks EVERY scenario category the project's own fault-injection
infrastructure already cares about, grounded in real geometry rather than
an ad hoc metric: `experiments/configs/turn_zones.json` (turn_zones,
bias_leadin_zones, lane_change_zones, curved_road_zones -- computed by
compute_turn_zones.py from real driven trajectories, curvature-thresholded)
and `experiments/configs/tl_zones.json` (real traffic-light intersections).
turn_zones and bias_leadin_zones are exactly what fault_injector.py gates
IMU faults on; tl_zones is exactly what TL faults are gated near -- i.e.
these are not an arbitrary taxonomy, they're the actual loci fault
campaigns already target. lane_change_zones/curved_road_zones are
currently NOT fault-targeted but are real, geometrically distinct minority
scenarios worth checking anyway (Kalpit's "other minority scenes" concern).

For each calibration window, label it by which zone category its actual
position falls in (pooling zone points across all 26 goals -- these are
fixed physical map locations, not goal-specific, so pooling is correct,
not an approximation), then report per-category: how much of the dataset
it is, mean residual, and per-step coverage AGAINST THE ALREADY-DEPLOYED
GLOBAL quantile (conformal_report.json's mean_fold_quantile_by_step --
reused as-is, not recomputed, since the question here is "how does the
calibration we already shipped perform per scenario," matching how a real
deployment would use it). A category with coverage well under the 90%
target relative to its size is a hidden gap the same way turns were.

Usage (repo venv sourced, no ROS needed):
  source .venv/bin/activate
  python3 experiments/scripts/audit_minority_scenarios.py \
      --model st_gat/checkpoints/h30_30_pointpred_v1/mean_warmup.pth
"""

import argparse
import json
import os
import sys

import numpy as np
import torch
from torch.utils.data import DataLoader

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
sys.path.insert(0, REPO_DIR)
sys.path.insert(0, SCRIPT_DIR)
sys.path.insert(0, os.path.join(REPO_DIR, 'experiments', 'lib'))

from st_gat.pipeline import config as cfg  # noqa: E402
from st_gat.model import STGAT, TrajectoryDataset  # noqa: E402
from conformal_horizon_calibration import _FEATURES, _extract_series  # noqa: E402
import scenario_zones  # noqa: E402

CONFORMAL_REPORT = os.path.join(REPO_DIR, 'experiments', 'analysis',
                                 'conformal_horizon_calibration', 'conformal_report.json')
DEFAULT_OUTPUT_DIR = os.path.join(REPO_DIR, 'experiments', 'analysis', 'minority_scenario_audit')

# Zone geometry (loading, pooling, radii) now lives in experiments/lib/
# scenario_zones.py -- shared with st_gat/train.py's --zone-weighted-sampling
# (2026-08-24) so the audit and the fix it motivated use the exact same
# definition of "near a turn/intersection zone."
_RADIUS_M = scenario_zones.RADIUS_M
_FAULT_TARGETED = {
    'turn_zones': True, 'bias_leadin_zones': True, 'lane_change_zones': False,
    'curved_road_zones': False, 'tl_zones': True,
}


def _load_model(model_path):
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model_cfg = cfg.build_inference_model_cfg(device)
    model = STGAT(model_cfg).to(device)
    model.load_state_dict(torch.load(model_path, map_location=device, weights_only=True))
    model.eval()
    print(f"Loaded model: {model_path} ({model.count_parameters():,} params)")
    return model, device


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--model', default=cfg.MODEL_CONFIG['model_path'])
    ap.add_argument('--batch', type=int, default=256)
    ap.add_argument('--output-dir', default=DEFAULT_OUTPUT_DIR)
    args = ap.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    with open(CONFORMAL_REPORT) as f:
        report = json.load(f)
    quantiles = {row['feature']: np.array(row['mean_fold_quantile_by_step']) for row in report['features']}

    trees = scenario_zones.load_zone_trees()
    for cat, (_, npts) in trees.items():
        print(f"  {cat}: {npts} pooled zone points (radius {_RADIUS_M[cat]}m)")

    model, device = _load_model(args.model)
    ds = TrajectoryDataset(cfg.CAL_DIR)
    n = len(ds)
    print(f"\n{n} calibration windows")

    loader = DataLoader(ds, batch_size=args.batch, shuffle=False, num_workers=0)
    errs = {k: [] for k in _FEATURES}
    with torch.no_grad():
        for past, future, graph, _bounds in loader:
            past   = {k: v.to(device) for k, v in past.items()}
            future = {k: v.to(device) for k, v in future.items()}
            graph  = {k: v.to(device) for k, v in graph.items()}
            preds  = model(past, graph)
            for key, dims in _FEATURES.items():
                mean   = preds[f'{key}_mean']
                actual = future[key]
                if dims == 1 and actual.dim() == 3 and actual.size(-1) == 1:
                    actual = actual.squeeze(-1)
                errs[key].append((actual - mean).cpu().numpy())
    errs = {k: np.concatenate(v, axis=0) for k, v in errs.items()}
    all_resid = _extract_series(errs)   # {report_key: (N, T_out)}

    # Real-world (x, y) at the window's start (last observed frame) -- the
    # moment fault_injector.py's own zone-arming logic checks against, so
    # this mirrors the actual live gating condition rather than an
    # after-the-fact reconstruction from the future trajectory.
    print("Computing per-window real-world position...")
    xy = scenario_zones.window_start_xy(ds, cfg)
    membership = scenario_zones.zone_membership(xy, trees)

    none_mask = ~np.any(np.stack(list(membership.values())), axis=0)
    membership['open_road (none of the above)'] = none_mask

    check_series = ['position', 'steering', 'velocity_longitudinal', 'acceleration']
    print(f"\n{'category':<30}{'n':>7}{'% data':>8}{'fault-targeted':>16}   " +
          "  ".join(f"{k} cov" for k in check_series))
    print('-' * (30 + 7 + 8 + 16 + 3 + sum(len(k) + 6 for k in check_series)))

    rows = []
    for cat, mask in membership.items():
        n_cat = int(mask.sum())
        if n_cat == 0:
            continue
        pct = 100 * n_cat / n
        targeted = _FAULT_TARGETED.get(cat, None)
        targeted_str = ('YES' if targeted else 'no') if targeted is not None else '--'
        cov_strs = []
        row = {'category': cat, 'n': n_cat, 'pct_of_data': pct, 'fault_targeted': targeted}
        for key in check_series:
            r = all_resid[key][mask]         # (n_cat, T_out)
            q = quantiles[key]               # (T_out,)
            cov = float((r <= q[np.newaxis, :]).mean())
            cov_strs.append(f"{cov:6.3f}")
            row[f'{key}_coverage'] = cov
            # all_resid[key] is already the reduced scalar MAGNITUDE per (window, step)
            # (see conformal_horizon_calibration.py's _extract_series -- l2-pooled for
            # position, abs() for scalar series), so no further reduction needed here.
            row[f'{key}_mean_resid'] = float(r.mean())
        print(f"{cat:<30}{n_cat:7d}{pct:7.1f}%{targeted_str:>16}   " + "  ".join(cov_strs))
        rows.append(row)

    out_path = os.path.join(args.output_dir, 'minority_scenario_audit.json')
    with open(out_path, 'w') as f:
        json.dump({'n_windows': n, 'target_coverage': 0.9, 'categories': rows}, f, indent=2)
    print(f"\nSaved {out_path}")

    flagged = [r for r in rows if any(r[f'{k}_coverage'] < 0.80 for k in check_series) and r['n'] >= 20]
    if flagged:
        print("\nFLAGGED (coverage <80% on some series, n>=20 -- worth investigating like the turn case):")
        for r in flagged:
            worst = min(check_series, key=lambda k: r[f'{k}_coverage'])
            print(f"  {r['category']}: {worst} coverage = {r[f'{worst}_coverage']:.3f} (n={r['n']})")
    else:
        print("\nNo other category flagged below 80% coverage at n>=20.")


if __name__ == '__main__':
    main()
