#!/usr/bin/env python3
"""
Model promotion gate (2026-08-25) -- connects the training and calibration
pipelines per Kalpit's explicit request: every time a new candidate model
(new architecture, more training data, retuned sampling) is produced, (1)
confirm it's actually not worse than what's currently deployed before
promoting it, and (2) if promoted, the conformal calibration MUST be
regenerated against the new model -- an old model's calibration report
means nothing once the residual distribution it was fit on no longer
matches reality (exactly the live bug this session hit: v2 was promoted
last turn without regenerating experiments/analysis/
conformal_horizon_calibration/conformal_report.json, leaving every script
that defaults to it silently using a stale, mismatched calibration).

Confirmation criteria (both must hold, tolerances configurable, not
buried as unchangeable constants):
  1. Overall mean position residual does not regress by more than
     --regression-tolerance (default 10%).
  2. The worst-performing zone category (turn_zones / bias_leadin_zones /
     tl_zones / curved_road_zones / open_road -- see experiments/lib/
     scenario_zones.py, the same real geometry fault_injector.py gates
     faults on) does not regress by more than --regression-tolerance
     either. This is deliberately about the SCENARIO that's currently
     worst, not just the global average -- a candidate that improves the
     average by getting even better at what's already easy while quietly
     regressing the hardest scenario should NOT pass.

On PASS: copies the candidate to --target (default
cfg.MODEL_CONFIG['model_path'], the canonical path every script falls
back to), then re-runs conformal_horizon_calibration.py against it,
overwriting the canonical conformal_report.json. On FAIL: refuses to
promote, prints why, exits nonzero -- the existing model stays in place.

Usage (repo venv sourced, no ROS needed):
  source .venv/bin/activate
  python3 experiments/scripts/promote_model.py \
      --candidate st_gat/checkpoints/<new_run>/mean_warmup.pth
"""

import argparse
import os
import shutil
import subprocess
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
import scenario_zones  # noqa: E402


def _load_model(model_path, device):
    model_cfg = cfg.build_inference_model_cfg(device)
    model = STGAT(model_cfg).to(device)
    model.load_state_dict(torch.load(model_path, map_location=device, weights_only=True))
    model.eval()
    return model


def _position_residuals(model, device, ds, xy):
    """Returns (N,) mean-over-horizon position residual in metres, and
    zone membership per window (reusing the SAME geometry the audit and
    the training sampler both use -- one definition, not three)."""
    loader = DataLoader(ds, batch_size=256, shuffle=False, num_workers=0)
    resid = []
    with torch.no_grad():
        for past, future, graph, _bounds in loader:
            past   = {k: v.to(device) for k, v in past.items()}
            future = {k: v.to(device) for k, v in future.items()}
            graph  = {k: v.to(device) for k, v in graph.items()}
            preds  = model(past, graph)
            mean   = preds['position_mean']
            actual = future['position']
            r = torch.linalg.norm(actual - mean, dim=-1).mean(dim=-1)  # (B,) mean over horizon
            resid.append(r.cpu().numpy())
    resid = np.concatenate(resid) * cfg.POSITION_DISPLACEMENT_RANGE_M

    trees = scenario_zones.load_zone_trees()
    membership = scenario_zones.zone_membership(xy, trees)
    return resid, membership


def _category_means(resid, membership):
    priority = ['turn_zones', 'bias_leadin_zones', 'tl_zones', 'curved_road_zones']
    assigned = np.full(len(resid), 'open_road', dtype=object)
    covered = np.zeros(len(resid), dtype=bool)
    for cat in priority:
        mask = membership[cat] & ~covered
        assigned[mask] = cat
        covered |= mask
    out = {}
    for cat in priority + ['open_road']:
        mask = assigned == cat
        if mask.sum() > 0:
            out[cat] = float(resid[mask].mean())
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--candidate', required=True)
    ap.add_argument('--current', default=cfg.MODEL_CONFIG['model_path'])
    ap.add_argument('--target', default=cfg.MODEL_CONFIG['model_path'],
                     help='where to copy the candidate if it passes -- default is the canonical '
                          'path every script falls back to')
    ap.add_argument('--regression-tolerance', type=float, default=0.10,
                     help='fraction a metric is allowed to get worse by and still pass (default 10%%)')
    ap.add_argument('--force', action='store_true',
                     help='promote and recalibrate even if the candidate fails the gate '
                          '(prints the failure reason regardless -- use deliberately, not by default)')
    ap.add_argument('--skip-recalibration', action='store_true',
                     help='promote without regenerating conformal_report.json -- DANGEROUS, only for '
                          'debugging the gate itself; leaves calibration stale for whatever is promoted')
    args = ap.parse_args()

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    ds = TrajectoryDataset(cfg.CAL_DIR)
    xy = scenario_zones.window_start_xy(ds, cfg)

    print(f"Evaluating candidate: {args.candidate}")
    cand_model = _load_model(args.candidate, device)
    cand_resid, membership = _position_residuals(cand_model, device, ds, xy)
    cand_cats = _category_means(cand_resid, membership)
    del cand_model

    if not os.path.exists(args.current):
        print(f"No current model at {args.current} -- nothing to compare against, promoting unconditionally.")
        passed, reasons = True, ['no current model to regress against']
        cur_cats = {}
    else:
        print(f"Evaluating current:   {args.current}")
        cur_model = _load_model(args.current, device)
        cur_resid, _ = _position_residuals(cur_model, device, ds, xy)
        cur_cats = _category_means(cur_resid, membership)
        del cur_model

        passed = True
        reasons = []
        tol = args.regression_tolerance

        overall_cur, overall_cand = float(cur_resid.mean()), float(cand_resid.mean())
        overall_regressed = overall_cand > overall_cur * (1 + tol)
        print(f"\n{'metric':<22}{'current (m)':>14}{'candidate (m)':>16}{'change':>10}   result")
        print('-' * 78)
        change = (overall_cand - overall_cur) / overall_cur if overall_cur else 0.0
        print(f"{'overall mean resid':<22}{overall_cur:14.4f}{overall_cand:16.4f}{change:+9.1%}   "
              f"{'FAIL' if overall_regressed else 'ok'}")
        if overall_regressed:
            passed = False
            reasons.append(f"overall mean residual regressed {change:+.1%} (tolerance {tol:.0%})")

        worst_cat = max(cur_cats, key=cur_cats.get) if cur_cats else None
        for cat in cur_cats:
            if cat not in cand_cats:
                continue
            c_cur, c_cand = cur_cats[cat], cand_cats[cat]
            c_change = (c_cand - c_cur) / c_cur if c_cur else 0.0
            flag = ' <- worst category' if cat == worst_cat else ''
            regressed = c_cand > c_cur * (1 + tol)
            print(f"{cat:<22}{c_cur:14.4f}{c_cand:16.4f}{c_change:+9.1%}   "
                  f"{'FAIL' if regressed else 'ok'}{flag}")
            if regressed and cat == worst_cat:
                passed = False
                reasons.append(f"worst category ({cat}) regressed {c_change:+.1%} (tolerance {tol:.0%})")

    print()
    if passed:
        print("PASS -- candidate is not worse than current on the gate criteria.")
    else:
        print("FAIL -- " + "; ".join(reasons))
        if not args.force:
            print("Refusing to promote. Re-run with --force to override (not recommended without "
                  "understanding why it regressed).")
            sys.exit(1)
        print("--force set: promoting anyway.")

    print(f"\nPromoting {args.candidate} -> {args.target}")
    os.makedirs(os.path.dirname(args.target), exist_ok=True)
    shutil.copyfile(args.candidate, args.target)

    if args.skip_recalibration:
        print("--skip-recalibration set: NOT regenerating conformal_report.json. "
              "The canonical calibration is now STALE relative to the promoted model. "
              "Re-run conformal_horizon_calibration.py before trusting any calibrated output.")
        return

    print("\nRegenerating canonical conformal calibration against the promoted model "
          "(conformal_horizon_calibration.py) -- required, not optional: a calibration fit on "
          "the OLD model's residuals is meaningless once the model has changed.")
    subprocess.run(
        [sys.executable, os.path.join(SCRIPT_DIR, 'conformal_horizon_calibration.py'),
         '--model', args.target],
        check=True, cwd=REPO_DIR,
    )
    print("\nDone. Canonical model and calibration are now in sync.")


if __name__ == '__main__':
    main()
