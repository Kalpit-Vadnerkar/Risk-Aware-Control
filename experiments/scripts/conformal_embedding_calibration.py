#!/usr/bin/env python3
"""
Embedding-conditioned conformal quantiles for SPECIFIC windows (2026-08-25)
-- turns conformal_scene_conditioning.py's validation study (which only
ever checked a random SAMPLE of test windows, never saved a reusable
calibration) into an actual artifact: for a given set of query windows
(default: the 5 canonical trust-example windows), compute each one's
k-NN-in-h_last-space conformal quantile for ALL 7 series, so
plot_layer1_trust_examples.py can render trust plots using the embedding
approach's bands, the same way it already does for vanilla and Mondrian.

Method, unchanged from conformal_scene_conditioning.py (see that file's
docstring for the full derivation): take the model's own h_last for a
query window, find its k nearest neighbors by Euclidean distance among
OTHER TRIALS' windows only (no leakage -- a window's own trial is excluded
from its neighbor pool even if the query window itself isn't in the
calibration set being searched), fit the conformal quantile per horizon
step on just that neighborhood's residuals.

Usage (repo venv sourced, no ROS needed):
  source .venv/bin/activate
  python3 experiments/scripts/conformal_embedding_calibration.py \
      --model st_gat/models/h30_30/st_gat_rise.pth
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

from st_gat.pipeline import config as cfg  # noqa: E402
from st_gat.model import STGAT, TrajectoryDataset  # noqa: E402
from conformal_horizon_calibration import (  # noqa: E402
    conformal_quantile, _trial_ids_for, _FEATURES, _SERIES, _extract_series,
)

DEFAULT_OUTPUT_DIR = os.path.join(REPO_DIR, 'experiments', 'analysis', 'conformal_embedding_calibration')
CANONICAL_WINDOWS = os.path.join(REPO_DIR, 'experiments', 'analysis', 'trust_example_windows.json')


def _load_model(model_path):
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model_cfg = cfg.build_inference_model_cfg(device)
    model = STGAT(model_cfg).to(device)
    model.load_state_dict(torch.load(model_path, map_location=device, weights_only=True))
    model.eval()
    print(f"Loaded model: {model_path} ({model.count_parameters():,} params)")
    return model, device


def _collect(model, device, loader):
    errs = {k: [] for k in _FEATURES}
    h_all = []
    with torch.no_grad():
        for past, future, graph, _bounds in loader:
            past   = {k: v.to(device) for k, v in past.items()}
            future = {k: v.to(device) for k, v in future.items()}
            graph  = {k: v.to(device) for k, v in graph.items()}
            h_last = model.encode_scene(past, graph)
            preds  = model(past, graph)
            h_all.append(h_last.cpu().numpy())
            for key, dims in _FEATURES.items():
                mean   = preds[f'{key}_mean']
                actual = future[key]
                if dims == 1 and actual.dim() == 3 and actual.size(-1) == 1:
                    actual = actual.squeeze(-1)
                errs[key].append((actual - mean).cpu().numpy())
    return ({k: np.concatenate(v, axis=0) for k, v in errs.items()},
            np.concatenate(h_all, axis=0))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--model', default=cfg.MODEL_CONFIG['model_path'])
    ap.add_argument('--batch', type=int, default=256)
    ap.add_argument('--alpha', type=float, default=0.1)
    ap.add_argument('--k-neighbors', type=int, default=150)
    ap.add_argument('--windows', type=int, nargs='+', default=None,
                     help='window indices to calibrate; default is the 5 canonical trust-example windows')
    ap.add_argument('--output-dir', default=DEFAULT_OUTPUT_DIR)
    args = ap.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    if args.windows is not None:
        query_idxs = args.windows
    else:
        with open(CANONICAL_WINDOWS) as f:
            canonical = json.load(f)
        query_idxs = list(canonical.values())
        print(f"Using canonical windows from {CANONICAL_WINDOWS}: {canonical}")

    model, device = _load_model(args.model)
    ds = TrajectoryDataset(cfg.CAL_DIR)
    n = len(ds)
    trial_id, pkl_files = _trial_ids_for(cfg.CAL_DIR)
    T_out = cfg.OUTPUT_SEQ_LEN
    series_keys = [s[0] for s in _SERIES]
    print(f"Calibration set: {len(pkl_files)} trials, {n} windows")

    loader = DataLoader(ds, batch_size=args.batch, shuffle=False, num_workers=0)
    print("Collecting residuals + scene embeddings (one pass)...")
    errs, h_all = _collect(model, device, loader)
    all_resid = _extract_series(errs)

    windows_out = {}
    for idx in query_idxs:
        own_trial = trial_id[idx]
        fit_mask = trial_id != own_trial   # exclude the query window's own trial -- no leakage
        fit_h = h_all[fit_mask]
        dists = np.linalg.norm(fit_h - h_all[idx][None, :], axis=1)
        nn = np.argsort(dists)[:args.k_neighbors]

        windows_out[str(idx)] = {}
        for key in series_keys:
            resid_fit = all_resid[key][fit_mask]
            neigh_resid = resid_fit[nn]   # (k, T_out)
            q = np.array([conformal_quantile(neigh_resid[:, t], args.alpha) for t in range(T_out)])
            windows_out[str(idx)][key] = q.tolist()
        print(f"  window {idx} (trial {pkl_files[own_trial]}): calibrated from {args.k_neighbors} "
              f"nearest neighbors among the other {len(pkl_files) - 1} trials")

    out_path = os.path.join(args.output_dir, 'conformal_embedding_report.json')
    with open(out_path, 'w') as f:
        json.dump({'method': 'embedding_knn', 'k': args.k_neighbors, 'alpha': args.alpha,
                   'windows': windows_out}, f, indent=2)
    print(f"\nSaved {out_path}")


if __name__ == '__main__':
    main()
