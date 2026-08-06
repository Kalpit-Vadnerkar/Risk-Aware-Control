#!/usr/bin/env python3
"""
z-score calibration diagnostic for the trained ST-GAT model's Gaussian heads,
on held-out CALIBRATION-split data (never seen in training).

For each Gaussian-headed feature (position, velocity, steering, acceleration,
traffic_light_color, traffic_light_confidence), computes z = (actual - predicted_mean) / predicted_std
at 1-step-ahead (t=0 of the future window — same convention st_gat/residuals.py
uses), flattened across all dims/examples, and reports:
  std(z)      — want ~1.0. >1 = predicted band too NARROW (overconfident);
                <1 = too WIDE (underconfident).
  frac|z|<1   — want ~0.68 for a well-calibrated Gaussian
  frac|z|<2   — want ~0.95

This existed only as an ad hoc, one-off computation earlier in the project's
history (see docs/research_notes/model_improvement_notes_2026.md for the
numbers it produced) — written here as a reusable script so it doesn't have
to be re-derived by hand every time the model changes (e.g. after the
2026-08-05 frame-gap fix + retrain).

Usage (ROS/Autoware + repo venv sourced — needed for STGAT/TrajectoryDataset
imports, though this script itself does no bag/ROS I/O):
  source /opt/ros/humble/setup.bash
  source /home/kvadner/Desktop/Dissertation/autoware/install/setup.bash
  source .venv/bin/activate
  python3 experiments/scripts/check_calibration.py
"""

import argparse
import os
import sys

import numpy as np
import torch
from torch.utils.data import DataLoader

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
sys.path.insert(0, REPO_DIR)

from st_gat.pipeline import config as cfg  # noqa: E402
from st_gat.model import STGAT, TrajectoryDataset  # noqa: E402

# (key, dims) — same Gaussian-headed features st_gat/residuals.py tracks.
_GAUSSIAN_FEATURES = {
    'position':                  2,
    'velocity':                  2,
    'steering':                  1,
    'acceleration':               1,
    'traffic_light_color':       1,
    'traffic_light_confidence':  1,
}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--model', default=cfg.MODEL_CONFIG['model_path'])
    ap.add_argument('--batch', type=int, default=256)
    ap.add_argument('--horizon-breakdown', action='store_true',
                    help='also report std(z) at t=0 (1-step) vs t=T_out-1 (full horizon) per feature')
    args = ap.parse_args()

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model_cfg = cfg.MODEL_CONFIG.copy()
    model_cfg.update({'d_model': 128, 'd_graph': 128, 'hidden_size': 128,
                      'num_layers': 2, 'nhead': 4, 'dropout_rate': 0.15, 'device': device})
    model = STGAT(model_cfg).to(device)
    model.load_state_dict(torch.load(args.model, map_location=device, weights_only=True))
    model.eval()
    print(f"Loaded model: {args.model} ({model.count_parameters():,} params)")

    ds = TrajectoryDataset(cfg.CAL_DIR)
    loader = DataLoader(ds, batch_size=args.batch, shuffle=False, num_workers=0, pin_memory=True)

    z_1step = {k: [] for k in _GAUSSIAN_FEATURES}
    z_full  = {k: [] for k in _GAUSSIAN_FEATURES}

    with torch.no_grad():
        for past, future, graph, _bounds in loader:
            past   = {k: v.to(device) for k, v in past.items()}
            future = {k: v.to(device) for k, v in future.items()}
            graph  = {k: v.to(device) for k, v in graph.items()}
            preds  = model(past, graph)

            for key, dims in _GAUSSIAN_FEATURES.items():
                mean = preds[f'{key}_mean']   # (B, T_out) or (B, T_out, dims)
                var  = preds[f'{key}_var']
                actual = future[key]
                if dims == 1 and actual.dim() == 3 and actual.size(-1) == 1:
                    actual = actual.squeeze(-1)

                z = (actual - mean) / torch.sqrt(var)
                z_1step[key].append(z[:, 0].flatten().cpu().numpy())
                z_full[key].append(z[:, -1].flatten().cpu().numpy())

    print(f"\n{'feature':<22} {'std(z)':>8} {'frac|z|<1':>10} {'frac|z|<2':>10}   (1-step-ahead, t=0)")
    print("-" * 60)
    for key in _GAUSSIAN_FEATURES:
        z = np.concatenate(z_1step[key])
        z = z[np.isfinite(z)]
        print(f"{key:<22} {z.std():8.3f} {np.mean(np.abs(z) < 1):10.3f} {np.mean(np.abs(z) < 2):10.3f}")

    if args.horizon_breakdown:
        print(f"\n{'feature':<22} {'std(z) @t=0':>12} {'std(z) @t=T_out-1':>18}")
        print("-" * 55)
        for key in _GAUSSIAN_FEATURES:
            z0 = np.concatenate(z_1step[key]); z0 = z0[np.isfinite(z0)]
            zf = np.concatenate(z_full[key]);  zf = zf[np.isfinite(zf)]
            print(f"{key:<22} {z0.std():12.3f} {zf.std():18.3f}")


if __name__ == '__main__':
    main()
