#!/usr/bin/env python3
"""
Calibration diagnostic for the trained ST-GAT model's Student-t heads, on
held-out CALIBRATION-split data (never seen in training).

Redesigned 2026-08-06 (Gaussian -> Student-t heads, see model.py's
docstring for the full reasoning): the old version computed z = (actual -
mean) / std and checked it against N(0,1) -- that assumes the predictive
family IS Gaussian, which is exactly the assumption the 2026-08-06
calibration findings (docs/research_notes/trust_and_signal_behavior_2026-08-06.md)
showed was false (leptokurtic residuals, Anderson-Darling decisively
rejects normality). Now that heads predict a per-sample dof as well as
mean/scale, a plain z-score has no fixed reference distribution to check
against (dof varies per sample) -- the correct generalization is the
PROBABILITY INTEGRAL TRANSFORM (PIT): u = F(actual; predicted params),
where F is the model's own predicted CDF for that exact sample (Student-t
with THAT sample's mean/scale/dof). If the model is correctly calibrated,
u ~ Uniform(0,1) regardless of the underlying family or how much dof
varies sample-to-sample -- this is standard practice in probabilistic
forecasting (Gneiting et al., "Probabilistic Forecasts, Calibration and
Sharpness," JRSS-B 2007) and is what plot_calibration_diagrams.py's
coverage curves are also built on now.

Reports, for each Student-t-headed feature (position, velocity, steering,
acceleration, traffic_light_color, traffic_light_confidence) at 1-step-ahead
(t=0, same convention st_gat/residuals.py uses):
  mean(u)         -- want ~0.5
  std(u)          -- want ~0.289 (1/sqrt(12), the Uniform(0,1) std)
  frac in [.16,.84] -- want ~0.68 (central-68% coverage check)
  frac in [.025,.975] -- want ~0.95 (central-95% coverage check)
  KS stat / p      -- Kolmogorov-Smirnov test against Uniform(0,1); a small
                      p-value (<0.05) rejects calibration

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
from scipy.stats import t as student_t, kstest
from torch.utils.data import DataLoader

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
sys.path.insert(0, REPO_DIR)

from st_gat.pipeline import config as cfg  # noqa: E402
from st_gat.model import STGAT, TrajectoryDataset  # noqa: E402

# (key, dims) — same Student-t-headed features st_gat/residuals.py tracks.
_STUDENT_T_FEATURES = {
    'position':                  2,
    'velocity':                  2,
    'steering':                  1,
    'acceleration':               1,
    'traffic_light_color':       1,
    'traffic_light_confidence':  1,
}


def _pit(actual: np.ndarray, mean: np.ndarray, scale: np.ndarray, dof: np.ndarray) -> np.ndarray:
    """Probability integral transform: F_t(actual; mean, scale, dof) per
    sample, using each sample's own predicted parameters (dof varies
    sample-to-sample, unlike a fixed-family z-score check)."""
    z = (actual - mean) / scale
    return student_t.cdf(z, df=dof)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--model', default=cfg.MODEL_CONFIG['model_path'])
    ap.add_argument('--batch', type=int, default=256)
    ap.add_argument('--horizon-breakdown', action='store_true',
                    help='also report mean(u)/std(u) at t=0 (1-step) vs t=T_out-1 (full horizon) per feature')
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

    u_1step = {k: [] for k in _STUDENT_T_FEATURES}
    u_full  = {k: [] for k in _STUDENT_T_FEATURES}

    with torch.no_grad():
        for past, future, graph, _bounds in loader:
            past   = {k: v.to(device) for k, v in past.items()}
            future = {k: v.to(device) for k, v in future.items()}
            graph  = {k: v.to(device) for k, v in graph.items()}
            preds  = model(past, graph)

            for key, dims in _STUDENT_T_FEATURES.items():
                mean  = preds[f'{key}_mean']    # (B, T_out) or (B, T_out, dims)
                scale = preds[f'{key}_scale']
                dof   = preds[f'{key}_dof']      # (B, T_out) always -- shared across dims
                actual = future[key]
                if dims == 1 and actual.dim() == 3 and actual.size(-1) == 1:
                    actual = actual.squeeze(-1)

                dof_b = dof.unsqueeze(-1) if mean.dim() == 3 else dof
                u = _pit(actual.cpu().numpy(), mean.cpu().numpy(), scale.cpu().numpy(),
                         dof_b.expand_as(mean).cpu().numpy())
                u_1step[key].append(u[:, 0].flatten())
                u_full[key].append(u[:, -1].flatten())

    print(f"\n{'feature':<24} {'mean(u)':>9} {'std(u)':>9} {'cov68%':>9} {'cov95%':>9} {'KS stat':>9} {'KS p':>10}   (1-step-ahead, t=0)")
    print("-" * 92)
    for key in _STUDENT_T_FEATURES:
        u = np.concatenate(u_1step[key])
        u = u[np.isfinite(u)]
        ks = kstest(u, 'uniform')
        cov68 = np.mean((u >= 0.16) & (u <= 0.84))
        cov95 = np.mean((u >= 0.025) & (u <= 0.975))
        print(f"{key:<24} {u.mean():9.3f} {u.std():9.3f} {cov68:9.3f} {cov95:9.3f} "
              f"{ks.statistic:9.3f} {ks.pvalue:10.2e}")

    if args.horizon_breakdown:
        print(f"\n{'feature':<24} {'std(u) @t=0':>14} {'std(u) @t=T_out-1':>20}")
        print("-" * 60)
        for key in _STUDENT_T_FEATURES:
            u0 = np.concatenate(u_1step[key]); u0 = u0[np.isfinite(u0)]
            uf = np.concatenate(u_full[key]);  uf = uf[np.isfinite(uf)]
            print(f"{key:<24} {u0.std():14.3f} {uf.std():20.3f}")


if __name__ == '__main__':
    main()
