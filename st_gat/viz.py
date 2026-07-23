"""
Visualization entry point for STGAT-RISE.

Generates a standard set of figures from a processed pkl sequence file and
trained model checkpoint.  Works offline — does NOT need ROS sourced.

Usage
-----
    # Basic: dashboard + fan for 9 random sequences
    python3 -m st_gat.viz \\
        --pkl  st_gat/data/sequences/calibration/goal_007_t1_...pkl \\
        --model st_gat/checkpoints/best_model.pth \\
        --out   st_gat/results/figures/goal_007_cal

    # From residual CSV (no pkl needed for the residual timeline plot)
    python3 -m st_gat.viz \\
        --csv  st_gat/results/residuals/obs_recovery/goal_007_obs_recovery_t1_...csv \\
        --out  st_gat/results/figures/obs_recovery_t1 \\
        --nll-threshold -6.37

    # Full report: all plots from a pkl + model
    python3 -m st_gat.viz \\
        --pkl  st_gat/data/sequences/calibration/goal_007_t1_...pkl \\
        --model st_gat/checkpoints/best_model.pth \\
        --out   st_gat/results/figures/report \\
        --seq   100 200 350   \\  # specific sequence indices for dashboard/fan
        --all                     # also write error_horizon + calibration

Outputs (saved to --out/)
---------
    dashboard_seq<N>.pdf     — 6-panel feature prediction dashboard
    fan_seq<N>.pdf           — trajectory fan
    grid.pdf                 — N-thumbnail prediction grid
    error_horizon.pdf        — RMSE/NLL vs prediction step
    calibration_position.pdf — calibration for position
    residual_timeline.pdf    — from --csv (timeline with NLL, L2, object proximity)
"""

import argparse
import os
import pickle
import sys

import torch


def _load_model(model_path: str, device: torch.device):
    from st_gat.model import STGAT
    from st_gat.pipeline import config as cfg

    model_cfg = cfg.MODEL_CONFIG.copy()
    model_cfg.update({
        'd_model':     128,
        'd_graph':     128,
        'hidden_size': 128,
        'num_layers':  2,
        'nhead':       4,
        'dropout_rate': 0.15,
        'device':      device,
    })
    model = STGAT(model_cfg).to(device)
    model.load_state_dict(
        torch.load(model_path, map_location=device, weights_only=True)
    )
    model.eval()
    print(f'[viz] model loaded: {model.count_parameters():,} params from {model_path}')
    return model


def _load_pkl(pkl_path: str):
    with open(pkl_path, 'rb') as f:
        sequences = pickle.load(f)
    print(f'[viz] loaded {len(sequences)} sequences from {pkl_path}')
    return sequences


def main():
    parser = argparse.ArgumentParser(
        description='STGAT-RISE visualization toolkit',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('--pkl',    type=str, help='Path to a .pkl sequence file')
    parser.add_argument('--model',  type=str, help='Path to model checkpoint (.pth)')
    parser.add_argument('--csv',    type=str, help='Path to residual CSV from infer.py')
    parser.add_argument('--out',    type=str, default='st_gat/results/figures',
                        help='Output directory for figures')
    parser.add_argument('--seq',    type=int, nargs='+',
                        help='Specific sequence indices to plot dashboard+fan for')
    parser.add_argument('--n-grid', type=int, default=9,
                        help='Number of sequences in the prediction grid (default 9)')
    parser.add_argument('--all',    action='store_true',
                        help='Also generate error_horizon and calibration plots')
    parser.add_argument('--nll-threshold', type=float, default=None,
                        help='Horizontal NLL threshold line on residual timeline')
    parser.add_argument('--fault-windows', type=str, default=None,
                        help='Comma-separated pairs start:end for fault shading, '
                             'e.g. "100:250,600:800"')
    parser.add_argument('--batch', type=int, default=256,
                        help='Batch size for model inference (default 256)')
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)

    # ── Residual CSV path ─────────────────────────────────────────────────
    if args.csv:
        import pandas as pd
        from st_gat.visualize import PredictionVisualizer

        df = pd.read_csv(args.csv, index_col=0)
        print(f'[viz] residual CSV: {len(df)} windows')

        fault_windows = None
        if args.fault_windows:
            fault_windows = [
                tuple(int(x) for x in pair.split(':'))
                for pair in args.fault_windows.split(',')
            ]

        PredictionVisualizer.plot_residual_timeline(
            df,
            title=os.path.basename(args.csv).replace('.csv', ''),
            fault_windows=fault_windows,
            nll_threshold=args.nll_threshold,
            out=os.path.join(args.out, 'residual_timeline.pdf'),
        )

    # ── pkl + model path ──────────────────────────────────────────────────
    if args.pkl:
        if not args.model:
            print('ERROR: --model is required when --pkl is given')
            sys.exit(1)

        sequences = _load_pkl(args.pkl)

        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        print(f'[viz] device: {device}')
        model = _load_model(args.model, device)

        from st_gat.visualize import PredictionVisualizer
        print('[viz] running inference...')
        viz = PredictionVisualizer.from_pkl_and_model(
            sequences, model, batch_size=args.batch, device=device
        )

        # Prediction grid (always)
        viz.plot_prediction_grid(
            n=args.n_grid,
            out=os.path.join(args.out, 'grid.pdf'),
        )

        # Per-sequence dashboard + fan
        seq_indices = args.seq
        if seq_indices is None:
            # Default: one early, one mid, one late
            n = len(sequences)
            seq_indices = [n // 8, n // 2, 7 * n // 8]

        for idx in seq_indices:
            if idx >= len(sequences):
                print(f'[viz] WARNING: seq {idx} out of range ({len(sequences)} seqs), skipping')
                continue
            viz.plot_prediction_dashboard(
                seq_idx=idx,
                out=os.path.join(args.out, f'dashboard_seq{idx}.pdf'),
            )
            viz.plot_trajectory_fan(
                seq_idx=idx,
                out=os.path.join(args.out, f'fan_seq{idx}.pdf'),
            )

        # Run-level fan (all sequences, subsampled)
        viz.plot_run_fan(
            out=os.path.join(args.out, 'run_fan.pdf'),
        )

        if args.all:
            viz.plot_error_horizon(
                out=os.path.join(args.out, 'error_horizon.pdf'),
            )
            for feat in ('position', 'velocity', 'steering', 'acceleration'):
                viz.plot_calibration(
                    feature=feat,
                    out=os.path.join(args.out, f'calibration_{feat}.pdf'),
                )

    if not args.pkl and not args.csv:
        parser.print_help()
        sys.exit(1)

    print(f'\n[viz] all figures saved to {args.out}/')


if __name__ == '__main__':
    main()
