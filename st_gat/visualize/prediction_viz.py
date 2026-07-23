"""
PredictionVisualizer — publication-quality plots for STGAT-RISE predictions.

Improvements over the T-ITS reference visualizer:

  * Physical units throughout (m, m/s, rad, m/s²) — no more [0,10]-scaled axes
  * Prediction dashboard: one compact figure per sequence, all features on a
    shared time axis, predicted-mean ± 1σ shaded bands vs actual future
  * Trajectory fan: prediction corridors from multiple anchor windows,
    opacity fading with prediction horizon, overlaid on the HD-map graph
  * Error vs horizon: how RMSE and NLL grow as prediction step increases;
    aggregated over a dataset for proper statistics
  * Calibration plot: empirical coverage vs nominal, per feature
  * Run residual timeline: NLL decomposition + CUSUM with optional fault-window
    shading (designed to match the dissertation's fault-analysis workflow)
  * All functions accept an optional ax/fig, so they compose into larger
    multi-panel figures cleanly
  * All output paths are arguments, not hardcoded strings

Usage
-----
    from st_gat.visualize import PredictionVisualizer
    import pickle, torch
    from st_gat.model import STGAT
    from st_gat.pipeline import config as cfg

    # Load data
    with open('st_gat/data/sequences/calibration/goal_007_t1_...pkl', 'rb') as f:
        sequences = pickle.load(f)

    # Load model and run predictions
    model = STGAT(cfg.MODEL_CONFIG)
    model.load_state_dict(torch.load('st_gat/checkpoints/best_model.pth'))

    viz = PredictionVisualizer.from_pkl_and_model(sequences, model)

    # Publication-ready figures
    viz.plot_prediction_dashboard(seq_idx=100, out='figs/dashboard.pdf')
    viz.plot_trajectory_fan(seq_idx=100, out='figs/fan.pdf')
    viz.plot_error_horizon(out='figs/error_horizon.pdf')
    viz.plot_calibration(out='figs/calibration.pdf')
"""

from __future__ import annotations

import math
import os
from typing import List, Optional, Sequence, Tuple

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.collections import LineCollection
import networkx as nx
import numpy as np
import torch
from torch.utils.data import DataLoader

from . import rescale as R

# ── Style ──────────────────────────────────────────────────────────────────

_STYLE = {
    'font.family':     'DejaVu Sans',
    'font.size':        12,
    'axes.titlesize':   13,
    'axes.labelsize':   12,
    'legend.fontsize':  10,
    'figure.dpi':       150,
    'axes.spines.top':  False,
    'axes.spines.right':False,
}

# Colorblind-safe palette (Okabe-Ito)
C_PAST    = '#999999'   # gray    — past/actual observed
C_PRED    = '#0072B2'   # blue    — predicted mean
C_ACTUAL  = '#000000'   # black   — actual future (ground truth)
C_BAND    = '#56B4E9'   # sky     — ±1σ band fill
C_GRAPH   = '#CCCCCC'   # light   — map graph edges
C_TL      = '#E69F00'   # orange  — traffic-light nodes
C_PATH    = '#009E73'   # green   — route nodes
C_ERROR   = '#D55E00'   # vermillion — error / fault signals

DT = 0.1   # seconds per timestep


# ── Internal tensor helpers ────────────────────────────────────────────────

def _to_np(t):
    if isinstance(t, torch.Tensor):
        return t.detach().cpu().numpy()
    return np.asarray(t)


def _seq_feature_arrays(seq: dict, key: str) -> np.ndarray:
    """Extract (T, dim) array from a sequence's past or future list."""
    return np.array([step[key] for step in seq])


def _unpack_past_future(seq: dict, bounds):
    """
    Returns everything needed for one sequence in physical units:
        past_pos, future_pos, past_vel, future_vel,
        past_steer, future_steer, past_accel, future_accel,
        past_unc
    All as (T, dim) or (T,) numpy arrays in physical units.
    """
    def _arr(split, key): return _seq_feature_arrays(seq, key) if False else \
        np.array([step[key] for step in seq[split]])

    pp  = R.pos_to_m(_arr('past', 'position'),   bounds)
    fp  = R.pos_to_m(_arr('future', 'position'),  bounds)
    pv  = R.vel_to_ms(_arr('past', 'velocity'))
    fv  = R.vel_to_ms(_arr('future', 'velocity'))
    ps  = R.steer_to_rad(np.array([s['steering'] for s in seq['past']]))
    fs  = R.steer_to_rad(np.array([s['steering'] for s in seq['future']]))
    pa  = R.accel_to_ms2(np.array([s['acceleration'] for s in seq['past']]))
    fa  = R.accel_to_ms2(np.array([s['acceleration'] for s in seq['future']]))
    pu  = R.uncertainty_to_m2(np.array([s.get('uncertainty', [0., 0.]) for s in seq['past']]))
    return pp, fp, pv, fv, ps, fs, pa, fa, pu


def _unpack_pred(pred: dict, bounds):
    """
    Convert model output dict from [0,1] to physical units.
    All means/variances as (T_out, dim) or (T_out,) arrays.
    """
    pos_mean  = R.pos_to_m(_to_np(pred['position_mean']),    bounds)
    pos_std   = np.sqrt(R.pos_var_to_m2(_to_np(pred['position_var']), bounds))
    vel_mean  = R.vel_to_ms(_to_np(pred['velocity_mean']))
    vel_std   = np.sqrt(R.vel_var_to_ms2(_to_np(pred['velocity_var'])))
    s_mean    = R.steer_to_rad(_to_np(pred['steering_mean']))
    s_std     = np.sqrt(R.steer_var_to_rad2(_to_np(pred['steering_var'])))
    a_mean    = R.accel_to_ms2(_to_np(pred['acceleration_mean']))
    a_std     = np.sqrt(R.accel_var_to_ms4(_to_np(pred['acceleration_var'])))
    return pos_mean, pos_std, vel_mean, vel_std, s_mean, s_std, a_mean, a_std


# ── Main class ─────────────────────────────────────────────────────────────

class PredictionVisualizer:
    """
    Holds a list of sequences and their corresponding model predictions,
    and provides methods for several plot types.

    Parameters
    ----------
    sequences  : list of sequence dicts (from run_pipeline / TrajectoryDataset)
    predictions: list of prediction dicts (model output, one per sequence)
                 Each dict has keys: position_mean, position_var, velocity_mean,
                 velocity_var, steering_mean, steering_var, acceleration_mean,
                 acceleration_var, object_distance, traffic_light_detected.
                 All values are numpy arrays (not tensors).
    """

    def __init__(
        self,
        sequences:   List[dict],
        predictions: List[dict],
    ):
        assert len(sequences) == len(predictions), \
            "sequences and predictions must be the same length"
        self.sequences   = sequences
        self.predictions = predictions

    # ── Factory ─────────────────────────────────────────────────────────────

    @classmethod
    def from_pkl_and_model(
        cls,
        sequences: List[dict],
        model,
        batch_size: int = 256,
        device: Optional[torch.device] = None,
    ) -> 'PredictionVisualizer':
        """
        Run `model` over all sequences and return a ready-to-use visualizer.
        `model` should already be on the desired device and in eval mode.
        """
        from st_gat.model.dataset import TrajectoryDataset

        if device is None:
            device = next(model.parameters()).device
        model.eval()

        # Wrap sequences in a temporary Dataset
        class _TmpDataset(torch.utils.data.Dataset):
            def __init__(self, seqs):
                self.seqs = seqs
                self._ds  = TrajectoryDataset.__new__(TrajectoryDataset)
                self._ds.sequences = seqs
            def __len__(self): return len(self.seqs)
            def __getitem__(self, i): return self._ds[i]

        ds     = _TmpDataset(sequences)
        loader = DataLoader(ds, batch_size=batch_size, shuffle=False, num_workers=0)

        all_preds = []
        with torch.no_grad():
            for past, _future, graph, _bounds in loader:
                past  = {k: v.to(device) for k, v in past.items()}
                graph = {k: v.to(device) for k, v in graph.items()}
                out   = model(past, graph)
                B = next(iter(out.values())).shape[0]
                for b in range(B):
                    all_preds.append({k: v[b].cpu().numpy() for k, v in out.items()})

        return cls(sequences, all_preds)

    # ── 1. Prediction Dashboard ─────────────────────────────────────────────

    def plot_prediction_dashboard(
        self,
        seq_idx: int,
        out: Optional[str] = None,
        fig: Optional[plt.Figure] = None,
    ) -> plt.Figure:
        """
        Six-panel figure for one sequence: pos_x, pos_y, vel_lon, vel_lat,
        steering, acceleration.  Each panel shows past (gray), predicted
        future mean ± 1σ (blue/band), and actual future (black dashed).

        All axes share the same time axis: past = [−3s, 0), future = [0, 3s].
        """
        with plt.rc_context(_STYLE):
            if fig is None:
                fig, axes = plt.subplots(3, 2, figsize=(12, 9), sharex=True)
            else:
                axes = np.array(fig.get_axes()).reshape(3, 2)

            seq    = self.sequences[seq_idx]
            pred   = self.predictions[seq_idx]
            bounds = seq['graph_bounds']

            T_in  = len(seq['past'])
            T_out = len(seq['future'])
            t_past   = np.arange(-T_in,   0)    * DT
            t_future = np.arange(0,        T_out) * DT

            pp, fp, pv, fv, ps, fs, pa, fa, pu = _unpack_past_future(seq, bounds)
            pm, ps2, vm, vs2, sm, ss2, am, as2 = _unpack_pred(pred, bounds)

            panels = [
                ('Position X',       'm',     t_past, pp[:, 0], t_future, fp[:, 0], pm[:, 0], ps2[:, 0]),
                ('Position Y',       'm',     t_past, pp[:, 1], t_future, fp[:, 1], pm[:, 1], ps2[:, 1]),
                ('Velocity long.',   'm/s',   t_past, pv[:, 0], t_future, fv[:, 0], vm[:, 0], vs2[:, 0]),
                ('Velocity lat.',    'm/s',   t_past, pv[:, 1], t_future, fv[:, 1], vm[:, 1], vs2[:, 1]),
                ('Steering',         'rad',   t_past, ps,        t_future, fs,        sm,        ss2),
                ('Acceleration',     'm/s²',  t_past, pa,        t_future, fa,        am,        as2),
            ]

            for ax, (title, ylabel, tp, past_v, tf, fut_v, mean_v, std_v) in \
                    zip(axes.flat, panels):
                # Past — observed
                ax.plot(tp, past_v, color=C_PAST, lw=1.4, label='Past (observed)')

                # Future — predicted
                ax.plot(tf, mean_v, color=C_PRED, lw=1.8, label='Predicted mean')
                ax.fill_between(tf,
                                mean_v - std_v,
                                mean_v + std_v,
                                color=C_BAND, alpha=0.35, label='±1σ')
                ax.fill_between(tf,
                                mean_v - 2 * std_v,
                                mean_v + 2 * std_v,
                                color=C_BAND, alpha=0.15, label='±2σ')

                # Future — actual
                ax.plot(tf, fut_v, color=C_ACTUAL, lw=1.2, ls='--', label='Actual future')

                # Divider at t=0
                ax.axvline(0, color='gray', lw=0.8, ls=':')

                ax.set_title(title)
                ax.set_ylabel(ylabel)
                ax.grid(True, alpha=0.25, lw=0.6)

            for ax in axes[-1]:
                ax.set_xlabel('Time relative to prediction origin (s)')

            # Single legend at the top right
            handles, labels = axes.flat[0].get_legend_handles_labels()
            fig.legend(handles, labels, loc='upper center',
                       ncol=4, fontsize=10, bbox_to_anchor=(0.5, 1.01))
            fig.suptitle(f'Prediction Dashboard — sequence {seq_idx}', y=1.05)
            fig.tight_layout()

            _save(fig, out)
        return fig

    # ── 2. Trajectory Fan ───────────────────────────────────────────────────

    def plot_trajectory_fan(
        self,
        seq_idx:   int,
        n_anchors: int = 6,
        out:       Optional[str] = None,
        ax:        Optional[plt.Axes] = None,
    ) -> plt.Figure:
        """
        Map-based plot for one sequence:
          - HD map context from the networkx graph (road nodes, TL nodes, route)
          - Past trajectory as a single colored line
          - Prediction fan: position prediction corridors (±1σ, ±2σ ellipses)
            at n_anchors evenly-spaced points along the past, fading toward t+3s
          - Actual future trajectory as a dashed line

        Works entirely in the sequence's own [x_min..x_max, y_min..y_max] frame.
        """
        with plt.rc_context(_STYLE):
            if ax is None:
                fig, ax = plt.subplots(figsize=(9, 9))
            else:
                fig = ax.figure

            seq    = self.sequences[seq_idx]
            pred   = self.predictions[seq_idx]
            bounds = seq['graph_bounds']

            pp, fp = (R.pos_to_m(np.array([s['position'] for s in seq['past']]),  bounds),
                      R.pos_to_m(np.array([s['position'] for s in seq['future']]), bounds))
            pm, ps2 = (R.pos_to_m(_to_np(pred['position_mean']), bounds),
                       R.pos_var_to_m2(_to_np(pred['position_var']), bounds))

            # ── HD map graph ──────────────────────────────────────────────
            G = seq['graph']
            _draw_graph(ax, G, bounds)

            # ── Past trajectory ───────────────────────────────────────────
            ax.plot(pp[:, 0], pp[:, 1],
                    color=C_PAST, lw=2.0, zorder=3, label='Past trajectory')
            ax.scatter(pp[0, 0], pp[0, 1],
                       color=C_PAST, s=50, marker='o', zorder=4)

            # ── Prediction fan from this sequence ─────────────────────────
            # The prediction is from the end of the past window → fp positions.
            # We plot it as a corridor: the mean path with ±1σ / ±2σ tubes.
            _draw_prediction_corridor(ax, pm, np.sqrt(ps2), alpha_base=0.85)

            # ── Actual future ─────────────────────────────────────────────
            ax.plot(fp[:, 0], fp[:, 1],
                    color=C_ACTUAL, lw=1.6, ls='--', zorder=4, label='Actual future')
            ax.scatter(fp[-1, 0], fp[-1, 1],
                       color=C_ACTUAL, s=80, marker='*', zorder=5, label='Future end')

            ax.set_aspect('equal')
            ax.set_xlabel('x  (m)')
            ax.set_ylabel('y  (m)')
            ax.set_title(f'Trajectory + Prediction Fan — sequence {seq_idx}')
            ax.legend(loc='best', fontsize=9)
            ax.grid(True, alpha=0.2)
            fig.tight_layout()

            _save(fig, out)
        return fig

    # ── 3. Multi-anchor fan across the full run ─────────────────────────────

    def plot_run_fan(
        self,
        anchor_stride: int = 30,
        max_anchors:   int = 12,
        out:           Optional[str] = None,
        ax:            Optional[plt.Axes] = None,
    ) -> plt.Figure:
        """
        Trajectory fan across all sequences:
        Every `anchor_stride`-th sequence provides a prediction fan.
        Good for visualising how the model tracks and anticipates the full run.

        All sequences must share the same graph_bounds (typical for a single run).
        """
        with plt.rc_context(_STYLE):
            if ax is None:
                fig, ax = plt.subplots(figsize=(12, 10))
            else:
                fig = ax.figure

            bounds = self.sequences[0]['graph_bounds']
            G      = self.sequences[0]['graph']
            _draw_graph(ax, G, bounds)

            n = len(self.sequences)
            anchor_idxs = range(0, n, anchor_stride)[:max_anchors]

            # Full observed trajectory (past of each sequence: dense coverage)
            all_past = np.concatenate([
                R.pos_to_m(np.array([s['position'] for s in seq['past']]), bounds)
                for seq in self.sequences[::max(1, n // 300)]  # subsample for speed
            ], axis=0)
            ax.plot(all_past[:, 0], all_past[:, 1],
                    color=C_PAST, lw=1.2, alpha=0.5, zorder=2, label='Observed trajectory')

            cmap = plt.get_cmap('plasma')
            for k, i in enumerate(anchor_idxs):
                pred   = self.predictions[i]
                pm     = R.pos_to_m(_to_np(pred['position_mean']), bounds)
                ps2    = R.pos_var_to_m2(_to_np(pred['position_var']), bounds)
                color  = cmap(k / max(1, len(anchor_idxs) - 1))
                _draw_prediction_corridor(ax, pm, np.sqrt(ps2),
                                          alpha_base=0.7, mean_color=color)

            ax.set_aspect('equal')
            ax.set_xlabel('x  (m)')
            ax.set_ylabel('y  (m)')
            ax.set_title('Run Trajectory with Prediction Fans')
            ax.legend(loc='best', fontsize=9)
            ax.grid(True, alpha=0.2)
            fig.tight_layout()

            _save(fig, out)
        return fig

    # ── 4. Error vs prediction horizon ─────────────────────────────────────

    def plot_error_horizon(
        self,
        out:     Optional[str] = None,
        fig:     Optional[plt.Figure] = None,
        subsample: int = 1,
    ) -> plt.Figure:
        """
        How prediction error grows with horizon (1 to T_out steps = 0.1s to 3s).

        Top panel:    RMSE in position (metres) at each step.
        Bottom panel: Gaussian NLL for position at each step.

        Shaded band = mean ± 1 SD across sequences, solid line = mean.
        """
        with plt.rc_context(_STYLE):
            if fig is None:
                fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
            else:
                ax1, ax2 = fig.axes[:2]

            seqs_sub = self.sequences[::subsample]
            pred_sub = self.predictions[::subsample]
            T_out = len(pred_sub[0]['position_mean'])

            rmse_all = []
            nll_all  = []

            for seq, pred in zip(seqs_sub, pred_sub):
                bounds = seq['graph_bounds']
                fp     = R.pos_to_m(np.array([s['position'] for s in seq['future']]), bounds)
                pm     = R.pos_to_m(_to_np(pred['position_mean']), bounds)
                pv     = R.pos_var_to_m2(_to_np(pred['position_var']), bounds)

                # (T_out, 2) → (T_out,)
                rmse = np.sqrt(np.sum((pm - fp) ** 2, axis=-1))
                nll  = 0.5 * np.sum(np.log(pv) + (pm - fp) ** 2 / pv, axis=-1)

                rmse_all.append(rmse)
                nll_all.append(nll)

            rmse_mat = np.stack(rmse_all)   # (N, T_out)
            nll_mat  = np.stack(nll_all)

            t = np.arange(1, T_out + 1) * DT

            for ax, mat, ylabel, title in [
                (ax1, rmse_mat, 'Position RMSE  (m)',  'Position error vs prediction horizon'),
                (ax2, nll_mat,  'Gaussian NLL',        'NLL vs prediction horizon'),
            ]:
                mean = mat.mean(axis=0)
                std  = mat.std(axis=0)
                ax.plot(t, mean, color=C_PRED, lw=2.0)
                ax.fill_between(t, mean - std, mean + std,
                                color=C_BAND, alpha=0.35, label='Mean ± 1 SD')
                ax.set_ylabel(ylabel)
                ax.set_title(title)
                ax.grid(True, alpha=0.25)
                ax.legend(fontsize=9)

            ax2.set_xlabel('Prediction horizon  (s)')
            fig.tight_layout()

            _save(fig, out)
        return fig

    # ── 5. Calibration check ────────────────────────────────────────────────

    def plot_calibration(
        self,
        feature:  str = 'position',
        n_levels: int = 20,
        out:      Optional[str] = None,
        ax:       Optional[plt.Axes] = None,
    ) -> plt.Figure:
        """
        Empirical coverage vs nominal coverage.

        For a well-calibrated Gaussian, the fraction of ground-truth values
        that fall within the predicted ±k-σ interval should equal the
        expected Gaussian coverage at that level.

        Plots: nominal (diagonal) vs observed coverage for each step-ahead,
        averaged over sequences (line) + per-step spread (band).

        feature: 'position', 'velocity', 'steering', or 'acceleration'
        """
        from scipy.stats import norm as _norm

        with plt.rc_context(_STYLE):
            if ax is None:
                fig, ax = plt.subplots(figsize=(6, 6))
            else:
                fig = ax.figure

            alphas   = np.linspace(0.05, 0.99, n_levels)
            k_vals   = _norm.ppf((1 + alphas) / 2)   # half-widths in σ units

            coverages = []   # list of (T_out, n_levels) arrays

            for seq, pred in zip(self.sequences, self.predictions):
                bounds = seq['graph_bounds']

                if feature == 'position':
                    fut = R.pos_to_m(
                        np.array([s['position'] for s in seq['future']]), bounds)
                    mu  = R.pos_to_m(_to_np(pred['position_mean']), bounds)
                    sig = np.sqrt(R.pos_var_to_m2(_to_np(pred['position_var']), bounds))
                    # collapse x/y: use mean L2 z-score
                    z = np.sqrt(np.sum(((fut - mu) / sig) ** 2, axis=-1))
                    # convert to per-dim: use chi2(2) CDF instead of Gaussian
                    # but for simplicity, treat each dim independently
                    z_x = np.abs((fut[:, 0] - mu[:, 0]) / sig[:, 0])
                    z_y = np.abs((fut[:, 1] - mu[:, 1]) / sig[:, 1])
                    z   = np.stack([z_x, z_y], axis=-1)   # (T_out, 2)
                elif feature == 'velocity':
                    fut = R.vel_to_ms(np.array([s['velocity'] for s in seq['future']]))
                    mu  = R.vel_to_ms(_to_np(pred['velocity_mean']))
                    sig = np.sqrt(R.vel_var_to_ms2(_to_np(pred['velocity_var'])))
                    z   = np.abs((fut - mu) / (sig + 1e-8))
                elif feature == 'steering':
                    fut = R.steer_to_rad(np.array([s['steering'] for s in seq['future']]))
                    mu  = R.steer_to_rad(_to_np(pred['steering_mean']))
                    sig = np.sqrt(R.steer_var_to_rad2(_to_np(pred['steering_var'])))
                    z   = np.abs((fut - mu) / (sig + 1e-8))[:, None]
                elif feature == 'acceleration':
                    fut = R.accel_to_ms2(np.array([s['acceleration'] for s in seq['future']]))
                    mu  = R.accel_to_ms2(_to_np(pred['acceleration_mean']))
                    sig = np.sqrt(R.accel_var_to_ms4(_to_np(pred['acceleration_var'])))
                    z   = np.abs((fut - mu) / (sig + 1e-8))[:, None]
                else:
                    raise ValueError(f"Unknown feature '{feature}'")

                # z: (T_out, d)  — z-scores per timestep per dim
                # Coverage at level k: mean over timesteps & dims of (z <= k)
                cvg = np.array([
                    float(np.mean(z <= k)) for k in k_vals
                ])   # (n_levels,)
                coverages.append(cvg)

            coverages = np.stack(coverages)   # (N_seq, n_levels)
            mean_cvg  = coverages.mean(axis=0)
            std_cvg   = coverages.std(axis=0)

            ax.plot([0, 1], [0, 1], 'k--', lw=1.2, label='Perfect calibration')
            ax.plot(alphas, mean_cvg, color=C_PRED, lw=2.0, label='Observed coverage')
            ax.fill_between(alphas, mean_cvg - std_cvg, mean_cvg + std_cvg,
                            color=C_BAND, alpha=0.35)

            ax.set_xlabel('Nominal coverage')
            ax.set_ylabel('Empirical coverage')
            ax.set_title(f'Calibration — {feature}')
            ax.legend()
            ax.set_xlim(0, 1)
            ax.set_ylim(0, 1)
            ax.grid(True, alpha=0.25)
            fig.tight_layout()

            _save(fig, out)
        return fig

    # ── 6. Residual timeline from infer.py CSV ──────────────────────────────

    @staticmethod
    def plot_residual_timeline(
        df,
        title:        str = 'Residual Timeline',
        fault_windows: Optional[List[Tuple[int, int]]] = None,
        nll_threshold: Optional[float] = None,
        out:           Optional[str] = None,
        fig:           Optional[plt.Figure] = None,
    ) -> plt.Figure:
        """
        Three-panel timeline from an infer.py residual DataFrame:
          Top:    Combined NLL + CUSUM (optional)
          Middle: Feature-level NLL breakdown (pos, vel, steer)
          Bottom: Position L2 error (metres) + object distance

        fault_windows: list of (start_idx, end_idx) for shading fault periods.
        nll_threshold: horizontal dashed line on the NLL panel (e.g. 3σ level).
        """
        import pandas as pd

        with plt.rc_context(_STYLE):
            if fig is None:
                fig, axes = plt.subplots(3, 1, figsize=(13, 8), sharex=True)
            else:
                axes = fig.axes[:3]

            x = df.index.values if hasattr(df, 'index') else np.arange(len(df))

            def _col(name, default=None):
                if name in df.columns:
                    return df[name].values.astype(float)
                return default

            nll     = _col('combined_nll')
            cusum   = _col('cusum_combined')
            pos_nll = _col('pos_nll')
            vel_nll = _col('vel_nll')
            str_nll = _col('steer_nll')
            pos_l2  = _col('pos_l2')
            obj_d   = _col('obj_dist_actual')

            # ── Top: NLL ──────────────────────────────────────────────────
            ax0 = axes[0]
            if nll is not None:
                ax0.plot(x, nll, color=C_PRED, lw=1.4, alpha=0.9, label='Combined NLL')
            if cusum is not None:
                ax0r = ax0.twinx()
                ax0r.plot(x, cusum, color=C_ERROR, lw=1.2, ls=':', alpha=0.7, label='CUSUM')
                ax0r.set_ylabel('CUSUM', color=C_ERROR, fontsize=10)
                ax0r.tick_params(axis='y', labelcolor=C_ERROR)
            if nll_threshold is not None:
                ax0.axhline(nll_threshold, color='gray', ls='--', lw=1.0,
                            label=f'Threshold ({nll_threshold:.2f})')
            ax0.set_ylabel('Combined NLL')
            ax0.set_title(title)
            ax0.legend(loc='upper left', fontsize=9)
            ax0.grid(True, alpha=0.22)

            # ── Middle: feature NLL breakdown ─────────────────────────────
            ax1 = axes[1]
            colors = [C_PRED, '#009E73', '#E69F00']
            for arr, label, color in [
                (pos_nll, 'Position NLL', colors[0]),
                (vel_nll, 'Velocity NLL', colors[1]),
                (str_nll, 'Steering NLL', colors[2]),
            ]:
                if arr is not None:
                    ax1.plot(x, arr, color=color, lw=1.2, alpha=0.85, label=label)
            ax1.set_ylabel('Per-feature NLL')
            ax1.legend(loc='upper left', fontsize=9)
            ax1.grid(True, alpha=0.22)

            # ── Bottom: pos L2 + object distance ──────────────────────────
            ax2 = axes[2]
            if pos_l2 is not None:
                ax2.plot(x, pos_l2, color=C_PRED, lw=1.4, label='Position L2 error')
                ax2.set_ylabel('Position L2  (scaled)')
            if obj_d is not None:
                ax2r = ax2.twinx()
                ax2r.fill_between(x, 0, 1 - obj_d, color=C_ERROR, alpha=0.18,
                                  label='Object proximity (1−dist)')
                ax2r.set_ylabel('Object proximity', color=C_ERROR, fontsize=10)
                ax2r.tick_params(axis='y', labelcolor=C_ERROR)
                ax2r.set_ylim(0, 1)
            ax2.legend(loc='upper left', fontsize=9)
            ax2.set_xlabel('Window index  (1 window = 0.1 s at 10 Hz)')
            ax2.grid(True, alpha=0.22)

            # ── Fault window shading ──────────────────────────────────────
            if fault_windows:
                for a_ax in axes:
                    for start, end in fault_windows:
                        a_ax.axvspan(start, end, color=C_ERROR, alpha=0.10, zorder=0)

            fig.tight_layout()
            _save(fig, out)
        return fig

    # ── 7. Ensemble grid ────────────────────────────────────────────────────

    def plot_prediction_grid(
        self,
        indices:     Optional[List[int]] = None,
        n:           int = 9,
        out:         Optional[str] = None,
    ) -> plt.Figure:
        """
        N thumbnail trajectory fans in a grid — quick model-quality overview.
        If indices is None, samples n evenly from the full sequence set.
        """
        if indices is None:
            step = max(1, len(self.sequences) // n)
            indices = list(range(0, len(self.sequences), step))[:n]

        cols = math.ceil(math.sqrt(len(indices)))
        rows = math.ceil(len(indices) / cols)

        with plt.rc_context(_STYLE):
            fig, axes = plt.subplots(rows, cols,
                                     figsize=(4.5 * cols, 4.5 * rows))
            axes_flat = np.array(axes).flat

            for ax, idx in zip(axes_flat, indices):
                seq    = self.sequences[idx]
                pred   = self.predictions[idx]
                bounds = seq['graph_bounds']

                _draw_graph(ax, seq['graph'], bounds)

                pp = R.pos_to_m(np.array([s['position'] for s in seq['past']]),  bounds)
                fp = R.pos_to_m(np.array([s['position'] for s in seq['future']]), bounds)
                pm = R.pos_to_m(_to_np(pred['position_mean']), bounds)
                ps = np.sqrt(R.pos_var_to_m2(_to_np(pred['position_var']), bounds))

                ax.plot(pp[:, 0], pp[:, 1], color=C_PAST,   lw=1.2, zorder=3)
                ax.plot(fp[:, 0], fp[:, 1], color=C_ACTUAL, lw=1.2, ls='--', zorder=4)
                _draw_prediction_corridor(ax, pm, ps, alpha_base=0.75, lw=1.5)
                ax.set_aspect('equal')
                ax.set_title(f'seq {idx}', fontsize=9)
                ax.axis('off')

            for ax in list(axes_flat)[len(indices):]:
                ax.set_visible(False)

            fig.suptitle('Prediction Grid  (gray=past, black--=actual, blue=predicted)', y=1.01)
            fig.tight_layout()
            _save(fig, out)
        return fig


# ── Standalone residual plot (no PredictionVisualizer needed) ─────────────

def plot_residual_timeline(df, **kwargs) -> plt.Figure:
    """Module-level convenience wrapper around PredictionVisualizer.plot_residual_timeline."""
    return PredictionVisualizer.plot_residual_timeline(df, **kwargs)


# ── Internal drawing helpers ───────────────────────────────────────────────

def _draw_graph(ax: plt.Axes, G, bounds, zorder: int = 1):
    """Draw the HD-map context graph on ax in physical units."""
    x_min, x_max, y_min, y_max = bounds

    def _unscale(nx_data):
        rx = nx_data['x'] * (x_max - x_min) + x_min
        ry = nx_data['y'] * (y_max - y_min) + y_min
        return rx, ry

    # Collect edge segments for a single LineCollection call (fast)
    edge_segs = []
    for u, v in G.edges():
        xu, yu = _unscale(G.nodes[u])
        xv, yv = _unscale(G.nodes[v])
        edge_segs.append([(xu, yu), (xv, yv)])

    if edge_segs:
        lc = LineCollection(edge_segs, colors=C_GRAPH, lw=0.5,
                            alpha=0.6, zorder=zorder)
        ax.add_collection(lc)

    # Nodes by type
    regular_xy, tl_xy, path_xy = [], [], []
    for nid, data in G.nodes(data=True):
        rx, ry = _unscale(data)
        if data.get('traffic_light_detection_node', 0):
            tl_xy.append((rx, ry))
        elif data.get('path_node', 0):
            path_xy.append((rx, ry))
        else:
            regular_xy.append((rx, ry))

    for xy, color, sz in [
        (regular_xy, C_GRAPH,  4),
        (tl_xy,      C_TL,    14),
        (path_xy,    C_PATH,  10),
    ]:
        if xy:
            xs, ys = zip(*xy)
            ax.scatter(xs, ys, c=color, s=sz, zorder=zorder + 1)


def _draw_prediction_corridor(
    ax: plt.Axes,
    mean: np.ndarray,   # (T_out, 2)  x, y in metres
    std:  np.ndarray,   # (T_out, 2)  σ_x, σ_y in metres
    alpha_base:  float = 0.8,
    mean_color = C_PRED,
    lw:          float = 2.0,
):
    """
    Draw the predicted trajectory as a coloured line (mean) with ±1σ and ±2σ
    confidence tubes along x and y independently, using fill_between.
    Opacity fades toward the end of the horizon (most uncertain).
    """
    T = mean.shape[0]
    t = np.arange(T)

    # For spatial corridors we split along x and y separately and use the
    # "ribbon" approach: sort points by x or y and shade the band.
    # A cleaner map representation: draw the mean path + ellipses at intervals.
    ax.plot(mean[:, 0], mean[:, 1],
            color=mean_color, lw=lw, zorder=5, alpha=alpha_base,
            label='Predicted mean')

    # Ellipses at every 5th step (=0.5s), size from σ, opacity from horizon
    from matplotlib.patches import Ellipse
    for t_i in range(0, T, max(1, T // 8)):
        alpha_i = alpha_base * (1.0 - 0.6 * t_i / T)
        e1 = Ellipse(
            xy=(mean[t_i, 0], mean[t_i, 1]),
            width=2 * std[t_i, 0],
            height=2 * std[t_i, 1],
            facecolor=C_BAND, edgecolor=mean_color,
            alpha=alpha_i * 0.35, linewidth=0.6, zorder=4,
        )
        e2 = Ellipse(
            xy=(mean[t_i, 0], mean[t_i, 1]),
            width=4 * std[t_i, 0],
            height=4 * std[t_i, 1],
            facecolor=C_BAND, edgecolor=mean_color,
            alpha=alpha_i * 0.15, linewidth=0.4, zorder=3,
        )
        ax.add_patch(e1)
        ax.add_patch(e2)


# ── Utility ────────────────────────────────────────────────────────────────

def _save(fig: plt.Figure, path: Optional[str]):
    if path is not None:
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        fig.savefig(path, bbox_inches='tight')
        print(f'[viz] saved → {path}')
