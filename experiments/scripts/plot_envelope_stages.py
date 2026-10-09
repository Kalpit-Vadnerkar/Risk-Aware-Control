"""How the envelope changes as each module is added, on one fault trial.

  envelope_stages_<trial>.png   rows = modules, columns = moments in the fault:
      1. Layer 1 only: ST-GAT forecast + its 90% conformal band per horizon step
      2. + 2a: scene-conditioned counterfactual futures (forecast + recorded
         errors of the 150 most similar scenes)
      3. + 2b: the same futures moved by the estimated pose error
      4. + correction (Layer 3 preview): the vehicle's past path corrected
         by the estimated pose error, against the true path
  envelope_explainer_<trial>.png  one moment: the envelope as samples and as
      the distribution they represent (density, 50/90% contours)

  STGAT_GRAPH_FRAME=ego_v6 python3 experiments/scripts/plot_envelope_stages.py \
      --trial imu_fault_scale/goal_007
"""
import argparse
import os
import sys
from types import SimpleNamespace

import numpy as np
import torch
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

sys.path.insert(0, os.path.dirname(__file__))
import layer2_envelope as LE  # noqa: E402
L2, cfg = LE.L2, LE.cfg
HALF = 28
C = {'past': 'k', 'true': '#ff7f0e', 'fc': '#1f77b4', 'band': '#1f77b4', 'ok': '#2ca02c', 'bad': '#d62728',
     'corr': '#9467bd', 'obj': 'm'}
ROWS = ['1  Layer 1 only:\nforecast + 90% conformal band',
        '2  + 2a: scene-conditioned\ncounterfactual futures',
        '3  + 2b: futures moved by the\nestimated pose error',
        '4  + correction (Layer 3 preview):\npose corrected by the estimate']


def _map(ax, drivable, ref, half=HALF, center=None):
    c = ref if center is None else center
    x0, y0 = c[0] - half, c[1] - half
    i0, j0 = int((x0 - drivable.x0) / drivable.res), int((y0 - drivable.y0) / drivable.res)
    n = int(2 * half / drivable.res)
    ax.imshow(drivable.mask[j0:j0 + n, i0:i0 + n], origin='lower', extent=[x0, x0 + 2 * half, y0, y0 + 2 * half],
              cmap='Greys', vmin=0, vmax=2.5)
    ax.set_xlim(x0, x0 + 2 * half); ax.set_ylim(y0, y0 + 2 * half)
    ax.set_aspect('equal'); ax.set_xticks([]); ax.set_yticks([])


def _true_path(ax, tr, i, back=3, fwd=3):
    t_abs = tr['t'][i] + tr['t0']
    gs = (tr['gt'][:, 0] >= t_abs - back) & (tr['gt'][:, 0] <= t_abs + fwd)
    ax.plot(tr['gt'][gs, 1], tr['gt'][gs, 2], color=C['true'], lw=2.2, ls='--', zorder=5)


def corrected_past(tr, i, n=30):
    """True-pose estimate for the last n windows: estimate minus the pose
    error (est - true, in the estimate's heading frame)."""
    ev = tr['ev']
    out = []
    for j in range(max(0, i - n + 1), i + 1):
        p = L2._ref(tr['seqs'][j])
        th = tr['seqs'][j]['past'][-1]['yaw']
        x, y = ev['X'][j], ev['Y'][j]
        out.append(p - np.array([np.cos(th) * x - np.sin(th) * y, np.sin(th) * x + np.cos(th) * y]))
    return np.array(out)


def band_radius(pool, q=0.9):
    """Layer 1's calibrated position band: per-step quantile of the
    calibration error norm (m)."""
    return np.quantile(np.linalg.norm(pool['err_local'], axis=-1), q, axis=0) * LE.R_POS


def panel(ax, row, tr, g, i, pool, drivable, map_data, args, rad):
    seq = tr['seqs'][i]
    ref = L2._ref(seq)
    rng = np.random.default_rng(args.seed + i)
    out, draws, objs = LE.evaluate(seq, tr['inf']['pos_mean'][i], tr['inf']['h'][i], pool, tr['ev'], i,
                                   drivable, map_data, rng, args, keep=True)
    mu = ref + tr['inf']['pos_mean'][i] * LE.R_POS
    _map(ax, drivable, ref)
    if row == 0:
        for k in range(0, len(mu), 3):
            ax.add_patch(plt.Circle(mu[k], rad[k], color=C['band'], alpha=0.10, lw=0))
    elif row in (1, 2, 3):
        traj, viol = draws['nominal' if row == 1 else 'evidence']
        for k in range(0, len(viol), 2):
            ax.plot(traj[k, :, 0], traj[k, :, 1], color=C['bad'] if viol[k] else C['ok'], lw=0.6, alpha=0.45)
    past = L2._past_xy(seq)
    ax.plot(past[:, 0], past[:, 1], color=C['past'], lw=2.2, zorder=6)
    if row == 3:
        cp = corrected_past(tr, i)
        ax.plot(cp[:, 0], cp[:, 1], color=C['corr'], lw=2.6, zorder=7)
    ax.plot(mu[:, 0], mu[:, 1], color=C['fc'], lw=2.2, zorder=6)
    _true_path(ax, tr, i)
    for m in range(len(objs[2])):
        ax.plot(objs[0][m, :, 0], objs[0][m, :, 1], color=C['obj'], lw=1)
    txt = {0: '', 1: f"P = {out['nominal']:.2f}", 2: f"P = {out['evidence']:.2f}", 3: f"P = {out['evidence']:.2f}"}[row]
    if txt:
        ax.text(0.03, 0.04, txt, transform=ax.transAxes, fontsize=12, weight='bold',
                bbox=dict(fc='white', ec='none', alpha=0.85))


def stages(tr, g, pool, drivable, map_data, args, out_dir):
    fs = min(w['start'] for w in tr['fault_windows'])
    dep = g['departure']
    times = [fs - 2, (fs + dep) / 2, dep - 2, dep - 0.3] if dep else [fs - 2, fs + 3, fs + 6, fs + 9]
    idxs = sorted({int(np.argmin(abs(tr['t'] - x))) for x in times})
    rad = band_radius(pool)
    fig, axes = plt.subplots(4, len(idxs), figsize=(3.9 * len(idxs) + 2.2, 16.5))
    for c, i in enumerate(idxs):
        for r in range(4):
            panel(axes[r, c], r, tr, g, i, pool, drivable, map_data, args, rad)
        ev = tr['ev']
        when = f"departure {tr['t'][i] - dep:+.1f} s" if dep else f"fault {tr['t'][i] - fs:+.1f} s"
        axes[0, c].set_title(f"t = {tr['t'][i]:.1f} s  ({when})\nheading error: true {np.degrees(g['true_psi'][i]):+.1f}°, "
                             f"estimated {np.degrees(ev['Psi'][i]):+.1f}°", fontsize=11)
    for r in range(4):
        axes[r, 0].set_ylabel(ROWS[r], fontsize=12, rotation=0, ha='right', va='center', labelpad=12)
    handles = [Line2D([], [], color=C['past'], lw=2.5, label="past path, as the vehicle's own estimate saw it"),
               Line2D([], [], color=C['true'], lw=2.5, ls='--', label='TRUE path (AWSIM ground truth)'),
               Line2D([], [], color=C['fc'], lw=2.5, label='ST-GAT forecast, next 3 s'),
               Patch(color=C['band'], alpha=0.25, label='row 1: Layer 1 90% band per horizon step'),
               Line2D([], [], color=C['ok'], lw=2, label='rows 2-4: counterfactual future stays on the road'),
               Line2D([], [], color=C['bad'], lw=2, label='rows 2-4: counterfactual future leaves the road'),
               Line2D([], [], color=C['corr'], lw=3, label='row 4: past path corrected by the estimated pose error'),
               Line2D([], [], color=C['obj'], lw=1.5, label='other road users'),
               Patch(color='#bdbdbd', label='drivable area')]
    fig.legend(handles=handles, loc='lower center', ncol=3, fontsize=12.5, frameon=False, handlelength=3)
    fig.suptitle(f"{tr['rel']}: what each module adds (P = share of futures that leave the road within 3 s)",
                 fontsize=14)
    fig.tight_layout(rect=[0, 0.07, 1, 0.97])
    fig.savefig(os.path.join(out_dir, f"envelope_stages_{tr['rel'].split('/t')[0].replace('/', '__')}.png"), dpi=110)
    plt.close(fig)


def explainer(tr, g, pool, drivable, map_data, args, out_dir, at):
    i = int(np.argmin(abs(tr['t'] - at)))
    seq = tr['seqs'][i]
    ref = L2._ref(seq)
    rng = np.random.default_rng(args.seed + i)
    out, draws, objs = LE.evaluate(seq, tr['inf']['pos_mean'][i], tr['inf']['h'][i], pool, tr['ev'], i,
                                   drivable, map_data, rng, args, keep=True)
    mu = ref + tr['inf']['pos_mean'][i] * LE.R_POS
    fig, axes = plt.subplots(1, 4, figsize=(21, 6.8))
    titles = ['a  2a futures as samples:\nforecast + one recorded error trajectory each',
              'b  the distribution they represent:\ndensity of positions, 50% / 90% at 3 s',
              'c  2b moves the distribution:\nsamples after the pose-error draw',
              'd  P(violation) = share of the\ndistribution outside the road']
    centre = ref + 0.5 * (mu[-1] - ref)
    for a, (name, mode) in zip(axes, [('nominal', 's'), ('nominal', 'd'), ('evidence', 's'), ('evidence', 'v')]):
        _map(a, drivable, ref, half=13, center=centre)
        traj, viol = draws[name]
        if mode == 's':
            for k in range(len(traj)):
                a.plot(traj[k, :, 0], traj[k, :, 1], color='#555' if name == 'nominal' else C['corr'], lw=0.5, alpha=0.35)
        elif mode == 'd':
            pts = traj.reshape(-1, 2)
            a.hist2d(pts[:, 0], pts[:, 1], bins=90, range=[[centre[0] - 13, centre[0] + 13], [centre[1] - 13, centre[1] + 13]],
                     cmap='Blues', cmin=1, alpha=0.9)
            end = traj[:, -1]
            m, S = end.mean(0), np.cov(end.T)
            w, V = np.linalg.eigh(S)
            for q, ls in [(1.386, '-'), (4.605, '--')]:                     # chi2(2) 50% and 90%
                tt = np.linspace(0, 2 * np.pi, 100)
                e = (V @ (np.sqrt(w * q)[:, None] * np.stack([np.cos(tt), np.sin(tt)]))).T + m
                a.plot(e[:, 0], e[:, 1], color='k', ls=ls, lw=1.2)
        else:
            for k in range(len(traj)):
                a.plot(traj[k, :, 0], traj[k, :, 1], color=C['bad'] if viol[k] else C['ok'], lw=0.5, alpha=0.4)
            a.text(0.03, 0.04, f"{int(viol.sum())} of {len(viol)} leave the road:  P = {viol.mean():.2f}",
                   transform=a.transAxes, fontsize=12, weight='bold', bbox=dict(fc='white', ec='none', alpha=0.85))
        a.plot(L2._past_xy(seq)[:, 0], L2._past_xy(seq)[:, 1], 'k', lw=2.2)
        a.plot(mu[:, 0], mu[:, 1], color=C['fc'], lw=2.2)
        _true_path(a, tr, i)
        a.set_title(titles[list(axes).index(a)], fontsize=12)
    handles = [Line2D([], [], color='k', lw=2.5, label="past path (vehicle's estimate)"),
               Line2D([], [], color=C['true'], lw=2.5, ls='--', label='TRUE path'),
               Line2D([], [], color=C['fc'], lw=2.5, label='ST-GAT forecast'),
               Line2D([], [], color='#555', lw=1.5, label='a: 200 counterfactual futures (2a)'),
               Line2D([], [], color='k', lw=1.2, label='b: 50% region of 3 s endpoints'),
               Line2D([], [], color='k', lw=1.2, ls='--', label='b: 90% region'),
               Line2D([], [], color=C['corr'], lw=1.5, label='c: futures moved by 2b'),
               Line2D([], [], color=C['ok'], lw=1.5, label='d: stays on road'),
               Line2D([], [], color=C['bad'], lw=1.5, label='d: leaves road')]
    fig.legend(handles=handles, loc='lower center', ncol=5, fontsize=12, frameon=False, handlelength=3)
    fig.suptitle(f"The envelope is a distribution, carried by samples ({tr['rel'].split('/t')[0]}, t = {tr['t'][i]:.1f} s)",
                 fontsize=14)
    fig.tight_layout(rect=[0, 0.1, 1, 0.94])
    fig.savefig(os.path.join(out_dir, f"envelope_explainer_{tr['rel'].split('/t')[0].replace('/', '__')}.png"), dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--trial', default='imu_fault_scale/goal_007')
    ap.add_argument('--explain-at', type=float, default=-2.0, help='seconds relative to departure (or fault onset)')
    a = ap.parse_args()
    out_dir = os.path.join(cfg.ANALYSIS_ROOT, 'highlights')
    os.makedirs(out_dir, exist_ok=True)
    args = SimpleNamespace(model=cfg.MODEL_CONFIG['model_path'], batch=256, no_cache=False, kappa_q=0.99,
                           max_episodes_per_hour=6.0, seed=20261007, k_neighbors=150, k_draws=200,
                           output_dir=os.path.join(cfg.ANALYSIS_ROOT, 'layer2_envelope'))
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = L2._load_model(args.model, device)
    map_data = L2.margin_lib.load_map(cfg.MAP_FILE)
    drivable = L2.margin_lib.DrivableArea(map_data)
    pool = LE.build_pool(model, device, args)
    from st_gat.pipeline.State_Estimator.MapProcessor import MapProcessor
    from st_gat.pipeline.sequence_builder import SequenceBuilder
    builder = SequenceBuilder(MapProcessor(cfg.MAP_FILE).map_data, route=[])
    camps = sorted({a.trial.split('/')[0]})
    run_dir = next(r for r in L2._fault_trials(camps) if os.path.relpath(r, L2.DATA_DIR).startswith(a.trial))
    tr = LE.process_trial(run_dir, model, device, pool, builder, args)
    g = LE.ground_truth(tr, drivable)
    stages(tr, g, pool, drivable, map_data, args, out_dir)
    ref_t = g['departure'] if g['departure'] else min(w['start'] for w in tr['fault_windows'])
    explainer(tr, g, pool, drivable, map_data, args, out_dir, ref_t + a.explain_at)
    print('saved to', out_dir)


if __name__ == '__main__':
    main()
