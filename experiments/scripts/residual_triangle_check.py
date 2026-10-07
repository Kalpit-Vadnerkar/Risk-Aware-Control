#!/usr/bin/env python3
"""
Residual-triangle check (2026-10-07) -- does the ST-GAT's own live residual
evidence carry the position/heading signal Layer 2 needs, early enough and
in the right direction?

Context: Layer 2's pose coupling (layer2_consequence_estimation.py, 'pose')
catches all 4 real IMU road departures, but its signal is a physics
consistency check (estimated pose vs bicycle dead-reckoning), not the
learned model -- Layer 1 is bypassed. Kalpit's proposal: expand the
envelope in position/heading using Layer 1's OWN residuals, and use all of
them -- at 10 Hz there are 30 overlapping forecasts in flight, the one made
k steps ago has its first k steps observed: ~435 observed (lag, step)
residual points at every instant, of which the old 'adaptive' coupling used
one forecast's first 10 steps.

For every forecast j and step s: residual r = observed - predicted
position, split into LATERAL / ALONG-track in forecast j's own heading
frame (localized yaw), plus a HEADING residual (observed vs predicted
direction of travel over 0.4s, steps >= 4, only when displacement > 1m).
Nominal per-step 90% quantiles of |lat|, |along|, |head| come from the
CAL set. At each time t:
  E_lat/E_along/E_head  fraction of the triangle's observed points beyond
                        their per-step nominal quantile (~0.10 nominally,
                        serially correlated) -- significance = nominal p99
                        of the same statistic over CAL windows
  L_recent, H_recent    signed mean lateral / heading residual of the most
                        recent observed step of the forecasts made in the
                        last 1s -- WHICH WAY the evidence points
Ground truth for the direction check: AWSIM pose vs Autoware estimate,
expressed in the estimate's frame (true lateral offset, true - estimated
heading). Baseline for timing: the dead-reckoning pose_error() reading.

Usage (ROS + venv, STGAT_GRAPH_FRAME=ego_v3):
  python3 experiments/scripts/residual_triangle_check.py
"""

import argparse
import json
import math
import os
import sys

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
sys.path[:0] = [REPO_DIR, SCRIPT_DIR, os.path.join(REPO_DIR, 'experiments', 'lib')]

import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import torch  # noqa: E402

from st_gat.pipeline import config as cfg  # noqa: E402
from st_gat.model import TrajectoryDataset  # noqa: E402
import layer2_consequence_estimation as L2  # noqa: E402
from conformal_horizon_calibration import _trial_ids_for  # noqa: E402

T_OUT = cfg.OUTPUT_SEQ_LEN
HEAD_LAG = 4          # heading residual over 0.4s
NOMINAL_Q = 0.90
DEFAULT_TRIALS = [
    'imu_fault_s3/goal_007', 'imu_fault_scale/goal_007', 'imu_fault_stuck/goal_007', 'imu_fault_stuck/goal_012',
    'imu_fault_s1/goal_007', 'imu_fault_s3/goal_012', 'imu_fault_scale/goal_012', 'imu_fault_s1/goal_012',
    'tl_fault_fixed_070/goal_007/t1', 'tl_fault_s4/goal_012', 'tl_fault_s2/goal_026',
]
wrap = lambda a: (a + np.pi) % (2 * np.pi) - np.pi  # noqa: E731


def residual_components(seqs, pos_mean_norm):
    """Per forecast j: (N,T) lateral, along, heading residuals (m, m, rad;
    heading NaN where undefined) in forecast j's own localized-yaw frame."""
    ref = np.array([L2._ref(s) for s in seqs])
    yaw = np.array([s['past'][-1]['yaw'] for s in seqs])
    mu = ref[:, None] + pos_mean_norm * cfg.POSITION_DISPLACEMENT_RANGE_M
    act = np.array([L2._future_xy(s) for s in seqs])
    r = act - mu
    u = np.stack([np.cos(yaw), np.sin(yaw)], -1)[:, None]
    n = np.stack([-np.sin(yaw), np.cos(yaw)], -1)[:, None]
    lat, along = (r * n).sum(-1), (r * u).sum(-1)
    head = np.full(lat.shape, np.nan)
    da, dm = act[:, HEAD_LAG:] - act[:, :-HEAD_LAG], mu[:, HEAD_LAG:] - mu[:, :-HEAD_LAG]
    ok = (np.linalg.norm(da, axis=-1) > 1.0) & (np.linalg.norm(dm, axis=-1) > 1.0)
    h = wrap(np.arctan2(da[..., 1], da[..., 0]) - np.arctan2(dm[..., 1], dm[..., 0]))
    head[:, HEAD_LAG:] = np.where(ok, h, np.nan)
    return lat, along, head


def triangle_stats(comp, q, prev_ok):
    """comp: {'lat','along','head'} (N,T). q: same keys -> (T,) nominal quantiles.
    prev_ok[k] (N,) bool: window i-k exists, same trial, ~k steps earlier.
    Returns per-window exceed fractions E_* and recent signed means."""
    N = comp['lat'].shape[0]
    out = {}
    for key in ('lat', 'along', 'head'):
        X = np.abs(comp[key]) > q[key][None, :]
        valid = ~np.isnan(comp[key])
        X = X & valid
        num, den = np.zeros(N), np.zeros(N)
        cx, cv = np.cumsum(X, axis=1), np.cumsum(valid, axis=1)   # prefix sums over steps
        for k in range(1, T_OUT):
            ok = prev_ok[k]
            src = np.arange(N) - k
            num[ok] += cx[src[ok], k - 1]
            den[ok] += cv[src[ok], k - 1]
        out[f'E_{key}'] = np.where(den > 0, num / np.maximum(den, 1), np.nan)
    # most recent observed step (s = k-1) of forecasts made 1..10 steps ago
    for key, name in (('lat', 'L_recent'), ('head', 'H_recent')):
        acc, cnt = np.zeros(N), np.zeros(N)
        for k in range(1, 11):
            ok = prev_ok[k]
            src = np.arange(N) - k
            v = comp[key][src[ok], k - 1]
            good = ~np.isnan(v)
            idx = np.where(ok)[0][good]
            acc[idx] += v[good]
            cnt[idx] += 1
        out[name] = np.where(cnt > 0, acc / np.maximum(cnt, 1), np.nan)
    return out


def prev_ok_from_trial(trial_id):
    N = len(trial_id)
    res = {}
    for k in range(1, T_OUT):
        src = np.arange(N) - k
        res[k] = (src >= 0) & (trial_id[np.maximum(src, 0)] == trial_id)
    return res


def prev_ok_from_time(t_rel):
    N = len(t_rel)
    res = {}
    for k in range(1, T_OUT):
        src = np.arange(N) - k
        ok = src >= 0
        dt = t_rel - t_rel[np.maximum(src, 0)]
        res[k] = ok & (np.abs(dt - k * L2.DT) < 0.3)
    return res


def read_pose_yaw(bag_dir, topic):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=''), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    T = get_message(types[topic])
    out = []
    while r.has_next():
        _, data, ts = r.read_next()
        p = deserialize_message(data, T).pose.pose
        q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        out.append((ts * 1e-9, p.position.x, p.position.y, yaw))
    return np.array(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--model', default=cfg.MODEL_CONFIG['model_path'])
    ap.add_argument('--trials', nargs='*', default=DEFAULT_TRIALS)
    ap.add_argument('--batch', type=int, default=256)
    ap.add_argument('--output-dir', default=os.path.join(cfg.ANALYSIS_ROOT, 'residual_triangle'))
    args = ap.parse_args()
    assert cfg.EGO_FRAME, "needs per-step yaw (run with STGAT_GRAPH_FRAME=ego_v3)"
    os.makedirs(args.output_dir, exist_ok=True)
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = L2._load_model(args.model, device)

    # ── nominal: quantiles + significance levels ──
    ds = TrajectoryDataset(cfg.CAL_DIR)
    trial_id, _ = _trial_ids_for(cfg.CAL_DIR)
    inf = L2.infer(model, device, ds, args.batch)
    lat, along, head = residual_components(ds.sequences, inf['pos_mean'])
    comp = {'lat': lat, 'along': along, 'head': head}
    q = {k: np.nanquantile(np.abs(v), NOMINAL_Q, axis=0) for k, v in comp.items()}
    q['head'] = np.nan_to_num(q['head'], nan=np.inf)
    nom = triangle_stats(comp, q, prev_ok_from_trial(trial_id))
    sig = {k: float(np.nanquantile(np.abs(v), 0.99)) for k, v in nom.items()}
    print("nominal per-step q90 at 0.5s/1.5s/3s:  lat " + ' '.join(f"{q['lat'][s]:.2f}" for s in (4, 14, 29))
          + "m  along " + ' '.join(f"{q['along'][s]:.2f}" for s in (4, 14, 29))
          + "m  head " + ' '.join(f"{np.degrees(q['head'][s]):.1f}" for s in (4, 14, 29)) + "deg")
    print("nominal p99 (significance): " + ', '.join(f"{k} {v:.3f}" for k, v in sig.items())
          + f"   [E_* nominal mean: " + ', '.join(f"{k} {np.nanmean(v):.3f}" for k, v in nom.items() if k.startswith('E_')) + "]")
    del ds

    # ── fault trials ──
    from inspect_fault_predictions import process_trial
    from fault_log import campaign_fault_kind
    from st_gat.pipeline.State_Estimator.MapProcessor import MapProcessor
    from st_gat.pipeline.sequence_builder import SequenceBuilder
    builder = SequenceBuilder(MapProcessor(cfg.MAP_FILE).map_data, route=[])
    drivable = L2.margin_lib.DrivableArea(L2.margin_lib.load_map(cfg.MAP_FILE))
    results = []
    for run_dir in L2._fault_trials(sorted(os.listdir(L2.DATA_DIR))):
        rel = os.path.relpath(run_dir, L2.DATA_DIR)
        if not any(rel.startswith(x) for x in args.trials):
            continue
        camp = rel.split('/')[0]
        tr = process_trial(run_dir, camp, builder, campaign_fault_kind(os.path.join(L2.DATA_DIR, camp)))
        if tr is None:
            continue
        seqs, t = tr['sequences'], tr['t_rel']
        inf = L2.infer(model, device, L2._SeqList(seqs), args.batch)
        lat, along, head = residual_components(seqs, inf['pos_mean'])
        st = triangle_stats({'lat': lat, 'along': along, 'head': head}, q, prev_ok_from_time(t))
        pe = np.array([L2.pose_error(s) for s in seqs])     # dead-reckoning baseline
        bag = os.path.join(run_dir, 'rosbag')
        gt, est = read_pose_yaw(bag, L2.GT_TOPIC), read_pose_yaw(bag, L2.EST_TOPIC)
        gt_rm = L2.margin_lib.road_margin(drivable.signed_distance(gt[:, 1], gt[:, 2]))
        dep = float(gt[np.argmax(gt_rm < 0), 0] - tr['t0']) if (gt_rm < 0).any() else None
        # true deviation in the estimate's frame, at each window time
        ta = t + tr['t0']
        gi = np.searchsorted(gt[:, 0], ta).clip(0, len(gt) - 1)
        ei = np.searchsorted(est[:, 0], ta).clip(0, len(est) - 1)
        d = gt[gi, 1:3] - est[ei, 1:3]
        ny = np.stack([-np.sin(est[ei, 3]), np.cos(est[ei, 3])], -1)
        true_lat = (d * ny).sum(-1)
        true_head = wrap(gt[gi, 3] - est[ei, 3])
        fs = min((w['start'] for w in tr['fault_windows']), default=None)

        def first_sig(x, thr):
            m = (np.abs(x) > thr) & (t >= (fs - 5 if fs is not None else 0))
            return float(t[np.argmax(m)]) if m.any() else None
        row = {'trial': rel, 'fault_start': fs, 'departure': dep,
               'first_sig': {k: first_sig(st[k], sig[k]) for k in st},
               'first_sig_deadreckon_psi': first_sig(pe[:, 0], 0.0264)}
        if dep is not None:
            w = (t >= dep - 3) & (t <= dep)
            for k, truth in (('L_recent', true_lat), ('H_recent', true_head)):
                v, g = st[k][w], truth[w]
                ok = ~np.isnan(v) & (np.abs(g) > (0.2 if k == 'L_recent' else np.radians(2)))
                row[f'{k}_sign_agreement_last3s'] = float(np.mean(np.sign(v[ok]) == np.sign(g[ok]))) if ok.any() else None
                row[f'{k}_corr_last3s'] = float(np.corrcoef(v[ok], g[ok])[0, 1]) if ok.sum() > 3 else None
        results.append(row)
        print(f"\n[{rel}] fault {fs}  departure {dep}")
        print("  first significant: " + ', '.join(f"{k} {('%.1f' % v) if v is not None else '-'}" for k, v in row['first_sig'].items())
              + f" | dead-reckoning {row['first_sig_deadreckon_psi']}")
        if dep is not None:
            print(f"  direction (last 3s before departure): L_recent sign-agree {row['L_recent_sign_agreement_last3s']}, "
                  f"corr {row['L_recent_corr_last3s']} | H_recent sign-agree {row['H_recent_sign_agreement_last3s']}, "
                  f"corr {row['H_recent_corr_last3s']}")

        fig, ax = plt.subplots(4, 1, figsize=(13, 11), sharex=True)
        for k, c in (('E_lat', 'C0'), ('E_along', 'C1'), ('E_head', 'C2')):
            ax[0].plot(t, st[k], c, lw=1, label=k)
            ax[0].axhline(sig[k], color=c, ls=':', lw=0.8)
        ax[0].set_ylabel('triangle exceed fraction')
        ax[0].legend(fontsize=7)
        ax[1].plot(t, st['L_recent'], 'C0', lw=1, label='L_recent (residual lateral, m)')
        ax[1].plot(t, true_lat, 'k', lw=1, label='TRUE lateral offset of real car vs estimate (m)')
        ax[1].set_ylim(-5, 5)
        ax[1].legend(fontsize=7)
        ax[2].plot(t, np.degrees(st['H_recent']), 'C2', lw=1, label='H_recent (residual heading, deg)')
        ax[2].plot(t, np.degrees(true_head), 'k', lw=1, label='TRUE heading of real car - estimate (deg)')
        ax[2].plot(t, np.degrees(pe[:, 0]), 'm', lw=0.8, label='dead-reckoning psi_acc (deg, baseline)')
        ax[2].set_ylim(-60, 60)
        ax[2].legend(fontsize=7)
        tg = gt[:, 0] - tr['t0']
        ax[3].plot(tg, gt_rm, 'r', lw=1, label='ground-truth road margin (m)')
        ax[3].axhline(0, color='grey', lw=0.7)
        ax[3].set_ylim(-8, 8)
        ax[3].legend(fontsize=7)
        for a in ax:
            for wdw in tr['fault_windows']:
                a.axvspan(wdw['start'], wdw['end'], color='#ff7f0e', alpha=0.12)
            if dep is not None:
                a.axvline(dep, color='r', ls='--', lw=1)
        ax[3].set_xlabel('s since trial start')
        fig.suptitle(f'{rel} -- residual triangle evidence vs ground truth', fontsize=11)
        fig.tight_layout(rect=[0, 0, 1, 0.97])
        fig.savefig(os.path.join(args.output_dir, f"triangle_{rel.replace('/', '__')}.png"), dpi=110)
        plt.close(fig)
    with open(os.path.join(args.output_dir, 'residual_triangle_summary.json'), 'w') as f:
        json.dump({'nominal_q90': {k: v.tolist() for k, v in q.items()}, 'nominal_p99': sig,
                   'trials': results}, f, indent=2, default=str)


if __name__ == '__main__':
    main()
