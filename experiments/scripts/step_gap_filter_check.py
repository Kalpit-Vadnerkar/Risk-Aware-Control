"""2b pose-error filter driven by corrected per-step gaps vs the consistency
heads' residuals, against AWSIM truth (follow-up of
pose_error_derivative_check.py).

Inputs to the same pose_error_filter (layer2_envelope.py), per Layer 1 window:
  head   the ego_v6 consistency heads' residuals (current method)
  step   per-step gaps from the raw bag: the estimate's pose increment between
         consecutive windows minus the vehicle report averaged over that
         interval, lateral moved to base_link (lever arm L_CM); minus the
         nominal mean (leave-one-trial-out)
Each variant gets its own process noise (nominal drift bands of its own
integrator) so neither is judged with the other's noise model; the LiDAR
health weights and R0 are shared.

Scores (truth = estimate - AWSIM truth in the estimate's heading frame):
  nominal  |delta_hat - delta_true| p50/p99, share of windows outside the
           nominal band (I2)
  faults   |delta_true - delta_hat| at departure / max over fault (I6),
           Autoware's own error for reference

Needs pose_traces.pkl (extract_pose_error_traces.py).
  STGAT_GRAPH_FRAME=ego_v6 python3 experiments/scripts/step_gap_filter_check.py
"""
import glob
import json
import os
import pickle
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import layer2_envelope as LE  # noqa: E402
import pose_error_derivative_check as PD  # noqa: E402
from inspect_fault_predictions import compute_window_times  # noqa: E402
from st_gat.pipeline.bag_reader import read_bag as read_frames  # noqa: E402
from st_gat.pipeline.sequence_builder import SequenceBuilder  # noqa: E402
cfg = LE.cfg
W = lambda a: (a + np.pi) % (2 * np.pi) - np.pi  # noqa: E731


def read_bag_rx(bag_dir):
    """Est pose, AWSIM truth and the vehicle report on the bag's receive
    clock (the clock of the Layer 1 window times)."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=''), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    topics = [PD.GT_TOPIC, PD.EST_TOPIC, PD.VEL_TOPIC]
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    mt = {k: get_message(types[k]) for k in topics}
    out = {k: [] for k in topics}
    while r.has_next():
        k, data, ts = r.read_next()
        m = deserialize_message(data, mt[k])
        if k == PD.VEL_TOPIC:
            out[k].append((ts * 1e-9, m.longitudinal_velocity, m.lateral_velocity, -m.heading_rate))
        else:
            p, q = m.pose.pose.position, m.pose.pose.orientation
            yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
            out[k].append((ts * 1e-9, p.x, p.y, yaw))
    return {k: np.array(v) for k, v in out.items()}


def step_gaps_at(B, ta):
    """Per-step gaps at absolute window times ta, plus the true pose error."""
    est, vel, gt = B[PD.EST_TOPIC], B[PD.VEL_TOPIC], B[PD.GT_TOPIC]
    ey = np.unwrap(est[:, 3])
    exy = np.stack([np.interp(ta, est[:, 0], est[:, c]) for c in (1, 2)], -1)
    eyaw = np.interp(ta, est[:, 0], ey)
    # report averaged over each interval: cumulative trapezoid, then differences
    rep = vel[:, 1:].copy()
    rep[:, 1] -= PD.L_CM * rep[:, 2]
    cum = np.concatenate([np.zeros((1, 3)), np.cumsum(0.5 * (rep[1:] + rep[:-1]) * np.diff(vel[:, 0])[:, None], 0)])
    C = np.stack([np.interp(ta, vel[:, 0], cum[:, c]) for c in range(3)], -1)
    g = np.zeros((len(ta), 3))
    dt = np.diff(ta)
    ok = dt > 1e-3
    hm = 0.5 * (eyaw[1:] + eyaw[:-1])
    d = np.diff(exy, axis=0)
    dC = np.diff(C, axis=0)
    g1 = np.stack([np.diff(eyaw) - dC[:, 2],
                   d[:, 0] * np.cos(hm) + d[:, 1] * np.sin(hm) - dC[:, 0],
                   -d[:, 0] * np.sin(hm) + d[:, 1] * np.cos(hm) - dC[:, 1]], -1)
    g[1:][ok] = g1[ok] / dt[ok, None]
    # windows across a frame gap (> 0.3 s): no increment (the filter clips dt too)
    g[1:][dt > 0.3] = 0.0
    # true error
    gy = np.interp(ta, gt[:, 0], np.unwrap(gt[:, 3]))
    gxy = np.stack([np.interp(ta, gt[:, 0], gt[:, c]) for c in (1, 2)], -1)
    dd = exy - gxy
    tx = dd[:, 0] * np.cos(eyaw) + dd[:, 1] * np.sin(eyaw)
    tl = -dd[:, 0] * np.sin(eyaw) + dd[:, 1] * np.cos(eyaw)
    return g, np.stack([W(eyaw - gy), tx, tl], -1)


def bands_from(series):
    """Nominal 95% |.| of the plain integral of (psi, x, y) rates vs time since
    start (starts every 10 s, up to 60 s): the drift_band format."""
    grid = np.arange(1, 601) * LE.DT
    acc = {k: [[] for _ in grid] for k in ('Psi', 'X', 'Y')}
    for t, g in series:
        for s0 in range(0, len(t), 100):
            tt, gg = t[s0:s0 + 600], g[s0:s0 + 600]
            dt = np.r_[0, np.clip(np.diff(tt), 0, 0.3)]
            I = np.cumsum(gg * dt[:, None], 0)
            for i in range(len(tt)):
                for c, k in enumerate(('Psi', 'X', 'Y')):
                    acc[k][i].append(abs(I[i, c]))
    b = {'tau': grid}
    for k in acc:
        v = np.array([np.quantile(a, 0.95) if len(a) else np.nan for a in acc[k]])
        b[k] = np.maximum.accumulate(np.nan_to_num(v, nan=np.nanmax(v)))
    return b


def run_filter(t, g, v, w, cal, bands):
    c = dict(cal); c['bands'] = bands
    d, sd = LE.pose_error_filter(t, g, v, w, c, r_hat=np.zeros(len(t)))
    return d, sd


def find_run(name):
    m = glob.glob(os.path.join(LE.L2.DATA_DIR, 'nom*', 'goal_*', name))
    return m[0] if m else None


def main():
    out_dir = os.path.join(cfg.ANALYSIS_ROOT, 'layer2_envelope')
    d = pickle.load(open(os.path.join(out_dir, 'pose_traces.pkl'), 'rb'))
    cal = d['cal']
    lq = np.concatenate([n['lq'] for n in d['nominal']])
    cal['lq_ref'] = (np.quantile(lq[:, 0], .5), np.quantile(lq[:, 0], .99), np.quantile(lq[:, 1], .01), np.quantile(lq[:, 1], .5))
    ic = list(cal['int_ch'])
    names = sorted(f[:-4] for f in os.listdir(cfg.CAL_DIR) if f.endswith('.pkl'))
    assert len(names) == len(d['nominal']), (len(names), len(d['nominal']))

    # nominal: window times from the same frame extraction the sequences used
    nom = []
    for name, n in zip(names, d['nominal']):
        run = find_run(name)
        frames = read_frames(os.path.join(run, 'rosbag'), goal_id=os.path.basename(os.path.dirname(run)), verbose=False)
        ta = np.array(compute_window_times(frames, SequenceBuilder._MAX_FRAME_GAP_SEC))
        if len(ta) != len(n['t']):
            print(f'  {name}: {len(ta)} window times vs {len(n["t"])} windows, skipped')
            continue
        g, true = step_gaps_at(read_bag_rx(os.path.join(run, 'rosbag')), ta)
        nom.append({'name': name, 't': ta - ta[0], 'g': g, 'true': true, 'n': n})
        print(f'  nominal {name}: {len(ta)} windows', flush=True)

    # leave-one-trial-out nominal mean of the step gaps (moving only)
    def bias_excl(i):
        G = np.concatenate([x['g'][x['n']['v'] > 1.0] for j, x in enumerate(nom) if j != i])
        return G.mean(0)
    bias_all = np.concatenate([x['g'][x['n']['v'] > 1.0] for x in nom]).mean(0)
    print('nominal mean step gap (deg/s, m/s, m/s):', np.degrees(bias_all[0]), bias_all[1:])

    bands_step = bands_from([(x['t'], x['g'] - bias_all) for x in nom])
    bands_head = cal['bands']
    for nm, b in (('head', bands_head), ('step', bands_step)):
        print(f'  {nm} band at 3 / 10 s: heading {np.degrees(LE.drift_band(b, 3, "Psi")):.2f} / '
              f'{np.degrees(LE.drift_band(b, 10, "Psi")):.2f} deg, lateral {LE.drift_band(b, 3, "Y"):.2f} / '
              f'{LE.drift_band(b, 10, "Y"):.2f} m')

    res = {'nominal': {}, 'faults': []}
    # nominal scores
    for var in ('head', 'step'):
        err, out_band, out_fixed, mags = [], [], [], []
        for i, x in enumerate(nom):
            n = x['n']
            w = LE.reference_health(n['lq'], n['D'], cal)
            if var == 'head':
                dh, _ = run_filter(n['t'], n['e'][:, ic], n['v'], w, cal, bands_head)
                b = bands_head
            else:
                dh, _ = run_filter(x['t'], x['g'] - bias_excl(i), n['v'], w, cal, bands_step)
                b = bands_step
            e = np.stack([W(dh[:, 0] - x['true'][:, 0]), dh[:, 2] - x['true'][:, 2]], -1)
            err.append(np.abs(e))
            out_band.append((np.abs(dh[:, 0]) > LE.drift_band(b, 3, 'Psi')) | (np.abs(dh[:, 2]) > LE.drift_band(b, 3, 'Y')))
            # same yardstick for both: the current (head) band
            out_fixed.append((np.abs(dh[:, 0]) > LE.drift_band(bands_head, 3, 'Psi')) | (np.abs(dh[:, 2]) > LE.drift_band(bands_head, 3, 'Y')))
            mags.append(np.abs(dh[:, [0, 2]]))
        E = np.concatenate(err); O = np.concatenate(out_band)
        res['nominal'][var] = {'psi_p50_deg': float(np.degrees(np.median(E[:, 0]))), 'psi_p99_deg': float(np.degrees(np.quantile(E[:, 0], .99))),
                               'lat_p50_m': float(np.median(E[:, 1])), 'lat_p99_m': float(np.quantile(E[:, 1], .99)),
                               'outside_own_band': float(O.mean()),
                               'outside_head_band': float(np.concatenate(out_fixed).mean()),
                               'est_psi_p999_deg': float(np.degrees(np.quantile(np.concatenate(mags)[:, 0], .999))),
                               'est_lat_p999_m': float(np.quantile(np.concatenate(mags)[:, 1], .999))}
    T = np.concatenate([np.abs(x['true']) for x in nom])
    res['nominal']['autoware'] = {'psi_p50_deg': float(np.degrees(np.median(T[:, 0]))), 'psi_p99_deg': float(np.degrees(np.quantile(T[:, 0], .99))),
                                  'lat_p50_m': float(np.median(T[:, 2])), 'lat_p99_m': float(np.quantile(T[:, 2], .99))}

    # faults
    for f in d['faults']:
        run = os.path.join(LE.L2.DATA_DIR, f['trial'])
        ta = f['t'] + f['t0']
        g, true = step_gaps_at(read_bag_rx(os.path.join(run, 'rosbag')), ta)
        w = LE.reference_health(f['lq'], f['D'], cal)
        dh_h, _ = run_filter(f['t'], f['e'][:, ic], f['v'], w, cal, bands_head)
        dh_s, _ = run_filter(f['t'], g - bias_all, f['v'], w, cal, bands_step)
        fs = min(x['start'] for x in f['fault_windows'])
        fe = f['departure'] if f['departure'] else max(x['end'] for x in f['fault_windows'])
        m = (f['t'] >= fs) & (f['t'] <= fe)
        r = {'trial': f['trial'], 'departure': f['departure']}
        i = int(np.argmin(np.abs(f['t'] - f['departure']))) if f['departure'] else None
        for nm, dh in (('autoware', np.zeros_like(true)), ('head', dh_h), ('step', dh_s)):
            ep = np.degrees(np.abs(W(true[:, 0] - dh[:, 0])))
            el = np.abs(true[:, 2] - dh[:, 2])
            r[nm] = {'psi_max': float(ep[m].max()), 'lat_max': float(el[m].max()),
                     'psi_dep': float(ep[i]) if i is not None else None, 'lat_dep': float(el[i]) if i is not None else None}
        res['faults'].append(r)
        f['_step'] = dh_s; f['_head'] = dh_h; f['_true'] = true

    with open(os.path.join(out_dir, 'step_gap_filter_check.json'), 'w') as fh:
        json.dump(res, fh, indent=1)
    print('\nNOMINAL (17 cal trials): |estimate - true pose error|')
    for k, v in res['nominal'].items():
        print(f'  {k:9s} ' + ', '.join(f'{a} {b:.3f}' for a, b in v.items()))
    print('\nFAULTS: error left after correction (heading deg / lateral m); at departure, else max over fault')
    print(f"{'trial':45s} {'autoware':>14s} {'head':>14s} {'step':>14s}")
    for r in res['faults']:
        key = ('psi_dep', 'lat_dep') if r['departure'] else ('psi_max', 'lat_max')
        tag = 'dep' if r['departure'] else 'max'
        print(f"{r['trial'][:45]:45s} " + ' '.join(f"{r[k][key[0]]:6.1f}/{r[k][key[1]]:5.2f}  " for k in ('autoware', 'head', 'step')) + tag)
    plot(d['faults'], out_dir)


def plot(faults, out_dir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, len(faults), figsize=(3.2 * len(faults), 6), sharex='col')
    for c, f in enumerate(faults):
        fs = min(x['start'] for x in f['fault_windows'])
        end = f['departure'] + 3 if f['departure'] else fs + 45
        m = (f['t'] >= fs - 5) & (f['t'] <= end)
        t = f['t'][m]
        tr = f['_true'][m]
        for r, (k, sc, u) in enumerate(((0, np.degrees(1), 'deg'), (2, 1, 'm'))):
            a = ax[r, c]
            raw = np.abs(W(tr[:, 0])) * sc if k == 0 else np.abs(tr[:, k])
            a.plot(t, raw, color='#555', lw=1.4, label="Autoware's error")
            for nm, col in (('_head', '#9467bd'), ('_step', '#2ca02c')):
                dh = f[nm][m]
                left = np.abs(W(tr[:, 0] - dh[:, 0])) * sc if k == 0 else np.abs(tr[:, k] - dh[:, k])
                a.plot(t, left, color=col, lw=1.4, label='after correction: ' + ('head residuals' if nm == '_head' else 'corrected step gaps'))
            if f['departure']:
                a.axvline(f['departure'], color='#d62728', ls='--')
            a.set_yscale('symlog', linthresh=1 if k == 0 else 0.2)
            a.grid(alpha=0.3)
        ax[0, c].set_title(f['trial'].split('/t1')[0].replace('imu_fault_', ''), fontsize=9)
    ax[0, 0].set_ylabel('|heading error| deg'); ax[1, 0].set_ylabel('|lateral error| m')
    h, l = ax[0, 0].get_legend_handles_labels()
    fig.legend(h, l, loc='lower center', ncol=3, frameon=False)
    fig.tight_layout(rect=[0, 0.05, 1, 1])
    p = os.path.join(out_dir, 'step_gap_filter_check.png')
    fig.savefig(p, dpi=110)
    print('saved', p)


if __name__ == '__main__':
    main()
