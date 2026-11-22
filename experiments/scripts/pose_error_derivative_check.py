"""Check the pose-error dynamics at the derivative level, against AWSIM truth.

The 2b estimator integrates delta_dot = f(delta, gaps, v) (docs/architecture.md,
Formal model). Comparing the integrated delta with the truth mixes every rate
error with its whole history; this compares the RATES instead, sample by
sample, so each term of f can be judged on its own:

  truth   delta_dot from the true pose error (estimate - AWSIM truth, in the
          estimate's heading frame), central difference
  exact   f with the exact kinematics (incl. v_l and rotating-frame terms),
          driven by AWSIM's true twist -> must match truth (checks the algebra
          and the timing)
  report  same f, driven by the vehicle's own report (wheel speed, lateral
          speed, yaw rate) -> what an onboard system can actually compute
  code    the terms the filter uses (no r_hat / v_l terms), driven by the
          pipeline's own gaps (1 s backward differences, mid-window heading)

Then integrates each variant open loop from the fault onset to show how a
rate error becomes divergence. Raw bag topics only; no model, no Layer 1.

  python3 experiments/scripts/pose_error_derivative_check.py
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
DATA = os.path.join(ROOT, 'experiments', 'data')
GT_TOPIC = '/awsim/ground_truth/localization/kinematic_state'
EST_TOPIC = '/localization/kinematic_state'
VEL_TOPIC = '/vehicle/status/velocity_status'
DT = 0.1
sys.path.insert(0, os.path.join(ROOT, 'experiments', 'lib'))
from pose_gaps import L_CM  # noqa: E402  (centre-of-mass lever arm, see pose_gaps.py)


def read_bag(bag_dir):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=''), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    topics = [GT_TOPIC, EST_TOPIC, VEL_TOPIC]
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    msg_t = {k: get_message(types[k]) for k in topics}
    out = {k: [] for k in topics}
    while r.has_next():
        k, data, ts = r.read_next()
        m = deserialize_message(data, msg_t[k])
        ts = m.header.stamp.sec * 1e9 + m.header.stamp.nanosec   # sim time, same clock as fault_log
        if k == VEL_TOPIC:
            # AWSIM heading_rate has the opposite sign to ROS (findings.md 2)
            out[k].append((ts * 1e-9, m.longitudinal_velocity, m.lateral_velocity, -m.heading_rate))
        else:
            p, q, tw = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
            out[k].append((ts * 1e-9, p.x, p.y, yaw, tw.linear.x, tw.linear.y, tw.angular.z))
    return {k: np.array(v) for k, v in out.items()}


def fault_window(trial_dir, t_lo, t_hi):
    """First fault onset inside this bag (the fault_log is cumulative over a
    campaign): the first imu_bias_on whose sim time falls in [t_lo, t_hi]
    (imu_fault_activated only arms the injector; the fault starts at
    imu_bias_on). Fault phase = onset to the matching imu_bias_off, or the
    configured duration."""
    on, off, dur = None, None, 45.0
    with open(os.path.join(trial_dir, 'fault_log.jsonl')) as f:
        ev = [json.loads(x) for x in f if x.strip()]
    for e in ev:
        if e.get('event') == 'startup':
            dur = float(e.get('fault_duration') or dur)
    for e in ev:
        ts = e.get('sim_time_sec')
        if ts is None or not (t_lo <= ts <= t_hi):
            continue
        if e['event'] == 'imu_bias_on' and on is None:
            on = float(ts)
        elif e['event'] == 'imu_bias_off' and on is not None and off is None:
            off = float(ts)
    return on, ((off - on) if off is not None else dur) if on is not None else dur


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def resample(B):
    gt, est, vel = B[GT_TOPIC], B[EST_TOPIC], B[VEL_TOPIC]
    t0, t1 = max(gt[0, 0], est[0, 0], vel[0, 0]), min(gt[-1, 0], est[-1, 0], vel[-1, 0])
    t = np.arange(t0, t1, DT)
    def lin(a, cols):
        return np.stack([np.interp(t, a[:, 0], a[:, c]) for c in cols], -1)
    def yaw(a):
        return np.interp(t, a[:, 0], np.unwrap(a[:, 3]))
    return {'t': t,
            'gt_xy': lin(gt, [1, 2]), 'gt_yaw': yaw(gt), 'gt_tw': lin(gt, [4, 5, 6]),
            'est_xy': lin(est, [1, 2]), 'est_yaw': yaw(est),
            'vel': lin(vel, [1, 2, 3])}


def central(y, t, h=2):
    """Central difference over +/- h samples (needs >= 0.3 s: forward-filled
    pose jitters, findings.md 1)."""
    d = np.full_like(y, np.nan)
    d[h:-h] = (y[2 * h:] - y[:-2 * h]) / (t[2 * h:] - t[:-2 * h])[:, None] if y.ndim > 1 else \
        (y[2 * h:] - y[:-2 * h]) / (t[2 * h:] - t[:-2 * h])
    return d


def true_error(S):
    th = S['est_yaw']
    d = S['est_xy'] - S['gt_xy']
    x = d[:, 0] * np.cos(th) + d[:, 1] * np.sin(th)
    y = -d[:, 0] * np.sin(th) + d[:, 1] * np.cos(th)
    return np.stack([wrap(S['est_yaw'] - S['gt_yaw']), x, y], -1)


def est_rates(S, h=2):
    """Estimate's own yaw rate and velocity in its own heading frame."""
    t, th = S['t'], S['est_yaw']
    r_hat = central(th, t, h)
    pd = central(S['est_xy'], t, h)
    a = pd[:, 0] * np.cos(th) + pd[:, 1] * np.sin(th)
    l = -pd[:, 0] * np.sin(th) + pd[:, 1] * np.cos(th)
    return r_hat, a, l


def pipeline_gaps(S, report, yaw_lag=5, pos_lag=10, matched=False, lever=0.0):
    """The gaps as sequence_builder._add_pose_consistency computes them
    (causal backward differences, mid-window heading), at 10 Hz.
    matched: compare the pose difference with the report AVERAGED over the
    same window (the plain version compares a window-average with an
    instantaneous value, a lag of half the window: ~0.5 s x acceleration).
    lever: move the reported lateral velocity to base_link (v_l - lever * r)."""
    t, yaw, xy = S['t'], S['est_yaw'], S['est_xy']
    rep = report.copy()
    rep[:, 1] = rep[:, 1] - lever * rep[:, 2]
    n = len(t)
    g = np.zeros((n, 3))
    for j in range(n):
        a, b = max(0, j - yaw_lag), max(0, j - pos_lag)
        dty, dtp = t[j] - t[a], t[j] - t[b]
        pr = (yaw[j] - yaw[a]) / dty if dty > 0.05 else 0.0
        d = (xy[j] - xy[b]) / dtp if dtp > 0.05 else np.zeros(2)
        hm = 0.5 * (yaw[j] + yaw[b])
        al = d[0] * math.cos(hm) + d[1] * math.sin(hm)
        la = -d[0] * math.sin(hm) + d[1] * math.cos(hm)
        if matched:
            ry = rep[a + 1:j + 1, 2].mean() if j > a else rep[j, 2]
            rv = rep[b + 1:j + 1, :2].mean(0) if j > b else rep[j, :2]
        else:
            ry, rv = rep[j, 2], rep[j, :2]
        g[j] = [pr - ry, al - rv[0], la - rv[1]]
    return g


def step_gaps(S, report, lever=L_CM):
    """Per-step gaps: the estimate's change over ONE sample minus the report
    averaged over that same sample, lateral moved to base_link. Summed, the
    pose increments telescope, so the forward-fill jitter that forces 1 s
    windows on a rate cancels in the integral and nothing lags."""
    t, yaw, xy = S['t'], S['est_yaw'], S['est_xy']
    rep = report.copy()
    rep[:, 1] = rep[:, 1] - lever * rep[:, 2]
    dt = np.diff(t)
    hm = 0.5 * (yaw[1:] + yaw[:-1])
    d = np.diff(xy, axis=0) / dt[:, None]
    rm = 0.5 * (rep[1:] + rep[:-1])
    g = np.zeros((len(t), 3))
    g[1:, 0] = np.diff(yaw) / dt - rm[:, 2]
    g[1:, 1] = d[:, 0] * np.cos(hm) + d[:, 1] * np.sin(hm) - rm[:, 0]
    g[1:, 2] = -d[:, 0] * np.sin(hm) + d[:, 1] * np.cos(hm) - rm[:, 1]
    return g


def f_exact(delta, r_hat, a, l, v, vl, r):
    psi, x, y = delta.T
    return np.stack([r_hat - r,
                     (a - v) + v * (1 - np.cos(psi)) - vl * np.sin(psi) + r_hat * y,
                     (l - vl) + v * np.sin(psi) + vl * (1 - np.cos(psi)) - r_hat * x], -1)


def f_code(delta, g, v):
    psi = delta[:, 0]
    return np.stack([g[:, 0], g[:, 1] + v * (1 - np.cos(psi)), g[:, 2] + v * np.sin(psi)], -1)


def open_loop(rate_fn, delta0, n0, n1, t):
    """Integrate a rate model from delta0 at n0 using its OWN state in the
    nonlinear terms (as the filter does without LiDAR updates)."""
    d = np.array(delta0, float)
    out = [d.copy()]
    for k in range(n0 + 1, n1):
        d = d + np.nan_to_num(rate_fn(d[None], k)[0]) * (t[k] - t[k - 1])
        out.append(d.copy())
    return np.array(out)


def analyse(trial_dir, out_dir, horizon=10.0):
    B = read_bag(os.path.join(trial_dir, 'rosbag'))
    S = resample(B)
    t = S['t']
    on, dur = fault_window(trial_dir, t[0], t[-1])
    delta = true_error(S)
    delta_u = delta.copy(); delta_u[:, 0] = np.unwrap(delta[:, 0])
    truth = central(delta_u, t)
    r_hat, a, l = est_rates(S)
    gv, gvl, gr = S['gt_tw'][:, 0], S['gt_tw'][:, 1], S['gt_tw'][:, 2]
    vv, vvl, vr = S['vel'][:, 0], S['vel'][:, 1], S['vel'][:, 2]
    rates = {
        'exact': f_exact(delta, r_hat, a, l, gv, gvl, gr),
        'report': f_exact(delta, r_hat, a, l, vv, vvl, vr),
        'report_lever': f_exact(delta, r_hat, a, l, vv, vvl - L_CM * vr, vr),
    }
    g_pipe = pipeline_gaps(S, S['vel'])
    g_fix = pipeline_gaps(S, S['vel'], matched=True, lever=L_CM)
    rates['code'] = f_code(delta, g_pipe, vv)
    rates['code_fixed'] = f_code(delta, g_fix, vv)
    g_step = step_gaps(S, S['vel'])
    rates['step'] = f_code(delta, g_step, vv)
    # term split of the lateral rate (the advisor's case: psi ~ 0, offset y)
    terms_y = {'g_l': l - vvl, 'v sin psi': vv * np.sin(delta[:, 0]), '-r_hat x': -r_hat * delta[:, 1],
               'v_l(1-cos psi)': vvl * (1 - np.cos(delta[:, 0]))}

    fault = (t >= on) & (t <= on + dur) if on is not None else np.zeros(len(t), bool)
    pre = (t < on) if on is not None else np.ones(len(t), bool)
    moving = vv > 1.0
    res = {'trial': os.path.relpath(trial_dir, DATA), 'onset': on}
    for name, f in rates.items():
        e = f - truth
        for tag, m in (('pre', pre & moving), ('fault', fault)):
            ok = m & np.isfinite(e).all(1)
            res[f'{name}_{tag}_rms'] = np.sqrt(np.nanmean(e[ok] ** 2, 0)).tolist() if ok.any() else None
            res[f'{name}_{tag}_mean'] = np.nanmean(e[ok], 0).tolist() if ok.any() else None
    # sensor-report error alone (what the 'report' variant inherits)
    rep_err = np.stack([vr - gr, vv - gv, vvl - gvl], -1)
    res['report_minus_true_twist_fault_mean'] = np.nanmean(rep_err[fault], 0).tolist() if fault.any() else None
    res['report_minus_true_twist_pre_mean'] = np.nanmean(rep_err[pre & moving], 0).tolist()

    # open-loop divergence over the first `horizon` s of the fault
    if on is not None:
        n0 = int(np.searchsorted(t, on)); n1 = min(len(t), n0 + int(horizon / DT))
        ol = {}
        ol['exact'] = open_loop(lambda d, k: f_exact(d, r_hat[k:k+1], a[k:k+1], l[k:k+1], gv[k:k+1], gvl[k:k+1], gr[k:k+1]),
                                delta_u[n0], n0, n1, t)
        ol['report'] = open_loop(lambda d, k: f_exact(d, r_hat[k:k+1], a[k:k+1], l[k:k+1], vv[k:k+1], vvl[k:k+1], vr[k:k+1]),
                                 delta_u[n0], n0, n1, t)
        ol['report_lever'] = open_loop(lambda d, k: f_exact(d, r_hat[k:k+1], a[k:k+1], l[k:k+1], vv[k:k+1], vvl[k:k+1] - L_CM * vr[k:k+1], vr[k:k+1]),
                                       delta_u[n0], n0, n1, t)
        ol['code'] = open_loop(lambda d, k: f_code(d, g_pipe[k:k+1], vv[k:k+1]), delta_u[n0], n0, n1, t)
        ol['step'] = open_loop(lambda d, k: f_code(d, g_step[k:k+1], vv[k:k+1]), delta_u[n0], n0, n1, t)
        ol['code_fixed'] = open_loop(lambda d, k: f_code(d, g_fix[k:k+1], vv[k:k+1]), delta_u[n0], n0, n1, t)
        tr = delta_u[n0:n1]
        res['open_loop_end_true'] = tr[-1].tolist()
        for k, v in ol.items():
            res[f'open_loop_end_{k}'] = v[-1].tolist()
            res[f'open_loop_maxerr_{k}'] = np.abs(v - tr).max(0).tolist()
        plot(trial_dir, out_dir, t, on, dur, delta_u, truth, rates, terms_y, ol, n0, n1)
    return res


VARIANTS = (('exact', 'tab:green'), ('report', 'tab:blue'), ('report_lever', 'tab:cyan'),
            ('code', 'tab:red'), ('code_fixed', 'tab:orange'), ('step', 'tab:purple'))


def plot(trial_dir, out_dir, t, on, dur, delta, truth, rates, terms_y, ol, n0, n1):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rel = t - on
    m = (rel > -10) & (rel < min(dur, 40))
    fig, ax = plt.subplots(4, 3, figsize=(15, 12), sharex='col')
    names = ['psi (deg)', 'x along (m)', 'y lateral (m)']
    sc = [np.degrees(1), 1, 1]
    for c in range(3):
        ax[0, c].plot(rel[m], delta[m, c] * sc[c], 'k', label='true error')
        ax[0, c].set_title(names[c])
        ax[1, c].plot(rel[m], truth[m, c] * sc[c], 'k', lw=1.5, label='true rate')
        for k, col in VARIANTS:
            ax[1, c].plot(rel[m], rates[k][m, c] * sc[c], color=col, lw=0.8, label=k)
            ax[2, c].plot(rel[m], (rates[k][m, c] - truth[m, c]) * sc[c], color=col, lw=0.8, label=k)
        tt = t[n0:n1] - on
        ax[3, c].plot(tt, delta[n0:n1, c] * sc[c], 'k', lw=1.5, label='true')
        for k, col in VARIANTS:
            ax[3, c].plot(tt, ol[k][:, c] * sc[c], color=col, lw=0.8, label=k)
    ax[1, 0].set_ylabel('rate (/s)'); ax[2, 0].set_ylabel('rate error (model - true)')
    ax[3, 0].set_ylabel('open loop from onset')
    ax[0, 2].twinx()
    for k, v in terms_y.items():
        ax[0, 2].plot(rel[m], np.nan_to_num(v[m]), lw=0.6, alpha=0.6, label=f'ydot term: {k}')
    for a_ in ax.ravel():
        a_.axvline(0, color='grey', ls=':')
        a_.grid(alpha=0.3)
    ax[1, 0].legend(fontsize=7); ax[0, 2].legend(fontsize=6); ax[3, 0].legend(fontsize=7)
    for c in range(3):
        ax[3, c].set_xlabel('s since fault onset')
    name = os.path.relpath(trial_dir, DATA).replace(os.sep, '__')
    fig.suptitle(f'{name}: pose-error dynamics at the derivative level', fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, f'deriv_{name}.png'), dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--campaigns', nargs='*', default=None, help='default: every IMU campaign')
    ap.add_argument('--horizon', type=float, default=10.0, help='open-loop integration length from onset (s)')
    ap.add_argument('--output-dir', default=os.path.join(ROOT, 'experiments', 'analysis', 'pose_error_derivative'))
    args = ap.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)
    from trials import campaigns, trial_dirs
    out = []
    for c in args.campaigns or campaigns('imu'):
        for d in trial_dirs(c):
            print(d, flush=True)
            out.append(analyse(d, args.output_dir, args.horizon))
    with open(os.path.join(args.output_dir, 'summary.json'), 'w') as f:
        json.dump(out, f, indent=1)
    fmt = lambda v, s=(np.degrees(1), 1, 1): '/'.join(f'{a * b:.2f}' for a, b in zip(v, s)) if v else '-'
    names = [k for k, _ in VARIANTS]
    print('\nrate error RMS during the fault (psi deg/s / x m/s / y m/s)')
    print(f"{'trial':45s} " + ' '.join(f'{k:>17s}' for k in names))
    for r in out:
        print(f"{r['trial']:45s} " + ' '.join(f"{fmt(r[k + '_fault_rms']):>17s}" for k in names))
    print(f'\nopen loop, {args.horizon:.0f} s from onset: max |model - true| (psi deg / x m / y m); true end error first')
    print(f"{'trial':45s} {'true end':>17s} " + ' '.join(f'{k:>17s}' for k in names))
    for r in out:
        if 'open_loop_end_true' in r:
            print(f"{r['trial']:45s} {fmt(r['open_loop_end_true']):>17s} " +
                  ' '.join(f"{fmt(r['open_loop_maxerr_' + k]):>17s}" for k in names))
    print('\nvehicle report - true twist, mean (yaw rate deg/s / v m/s / v_l m/s): pre | fault')
    for r in out:
        print(f"{r['trial']:45s} {fmt(r['report_minus_true_twist_pre_mean'])} | {fmt(r['report_minus_true_twist_fault_mean'])}")

if __name__ == '__main__':
    main()
