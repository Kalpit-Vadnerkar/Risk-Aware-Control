"""The 5 ego_v6 consistency gaps under nominal and faulty driving.

Rows: the 5 gaps (stack estimate minus vehicle report). Columns: one nominal
calibration trial (around its sharpest turn), the harmless IMU fault
(imu_fault_s1) and three IMU faults that end in a road departure. Grey: the
observed gap (the model input). Purple: the residual, observed minus the
consistency head's predicted nominal gap (what Layer 1 scores and the 2b
filter integrates). Band: the nominal 99% residual quantile (leave-one-trial-
out calibration). Orange shading: fault active; red line: real departure.

Reads <ANALYSIS_ROOT>/layer2_envelope/pose_traces.pkl
(extract_pose_error_traces.py).

  python3 experiments/scripts/plot_consistency_gaps.py [--traces PKL] [--out PNG]
"""
import argparse
import os
import pickle

import matplotlib.pyplot as plt
import numpy as np

ROOT = os.path.join(os.path.dirname(__file__), '..', 'analysis', 'h30_30_ego6')
CH = [('1  EKF yaw rate\n− vehicle yaw rate', 'deg/s', np.degrees),
      ('2  EKF speed\n− wheel speed', 'm/s', lambda x: x),
      ('3  pose yaw rate\n− vehicle yaw rate', 'deg/s', np.degrees),
      ('4  pose along speed\n− wheel speed', 'm/s', lambda x: x),
      ('5  pose lateral speed\n− vehicle lateral', 'm/s', lambda x: x)]
FAULTS = [('imu_fault_s1/goal_007', 'IMU bias 0.03 rad/s (s1)\nharmless'),
          ('imu_fault_s3/goal_007', 'IMU bias 0.08 rad/s (s3)\nleaves the road'),
          ('imu_fault_scale/goal_007', 'IMU scale ×1.8\nleaves the road'),
          ('imu_fault_stuck/goal_007', 'IMU stuck\nleaves the road')]


def nominal_window(n, span=60.0):
    """The span-second stretch of a nominal trial with the largest heading change."""
    y, t = np.unwrap(n['yaw']), n['t']
    best, lo = -1, t[0]
    for i in range(0, len(t), 10):
        m = (t >= t[i]) & (t < t[i] + span)
        if m.sum() > 0.8 * span * 10 and np.ptp(y[m]) > best:
            best, lo = np.ptp(y[m]), t[i]
    return lo, lo + span


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--traces', default=os.path.join(ROOT, 'layer2_envelope', 'pose_traces.pkl'))
    ap.add_argument('--nominal-trial', type=int, default=0)
    ap.add_argument('--out', default=os.path.join(ROOT, 'highlights', 'consistency_gaps.png'))
    args = ap.parse_args()
    d = pickle.load(open(args.traces, 'rb'))
    q99 = np.asarray(d['cal']['q99'])
    nom = [n for n in d['nominal'] if n['trial'] == args.nominal_trial][0]
    byname = {f['trial'].rsplit('/', 1)[0]: f for f in d['faults']}

    cols = [('nominal', f'Nominal driving\n(calibration trial {args.nominal_trial}, sharpest turn)', nom,
             nominal_window(nom), [], None)]
    for key, label in FAULTS:
        f = byname[key]
        fw = [(w['start'], w['end']) for w in f['fault_windows']]
        dep = f['departure']
        if dep is not None:
            first = [s for s, e in fw if s < dep][-1]
            win = (first - 12, dep + 4)
        else:
            win = (fw[0][0] - 12, fw[0][1] + 20)
        cols.append((key, label, f, win, fw, dep))

    fig, axs = plt.subplots(5, len(cols), figsize=(4.1 * len(cols), 12.5), sharex='col', squeeze=False)
    for j, (key, label, tr, (lo, hi), fw, dep) in enumerate(cols):
        t = tr['t']
        m = (t >= lo) & (t <= hi)
        tt = t[m] - lo
        for i, (name, unit, conv) in enumerate(CH):
            a = axs[i, j]
            g, e = conv(tr['g_raw'][m, i]), conv(tr['e'][m, i])
            b = conv(q99[i])
            a.axhspan(-b, b, color='0.88', zorder=0)
            for s, en in fw:
                if en < lo or s > hi:
                    continue
                a.axvspan(max(s, lo) - lo, min(en, hi) - lo, color='#F5A623', alpha=0.18, lw=0, zorder=0)
            if dep is not None:
                a.axvline(dep - lo, color='#d62728', lw=1.5)
            a.plot(tt, g, color='0.45', lw=0.8)
            a.plot(tt, e, color='#6B3FA0', lw=1.2)
            out = np.abs(e) > b
            if fw:
                inf = np.zeros_like(out)
                for s, en in fw:
                    inf |= (t[m] >= s) & (t[m] <= en)
                pct = 100 * np.nanmean(out[inf]) if inf.any() else np.nan
                a.text(0.02, 0.95, f'{pct:.0f}% of fault time outside band', transform=a.transAxes,
                       fontsize=9, va='top', color='#6B3FA0')
            else:
                a.text(0.02, 0.95, f'{100 * np.nanmean(out):.1f}% outside band', transform=a.transAxes,
                       fontsize=9, va='top', color='#6B3FA0')
            lim = max(3 * b, np.nanpercentile(np.abs(np.r_[g, e]), 99.5) * 1.15)
            a.set_ylim(-lim, lim)
            if j == 0:
                a.set_ylabel(f'{name} [{unit}]', fontsize=11)
            if i == 0:
                a.set_title(label, fontsize=11)
            if i == 4:
                a.set_xlabel('seconds')
            a.tick_params(labelsize=8)
    h = [plt.Line2D([], [], color='0.45', lw=1, label='observed gap (model input)'),
         plt.Line2D([], [], color='#6B3FA0', lw=1.5, label='residual = observed − predicted nominal gap'),
         plt.Rectangle((0, 0), 1, 1, color='0.88', label='nominal 99% residual band'),
         plt.Rectangle((0, 0), 1, 1, color='#F5A623', alpha=0.3, label='fault active'),
         plt.Line2D([], [], color='#d62728', lw=1.5, label='real departure')]
    fig.legend(handles=h, loc='lower center', ncol=5, fontsize=12, frameon=False)
    fig.suptitle('The 5 consistency gaps (stack estimate − vehicle report), nominal vs IMU faults', fontsize=15)
    fig.tight_layout(rect=(0, 0.035, 1, 0.97))
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    fig.savefig(args.out, dpi=130)
    print(args.out)


if __name__ == '__main__':
    main()
