"""Offline correction: Autoware's pose error vs the error left after
subtracting the 2b pose-error estimate (formal-model filter), against AWSIM
ground truth. Open loop: the vehicle did not use the correction.
Writes <ANALYSIS_ROOT>/highlights/correction_offline.png.
Needs pose_traces.pkl (extract_pose_error_traces.py)."""
import os
import pickle
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(__file__))
import layer2_envelope as LE  # noqa: E402
cfg = LE.cfg
W = lambda a: (a + np.pi) % (2 * np.pi) - np.pi  # noqa: E731


def main():
    src = os.path.join(cfg.ANALYSIS_ROOT, 'layer2_envelope', 'pose_traces.pkl')
    d = pickle.load(open(src, 'rb'))
    cal = d['cal']
    lq = np.concatenate([n['lq'] for n in d['nominal']])
    cal['lq_ref'] = (np.quantile(lq[:, 0], .5), np.quantile(lq[:, 0], .99), np.quantile(lq[:, 1], .01), np.quantile(lq[:, 1], .5))
    ic = list(cal['int_ch'])
    order = ['scale/goal_007', 's3/goal_007', 'stuck/goal_007', 'stuck/goal_012', 's1/goal_007', 'scale/goal_012']
    sel = [next(f for f in d['faults'] if f['trial'].startswith('imu_fault_' + o)) for o in order]
    fig, ax = plt.subplots(2, len(sel), figsize=(3.6 * len(sel), 6.4), sharex='col')
    for c, f in enumerate(sel):
        w = LE.reference_health(f['lq'], f['D'], cal)
        de = LE.pose_error_filter(f['t'], f['e'][:, ic], f['v'], w, cal, r_hat=np.zeros(len(f['t'])))[0]
        fs = min(x['start'] for x in f['fault_windows'])
        end = f['departure'] + 3 if f['departure'] else fs + 40
        m = (f['t'] >= fs - 5) & (f['t'] <= end)
        t = f['t'][m]
        for r, (raw, cor, unit) in enumerate([(np.degrees(np.abs(W(f['true_psi'][m]))), np.degrees(np.abs(W(f['true_psi'][m] - de[m, 0]))), '°'),
                                              (np.abs(f['true_lat'][m]), np.abs(f['true_lat'][m] - de[m, 2]), ' m')]):
            a = ax[r, c]
            for x in f['fault_windows']:
                a.axvspan(x['start'], x['end'], color='#ff7f0e', alpha=0.12, lw=0)
            a.plot(t, raw, color='#555', lw=1.6, label="Autoware's own estimate")
            a.plot(t, cor, color='#9467bd', lw=2.0, label='after subtracting the 2b estimate')
            if f['departure']:
                a.axvline(f['departure'], color='#d62728', ls='--', lw=1.2)
                i = np.argmin(abs(t - f['departure']))
                a.text(0.03, 0.92, f"at departure: {raw[i]:.1f}{unit} → {cor[i]:.1f}{unit}", transform=a.transAxes,
                       fontsize=9.5, va='top', bbox=dict(fc='white', ec='none', alpha=0.8))
            a.set_xlim(t[0], t[-1])
        name = f['trial'].split('/')
        ax[0, c].set_title(f"{name[0].replace('imu_fault_', 'IMU ')} / {name[1]}" + ('  (departs)' if f['departure'] else '  (harmless)'),
                           fontsize=10)
        ax[1, c].set_xlabel('s since trial start')
    ax[0, 0].set_ylabel('|heading error| (deg)')
    ax[1, 0].set_ylabel('|lateral error| (m)')
    h, l = ax[0, 0].get_legend_handles_labels()
    fig.legend(h, l, loc='lower center', ncol=2, fontsize=11, frameon=False)
    fig.suptitle('Offline correction against AWSIM ground truth (open loop: orange = fault active, red = real departure)',
                 fontsize=12)
    fig.tight_layout(rect=[0, 0.06, 1, 0.95])
    out = os.path.join(cfg.ANALYSIS_ROOT, 'highlights', 'correction_offline.png')
    fig.savefig(out, dpi=130)
    print('saved', out)


if __name__ == '__main__':
    main()
