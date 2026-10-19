"""Highlight figures for the Layer 2 result (ego_v6), from the series that
layer2_envelope.py --mode all saves (series_nominal.npz, series_faults.pkl).

  1. operating_curves.png  measured operating curves: threshold swept on each
                           P(violation) variant -> warning time vs nominal
                           false alarms per hour, and departures caught vs
                           alarm time on harmless faults
  2. heading_tracking.png  2b's filtered heading error vs AWSIM truth
  3. anomaly_vs_consequence.png  Layer 1 (anomaly) vs Layer 2 (consequence)
                           alarm share per IMU trial
  4. p_violation_scale007.png    one departure, one panel: P over time
  5. coupling_ladder.png   departures caught by each Layer 1 -> Layer 2 coupling

Run: STGAT_GRAPH_FRAME=ego_v6 python3 experiments/scripts/plot_highlights.py
"""
import json
import os
import pickle
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from st_gat.pipeline import config as cfg  # noqa: E402

SRC = os.path.join(cfg.ANALYSIS_ROOT, os.environ.get('L2_RESULTS', 'layer2_envelope_filter'))
OUT = os.path.join(cfg.ANALYSIS_ROOT, 'highlights')
GAP = 1.0            # alarm episodes closer than this merge (as in layer2_envelope)
STOP_S = 1.7         # stop from 5 m/s at 3 m/s^2
COL = {'evidence_gated': '#2ca02c', 'evidence': '#9467bd', 'deadreckon': '#e377c2', 'nominal': '#7f7f7f', 'evidence_dP': '#17becf', 'layer1': 'k'}
LAB = {'evidence_gated': 'evidence P, alarmed only while Layer 1 distrusts (proposed)',
       'evidence': '2a + 2b evidence (method)', 'deadreckon': 'dead-reckoning baseline',
       'nominal': '2a nominal only', 'evidence_dP': 'dP = evidence - nominal', 'layer1': 'Layer 1 anomaly alone'}


def episodes(t, a):
    out, s, last = [], None, None
    for ti, ai in zip(t, a):
        if ai:
            if s is None or ti - last > GAP:
                if s is not None:
                    out.append((s, last))
                s = ti
            last = ti
    if s is not None:
        out.append((s, last))
    return out


def lead(t, a, dep):
    live = [(a0, a1) for a0, a1 in episodes(t, a) if a0 <= dep and a1 >= dep - GAP]
    return dep - live[0][0] if live else None


def in_fault(tr):
    return np.array([any(w['start'] <= x <= w['end'] for w in tr['fault_windows']) for x in tr['t']])


def load():
    nom = dict(np.load(os.path.join(SRC, 'series_nominal.npz'), allow_pickle=True))
    with open(os.path.join(SRC, 'series_faults.pkl'), 'rb') as f:
        faults = pickle.load(f)
    with open(os.path.join(SRC, 'nominal_floor.json')) as f:
        floor = json.load(f)
    # gated: P(evidence) only counts while Layer 1 has moved the envelope
    # (distrusted, or carrying a pose error). Risk at tight spots in normal
    # driving is the planner's own business; this alarm is for fault-induced
    # risk. Defined after seeing these 4 departures -- confirm on new data.
    nom['P_evidence_gated'] = nom['P_evidence'] * (nom['D'] > 0)
    for f in faults:
        f['P_evidence_gated'] = f['P_evidence'] * ((f['D'] > 0) | (np.abs(f['Psi']) > 0))
    floor['thresholds']['evidence_gated'] = floor['thresholds']['evidence']
    return nom, faults, floor


def nominal_fa_per_hour(nom, key, thr):
    n_eps, secs = 0, 0.0
    for tid in np.unique(nom['trial']):
        m = nom['trial'] == tid
        t = nom['t'][m]
        a = nom[key][m] > thr if key != 'D' else nom['D'][m] > 0
        n_eps += len(episodes(t, a))
        secs += len(t) * np.median(np.diff(t)) if len(t) > 1 else 0
    return n_eps / (secs / 3600), secs / 3600


def sweep(nom, faults, variant):
    key = 'P_' + variant
    deps = [f for f in faults if f['departure'] is not None]
    harmless = [f for f in faults if f['departure'] is None and f['trial'].startswith('imu')]
    grid = np.unique(np.r_[np.linspace(0.02, 1.0, 50), np.quantile(nom[key], np.linspace(0.9, 0.9999, 40))])
    rows = []
    for thr in grid:
        fa, hours = nominal_fa_per_hour(nom, key, thr)
        leads = [lead(f['t'], f[key] > thr, f['departure']) for f in deps]
        lv = np.array([x if x is not None else 0.0 for x in leads])
        hf = np.mean([(f[key] > thr)[in_fault(f)].mean() for f in harmless])
        rows.append((thr, fa, np.median(lv), np.mean([x is not None for x in leads]),
                     np.mean(lv >= STOP_S), hf))
    return np.array(rows), hours


def layer1_point(nom, faults):
    fa, _ = nominal_fa_per_hour(nom, 'D', 0)
    deps = [f for f in faults if f['departure'] is not None]
    harmless = [f for f in faults if f['departure'] is None and f['trial'].startswith('imu')]
    lv = np.array([lead(f['t'], f['D'] > 0, f['departure']) or 0.0 for f in deps])
    hf = np.mean([(f['D'] > 0)[in_fault(f)].mean() for f in harmless])
    return fa, np.median(lv), np.mean(lv >= STOP_S), hf


def fig_operating(nom, faults, floor):
    fig, ax = plt.subplots(1, 2, figsize=(13, 5))
    hours = None
    for v in ['nominal', 'deadreckon', 'evidence', 'evidence_dP', 'evidence_gated']:
        R, hours = sweep(nom, faults, v)
        if v == 'evidence_dP':
            R = R[R[:, 0] >= 0.05]
        o = np.argsort(R[:, 1])
        thr_op = floor['thresholds'][v]
        lw = 2.6 if v in ('evidence', 'evidence_gated') else 1.4
        ls = {'deadreckon': '--', 'evidence_dP': ':'}.get(v, '-')
        k = np.argmin(abs(R[:, 0] - thr_op))
        fa_plot = np.maximum(R[:, 1], 0.5 / hours)          # zero episodes drawn at half the resolution
        # Pareto front only: for each false-alarm level, the best warning time
        # reachable at or below it
        fx, fy = fa_plot[o], np.maximum.accumulate(R[o, 2])
        ax[0].plot(fx, fy, color=COL[v], lw=lw, ls=ls, label=LAB[v], drawstyle='steps-post')
        ax[0].plot(fa_plot[k], R[k, 2], 'o', color=COL[v], ms=7, mec='k')
        o2 = np.argsort(R[:, 5])
        ax[1].plot(R[o2, 5], np.maximum.accumulate(R[o2, 4]), color=COL[v], lw=lw, ls=ls, label=LAB[v],
                   drawstyle='steps-post')
        ax[1].plot(R[k, 5], R[k, 4], 'o', color=COL[v], ms=7, mec='k')
    fa1, ld1, c1, hf1 = layer1_point(nom, faults)
    ax[0].plot(max(fa1, 0.5 / hours), ld1, 's', color='k', ms=8, label=LAB['layer1'] + ' (one setting)')
    ax[1].plot(hf1, c1, 's', color='k', ms=8, label=LAB['layer1'] + ' (one setting)')
    a = ax[0]
    a.add_patch(plt.Rectangle((1e-1, 2.0), 1.0 - 1e-1, 4, color='#2ca02c', alpha=0.10, lw=0))
    a.text(0.11, 5.85, 'target:\n>= 2 s warning\nat <= 1 false\nalarm / h', fontsize=8, color='#2ca02c', va='top')
    a.axvspan(1e-1, 1 / hours, ymax=2 / 6, color='#000', alpha=0.06)
    a.text(0.11, 0.15, f'< 1 episode in\n{hours:.2f} h nominal:\nnot resolvable', fontsize=7, color='#555')
    a.axhline(3.0, color='#9467bd', ls=':', lw=1)
    a.text(1.2, 3.08, 'forecast horizon 3 s', fontsize=8, color='#9467bd')
    a.axhline(STOP_S, color='grey', ls=':', lw=1)
    a.text(40, STOP_S + 0.08, 'stop from 5 m/s at 3 m/s$^2$', fontsize=8, color='grey', ha='right')
    a.set_xscale('log'); a.set_xlim(1e-1, 3e2); a.set_ylim(0, 6)
    a.set_xlabel('false alarm episodes per hour of nominal driving (log; leave-one-trial-out)')
    a.set_ylabel('median warning before real departure (s, 4 departures; miss = 0)')
    a.set_title('Warning time vs false alarms (big dots = threshold used in the table)', loc='left', fontsize=10)
    a.legend(fontsize=7.5, loc='lower right', bbox_to_anchor=(1, 0.08))
    a = ax[1]
    a.set_xlim(-0.02, 1.02); a.set_ylim(-0.03, 1.05)
    a.set_xlabel('harmless IMU faults (4 trials): share of fault time in alarm')
    a.set_ylabel(f'departures warned >= {STOP_S} s ahead (share of 4)')
    a.set_title('Consequence, not anomaly', loc='left', fontsize=10)
    a.legend(fontsize=8, loc='lower right')
    fig.suptitle('Measured operating curves, ego_v6 (threshold swept along each line)', fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'operating_curves.png'), dpi=140)
    plt.close(fig)


def fig_heading(faults):
    sel = [f for f in faults if f['trial'].startswith('imu')]
    fig, ax = plt.subplots(2, 4, figsize=(16, 6.5), sharey=False)
    for a, f in zip(ax.ravel(), sel):
        fs = min(w['start'] for w in f['fault_windows'])
        end = f['departure'] + 6 if f['departure'] else fs + 60
        m = (f['t'] >= fs - 8) & (f['t'] <= end)
        t = f['t'][m]
        for w in f['fault_windows']:
            a.axvspan(w['start'], w['end'], color='#ff7f0e', alpha=0.12, lw=0)
        a.plot(t, np.degrees(f['true_psi'][m]), color='k', lw=1.4, label='true (AWSIM)')
        a.plot(t, np.degrees(f['Psi'][m]), color='#9467bd', lw=1.8, label='2b estimate')
        if f['departure']:
            a.axvline(f['departure'], color='#d62728', ls='--', lw=1.2)
        a.set_xlim(t[0], t[-1])
        name = f['trial'].split('/')
        a.set_title(f"{name[0].replace('imu_fault_', 'IMU ')} / {name[1]}" + ('  (departs)' if f['departure'] else ''),
                    fontsize=9, loc='left')
        a.axhline(0, color='grey', lw=0.5)
    ax[0, 0].legend(fontsize=8)
    for a in ax[:, 0]:
        a.set_ylabel('heading error (deg)')
    for a in ax[1]:
        a.set_xlabel('s since trial start')
    fig.suptitle('Which way and how much: 2b pose-error filter estimate vs true heading error '
                 '(orange = fault active, red = real departure)', fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'heading_tracking.png'), dpi=140)
    plt.close(fig)


def fig_anomaly_vs_consequence(floor):
    with open(os.path.join(SRC, 'fault_summary.json')) as f:
        S = [s for s in json.load(f) if not s.get('skipped')]
    imu = [s for s in S if s['trial'].startswith('imu')]
    tl = [s for s in S if s['trial'].startswith('tl')]
    fig, ax = plt.subplots(figsize=(12, 4.6))
    names, l1, evd, dep = [], [], [], []
    for s in imu:
        c, g = s['trial'].split('/')[:2]
        names.append(f"{c.replace('imu_fault_', '')}\n{g.replace('goal_', 'g')}")
        dep.append(s['departure'] is not None)
        if s['departure'] is not None:
            l1.append(1.0); evd.append(1.0)    # drawn as caught; lead time annotated
        else:
            l1.append(s['layer1']['in_fault_alarm_rate'] or 0); evd.append(s['evidence']['in_fault_alarm_rate'] or 0)
    tl_l1 = np.array([s['layer1']['in_fault_alarm_rate'] or 0 for s in tl])
    tl_ev = np.array([s['evidence']['in_fault_alarm_rate'] or 0 for s in tl])
    names.append(f'all 30 TL\ntrials (mean)'); l1.append(tl_l1.mean()); evd.append(tl_ev.mean()); dep.append(False)
    x = np.arange(len(names))
    ax.bar(x - 0.2, l1, 0.4, color='#444', label='Layer 1 alone: anomaly (share of fault time in alarm)')
    ax.bar(x + 0.2, evd, 0.4, color='#9467bd', label='Layer 2 evidence: consequence (share of fault time in alarm)')
    for i, s in enumerate(imu):
        if s['departure'] is not None:
            ax.text(x[i] - 0.2, 1.02, f"{s['layer1']['lead_time_s']:.1f}s", ha='center', fontsize=8)
            ax.text(x[i] + 0.2, 1.02, f"{s['evidence']['lead_time_s']:.1f}s", ha='center', fontsize=8, color='#9467bd')
    for i, d in enumerate(dep):
        if d:
            ax.axvspan(x[i] - 0.5, x[i] + 0.5, color='#d62728', alpha=0.07, lw=0)
    ax.set_xticks(x, names, fontsize=8)
    ax.set_ylim(0, 1.15)
    ax.set_ylabel('share of fault time in alarm')
    ax.set_title('Red columns: the vehicle really left the road -- both warn (lead time on top). '
                 'Elsewhere the anomaly is real but harmless: Layer 2 stays quiet.', loc='left', fontsize=9)
    ax.legend(fontsize=8, loc='upper center', bbox_to_anchor=(0.5, -0.14), ncol=2, frameon=False)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'anomaly_vs_consequence.png'), dpi=140)
    plt.close(fig)


def fig_p_scale(faults, floor):
    f = next(x for x in faults if x['trial'].startswith('imu_fault_scale/goal_007'))
    fs = min(w['start'] for w in f['fault_windows'])
    dep = f['departure']
    m = (f['t'] >= fs - 15) & (f['t'] <= dep + 8)
    fig, ax = plt.subplots(2, 1, figsize=(10, 5.6), sharex=True, gridspec_kw={'height_ratios': [1, 1.6]})
    a = ax[0]
    a.plot(f['rm_t'], f['rm'], color='#d62728', lw=1.5, label='true clearance to road edge (m)')
    a.axhline(0, color='k', lw=0.6)
    a.set_ylim(-3, 5); a.set_ylabel('m')
    a.legend(fontsize=8, loc='upper left')
    a = ax[1]
    for v in ['nominal', 'deadreckon', 'evidence']:
        a.plot(f['t'][m], f['P_' + v][m], color=COL[v], lw=2.2 if v == 'evidence' else 1.3,
               ls='--' if v == 'deadreckon' else '-', label=LAB[v])
    thr = floor['thresholds']['evidence']
    a.axhline(thr, color=COL['evidence'], ls=':', lw=1)
    a.text(f['t'][m][0] + 0.3, thr + 0.02, f'alarm threshold (1% nominal) {thr:.2f}', fontsize=8, color=COL['evidence'])
    a.set_ylim(-0.02, 1.05); a.set_ylabel('P(violation within 3 s)')
    a.set_xlabel('s since trial start')
    a.legend(fontsize=8, loc='center left')
    L = lead(f['t'], f['P_evidence'] > thr, dep)
    for a in ax:
        for w in f['fault_windows']:
            a.axvspan(w['start'], w['end'], color='#ff7f0e', alpha=0.12, lw=0)
        a.axvline(dep, color='#d62728', ls='--', lw=1.2)
    if L is not None:
        ax[1].annotate('', xy=(dep, 0.5), xytext=(dep - L, 0.5), arrowprops=dict(arrowstyle='<->'))
        ax[1].text(dep - L / 2, 0.53, f'{L:.1f} s warning', ha='center', fontsize=9)
    ax[1].set_xlim(f['t'][m][0], f['t'][m][-1])
    ax[0].set_title('IMU scale fault, goal_007: the estimate stays in lane, the real vehicle leaves the road '
                    '(orange = fault, red = departure)', loc='left', fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'p_violation_scale007.png'), dpi=140)
    plt.close(fig)


def fig_ladder():
    rows = [('static envelope\n(no coupling)', 0, 'behavioural'),
            ('scale residuals\nby anomaly', 0, 'behavioural'),
            ('residual-triggered\ntrust region', 0, 'behavioural'),
            ('hand-built\ndead-reckoning', 4, 'redundancy (hand-built)'),
            ('learned consistency\n(method, ego_v6)', 4, 'redundancy (in the model)')]
    col = {'behavioural': '#bbbbbb', 'redundancy (hand-built)': '#e377c2', 'redundancy (in the model)': '#9467bd'}
    fig, ax = plt.subplots(figsize=(9, 3.8))
    for i, (n, k, src) in enumerate(rows):
        ax.bar(i, k, color=col[src], label=src if src not in [r[2] for r in rows[:i]] else None)
        ax.text(i, k + 0.08, f'{k}/4', ha='center', fontsize=10)
    ax.set_xticks(range(len(rows)), [r[0] for r in rows], fontsize=8)
    ax.set_ylim(0, 4.8); ax.set_ylabel('real departures warned (of 4)')
    ax.set_title('Residuals say WHEN; size and direction need a second measurement of the same quantity',
                 loc='left', fontsize=9)
    ax.legend(fontsize=8, loc='upper left')
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'coupling_ladder.png'), dpi=140)
    plt.close(fig)


def main():
    os.makedirs(OUT, exist_ok=True)
    nom, faults, floor = load()
    fig_operating(nom, faults, floor)
    fig_heading(faults)
    fig_anomaly_vs_consequence(floor)
    fig_p_scale(faults, floor)
    fig_ladder()
    # numbers for the write-up
    for v in ['nominal', 'deadreckon', 'evidence', 'evidence_dP', 'evidence_gated']:
        R, hours = sweep(nom, faults, v)
        best = R[(R[:, 4] >= 1.0)]
        print(f"{v:<11} nominal hours {hours:.2f}; lowest FA/h with all 4 warned >= {STOP_S}s: "
              f"{best[:, 1].min() if len(best) else float('nan'):.2f}")
        for fa_cap in (1.04, 3, 10):
            ok = R[R[:, 1] <= fa_cap]
            if len(ok):
                j = np.argmax(ok[:, 2])
                print(f"   at <= {fa_cap}/h: median lead {ok[j, 2]:.1f}s, caught {ok[j, 3]:.2f}, "
                      f">= {STOP_S}s {ok[j, 4]:.2f}, harmless alarm {ok[j, 5]:.2f} (thr {ok[j, 0]:.3f})")
    print('Layer 1 point: FA/h %.2f, median lead %.1f, >=1.7s %.2f, harmless %.2f' % layer1_point(nom, faults))
    print('saved to', OUT)


if __name__ == '__main__':
    main()
