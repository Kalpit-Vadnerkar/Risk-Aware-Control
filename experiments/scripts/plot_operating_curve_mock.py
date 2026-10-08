"""MOCK operating curve (Pareto front) for the Layer 2 claim: what the figure
would look like and where 'good' sits. Numbers are illustrative, not
measured -- the measured curve is plot_highlights.py."""
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT = os.path.join(os.path.dirname(__file__), '..', 'analysis', 'h30_30_ego6', 'highlights')
fa = np.logspace(-2, 2, 200)            # false alarms per hour of nominal driving


def curve(lead_max, k, fa50):
    return lead_max / (1 + (fa50 / fa) ** k)


fig, ax = plt.subplots(1, 2, figsize=(13, 5))
a = ax[0]
a.axvspan(1e-2, 1, ymin=2 / 6, ymax=1, color='#2ca02c', alpha=0.10)
a.text(0.012, 5.6, 'target: >= 2 s warning\nat <= 1 false alarm / h', color='#2ca02c', fontsize=9, va='top')
a.axhline(1.7, color='grey', ls=':', lw=1)
a.text(30, 1.75, 'stop from 5 m/s at 3 m/s^2 (1.7 s)', fontsize=8, color='grey', ha='right')
a.axhline(3.0, color='#9467bd', ls='--', lw=1)
a.text(0.012, 3.05, 'forecast horizon (3 s): P(violation within 3 s) cannot warn earlier', fontsize=8, color='#9467bd')
a.axvspan(1e-2, 1.04, color='k', alpha=0.05, hatch='//', fill=False, lw=0)
a.text(0.5, 0.3, 'cannot be measured with\n0.96 h of nominal data\n(1 episode = 1.04 / h)', fontsize=8, ha='center', color='#555')
a.plot(fa, curve(3.0, 1.2, 0.05), color='#9467bd', lw=2.4, label='2a + 2b evidence (method)')
a.plot(fa, curve(2.6, 1.0, 0.6), color='#e377c2', lw=1.6, ls='--', label='dead-reckoning baseline')
a.plot(fa, curve(1.6, 0.9, 3.0), color='grey', lw=1.6, label='2a nominal only')
a.plot(fa, curve(5.0, 1.0, 40), color='k', lw=1.4, ls='-.', label='Layer 1 anomaly alone')
a.set_xscale('log'); a.set_ylim(0, 6); a.set_xlim(1e-2, 1e2)
a.set_xlabel('false alarms per hour of nominal driving (log)')
a.set_ylabel('median warning before real departure (s)')
a.set_title('Warning time vs false alarms', loc='left', fontsize=10)
a.legend(fontsize=8, loc='upper right')
a = ax[1]
a.axvspan(0, 0.1, ymin=0.9, ymax=1, color='#2ca02c', alpha=0.15)
a.annotate('target: every departure warned >= 1.7 s ahead,\nharmless faults in alarm < 10% of fault time', xy=(0.05, 0.96),
           xytext=(0.30, 0.55), fontsize=8, color='#2ca02c', arrowprops=dict(arrowstyle='->', color='#2ca02c'))
x = np.linspace(0, 1, 200)
for lab, c, ls, k in [('2a + 2b evidence (method)', '#9467bd', '-', 12), ('dead-reckoning baseline', '#e377c2', '--', 4),
                      ('2a nominal only', 'grey', '-', 2.0), ('Layer 1 anomaly alone', 'k', '-.', 0.6)]:
    a.plot(x, 1 - (1 - x) ** k, color=c, ls=ls, lw=2.4 if k == 12 else 1.5, label=lab)
a.set_xlim(0, 1); a.set_ylim(0, 1.02)
a.set_xlabel('harmless faults: share of fault time in alarm')
a.set_ylabel('departures warned >= 1.7 s ahead (share)')
a.set_title('Consequence, not anomaly: catch departures, stay quiet on harmless faults', loc='left', fontsize=10)
a.legend(fontsize=8, loc='lower right')
fig.suptitle('MOCK -- illustrative operating curves (threshold swept along each line); not measured', color='#d62728')
fig.tight_layout()
fig.savefig(os.path.join(OUT, 'operating_curve_MOCK.png'), dpi=140)
print('saved')
