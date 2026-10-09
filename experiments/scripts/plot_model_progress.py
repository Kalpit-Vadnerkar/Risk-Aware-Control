"""Before / after for the Layer 1 model fixes, from the safety benchmark
table: original model vs ego-frame map + route (ego_v3) vs multi-source
(ego_v6). Writes experiments/analysis/h30_30_ego6/highlights/model_progress.png."""
import json
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

B = os.path.join(os.path.dirname(__file__), '..', 'analysis', 'model_safety_benchmark')
OUT = os.path.join(os.path.dirname(__file__), '..', 'analysis', 'h30_30_ego6', 'highlights', 'model_progress.png')
MODELS = [('v2_zone_weighted_canonical', 'original\n(bbox map frame)', '#9e9e9e'),
          ('ego_v3_routefix+route_ahead_normsel', 'ego-frame map\n+ route ahead (v3)', '#1f77b4'),
          ('ego_v6_planar_pose', '+ redundant sources\n+ consistency heads (v6)', '#9467bd')]
METRICS = [('lat_err_3s_p50', 'lateral error at 3 s,\nmedian (m)', 0.5),
           ('offroad_rate', 'forecasts that leave\nthe road (share)', 0.02),
           ('wrong_direction_rate', 'turns predicted the\nwrong way (share)', 0.05),
           ('phantom_turn_rate', 'turns predicted where\nthere is none (share)', 0.05)]


def row(tag):
    with open(os.path.join(B, f'benchmark_{tag}.json')) as f:
        d = json.load(f)
    return d.get('metrics', d)


rows = {t: row(t) for t, _, _ in MODELS}
fig, axes = plt.subplots(1, 4, figsize=(16, 4.6))
for a, (k, lab, gate) in zip(axes, METRICS):
    vals = [rows[t][k] for t, _, _ in MODELS]
    bars = a.bar(range(3), vals, color=[c for _, _, c in MODELS])
    a.axhline(gate, color='#d62728', ls='--', lw=1.2)
    a.text(2.45, gate, 'safety gate', color='#d62728', fontsize=9, ha='right', va='bottom')
    for b, v in zip(bars, vals):
        a.text(b.get_x() + b.get_width() / 2, v, f'{v:.2f}' if v >= 0.1 else f'{v:.3f}', ha='center', va='bottom', fontsize=10)
    a.set_xticks([])
    a.set_title(lab, fontsize=11)
    a.set_ylim(0, max(max(vals), gate) * 1.25)
from matplotlib.patches import Patch
fig.legend(handles=[Patch(color=c, label=n.replace('\n', ' ')) for _, n, c in MODELS], loc='lower center', ncol=3,
           fontsize=11, frameon=False)
fig.suptitle('Fixing the predictor at the model level (held-out nominal windows; lower is better)', fontsize=12)
fig.tight_layout(rect=[0, 0.08, 1, 0.93])
fig.savefig(OUT, dpi=130)
print('saved', OUT)
