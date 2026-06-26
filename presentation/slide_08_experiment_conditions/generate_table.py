"""
Generate the two-phase experiment conditions table for advisor presentation.
Run from the Risk-Aware-Control root:
    python3 presentation/slide_08_experiment_conditions/generate_table.py
"""

import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

OUT = os.path.dirname(os.path.abspath(__file__))

BLUE   = '#1565C0'
ORANGE = '#E65100'
GREEN  = '#2E7D32'
GRAY   = '#757575'
RED    = '#B71C1C'
PURPLE = '#6A1B9A'
TEAL   = '#00695C'

fig, axes = plt.subplots(2, 1, figsize=(15, 8.5),
                         gridspec_kw={'height_ratios': [4, 8], 'hspace': 0.55})

# ── Phase 1 table ────────────────────────────────────────────────────────────
ax1 = axes[0]
ax1.axis('off')
ax1.set_title('Phase 1 — Signal Validation (No Sensor Faults)',
              fontsize=11, fontweight='bold', pad=10, color='#263238')

p1_cols = ['Condition', 'Obstacle\nDistance', 'Road Type', 'Avoidance\nPolicy',
           'Ground Truth Label', 'Signal Tested', 'Status']

p1_rows = [
    ['obs_recovery',         '30 m',   'Multi-lane\n(adj. exists)',  'auto\n(enabled)',
     'Avoidance warranted\n+ executed',         'S3: Behavioral\nCUSUM',       '18 runs\n✓ Complete'],
    ['obs_noescape',         '30 m',   'Multi-lane\n(adj. exists)',  'manual\n(disabled)',
     'Avoidance warranted\n(suppressed)',        'S3: Behavioral\nCUSUM',       '18 runs\n✓ Complete'],
    ['obs_singlelane',       '30 m',   'Single-lane\n(no adjacent)', 'auto\n(enabled)',
     'MRC mandatory\n(topology constraint)',     'S1: HD map\ntopology',        '0 runs\n⚠ NEEDED'],
    ['obs_tooclosetoreact',  '5–8 m',  'Multi-lane\n(adj. exists)',  'auto\n(enabled)',
     'MRC mandatory\n(kinematic constraint)',    'S2: Kinematic\nTTC',          '0 runs\n⚠ NEEDED'],
]

p1_colors = [BLUE, ORANGE, GRAY, GRAY]

t1 = ax1.table(cellText=p1_rows, colLabels=p1_cols, loc='center', cellLoc='center')
t1.auto_set_font_size(False)
t1.set_fontsize(9.5)
t1.scale(1.0, 3.0)

for (row, col), cell in t1.get_celld().items():
    cell.set_edgecolor('#333333')
    cell.set_linewidth(0.7)
    if row == 0:
        cell.set_facecolor('#263238')
        cell.set_text_props(color='white', fontweight='bold', fontsize=9)
    else:
        base = p1_colors[row - 1]
        if base in (BLUE, ORANGE):
            cell.set_facecolor(base + '22')
        elif base == GRAY:
            cell.set_facecolor('#F5F5F5')
        # Status column
        if col == 6:
            if row <= 2:
                cell.set_facecolor('#A5D6A7')   # green — complete
            else:
                cell.set_facecolor('#FFCDD2')   # red — needed

ax1.text(0.5, -0.08,
         '⚠ Route note: current routes (goal_007/011/021) are similar segments. '
         'New collection needs routes with single-lane sections and varied traffic light patterns.',
         transform=ax1.transAxes, ha='center', va='top', fontsize=8.5,
         color='#B71C1C', style='italic')

# ── Phase 2 table ─────────────────────────────────────────────────────────────
ax2 = axes[1]
ax2.axis('off')
ax2.set_title('Phase 2 — Fault Robustness  (Confidence Values: P(correct | fault, magnitude))',
              fontsize=11, fontweight='bold', pad=10, color='#263238')

p2_cols = ['Base Scenario', 'Fault Type', 'Magnitude', 'Purpose', 'Status']

p2_rows = [
    # Camera fault
    ['nominal',        'Camera — TL dropout', '20%',         'Signal baseline: mild camera fault',           '0 — NEEDED'],
    ['nominal',        'Camera — TL dropout', '60%',         'Signal baseline: severe camera fault',         '0 — NEEDED'],
    ['obs_recovery',   'Camera — TL dropout', '20%',         'Avoidance detection: mild camera fault',       '0 — NEEDED'],
    ['obs_recovery',   'Camera — TL dropout', '60%',         'Avoidance detection: severe camera fault',     '0 — NEEDED'],
    # IMU fault
    ['nominal',        'IMU additive noise',  'σ=0.05 rad/s','Signal baseline: mild IMU noise',              '0 — NEEDED'],
    ['nominal',        'IMU additive noise',  'σ=0.25 rad/s','Signal baseline: severe IMU noise',            '0 — NEEDED'],
    ['obs_recovery',   'IMU additive noise',  'σ=0.05 rad/s','Avoidance detection: mild IMU noise',          '0 — NEEDED'],
    ['obs_recovery',   'IMU additive noise',  'σ=0.25 rad/s','Avoidance detection: severe IMU noise',        '0 — NEEDED'],
]

p2_section_colors = [TEAL, TEAL, TEAL, TEAL, PURPLE, PURPLE, PURPLE, PURPLE]

t2 = ax2.table(cellText=p2_rows, colLabels=p2_cols, loc='center', cellLoc='center')
t2.auto_set_font_size(False)
t2.set_fontsize(9.5)
t2.scale(1.0, 2.45)

for (row, col), cell in t2.get_celld().items():
    cell.set_edgecolor('#333333')
    cell.set_linewidth(0.7)
    if row == 0:
        cell.set_facecolor('#263238')
        cell.set_text_props(color='white', fontweight='bold', fontsize=9)
    else:
        base = p2_section_colors[row - 1]
        cell.set_facecolor(base + '18')
        # Add section divider line between camera and IMU rows
        if row == 4:
            cell.set_edgecolor('#333333')
            cell.set_linewidth(2.0)
        if col == 4:
            cell.set_facecolor('#FFCDD2')   # all phase 2 still needed

# Legend
legend_elements = [
    mpatches.Patch(facecolor=BLUE+'22',  edgecolor=BLUE,   label='obs_recovery (avoidance executed)'),
    mpatches.Patch(facecolor=ORANGE+'22',edgecolor=ORANGE, label='obs_noescape (avoidance suppressed)'),
    mpatches.Patch(facecolor='#F5F5F5', edgecolor=GRAY,   label='New Phase 1 conditions (needed)'),
    mpatches.Patch(facecolor=TEAL+'18', edgecolor=TEAL,   label='Phase 2 — Camera fault'),
    mpatches.Patch(facecolor=PURPLE+'18',edgecolor=PURPLE,label='Phase 2 — IMU fault'),
    mpatches.Patch(facecolor='#A5D6A7', edgecolor=GREEN,  label='Data complete'),
    mpatches.Patch(facecolor='#FFCDD2', edgecolor=RED,    label='Data needed'),
]
fig.legend(handles=legend_elements, loc='lower center',
           bbox_to_anchor=(0.5, 0.01), ncol=4, fontsize=8.5,
           framealpha=0.9)

fig.suptitle('Two-Phase Validation Design\n'
             'Phase 1: Signal Validation (Fault-Free)  |  '
             'Phase 2: Fault Robustness (Confidence Values)',
             fontsize=12, fontweight='bold', y=0.98)

out = os.path.join(OUT, 'experiment_conditions_table.png')
fig.savefig(out, bbox_inches='tight', dpi=150)
print(f'Saved: {out}')
plt.close(fig)
