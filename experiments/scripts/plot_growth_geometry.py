"""Where the sin / cos in the pose-error growth equations come from.

(a) One time step dt, in the believed frame: the stack's pose moves v*dt along
    its believed heading; the car really moves v*dt along the true heading,
    psi away. The true step splits into v*dt*cos(psi) along and v*dt*sin(psi)
    across the believed heading, so the error grows by v*dt*(1 - cos psi)
    along and v*dt*sin psi across (plus whatever the gaps g_a, g_l add).
(b) Integrated for a constant gyro bias b with no LiDAR reset:
    psi = b t,  y ~ v b t^2 / 2,  x ~ v b^2 t^3 / 6 (negligible).

  python3 experiments/scripts/plot_growth_geometry.py [--out PNG]
"""
import argparse
import os

import matplotlib.pyplot as plt
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(os.path.dirname(__file__), '..', 'analysis', 'h30_30_ego6',
                                                  'highlights', 'growth_geometry.png'))
    args = ap.parse_args()
    fig, (a, b) = plt.subplots(1, 2, figsize=(15, 6.2), gridspec_kw={'width_ratios': [1.15, 1]})

    # (a) geometry of one step, psi exaggerated for visibility
    psi = np.radians(22)
    L = 10.0
    a.set_aspect('equal')
    a.annotate('', xy=(L, 0), xytext=(0, 0), arrowprops=dict(arrowstyle='-|>', lw=3, color='0.45'))
    a.annotate('', xy=(L * np.cos(psi), -L * np.sin(psi)), xytext=(0, 0),
               arrowprops=dict(arrowstyle='-|>', lw=3, color='#E07B00'))
    a.plot([L * np.cos(psi)] * 2, [0, -L * np.sin(psi)], color='#C0392B', lw=2.5, ls='--')
    a.plot([L * np.cos(psi), L], [0.25, 0.25], color='#1F77B4', lw=4)
    th = np.linspace(-psi, 0, 30)
    a.plot(3.2 * np.cos(th), 3.2 * np.sin(th), color='k', lw=1.2)
    a.text(3.5, -0.75, 'ψ', fontsize=18)
    a.text(0.2, 1.9, 'stack\'s pose moves v·dt along its\nBELIEVED heading (grey)', color='0.3', fontsize=11.5)
    a.text(0.4, -L * np.sin(psi) - 0.6, 'car really moves v·dt along its TRUE heading', color='#E07B00', fontsize=11.5)
    a.text(L * np.cos(psi) + 0.25, -L * np.sin(psi) / 2 - 0.2, 'across:\nv·dt·sin ψ\n→ ẏ += v sin ψ',
           color='#C0392B', fontsize=12.5, weight='bold')
    a.text(L + 0.4, 0.6, 'along (blue bar):\nv·dt·(1 − cos ψ)\n→ ẋ += v (1 − cos ψ)', color='#1F77B4', fontsize=12.5,
           weight='bold', ha='left')
    a.set_xlim(-0.6, 15.5)
    a.set_ylim(-5.8, 3.4)
    a.axis('off')
    a.set_title('(a) One time step, seen from the believed heading\n(ψ exaggerated to 22° to be visible)', fontsize=13)
    a.text(-0.4, -5.6, 'At ψ = 6°: 10% of each metre driven goes sideways, 0.5% is lost along.\nSo heading error turns into SIDEWAYS error.',
           fontsize=11.5)

    # (b) integration for a constant gyro bias, no LiDAR reset
    v, bias = 11.0, 0.02
    t = np.linspace(0, 10, 201)
    psi_t = bias * t
    y_t = np.cumsum(v * np.sin(psi_t)) * (t[1] - t[0])
    x_t = np.cumsum(v * (1 - np.cos(psi_t))) * (t[1] - t[0])
    b.plot(t, np.degrees(psi_t), color='k', lw=2.5, label='heading error ψ [deg] = ∫ g_ψ dt')
    b2 = b.twinx()
    b2.plot(t, y_t, color='#C0392B', lw=2.5, label='lateral error y [m] = ∫ v sin ψ dt')
    b2.plot(t, x_t, color='#1F77B4', lw=2.5, label='along-track error x [m] = ∫ v (1 − cos ψ) dt')
    for tt in (3, 5, 10):
        i = np.searchsorted(t, tt)
        b2.plot(tt, y_t[i], 'o', color='#C0392B')
        b2.annotate(f'{y_t[i]:.1f} m', (tt, y_t[i]), textcoords='offset points', xytext=(-38, 6), color='#C0392B',
                    fontsize=11)
    b.set_xlabel('seconds since the fault started', fontsize=12)
    b.set_ylabel('heading error ψ [deg]', fontsize=12)
    b2.set_ylabel('position error [m]', fontsize=12)
    b.set_ylim(0, 13)
    b2.set_ylim(0, 12)
    h1, l1 = b.get_legend_handles_labels()
    h2, l2 = b2.get_legend_handles_labels()
    b.legend(h1 + h2, l1 + l2, loc='upper left', fontsize=11)
    b.set_title(f'(b) Gyro bias {bias} rad/s at {v:.0f} m/s, LiDAR not resetting:\n'
                'ψ grows linearly, lateral error grows with t² (≈ v·b·t²/2)', fontsize=13)
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    fig.savefig(args.out, dpi=140)
    print(args.out, f'y(3)={y_t[60]:.2f} y(5)={y_t[100]:.2f} y(10)={y_t[200]:.2f} x(10)={x_t[200]:.2f}')


if __name__ == '__main__':
    main()
