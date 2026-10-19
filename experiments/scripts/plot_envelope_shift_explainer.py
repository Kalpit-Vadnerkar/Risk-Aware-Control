"""Schematic of how 2b moves the envelope, step by step, with worked numbers.

Not data: a straight road, a forecast at 11 m/s over 3 s, and a pose error of
6 deg heading / 0.8 m lateral, chosen so the envelope sits on the road edge.
Shows the arithmetic in docs/architecture.md ("Envelope and risk"):
  tau_k = R(psi_k) (mu + eps_k) + (x_k, y_k),  delta_k ~ N(delta_hat, P)
and the rule of thumb behind it: a point d metres ahead moves sideways by
about y + d sin(psi).

Usage: python3 experiments/scripts/plot_envelope_shift_explainer.py [--out PNG]
"""
import argparse
import os

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon

HALF_W = 5.25         # road half-width (m): three 3.5 m lanes, car in the middle one
LANE_Y = 0.0          # centre of the middle lane
V, T, K = 11.0, 3.0, 200
PSI_DEG, Y_ERR = 6.0, 0.8            # estimated pose error (heading, lateral)
SIG_PSI_DEG, SIG_Y = 1.5, 0.3        # its uncertainty (filter covariance P)
CAR_L, CAR_W = 4.5, 1.9


def rot(a):
    return np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])


def car(ax, x, y, h, color, ls='-', alpha=1.0, label=None):
    c = np.array([[-CAR_L / 2, -CAR_W / 2], [CAR_L / 2, -CAR_W / 2],
                  [CAR_L / 2, CAR_W / 2], [-CAR_L / 2, CAR_W / 2]]) @ rot(h).T + [x, y]
    ax.add_patch(Polygon(c, closed=True, fill=False, ec=color, lw=2, ls=ls, alpha=alpha, label=label))
    ax.annotate('', xy=(x + 3.2 * np.cos(h), y + 3.2 * np.sin(h)), xytext=(x, y),
                arrowprops=dict(arrowstyle='->', color=color, lw=2, alpha=alpha))


def road(ax):
    ax.axhspan(-HALF_W, HALF_W, color='0.80', zorder=0)
    for yl in (-1.75, 1.75):
        ax.axhline(yl, color='white', lw=2, ls=(0, (6, 6)), zorder=0)
    ax.set_xlim(-6, 40)
    ax.set_ylim(-10.5, 7)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(
        os.path.dirname(__file__), '..', 'analysis', 'h30_30_ego6', 'highlights',
        'envelope_shift_explainer.png'))
    args = ap.parse_args()
    rng = np.random.default_rng(3)

    # 2a: forecast mu (straight down the believed lane) + K recorded-error-like
    # trajectories eps_k (random walks; spread ~0.3 m lateral, ~0.6 m along at 3 s)
    t = np.arange(1, 31) / 10.0
    mu = np.stack([V * t, np.zeros_like(t)], 1)
    eps = np.cumsum(rng.normal(0, [0.11, 0.055], (K, 30, 2)), axis=1)
    fut = mu[None] + eps                               # in the believed car frame
    start = np.array([0.0, LANE_Y])

    psi, y0 = np.deg2rad(PSI_DEG), Y_ERR
    psi_k = rng.normal(psi, np.deg2rad(SIG_PSI_DEG), K)
    y_k = rng.normal(y0, SIG_Y, K)

    def moved(f, a, dy):  # rotate about the car (sign: true heading = believed - psi), then shift
        return f @ rot(-a).T + [0.0, -dy]

    fig, axs = plt.subplots(2, 2, figsize=(15, 8.6))
    axs = axs.ravel()
    blue, purple, grey = '#1f77b4', '#6B3FA0', '0.35'

    # 1 believed
    ax = axs[0]; road(ax)
    for f in fut[:60]:
        ax.plot(*(f + start).T, color=grey, lw=0.6, alpha=0.5)
    ax.plot(*(mu + start).T, color=blue, lw=2.5)
    car(ax, *start, 0.0, 'k')
    ax.set_title('1  2a: what the stack believes\n'
                 'forecast + 200 recorded-error futures, all inside its lane', fontsize=12)
    ax.text(-5, -9.8, 'Autoware thinks it is centred and heading down the lane,\n'
            'so in its own frame every future looks fine:  P = 0.00', fontsize=11)

    # 2 rotate
    ax = axs[1]; road(ax)
    for f in fut[:60]:
        ax.plot(*(f + start).T, color=grey, lw=0.5, alpha=0.15)
    rf = fut[:60] @ rot(-psi).T + start
    for f in rf:
        ax.plot(*f.T, color=purple, lw=0.6, alpha=0.5)
    ax.plot(*((mu @ rot(-psi).T) + start).T, color=purple, lw=2.5)
    car(ax, *start, -psi, purple)
    d = V * T
    ax.annotate('', xy=(d * np.cos(psi), LANE_Y - d * np.sin(psi)), xytext=(d, LANE_Y),
                arrowprops=dict(arrowstyle='->', color='crimson', lw=1.8))
    ax.text(d - 21, LANE_Y - 5.0, f'33 m ahead × sin {PSI_DEG:.0f}° ≈ {d * np.sin(psi):.1f} m', color='crimson', fontsize=11)
    ax.set_title(f'2  2b, step 1: rotate every future by the heading error ψ̂ = {PSI_DEG:.0f}°\n'
                 f'about the car (the car really points {PSI_DEG:.0f}° off the lane)', fontsize=12)

    # 3 shift
    ax = axs[2]; road(ax)
    sf = moved(fut[:60], psi, y0) + start
    for f in sf:
        ax.plot(*f.T, color=purple, lw=0.6, alpha=0.5)
    ax.plot(*(moved(mu, psi, y0) + start).T, color=purple, lw=2.5)
    car(ax, *start, 0.0, 'k', ls='--', alpha=0.5)
    car(ax, start[0], start[1] - y0, -psi, purple)
    ax.annotate('', xy=(0, LANE_Y - y0), xytext=(0, LANE_Y),
                arrowprops=dict(arrowstyle='->', color='crimson', lw=1.8))
    ax.text(-5.5, LANE_Y - 2.6, f'ŷ = {y0:.1f} m', color='crimson', fontsize=11)
    ax.text(-5, -9.8, f'Sideways move of a point d metres ahead ≈ ŷ + d·sin ψ̂\n'
            f'at 3 s (33 m):  {y0:.1f} + {d * np.sin(psi):.1f} ≈ {y0 + d * np.sin(psi):.1f} m;  room to the edge: {HALF_W - CAR_W / 2:.1f} m', fontsize=11)
    ax.set_title(f'3  2b, step 2: shift by the position error (ŷ = {y0:.1f} m)\n'
                 'dashed = believed pose, purple = estimated true pose', fontsize=12)

    # 4 spread + count
    ax = axs[3]; road(ax)
    out = 0
    for k in range(K):
        f = moved(fut[k], psi_k[k], y_k[k]) + start
        bad = np.any(np.abs(f[:, 1]) > HALF_W - CAR_W / 2)
        out += bad
        ax.plot(*f.T, color='#d62728' if bad else '#2ca02c', lw=0.5, alpha=0.45)
    car(ax, start[0], start[1] - y0, -psi, purple)
    ax.text(-5, -9.8, f'{out} of {K} futures leave the road  →  P(violation) = {out / K:.2f}',
            fontsize=12, weight='bold')
    ax.set_title(f'4  each future gets its own error draw from N(δ̂, P)\n'
                 f'(ψ ± {SIG_PSI_DEG}°, y ± {SIG_Y} m), then 2c counts how many leave the road', fontsize=12)

    for a in axs:
        a.plot([], [], color='0.80', lw=8, label='road')
    h = [plt.Line2D([], [], color=blue, lw=2.5, label='ST-GAT forecast (believed frame)'),
         plt.Line2D([], [], color=grey, lw=1, label='2a futures (believed frame)'),
         plt.Line2D([], [], color=purple, lw=2.5, label='after 2b move'),
         plt.Line2D([], [], color='#2ca02c', lw=1.5, label='stays on road'),
         plt.Line2D([], [], color='#d62728', lw=1.5, label='leaves road'),
         plt.Line2D([], [], color='0.80', lw=8, label='road (three lanes)')]
    fig.legend(handles=h, loc='lower center', ncol=6, fontsize=12, frameon=False)
    fig.suptitle(f'How 2b moves the envelope (schematic, worked numbers: heading error {PSI_DEG:.0f}°, lateral error {Y_ERR} m, 11 m/s)',
                 fontsize=14)
    fig.tight_layout(rect=(0, 0.05, 1, 0.96))
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    fig.savefig(args.out, dpi=150)
    print(args.out, f'P={out / K:.2f}')


if __name__ == '__main__':
    main()
