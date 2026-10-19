"""Architecture diagram: where every model input comes from.

Physical sources (vehicle, sensors, maps) -> AV-stack modules -> model input
groups -> ST-GAT heads -> Layers 1-3. Wiring as in this Autoware install:
the twist estimator (gyro odometer) fuses the vehicle's speed with the IMU yaw
rate; the EKF fuses that twist with LiDAR-to-map (NDT) poses; the vehicle
interface reports speed / yaw rate / steering (in AWSIM: the simulated body's
true velocity; on a real car: wheel-speed sensors, ESC yaw-rate sensor or
steering + bicycle model). Fault injection points marked.

  python3 experiments/scripts/plot_architecture_sources.py [--out PNG]
"""
import argparse
import os

import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

C = {'veh': '#E07B00', 'imu': '#C0392B', 'lidar': '#2E8B57', 'cam': '#1F77B4', 'map': '#666666',
     'stack': '#4A4A8A', 'model': '#6B3FA0', 'layer': '#1B2430'}


def node(ax, x, y, w, h, text, ec, fc='white', fs=10.5, bold_first=True, lw=1.8):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.02,rounding_size=0.08',
                                fc=fc, ec=ec, lw=lw))
    lines = text.split('\n')
    if bold_first:
        ax.text(x + w / 2, y + h - 0.13, lines[0], ha='center', va='top', fontsize=fs, weight='bold', color=ec)
        if len(lines) > 1:
            ax.text(x + w / 2, y + h - 0.42, '\n'.join(lines[1:]), ha='center', va='top', fontsize=fs - 1.5,
                    color='#222222', linespacing=1.25)
    else:
        ax.text(x + w / 2, y + h / 2, text, ha='center', va='center', fontsize=fs, color='#222222')
    return (x, y, w, h)


def arrow(ax, a, b, color, lw=1.4, rad=0.0, ls='-'):
    ax.add_patch(FancyArrowPatch(a, b, arrowstyle='-|>', mutation_scale=11, color=color, lw=lw,
                                 connectionstyle=f'arc3,rad={rad}', ls=ls, shrinkA=2, shrinkB=2))


def R(b):  # right-middle
    return (b[0] + b[2], b[1] + b[3] / 2)


def L(b, dy=0.0):
    return (b[0], b[1] + b[3] / 2 + dy)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(os.path.dirname(__file__), '..', 'analysis', 'h30_30_ego6',
                                                  'highlights', 'architecture_sources.png'))
    args = ap.parse_args()
    fig, ax = plt.subplots(figsize=(17, 9.6))
    ax.set_xlim(0, 17)
    ax.set_ylim(0, 9.6)
    ax.axis('off')

    for x, t in [(1.25, 'PHYSICAL SOURCES'), (5.0, 'AV STACK MODULES (Autoware)'), (9.05, 'MODEL INPUTS'),
                 (13.6, 'OUR PIPELINE')]:
        ax.text(x, 9.35, t, ha='center', fontsize=12, weight='bold', color='#444444')

    # column 1: physical sources
    veh = node(ax, 0.1, 6.55, 2.3, 2.45, 'Vehicle (CAN bus)\nwheel-speed sensors\nyaw-rate sensor (ESC)\nor steering + bicycle\nmodel; steering sensor',
               C['veh'])
    imu = node(ax, 0.1, 5.0, 2.3, 1.1, 'IMU\ngyro (yaw rate),\naccelerometer', C['imu'])
    lid = node(ax, 0.1, 3.45, 2.3, 1.1, 'LiDAR\n3D point cloud', C['lidar'])
    cam = node(ax, 0.1, 2.15, 2.3, 0.9, 'Camera\ntraffic lights only', C['cam'])
    mp = node(ax, 0.1, 0.35, 2.3, 1.4, 'Maps\nHD lanelet map,\npoint-cloud map', C['map'])

    # column 2: stack modules
    vif = node(ax, 3.2, 7.55, 3.6, 1.45, 'Vehicle interface\nvelocity report: speed, lateral\nspeed, yaw rate · steering report',
               C['veh'])
    gyro = node(ax, 3.2, 6.15, 3.6, 1.05, 'Twist estimator (gyro odometer)\nspeed from vehicle + yaw rate from IMU', C['stack'])
    ndt = node(ax, 3.2, 4.75, 3.6, 1.05, 'Pose estimator (NDT scan matcher)\nLiDAR scan matched to point-cloud map', C['lidar'])
    ekf = node(ax, 3.2, 3.15, 3.6, 1.25, 'EKF localizer\nfuses twist + NDT pose → pose, twist,\ncovariance, gate (Mahalanobis) test', C['stack'])
    perc = node(ax, 3.2, 1.95, 3.6, 0.85, 'Perception\nobject tracking (LiDAR) · traffic-light recog.', C['cam'])
    plan = node(ax, 3.2, 0.35, 3.6, 1.15, 'Planning & control\nroute (mission planner) on the\nHD map · commanded acceleration', C['stack'])

    arrow(ax, R(veh), L(vif), C['veh'])
    arrow(ax, (2.4, 7.0), L(gyro, 0.2), C['veh'])
    arrow(ax, R(imu), L(gyro, -0.2), C['imu'])
    arrow(ax, R(lid), L(ndt), C['lidar'])
    arrow(ax, (2.4, 0.9), (3.2, 5.0), C['map'], rad=-0.25, ls='--')
    arrow(ax, (5.0, 6.15), (5.0, 4.4), C['stack'])
    arrow(ax, (5.6, 4.75), (5.6, 4.4), C['lidar'])
    arrow(ax, (2.4, 3.7), (3.2, 2.5), C['lidar'], rad=0.0)
    arrow(ax, R(cam), (3.2, 2.2), C['cam'])
    arrow(ax, R(mp), L(plan), C['map'])

    # fault markers
    ax.text(1.25, 4.72, '⚡ IMU faults injected here', fontsize=9.5, color=C['imu'], weight='bold', ha='center')
    ax.text(5.0, 1.62, '⚡ traffic-light faults injected at this output', fontsize=9.5, color=C['cam'],
            weight='bold', ha='center')

    # column 3: model inputs
    x3, w3 = 7.55, 3.0
    i_vel = node(ax, x3, 7.95, w3, 1.05, 'Ego motion (vehicle side)\nspeed, lateral speed, steering', C['veh'])
    i_pos = node(ax, x3, 4.95, w3, 0.8, 'Ego position (stack side)\nEKF pose history', C['stack'])
    i_con = node(ax, x3, 5.95, w3, 1.8, 'Consistency gaps (5)\n1 EKF − vehicle yaw rate\n2 EKF − wheel speed\n3–5 pose yaw rate / along / lateral\n   speed − vehicle report',
                 C['model'], fc='#F1EAF8', lw=2.6)
    i_loc = node(ax, x3, 3.75, w3, 1.0, 'Localization health\nEKF gate distance, NDT match\nprobability · EKF variance', C['lidar'])
    i_tl = node(ax, x3, 2.75, w3, 0.8, 'Traffic light & objects\ncolour, confidence · up to 8 objects', C['cam'])
    i_map = node(ax, x3, 0.95, w3, 1.6, 'Scene (HD map + planner)\nmap graph (150 m), adjacent\nlane, route ahead (20 pts),\ncommanded acceleration', C['map'])

    arrow(ax, R(vif), L(i_vel), C['veh'])
    arrow(ax, (6.8, 4.05), L(i_pos), C['stack'])
    arrow(ax, (6.8, 7.75), (x3, 7.2), C['veh'])
    arrow(ax, (6.8, 4.3), (x3, 6.4), C['stack'])
    arrow(ax, (6.8, 3.45), L(i_loc), C['stack'])
    arrow(ax, (6.8, 5.1), (x3, 4.4), C['lidar'])
    arrow(ax, R(perc), L(i_tl), C['cam'])
    arrow(ax, R(plan), L(i_map, -0.3), C['map'])

    # column 4: our pipeline
    x4 = 11.25
    stg = node(ax, x4, 6.3, 2.5, 2.2, 'ST-GAT\n(nominal data only)\n3 s history → 3 s\nforecast at 10 Hz', C['model'], fc='#F1EAF8')
    hb = node(ax, 14.1, 7.5, 2.8, 1.0, 'Behaviour heads\nposition, speed, steering, TL', C['model'])
    hc = node(ax, 14.1, 6.3, 2.8, 1.0, 'Consistency heads\npredicted NOMINAL gaps', C['model'], fc='#F1EAF8', lw=2.6)
    l1 = node(ax, x4, 4.35, 5.65, 1.4, 'Layer 1: conformal residual scores\nWHEN (all channels) · HOW MUCH / WHICH WAY\n(gap residual = observed − predicted nominal gap)',
              C['layer'])
    fl = node(ax, x4, 2.85, 5.65, 1.1, 'Pose-error filter: δ̂ = (ψ̂, x̂, ŷ), covariance P\ngrowth from gaps 3–5, reset by LiDAR health', C['layer'])
    l2 = node(ax, x4, 1.45, 5.65, 1.05, 'Layer 2: 2a futures → 2b moved by δ̂ → 2c road / objects\n→ P(violation)', C['layer'])
    l3 = node(ax, x4, 0.25, 5.65, 0.9, 'Layer 3: correct pose (believed − δ̂), or slow / stop', C['layer'], fs=10.5)

    for b in (i_vel, i_pos, i_con, i_loc, i_tl, i_map):
        arrow(ax, R(b), (x4, 7.4), '#999999', lw=1.0)
    arrow(ax, (13.75, 7.85), L(hb), C['model'])
    arrow(ax, (13.75, 6.9), L(hc), C['model'])
    arrow(ax, (15.5, 6.3), (15.5, 5.75), C['model'])
    arrow(ax, (12.4, 6.3), (12.4, 5.75), C['model'])
    arrow(ax, (14.0, 4.35), (14.0, 3.95), C['layer'])
    arrow(ax, (14.0, 2.85), (14.0, 2.5), C['layer'])
    arrow(ax, (14.0, 1.45), (14.0, 1.15), C['layer'])
    arrow(ax, (x3 + w3, 4.1), (x4, 3.4), C['lidar'], rad=0.1)
    arrow(ax, (x3 + w3, 6.6), (x4, 5.0), C['model'], rad=0.0)

    ax.text(8.5, -0.1, 'Note: the EKF speed is built from the vehicle\'s own speed, so gap 2 is nearly the same source twice; '
            'the yaw-rate gaps compare IMU-driven estimates with the vehicle report.', fontsize=9.5, ha='center',
            color='#444444', style='italic')
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    fig.savefig(args.out, dpi=140, bbox_inches='tight')
    print(args.out)


if __name__ == '__main__':
    main()
