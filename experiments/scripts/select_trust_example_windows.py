#!/usr/bin/env python3
"""
Select a fixed, small set of "interesting" nominal-driving windows to use
as the canonical examples for plot_layer1_trust_examples.py (2026-08-24,
per Kalpit: pick 3-4 unique/interesting scenarios once, reuse them for
every future plot regeneration instead of re-randomizing each time).

Scores every calibration window on four independent event types and picks
the single most extreme window per type, plus one calm baseline (low on
all four) for contrast:
  - turn:     largest real heading change over the horizon (robust,
              denoised the same way as diagnose_turn_learning.py)
  - tl_event: traffic light color actually changes during the window
  - braking:  largest longitudinal deceleration event
  - accel:    largest longitudinal acceleration event
  - calm:     lowest combined score across all four (percentile rank sum)

Writes experiments/analysis/trust_example_windows.json:
  {"turn": <idx>, "tl_event": <idx>, "braking": <idx>, "accel": <idx>, "calm": <idx>}
plot_layer1_trust_examples.py reads this file by default instead of
drawing a random sample, so the same 5 example windows are used every time
the pipeline is regenerated (unless --random is passed).
"""

import json
import os
import sys

import numpy as np
import torch

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
sys.path.insert(0, REPO_DIR)

from st_gat.pipeline import config as cfg  # noqa: E402
from st_gat.model import TrajectoryDataset  # noqa: E402

OUTPUT_PATH = os.path.join(REPO_DIR, 'experiments', 'analysis', 'trust_example_windows.json')

MIN_SEG_M = 0.5
K = 5


def heading(vec):
    return np.arctan2(vec[1], vec[0])


def wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def robust_turn_deg(xy, min_seg_m=MIN_SEG_M, k=K):
    start_vec = xy[k] - xy[0]
    end_vec   = xy[-1] - xy[-1 - k]
    if np.linalg.norm(start_vec) < min_seg_m or np.linalg.norm(end_vec) < min_seg_m:
        return 0.0
    return float(np.degrees(abs(wrap(heading(end_vec) - heading(start_vec)))))


def main():
    ds = TrajectoryDataset(cfg.CAL_DIR)
    n = len(ds)
    print(f"Scanning {n} calibration windows for canonical example scenarios...")

    turn_score, tl_score, brake_score, accel_score = (np.full(n, np.nan) for _ in range(4))

    for idx in range(n):
        seq = ds.sequences[idx]
        future = seq['future']
        position = np.array([f['position'] for f in future]) * cfg.POSITION_DISPLACEMENT_RANGE_M
        turn_score[idx] = robust_turn_deg(position)

        tl_color = np.array([f['traffic_light_color'] for f in future]).reshape(-1)
        tl_score[idx] = float(tl_color.max() - tl_color.min())  # 0 if constant, >0 if it changes

        long_vel = np.array([f['velocity'][0] for f in future])
        d_vel = np.diff(long_vel)
        brake_score[idx] = float(-d_vel.min()) if len(d_vel) else 0.0   # largest single-step decel
        accel_score[idx] = float(d_vel.max()) if len(d_vel) else 0.0    # largest single-step accel

    def rank(arr):
        order = np.argsort(arr)
        r = np.empty_like(order, dtype=float)
        r[order] = np.arange(len(arr)) / (len(arr) - 1)
        return r

    combined_rank = rank(turn_score) + rank(tl_score) + rank(brake_score) + rank(accel_score)

    selected = {
        'turn':     int(np.argmax(turn_score)),
        'tl_event': int(np.argmax(tl_score)),
        'braking':  int(np.argmax(brake_score)),
        'accel':    int(np.argmax(accel_score)),
        'calm':     int(np.argmin(combined_rank)),
    }

    print("\nSelected canonical example windows:")
    for name, idx in selected.items():
        print(f"  {name:9s}: window {idx:5d}  "
              f"(turn={turn_score[idx]:6.2f} deg, tl_delta={tl_score[idx]:.2f}, "
              f"brake={brake_score[idx]:.3f}, accel={accel_score[idx]:.3f})")

    with open(OUTPUT_PATH, 'w') as f:
        json.dump(selected, f, indent=2)
    print(f"\nSaved {OUTPUT_PATH}")


if __name__ == '__main__':
    main()
