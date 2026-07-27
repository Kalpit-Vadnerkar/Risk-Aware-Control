#!/usr/bin/env python3
"""
Post-hoc, independent verification that every IMU periodic on-cycle
(imu_bias/imu_scale_factor/imu_stuck_at) actually started inside its intended
zone — turn_zones for scale/stuck, bias_leadin_zones for bias (imu_fault_s1/
s3). This is a "trust but verify" regression check, not evidence the
mechanism is currently broken (fault_injector.py's _wait_for_zone already
guarantees this by construction, confirmed live 2026-07-26): the value is
catching a FUTURE bug in that gating logic (wrong zone file, radius drift,
generation-tracking bug across a trial reset, etc.) independently of
fault_injector.py's own code, using nothing but the position each
'imu_bias_on' event recorded at arming time (added 2026-07-26 specifically
for this) and the same turn_zones.json fault_injector.py itself loads.

Usage:
  python3 experiments/scripts/verify_imu_zone_arming.py --campaign imu_fault_scale
  python3 experiments/scripts/verify_imu_zone_arming.py --campaign imu_fault_s1 \
      --campaign imu_fault_s3 --campaign imu_fault_scale --campaign imu_fault_stuck
"""

import argparse
import glob
import json
import math
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
DATA_DIR = os.path.join(REPO_DIR, 'experiments', 'data')
TURN_ZONES_FILE = os.path.join(REPO_DIR, 'experiments', 'configs', 'turn_zones.json')
sys.path.insert(0, os.path.join(REPO_DIR, 'experiments', 'lib'))

# Imported from fault_injector.py when possible, not hand-copied — a
# hardcoded duplicate of these previously lived here and could silently
# drift from the live values without anyone noticing (found 2026-07-27
# auditing for exactly this class of disconnect between verification
# tooling and the code it's supposed to be checking). Falls back to a
# hardcoded copy only if ROS isn't sourced (fault_injector.py needs
# rclpy to import) — this script is documented as needing no ROS, and
# that's worth preserving even at the cost of a fallback duplicate.
try:
    from fault_injector import (  # noqa: E402
        _DEFAULT_IMU_TURN_ZONE_RADIUS_M as TURN_ZONE_RADIUS_M,
        _DEFAULT_IMU_LEADIN_ZONE_RADIUS_M as LEADIN_ZONE_RADIUS_M,
    )
except ImportError:
    TURN_ZONE_RADIUS_M = 15.0
    LEADIN_ZONE_RADIUS_M = 8.0

TOLERANCE_M = 1.0             # slack for GT sample spacing between the position that
                               # triggered arming and the logged position (both come
                               # from the same self._last_gt_xy, so this should be ~0
                               # in practice — a nonzero mismatch here would itself be
                               # a finding, not just noise to paper over)


def min_dist_to_zone(x, y, zone_points):
    if not zone_points:
        return float('inf')
    return min(math.hypot(x - zx, y - zy) for zx, zy in zone_points)


def verify_trial(trial_dir, zones_by_goal):
    goal_id = os.path.basename(os.path.dirname(trial_dir))
    trial_name = os.path.basename(trial_dir)
    fault_log = os.path.join(trial_dir, 'fault_log.jsonl')
    if not os.path.exists(fault_log):
        return None

    zones = zones_by_goal.get(goal_id)
    if zones is None:
        return {'goal': goal_id, 'trial': trial_name, 'checked': 0, 'mismatches': [],
                'error': f'no turn_zones.json entry for {goal_id}'}

    turn_pts = [(p['x'], p['y']) for p in zones.get('turn_zones', [])]
    leadin_pts = [(p['x'], p['y']) for p in zones.get('bias_leadin_zones', [])]

    checked = 0
    mismatches = []
    for line in open(fault_log):
        try:
            e = json.loads(line)
        except json.JSONDecodeError:
            continue
        if e.get('event') != 'imu_bias_on':
            continue
        pos = e.get('position')
        zone_kind = e.get('zone_kind')
        if pos is None or zone_kind is None:
            continue  # pre-2026-07-26 log, doesn't have position/zone_kind
        checked += 1
        pts, radius = (leadin_pts, LEADIN_ZONE_RADIUS_M) if zone_kind == 'bias_leadin' \
            else (turn_pts, TURN_ZONE_RADIUS_M)
        d = min_dist_to_zone(pos['x'], pos['y'], pts)
        if d > radius + TOLERANCE_M:
            mismatches.append({
                'cycle': e.get('cycle'), 'zone_kind': zone_kind,
                'position': pos, 'distance_to_nearest_zone_m': round(d, 1),
                'expected_radius_m': radius,
            })

    return {'goal': goal_id, 'trial': trial_name, 'checked': checked, 'mismatches': mismatches}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--campaign', action='append', required=True)
    ap.add_argument('--turn-zones-file', default=TURN_ZONES_FILE)
    args = ap.parse_args()

    zones_data = json.load(open(args.turn_zones_file))
    zones_by_goal = zones_data.get('goals', {})

    total_checked = 0
    total_mismatches = 0
    total_no_position = 0
    for campaign in args.campaign:
        trial_dirs = sorted(glob.glob(os.path.join(DATA_DIR, campaign, '*', 't*')))
        print(f'\n=== {campaign} ({len(trial_dirs)} trials) ===')
        for trial_dir in trial_dirs:
            result = verify_trial(trial_dir, zones_by_goal)
            if result is None:
                continue
            if 'error' in result:
                print(f'  {result["goal"]}/{result["trial"]}: ERROR — {result["error"]}')
                continue
            n_on_events = sum(
                1 for line in open(os.path.join(trial_dir, 'fault_log.jsonl'))
                if json.loads(line).get('event') == 'imu_bias_on'
            )
            no_pos = n_on_events - result['checked']
            total_no_position += no_pos
            total_checked += result['checked']
            total_mismatches += len(result['mismatches'])
            status = 'OK' if not result['mismatches'] else 'MISMATCH'
            extra = f'  ({no_pos} pre-2026-07-26 events without position, not checked)' if no_pos else ''
            print(f'  {result["goal"]}/{result["trial"]}: {result["checked"]}/{n_on_events} '
                  f'on-cycles verified in-zone — {status}{extra}')
            for m in result['mismatches']:
                print(f'    cycle {m["cycle"]} ({m["zone_kind"]}): position {m["position"]} is '
                      f'{m["distance_to_nearest_zone_m"]}m from nearest zone '
                      f'(expected <= {m["expected_radius_m"]}m)')

    print(f'\n{total_checked} on-cycles checked, {total_mismatches} mismatches, '
          f'{total_no_position} skipped (no position recorded — pre-2026-07-26 data).')
    if total_mismatches:
        print('MISMATCHES FOUND — the zone-gating guarantee does not hold for this data. '
              'Do not trust it for turn-vs-straight or lead-in analysis without investigating.')


if __name__ == '__main__':
    main()
