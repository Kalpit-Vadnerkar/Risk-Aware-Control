#!/usr/bin/env python3
"""
Post-hoc, independent verification that every TL fault activation
(tl_fault_start event) was scoped to a real, reachable traffic_light_group_id
for that goal's route — the TL-side counterpart to
verify_imu_zone_arming.py, checking the group-id scoping added 2026-07-27
(see fault_injector.py's _tl_fault_group_id) rather than trusting that
fault_injector.py's own logic got it right.

Two things checked per tl_fault_start event:
  1. group_id is not None — None means the fault fell back to unscoped
     (mutating every group in the message), which only happens if
     tl_zones.json had no data for this goal at the time of the run. Not
     inherently a failure, but worth knowing if it happened when it
     shouldn't have.
  2. group_id matches a zone actually in tl_zones.json for this goal, AND
     that zone is marked reachable (not the one consumed by the runway gate
     — see compute_tl_zones.py's `reachable` field). A group_id that
     matches nothing, or matches only an unreachable zone, would mean the
     live zone data and the precomputed file have drifted apart, or the
     runway/arming state machine's "can't arm on the first zone" guarantee
     (see compute_turn_zones.py's module docstring) has broken.

fault_log.jsonl doesn't record a position for TL events (unlike
imu_bias_on — TL arming is edge-triggered off zone entry, not a per-cycle
position snapshot), so this can't cross-check against the vehicle's actual
GT position the way the IMU script does; it checks the group_id against the
zone file only.

Usage:
  python3 experiments/scripts/verify_tl_zone_arming.py --campaign tl_fault_s2
  python3 experiments/scripts/verify_tl_zone_arming.py --campaign tl_fault_s2 \
      --campaign tl_fault_s3 --campaign tl_fault_s4 --campaign tl_fault_ramp
"""

import argparse
import glob
import json
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
DATA_DIR = os.path.join(REPO_DIR, 'experiments', 'data')
TL_ZONES_FILE = os.path.join(REPO_DIR, 'experiments', 'configs', 'tl_zones.json')


def verify_trial(trial_dir, zones_by_goal):
    goal_id = os.path.basename(os.path.dirname(trial_dir))
    trial_name = os.path.basename(trial_dir)
    fault_log = os.path.join(trial_dir, 'fault_log.jsonl')
    if not os.path.exists(fault_log):
        return None

    zones = zones_by_goal.get(goal_id)
    if zones is None:
        return {'goal': goal_id, 'trial': trial_name, 'checked': 0, 'issues': [],
                'error': f'no tl_zones.json entry for {goal_id}'}

    reachable_ids = {z['group_id'] for z in zones.get('tl_zones', []) if z.get('reachable', True)}
    unreachable_ids = {z['group_id'] for z in zones.get('tl_zones', []) if not z.get('reachable', True)}

    checked = 0
    issues = []
    for line in open(fault_log):
        try:
            e = json.loads(line)
        except json.JSONDecodeError:
            continue
        if e.get('event') != 'tl_fault_start':
            continue
        checked += 1
        gid = e.get('group_id')
        if gid is None:
            issues.append({'cycle': e.get('cycle'), 'issue': 'unscoped (group_id=None) — '
                            'no route-derived tl_zones.json data was loaded for this run'})
        elif gid in unreachable_ids:
            issues.append({'cycle': e.get('cycle'), 'issue': f'group_id={gid} matches a zone '
                            'flagged unreachable (before runway-clear) — should never arm here'})
        elif gid not in reachable_ids:
            issues.append({'cycle': e.get('cycle'), 'issue': f'group_id={gid} matches no zone '
                            f'in tl_zones.json for {goal_id} at all'})

    return {'goal': goal_id, 'trial': trial_name, 'checked': checked, 'issues': issues}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--campaign', action='append', required=True)
    ap.add_argument('--tl-zones-file', default=TL_ZONES_FILE)
    args = ap.parse_args()

    zones_data = json.load(open(args.tl_zones_file))
    zones_by_goal = zones_data.get('goals', {})

    total_checked = 0
    total_issues = 0
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
            total_checked += result['checked']
            total_issues += len(result['issues'])
            status = 'OK' if not result['issues'] else 'ISSUE'
            print(f'  {result["goal"]}/{result["trial"]}: {result["checked"]} TL fault '
                  f'activation(s) checked — {status}')
            for iss in result['issues']:
                print(f'    cycle {iss["cycle"]}: {iss["issue"]}')

    print(f'\n{total_checked} TL fault activations checked, {total_issues} issue(s).')
    if total_issues:
        print('ISSUES FOUND — group-id scoping does not hold for this data. Do not trust '
              'per-light labeling (or assume unrelated intersections were left untouched) '
              'without investigating.')


if __name__ == '__main__':
    main()
