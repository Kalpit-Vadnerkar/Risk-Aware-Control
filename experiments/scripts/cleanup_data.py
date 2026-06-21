#!/usr/bin/env python3
"""
One-time cleanup: removes experiment data that is no longer needed.

Keeps:
  baseline_all/         -- all 24 nominal runs (training + calibration data)
  static_obstacle/      -- only goal_{007,011,021} × dist=30m × CAR (6 runs)

Deletes:
  static_obstacle_dropout30/   -- fault injection data, out of scope
  static_obstacle/goal_003..006, 008, 009   -- goals not in test set
  static_obstacle/dist=20m, dist=50m        -- distances not in core scenario
  static_obstacle/TRUCK type                -- object type not in core scenario

Run with --dry-run first to review, then without to execute.
"""

import argparse
import glob
import json
import os
import shutil

DATA_DIR = os.path.join(os.path.dirname(__file__), '..', 'data')

KEEP_GOALS    = {'goal_007', 'goal_011', 'goal_021'}
KEEP_DISTANCE = 30.0
KEEP_OBJ_TYPE = 'CAR'


def classify_run(exp_dir: str):
    """Return ('keep', reason) or ('delete', reason) for a run directory."""
    meta_path = os.path.join(exp_dir, 'metadata.json')
    if not os.path.exists(meta_path):
        return 'delete', 'no metadata.json'

    with open(meta_path) as f:
        meta = json.load(f)

    goal_id = meta.get('goal_id', '')
    params  = meta.get('scenario_params', {})
    dist    = params.get('distance', None)
    obj     = params.get('obj_type', None)

    if goal_id not in KEEP_GOALS:
        return 'delete', f'goal {goal_id} not in test set'
    if dist != KEEP_DISTANCE:
        return 'delete', f'distance={dist}m (keep only {KEEP_DISTANCE}m)'
    if obj != KEEP_OBJ_TYPE:
        return 'delete', f'obj_type={obj} (keep only {KEEP_OBJ_TYPE})'
    return 'keep', 'core scenario run'


def human_size(path: str) -> str:
    total = 0
    for root, _, files in os.walk(path):
        for f in files:
            try:
                total += os.path.getsize(os.path.join(root, f))
            except OSError:
                pass
    if total >= 1e9:
        return f'{total/1e9:.1f} GB'
    return f'{total/1e6:.0f} MB'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dry-run', action='store_true',
                        help='Show what would be deleted without deleting')
    args = parser.parse_args()

    mode = 'DRY RUN' if args.dry_run else 'DELETING'
    print(f'=== Experiment data cleanup [{mode}] ===\n')

    total_freed = 0

    # 1. Delete entire fault injection campaign
    dropout_dir = os.path.join(DATA_DIR, 'static_obstacle_dropout30')
    if os.path.isdir(dropout_dir):
        size_str = human_size(dropout_dir)
        print(f'[DELETE] static_obstacle_dropout30/  ({size_str})')
        print(f'         reason: fault injection data, out of scope')
        if not args.dry_run:
            shutil.rmtree(dropout_dir)
            print(f'         DELETED')
    else:
        print('[SKIP]   static_obstacle_dropout30/ not found')

    print()

    # 2. Prune static_obstacle/ — keep only the core scenario runs
    obs_dir = os.path.join(DATA_DIR, 'static_obstacle')
    if not os.path.isdir(obs_dir):
        print('[SKIP]   static_obstacle/ not found')
        return

    keep_count = delete_count = 0
    for exp_dir in sorted(glob.glob(os.path.join(obs_dir, 'goal_*'))):
        action, reason = classify_run(exp_dir)
        name = os.path.basename(exp_dir)
        if action == 'delete':
            size_str = human_size(exp_dir)
            print(f'[DELETE] static_obstacle/{name}  ({size_str})')
            print(f'         reason: {reason}')
            if not args.dry_run:
                shutil.rmtree(exp_dir)
            delete_count += 1
        else:
            print(f'[KEEP]   static_obstacle/{name}')
            keep_count += 1

    print(f'\nSummary: keep={keep_count}  delete={delete_count}')
    if args.dry_run:
        print('\nRe-run without --dry-run to execute.')


if __name__ == '__main__':
    main()
