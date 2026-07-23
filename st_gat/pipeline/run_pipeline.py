"""
run_pipeline.py — entry point for the ST-GAT data processing pipeline.

What it does:
  1. Scans all NOMINAL_DATASETS under DATA_ROOT
  2. For each successful run, reads the rosbag → synchronized frames
  3. Builds sliding-window sequences (past 30 + future 30 at 10 Hz)
  4. Splits train/cal by CAL_FRACTION, stratified by goal
  5. Saves .pkl files to TRAIN_DIR and CAL_DIR

Usage (with Autoware workspace sourced):
  source /home/kvadner/Desktop/Dissertation/autoware/install/setup.bash
  python3 -m st_gat.pipeline.run_pipeline [--datasets baseline_all nom_v11] [--verbose]

Output pkl format:
  Each file is a list of sequence dicts:
    [{'past': [...], 'future': [...], 'graph': G, 'graph_bounds': [...]}, ...]

Notes:
  - Test datasets (obs_recovery, obs_noescape) are NEVER processed here.
  - Map loading is cached per Python process (~10s first time).
  - Each run's rosbag is read once; SequenceBuilder.from_bag() re-loads the map
    but reuses the same MapProcessor if you call it repeatedly — see below.
"""

import argparse
import json
import math
import os
import pickle
import random
import sys
from collections import defaultdict
from typing import Dict, List, Tuple

from . import config as cfg
from .bag_reader import read_bag
from .sequence_builder import SequenceBuilder


# ── Helpers ────────────────────────────────────────────────────────────────

def _find_run_dirs(dataset: str) -> List[str]:
    """Return sorted list of trial run directories for a dataset.

    Handles the nested layout used since 2026-07-22:
        <dataset>/<goal_NNN>/<tN_timestamp>/rosbag/
    Also handles the old flat layout (pre-2026-07-22):
        <dataset>/<goal_NNN_campaign_tN_timestamp>/rosbag/
    """
    dataset_dir = os.path.join(cfg.DATA_ROOT, dataset)
    if not os.path.isdir(dataset_dir):
        print(f"  [pipeline] WARNING: dataset dir not found: {dataset_dir}")
        return []

    runs = []
    for entry in sorted(os.listdir(dataset_dir)):
        top_dir = os.path.join(dataset_dir, entry)
        if not os.path.isdir(top_dir):
            continue
        # Old flat layout: rosbag/ is directly inside this dir
        if os.path.isdir(os.path.join(top_dir, 'rosbag')):
            runs.append(top_dir)
            continue
        # New nested layout: goal_NNN/tN_timestamp/rosbag/
        for trial_entry in sorted(os.listdir(top_dir)):
            trial_dir = os.path.join(top_dir, trial_entry)
            if os.path.isdir(trial_dir) and os.path.isdir(os.path.join(trial_dir, 'rosbag')):
                runs.append(trial_dir)
    return runs


def _load_result(run_dir: str) -> dict:
    result_file = os.path.join(run_dir, 'result.json')
    if not os.path.exists(result_file):
        return {}
    with open(result_file) as f:
        return json.load(f)


def _goal_from_run_dir(run_dir: str) -> str:
    """Extract goal_id from a trial run directory.

    New nested layout:  .../goal_007/t1_20260722_134547  → 'goal_007'
    Old flat layout:    .../goal_007_nom_v11_t1_<ts>     → 'goal_007'
    """
    basename = os.path.basename(run_dir)
    parts = basename.split('_')
    if len(parts) >= 2 and parts[0] == 'goal':
        # Old flat layout: basename starts with 'goal_NNN'
        return f"{parts[0]}_{parts[1]}"
    # New nested layout: trial dir is tN_<timestamp>; goal is the parent dirname
    parent = os.path.basename(os.path.dirname(run_dir))
    parent_parts = parent.split('_')
    if len(parent_parts) >= 2 and parent_parts[0] == 'goal':
        return f"{parent_parts[0]}_{parent_parts[1]}"
    return parent


def _train_cal_split(
    run_dirs_by_goal: Dict[str, List[str]],
    cal_fraction: float,
    seed: int = 42,
) -> Tuple[List[str], List[str]]:
    """
    Stratified split by goal: for each goal, put CAL_FRACTION of runs into cal set.

    Goals with only 1 run stay in train — we do NOT force a floor of 1 cal run
    per goal, because baseline_all has 1 run per goal × 22 goals, and forcing
    each to cal would put all diverse route coverage into cal and leave train
    with only the 3 nom-dataset goals (007/011/021). Calibration for conformal
    prediction is handled primarily by the nom_v11 held-out runs, which
    have 5-6 runs per goal and naturally contribute ≥1 run to cal per goal.
    """
    rng = random.Random(seed)
    train_dirs, cal_dirs = [], []

    for goal, dirs in sorted(run_dirs_by_goal.items()):
        shuffled = dirs[:]
        rng.shuffle(shuffled)
        # ceil so that 2-run goals (round(2*0.20)=0) still contribute 1 cal run.
        # Single-run goals stay in train (ceil(1*0.20)=1 would sacrifice the only run).
        if len(shuffled) >= 2:
            n_cal = math.ceil(len(shuffled) * cal_fraction)
        else:
            n_cal = 0
        cal_dirs.extend(shuffled[:n_cal])
        train_dirs.extend(shuffled[n_cal:])

    return train_dirs, cal_dirs


# ── Main pipeline ──────────────────────────────────────────────────────────

def process_dataset(
    dataset: str,
    shared_builder: SequenceBuilder,
    verbose: bool = False,
    filter_mrm: bool = False,
) -> Tuple[List[str], List[str]]:
    """
    Process all runs in a dataset. Returns (train_run_dirs, cal_run_dirs).
    Sequences are saved to EXTRACTED_DIR/<dataset>/<run_name>.pkl.
    """
    os.makedirs(cfg.EXTRACTED_DIR, exist_ok=True)
    out_dir = os.path.join(cfg.EXTRACTED_DIR, dataset)
    os.makedirs(out_dir, exist_ok=True)

    run_dirs = _find_run_dirs(dataset)
    print(f"\n[pipeline] Dataset: {dataset}  ({len(run_dirs)} runs found)")

    runs_by_goal: Dict[str, List[str]] = defaultdict(list)
    processed = 0

    for run_dir in run_dirs:
        # Build a unique name: for nested layout goal_007/t1_..., use goal_007_t1_...
        # For old flat layout goal_007_nom_v11_t1_..., basename is already unique.
        basename = os.path.basename(run_dir)
        parent   = os.path.basename(os.path.dirname(run_dir))
        if parent.startswith('goal_'):
            run_name = f"{parent}_{basename}"
        else:
            run_name = basename

        bag_dir  = os.path.join(run_dir, 'rosbag')
        out_pkl  = os.path.join(out_dir, f"{run_name}.pkl")

        # Skip already-processed runs
        if os.path.exists(out_pkl):
            if verbose:
                print(f"  [pipeline] skipping (cached): {run_name}")
            result = _load_result(run_dir)
            if result.get('status') == 'goal_reached':
                goal = _goal_from_run_dir(run_dir)
                runs_by_goal[goal].append(run_dir)
            continue

        # Only process successful runs
        result = _load_result(run_dir)
        if result.get('status') != 'goal_reached':
            if verbose:
                print(f"  [pipeline] skipping (status={result.get('status')}): {run_name}")
            continue

        print(f"  [pipeline] processing: {run_name}")

        try:
            frames = read_bag(bag_dir, verbose=verbose)
            if len(frames) < cfg.INPUT_SEQ_LEN + cfg.OUTPUT_SEQ_LEN:
                print(f"    WARNING: only {len(frames)} frames, skipping")
                continue

            from .sequence_builder import extract_route_from_bag
            from .State_Estimator.GraphBuilder import GraphBuilder
            route = extract_route_from_bag(bag_dir)
            shared_builder.route = route
            shared_builder.graph_builder = GraphBuilder(
                map_data              = shared_builder.map_data,
                route                 = route,
                min_dist_between_node = cfg.MIN_DIST_BETWEEN_NODES,
                connection_threshold  = cfg.CONNECTION_THRESHOLD,
                max_nodes             = cfg.MAX_GRAPH_NODES,
                min_nodes             = cfg.MIN_GRAPH_NODES,
            )

            sequences = shared_builder.build(frames, verbose=verbose,
                                             filter_mrm=filter_mrm)
            if not sequences:
                print(f"    WARNING: zero sequences built, skipping")
                continue

            with open(out_pkl, 'wb') as f:
                pickle.dump(sequences, f, protocol=4)

            print(f"    → {len(sequences)} sequences saved")
            goal = _goal_from_run_dir(run_dir)
            runs_by_goal[goal].append(run_dir)
            processed += 1

        except Exception as e:
            print(f"    ERROR processing {run_name}: {e}")
            if verbose:
                import traceback
                traceback.print_exc()

    print(f"  [pipeline] processed {processed} new runs")

    train_dirs, cal_dirs = _train_cal_split(
        runs_by_goal, cal_fraction=cfg.CAL_FRACTION
    )
    return train_dirs, cal_dirs


def assemble_splits(
    all_train_dirs: List[str],
    all_cal_dirs: List[str],
    verbose: bool = False,
):
    """
    Copy/symlink processed pkl files into TRAIN_DIR and CAL_DIR.
    Each pkl is a flat list of sequences; the split dirs collect them all.
    """
    os.makedirs(cfg.TRAIN_DIR, exist_ok=True)
    os.makedirs(cfg.CAL_DIR, exist_ok=True)

    def _run_name_for(run_dir: str) -> str:
        basename = os.path.basename(run_dir)
        parent   = os.path.basename(os.path.dirname(run_dir))
        return f"{parent}_{basename}" if parent.startswith('goal_') else basename

    def _link(run_dirs: List[str], dest_dir: str, tag: str):
        count = 0
        for run_dir in run_dirs:
            run_name = _run_name_for(run_dir)
            # Find which dataset this run belongs to
            for dataset in cfg.NOMINAL_DATASETS:
                src = os.path.join(cfg.EXTRACTED_DIR, dataset, f"{run_name}.pkl")
                if os.path.exists(src):
                    dst = os.path.join(dest_dir, f"{run_name}.pkl")
                    if not os.path.exists(dst):
                        os.symlink(src, dst)
                    count += 1
                    break
        print(f"  [pipeline] {tag}: {count} pkl files")

    _link(all_train_dirs, cfg.TRAIN_DIR,  "train set")
    _link(all_cal_dirs,   cfg.CAL_DIR,    "cal set")


def main():
    parser = argparse.ArgumentParser(description="Build ST-GAT training sequences from rosbags")
    parser.add_argument('--datasets', nargs='+', default=cfg.NOMINAL_DATASETS,
                        help="Datasets to process (default: all nominal)")
    parser.add_argument('--verbose', action='store_true')
    args = parser.parse_args()

    # Guard against accidentally processing test data
    bad = set(args.datasets) & set(cfg.TEST_DATASETS)
    if bad:
        print(f"ERROR: test datasets cannot be used for training: {bad}")
        sys.exit(1)

    print("[pipeline] Loading map data (one-time, ~10s)...")
    from .State_Estimator.MapProcessor import MapProcessor
    map_processor = MapProcessor(cfg.MAP_FILE)
    shared_builder = SequenceBuilder(map_processor.map_data, route=[])

    all_train_dirs: List[str] = []
    all_cal_dirs: List[str]   = []

    for dataset in args.datasets:
        train_dirs, cal_dirs = process_dataset(dataset, shared_builder,
                                               verbose=args.verbose,
                                               filter_mrm=False)
        all_train_dirs.extend(train_dirs)
        all_cal_dirs.extend(cal_dirs)

    print(f"\n[pipeline] Total — train: {len(all_train_dirs)} runs, cal: {len(all_cal_dirs)} runs")
    assemble_splits(all_train_dirs, all_cal_dirs, verbose=args.verbose)
    print("[pipeline] Done.")


if __name__ == '__main__':
    main()
