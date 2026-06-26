"""
run_pipeline.py — entry point for the ST-GAT data processing pipeline.

What it does:
  1. Scans all NOMINAL_DATASETS under DATA_ROOT
  2. For each successful run, reads the rosbag → synchronized frames
  3. Builds sliding-window sequences (past 30 + future 30 at 10 Hz)
  4. Splits train/cal by CAL_FRACTION, stratified by goal
  5. Saves .pkl files to TRAIN_DIR and CAL_DIR

Usage (with Autoware workspace sourced):
  source /home/df/Desktop/Kalpit-2026/Risk-Aware-Control/autoware/install/setup.bash
  python3 -m st_gat.pipeline.run_pipeline [--datasets baseline_all nom_v5] [--verbose]

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
import os
import pickle
import random
import sys
from collections import defaultdict
from typing import Dict, List, Tuple

import sys as _sys
import os as _os
_REF_ROOT = _os.path.normpath(_os.path.join(_os.path.dirname(__file__), '..', '..', '..', 'Graph-Scene-Representation-and-Prediction'))
if _REF_ROOT not in _sys.path:
    _sys.path.insert(0, _REF_ROOT)

from . import config as cfg
from .bag_reader import read_bag
from .sequence_builder import SequenceBuilder


# ── Helpers ────────────────────────────────────────────────────────────────

def _find_run_dirs(dataset: str) -> List[str]:
    """Return sorted list of run directories for a dataset."""
    dataset_dir = os.path.join(cfg.DATA_ROOT, dataset)
    if not os.path.isdir(dataset_dir):
        print(f"  [pipeline] WARNING: dataset dir not found: {dataset_dir}")
        return []

    runs = []
    for entry in sorted(os.listdir(dataset_dir)):
        run_dir = os.path.join(dataset_dir, entry)
        if not os.path.isdir(run_dir):
            continue
        bag_dir = os.path.join(run_dir, 'rosbag')
        if not os.path.isdir(bag_dir):
            continue
        runs.append(run_dir)
    return runs


def _load_result(run_dir: str) -> dict:
    result_file = os.path.join(run_dir, 'result.json')
    if not os.path.exists(result_file):
        return {}
    with open(result_file) as f:
        return json.load(f)


def _goal_from_run_dir(run_dir: str) -> str:
    """Extract goal_id from the directory name (e.g., 'goal_007_...' → 'goal_007')."""
    basename = os.path.basename(run_dir)
    parts = basename.split('_')
    if len(parts) >= 2 and parts[0] == 'goal':
        return f"{parts[0]}_{parts[1]}"
    return basename


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
    prediction is handled primarily by the nom_v5/v7/v10 held-out runs, which
    have 5-6 runs per goal and naturally contribute ≥1 run to cal per goal.
    """
    rng = random.Random(seed)
    train_dirs, cal_dirs = [], []

    for goal, dirs in sorted(run_dirs_by_goal.items()):
        shuffled = dirs[:]
        rng.shuffle(shuffled)
        n_cal = round(len(shuffled) * cal_fraction)   # 0 is fine for single-run goals
        cal_dirs.extend(shuffled[:n_cal])
        train_dirs.extend(shuffled[n_cal:])

    return train_dirs, cal_dirs


# ── Main pipeline ──────────────────────────────────────────────────────────

def process_dataset(
    dataset: str,
    shared_builder: SequenceBuilder,
    verbose: bool = False,
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
        run_name = os.path.basename(run_dir)
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
            from State_Estimator.GraphBuilder import GraphBuilder
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

            sequences = shared_builder.build(frames, verbose=verbose)
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

    def _link(run_dirs: List[str], dest_dir: str, tag: str):
        count = 0
        for run_dir in run_dirs:
            run_name = os.path.basename(run_dir)
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
    import Data_Curator.config as _dc_config
    _dc_config.config.MAP_FILE = cfg.MAP_FILE   # override relative path with absolute
    from State_Estimator.MapProcessor import MapProcessor
    map_processor = MapProcessor()
    shared_builder = SequenceBuilder(map_processor.map_data, route=[])

    all_train_dirs: List[str] = []
    all_cal_dirs: List[str]   = []

    for dataset in args.datasets:
        train_dirs, cal_dirs = process_dataset(dataset, shared_builder, verbose=args.verbose)
        all_train_dirs.extend(train_dirs)
        all_cal_dirs.extend(cal_dirs)

    print(f"\n[pipeline] Total — train: {len(all_train_dirs)} runs, cal: {len(all_cal_dirs)} runs")
    assemble_splits(all_train_dirs, all_cal_dirs, verbose=args.verbose)
    print("[pipeline] Done.")


if __name__ == '__main__':
    main()
