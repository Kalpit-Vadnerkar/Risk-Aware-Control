"""
run_pipeline.py — entry point for the ST-GAT data processing pipeline.

  1. For every valid trial of the nominal dataset(s) (experiments/data/<campaign>/<goal>/t<N>_<ts>/),
     reads the rosbag → synchronized 10 Hz frames
  2. Builds sliding-window sequences (past + future)
  3. Saves EXTRACTED_DIR/<dataset>/t<N>_<ts>.pkl (cached; re-runs skip done trials)
  4. Makes TRAIN_DIR / CAL_DIR exactly match
     experiments/configs/goal_split_manifest.json (goal_split.py); a goal with
     no assignment fails loudly — run
     `experiments/scripts/manage_goal_split.py generate --dataset nominal` first

Usage (Autoware workspace sourced):
  python3 -m st_gat.pipeline.run_pipeline [--datasets nominal] [--verbose]

Output pkl: a list of sequence dicts {'past': [...], 'future': [...], 'graph': G, ...}.
Fault campaigns are never processed here (training is nominal-only).
"""

import argparse
import json
import os
import pickle
import sys
from typing import List

from . import config as cfg
from . import goal_split as gs
from .bag_reader import read_bag
from .sequence_builder import SequenceBuilder



# ── Helpers ────────────────────────────────────────────────────────────────

def _load_result(run_dir: str) -> dict:
    result_file = os.path.join(run_dir, 'result.json')
    if not os.path.exists(result_file):
        return {}
    with open(result_file) as f:
        return json.load(f)


# ── Main pipeline ──────────────────────────────────────────────────────────

def process_dataset(
    dataset: str,
    shared_builder: SequenceBuilder,
    verbose: bool = False,
    filter_mrm: bool = False,
) -> None:
    """
    Process all runs in a dataset — extracts sequences to
    EXTRACTED_DIR/<dataset>/<run_name>.pkl. Does NOT assign a train/cal
    split (that moved to an explicit, separate step 2026-10-03 — see
    assemble_splits() and experiments/scripts/manage_goal_split.py).
    """
    os.makedirs(cfg.EXTRACTED_DIR, exist_ok=True)
    # Schema-versioning guard (2026-08-07, see config.py's check_schema_manifest
    # docstring) -- verifies (or, on a fresh/empty dir, creates) a manifest
    # BEFORE the cache-skip check below can silently reuse a stale .pkl written
    # under an old feature vector/graph schema.
    cfg.check_schema_manifest(cfg.EXTRACTED_DIR, write_if_missing=True)
    out_dir = os.path.join(cfg.EXTRACTED_DIR, dataset)
    os.makedirs(out_dir, exist_ok=True)

    run_dirs = gs.find_run_dirs(dataset)
    print(f"\n[pipeline] Dataset: {dataset}  ({len(run_dirs)} runs found)")

    processed = 0

    for run_dir in run_dirs:
        run_name = os.path.basename(run_dir)
        bag_dir  = os.path.join(run_dir, 'rosbag')
        out_pkl  = os.path.join(out_dir, f"{run_name}.pkl")

        # Skip already-processed runs
        if os.path.exists(out_pkl):
            if verbose:
                print(f"  [pipeline] skipping (cached): {run_name}")
            continue

        # Only valid trials that reached the goal
        result = _load_result(run_dir)
        if result.get('status') != 'goal_reached' or not result.get('valid'):
            if verbose:
                print(f"  [pipeline] skipping (status={result.get('status')}, "
                      f"valid={result.get('valid')}): {run_name}")
            continue

        print(f"  [pipeline] processing: {run_name}")

        try:
            goal_id = gs.goal_from_run_dir(run_dir)
            frames = read_bag(bag_dir, goal_id=goal_id, verbose=verbose)
            if len(frames) < cfg.INPUT_SEQ_LEN + cfg.OUTPUT_SEQ_LEN:
                print(f"    WARNING: only {len(frames)} frames, skipping")
                continue

            from .sequence_builder import extract_route_from_bag, TrafficLightExpectationChecker
            from .State_Estimator.GraphBuilder import GraphBuilder
            route = extract_route_from_bag(bag_dir)
            shared_builder.route = route
            shared_builder.graph_builder = GraphBuilder(
                map_data              = shared_builder.map_data,
                route                 = route,
                min_dist_between_node = cfg.MIN_DIST_BETWEEN_NODES,
                connection_threshold  = cfg.CONNECTION_THRESHOLD,
                max_nodes             = cfg.MAX_GRAPH_NODES,
                radius_m              = cfg.GRAPH_RADIUS_M,
                routing_graph         = shared_builder.routing_graph,
            )
            # Route-dependent, same as graph_builder above — must be rebuilt
            # per trial too, or traffic_light_discrepancy silently stays 0
            # for every window (see sequence_builder.py's SequenceBuilder.__init__
            # comment on routing_graph).
            shared_builder._tl_checker = TrafficLightExpectationChecker(
                shared_builder.map_data, route)

            sequences = shared_builder.build(frames, verbose=verbose,
                                             filter_mrm=filter_mrm)
            if not sequences:
                print(f"    WARNING: zero sequences built, skipping")
                continue

            with open(out_pkl, 'wb') as f:
                pickle.dump(sequences, f, protocol=4)

            print(f"    → {len(sequences)} sequences saved")
            processed += 1

        except Exception as e:
            print(f"    ERROR processing {run_name}: {e}")
            if verbose:
                import traceback
                traceback.print_exc()

    print(f"  [pipeline] processed {processed} new runs")


def assemble_splits(datasets: List[str], verbose: bool = False):
    """
    Reconcile TRAIN_DIR/CAL_DIR's symlinks to EXACTLY match
    experiments/configs/goal_split_manifest.json (2026-10-03 — replaces the
    old recompute-a-shuffle-and-only-ever-add-symlinks mechanism; see
    st_gat/pipeline/goal_split.py's module docstring and
    experiments/scripts/manage_goal_split.py for why). Adds missing
    symlinks AND removes stale ones, so re-running this after any manifest
    change (or any dataset subset) converges to the same state every time.

    Fails loudly (does not silently default) if any extracted goal has no
    manifest entry yet — run `manage_goal_split.py generate --dataset X`
    first.
    """
    os.makedirs(cfg.TRAIN_DIR, exist_ok=True)
    os.makedirs(cfg.CAL_DIR, exist_ok=True)
    # Schema-versioning guard (2026-08-07) -- TrajectoryDataset checks
    # SEQUENCES_DIR's manifest (the parent of TRAIN_DIR/CAL_DIR) before
    # loading, regardless of how the .pkl files it finds there got there;
    # this is what actually makes that check meaningful, not just EXTRACTED_DIR's.
    cfg.check_schema_manifest(cfg.SEQUENCES_DIR, write_if_missing=True)

    manifest = gs.load_manifest()
    expected_cal, expected_train, missing = gs.expected_split(datasets, manifest)
    if missing:
        for dataset, goal, n in missing:
            print(f"  [pipeline] ERROR: {dataset}/{goal} has {n} extracted "
                  f"run(s) but no split-manifest entry. Run: "
                  f"experiments/scripts/manage_goal_split.py generate --dataset {dataset}")
        sys.exit(1)

    def _src_for(run_name: str):
        for dataset in datasets:
            src = os.path.join(cfg.EXTRACTED_DIR, dataset, f"{run_name}.pkl")
            if os.path.exists(src):
                return src
        return None

    # Exact: anything not in the manifest's split for these datasets (a run
    # of another dataset, a since-deleted trial) is removed.
    def _reconcile(expected_names, dest_dir: str, tag: str):
        current = gs.current_symlink_targets(dest_dir)
        added = removed = 0
        for run_name in expected_names - current:
            os.symlink(_src_for(run_name), os.path.join(dest_dir, f"{run_name}.pkl"))
            added += 1
        for run_name in current - expected_names:
            os.remove(os.path.join(dest_dir, f"{run_name}.pkl"))
            removed += 1
        print(f"  [pipeline] {tag}: {len(expected_names)} pkl files "
              f"(+{added} added, -{removed} removed)")

    _reconcile(expected_train, cfg.TRAIN_DIR, "train set")
    _reconcile(expected_cal,   cfg.CAL_DIR,   "cal set")


def main():
    parser = argparse.ArgumentParser(description="Build ST-GAT training sequences from rosbags")
    parser.add_argument('--datasets', nargs='+', default=cfg.NOMINAL_DATASETS,
                        help="Datasets to process (default: all nominal)")
    parser.add_argument('--verbose', action='store_true')
    args = parser.parse_args()

    # Training is nominal-only
    bad = set(args.datasets) & set(cfg.FAULT_DATASETS)
    if bad:
        print(f"ERROR: fault-campaign datasets cannot be used for training: {bad}")
        sys.exit(1)

    print("[pipeline] Loading map data (one-time, ~10s)...")
    from .State_Estimator.MapProcessor import MapProcessor
    map_processor = MapProcessor(cfg.MAP_FILE)
    shared_builder = SequenceBuilder(map_processor.map_data, route=[])

    for dataset in args.datasets:
        process_dataset(dataset, shared_builder, verbose=args.verbose, filter_mrm=False)

    assemble_splits(args.datasets, verbose=args.verbose)
    print("[pipeline] Done.")


if __name__ == '__main__':
    main()
