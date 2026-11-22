"""
manage_goal_split.py — explicit, git-tracked train/calibration split
assignment, replacing the old "recompute-and-silently-append" mechanism.

Added 2026-10-03. Background: `st_gat/pipeline/run_pipeline.py`'s goal-split
policy used to be invoked fresh on every pipeline run, assigning each goal
to train/cal via a seeded shuffle over that run's full goal list — fine
until a dataset's goal set grows, at which point the shuffle's output for
ALL goals can change (adding one element to a Python list changes a
Fisher-Yates shuffle's entire output), with no record of what a goal was
assigned last time. Worse, the old symlink-assembly step only ever ADDED
symlinks into TRAIN_DIR/CAL_DIR, never removed one — so a changed
assignment left stale symlinks behind, undetected. This tool makes the
assignment an explicit, durable, auditable fact instead of a recomputed
one. The actual split logic lives in `st_gat/pipeline/goal_split.py`
(shared with run_pipeline.py's assemble_splits()) — this is the CLI over it.

  generate --dataset X                  — assign any goal in X not yet in
                                           the manifest, via the same
                                           seeded-shuffle policy as before,
                                           WITHOUT touching any existing
                                           assignment.
  set --dataset X --goal Y --split S    — explicit, auditable override
                                           (replaces manually moving a
                                           symlink).
  verify [--dataset X]                  — errors if any extracted goal has
                                           no manifest entry, or if
                                           TRAIN_DIR/CAL_DIR's actual
                                           symlinks don't exactly match what
                                           the manifest says.

Usage:
  python3 experiments/scripts/manage_goal_split.py generate --dataset nominal
  python3 experiments/scripts/manage_goal_split.py set --dataset nominal --goal goal_007 --split cal
  python3 experiments/scripts/manage_goal_split.py verify
"""
import argparse
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
sys.path.insert(0, REPO_DIR)

from st_gat.pipeline import config as cfg  # noqa: E402
from st_gat.pipeline import goal_split as gs  # noqa: E402


def cmd_generate(args):
    manifest = gs.load_manifest()
    ds_manifest = manifest.setdefault(args.dataset, {})
    by_goal = gs.extracted_runs_by_goal(args.dataset)
    unassigned = {g: names for g, names in by_goal.items() if g not in ds_manifest}

    if not unassigned:
        print(f"{args.dataset}: every extracted goal already has a manifest "
              f"entry — nothing to do.")
        return

    # Reuse the existing policy function UNCHANGED, applied only to the
    # unassigned subset — previously-recorded goals are never touched.
    _, cal_run_names = gs.train_cal_split(unassigned, cal_fraction=cfg.CAL_FRACTION)
    cal_set = set(cal_run_names)

    for goal, names in sorted(unassigned.items()):
        split = 'cal' if names[0] in cal_set else 'train'
        ds_manifest[goal] = split
        print(f"  {args.dataset}/{goal}: assigned '{split}' "
              f"(new goal, {len(names)} run(s))")

    gs.save_manifest(manifest)


def cmd_set(args):
    manifest = gs.load_manifest()
    ds_manifest = manifest.setdefault(args.dataset, {})
    prev = ds_manifest.get(args.goal, '(unassigned)')
    ds_manifest[args.goal] = args.split
    gs.save_manifest(manifest)
    print(f"{args.dataset}/{args.goal}: {prev} -> {args.split}")
    print(f"Re-run `python3 -m st_gat.pipeline.run_pipeline --datasets "
          f"{args.dataset}` then `manage_goal_split.py verify` to apply.")


def cmd_verify(args):
    manifest = gs.load_manifest()
    datasets = [args.dataset] if args.dataset else list(cfg.NOMINAL_DATASETS)
    expected_cal, expected_train, missing = gs.expected_split(datasets, manifest)
    ok = True

    for dataset, goal, n in missing:
        print(f"MISSING: {dataset}/{goal} has {n} extracted run(s) but no "
              f"manifest entry. Run: manage_goal_split.py generate --dataset {dataset}")
        ok = False

    if not args.dataset:
        # Only a full (all-datasets) check can compare against the full
        # TRAIN_DIR/CAL_DIR contents without false positives from datasets
        # that weren't scanned.
        actual_cal = gs.current_symlink_targets(cfg.CAL_DIR)
        actual_train = gs.current_symlink_targets(cfg.TRAIN_DIR)
        missing_cal, extra_cal = expected_cal - actual_cal, actual_cal - expected_cal
        missing_train, extra_train = expected_train - actual_train, actual_train - expected_train
        if missing_cal or extra_cal or missing_train or extra_train:
            ok = False
            if missing_cal:
                print(f"CAL_DIR is missing {len(missing_cal)} run(s) the manifest expects: {sorted(missing_cal)}")
            if extra_cal:
                print(f"CAL_DIR has {len(extra_cal)} STALE run(s) not in the manifest's cal set: {sorted(extra_cal)}")
            if missing_train:
                print(f"TRAIN_DIR is missing {len(missing_train)} run(s) the manifest expects: {sorted(missing_train)}")
            if extra_train:
                print(f"TRAIN_DIR has {len(extra_train)} STALE run(s) not in the manifest's train set: {sorted(extra_train)}")
            print("Re-run: python3 -m st_gat.pipeline.run_pipeline --datasets <dataset> to reconcile.")

    if ok:
        print("OK — every extracted goal has a manifest entry, and "
              "TRAIN_DIR/CAL_DIR match it exactly.")
    else:
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)

    p_gen = sub.add_parser('generate', help="Assign any not-yet-recorded goal in a dataset via the existing split policy.")
    p_gen.add_argument('--dataset', required=True, choices=cfg.NOMINAL_DATASETS)

    p_set = sub.add_parser('set', help="Explicitly (re)assign one goal's split — the auditable override.")
    p_set.add_argument('--dataset', required=True, choices=cfg.NOMINAL_DATASETS)
    p_set.add_argument('--goal', required=True)
    p_set.add_argument('--split', required=True, choices=['train', 'cal'])

    p_ver = sub.add_parser('verify', help="Check every extracted goal has a manifest entry and TRAIN_DIR/CAL_DIR match it.")
    p_ver.add_argument('--dataset', default=None, choices=cfg.NOMINAL_DATASETS)

    args = parser.parse_args()
    {'generate': cmd_generate, 'set': cmd_set, 'verify': cmd_verify}[args.command](args)


if __name__ == '__main__':
    main()
