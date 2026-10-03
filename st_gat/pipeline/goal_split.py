"""
goal_split.py — shared train/calibration split-assignment logic, used by
both run_pipeline.py (which reconciles TRAIN_DIR/CAL_DIR against it) and
experiments/scripts/manage_goal_split.py (the CLI that edits/audits it).
Split into its own module to avoid run_pipeline.py <-> manage_goal_split.py
importing each other.

The split is an explicit, git-tracked manifest
(experiments/configs/goal_split_manifest.json) rather than a value
recomputed fresh on every pipeline run — see manage_goal_split.py's module
docstring for why (goal-set growth could silently reshuffle previously
-assigned goals; the old symlink step only ever added, never removed, so a
changed assignment left stale symlinks behind undetected).
"""
import json
import os
import random
from collections import defaultdict
from typing import Dict, List, Set, Tuple

from . import config as cfg

MANIFEST_PATH = os.path.join(cfg.REPO_ROOT, 'experiments', 'configs', 'goal_split_manifest.json')


# ── Raw-data scanning (moved here from run_pipeline.py 2026-10-03 — also
#    used by extraction itself, imported back from there) ────────────────

def find_run_dirs(dataset: str) -> List[str]:
    """
    Return sorted list of trial run directories for a dataset.

    Trials are nested by goal: experiments/data/<dataset>/<goal_id>/<trial>/
    (revised 2026-07-22 — was flat with the campaign name repeated in every
    trial dirname; see CLAUDE.md's directory-conventions section). Fixed
    2026-08-01: this previously only looked one level deep
    (<dataset>/<entry>/rosbag), so it silently found 0 runs for every
    dataset on the current layout.
    """
    dataset_dir = os.path.join(cfg.DATA_ROOT, dataset)
    if not os.path.isdir(dataset_dir):
        print(f"  [pipeline] WARNING: dataset dir not found: {dataset_dir}")
        return []

    runs = []
    for goal_entry in sorted(os.listdir(dataset_dir)):
        goal_dir = os.path.join(dataset_dir, goal_entry)
        if not os.path.isdir(goal_dir) or not goal_entry.startswith('goal_'):
            continue
        for trial_entry in sorted(os.listdir(goal_dir)):
            run_dir = os.path.join(goal_dir, trial_entry)
            if not os.path.isdir(run_dir):
                continue
            bag_dir = os.path.join(run_dir, 'rosbag')
            if not os.path.isdir(bag_dir):
                continue
            runs.append(run_dir)
    return runs


def goal_from_run_dir(run_dir: str) -> str:
    """
    Extract goal_id for a trial run directory.

    run_dir is a trial dir (e.g. '.../nom_v11/goal_016/t1_20260722_141240') —
    the goal_id is its PARENT directory's name, not derivable from the trial
    dirname itself (fixed 2026-08-01: this used to assume a flat
    'goal_XXX_<campaign>_tN_<timestamp>' dirname, which hasn't been the
    on-disk layout since 2026-07-22).
    """
    parent = os.path.basename(os.path.dirname(run_dir))
    if parent.startswith('goal_'):
        return parent
    return os.path.basename(run_dir)


def train_cal_split(
    run_dirs_by_goal: Dict[str, List[str]],
    cal_fraction: float,
    seed: int = 42,
) -> Tuple[List[str], List[str]]:
    """
    Split at the GOAL level: whole goals assigned wholesale to train or cal,
    not individual runs within a goal (see run_pipeline.py's git history for
    why — splitting whole goals works the same whether a goal has 1, 2, or
    20 runs, and is safer against leakage: a held-out goal is a different
    route/geometry entirely, not just a repeat trial of one already seen in
    training).

    This is the POLICY function for a never-before-seen goal — see
    experiments/scripts/manage_goal_split.py's `generate` command, the only
    caller. Once a goal has an explicit manifest entry, it is never
    re-run through this function again, so growing a dataset's goal set
    cannot reshuffle an already-recorded assignment.
    """
    rng = random.Random(seed)
    goals = sorted(run_dirs_by_goal.keys())
    rng.shuffle(goals)
    n_cal_goals = max(1, round(len(goals) * cal_fraction)) if goals else 0
    cal_goals = set(goals[:n_cal_goals])

    train_dirs, cal_dirs = [], []
    for goal, dirs in sorted(run_dirs_by_goal.items()):
        (cal_dirs if goal in cal_goals else train_dirs).extend(dirs)

    return train_dirs, cal_dirs


# ── Manifest I/O ─────────────────────────────────────────────────────────

def load_manifest() -> dict:
    if not os.path.exists(MANIFEST_PATH):
        return {}
    with open(MANIFEST_PATH) as f:
        return json.load(f)


def save_manifest(manifest: dict):
    with open(MANIFEST_PATH, 'w') as f:
        json.dump(manifest, f, indent=2, sort_keys=True)
        f.write('\n')
    print(f"Wrote {MANIFEST_PATH}")


# ── Derived state ────────────────────────────────────────────────────────

def extracted_runs_by_goal(dataset: str) -> Dict[str, List[str]]:
    """goal_id -> [run_name, ...] for runs that exist BOTH in the raw
    experiments/data/<dataset>/goal_*/ layout AND as an already-extracted
    .pkl under EXTRACTED_DIR/<dataset>/ — i.e. exactly what assemble_splits
    can actually symlink."""
    by_goal = defaultdict(list)
    for run_dir in find_run_dirs(dataset):
        run_name = os.path.basename(run_dir)
        pkl_path = os.path.join(cfg.EXTRACTED_DIR, dataset, f"{run_name}.pkl")
        if os.path.exists(pkl_path):
            by_goal[goal_from_run_dir(run_dir)].append(run_name)
    return by_goal


def current_symlink_targets(dest_dir: str) -> Set[str]:
    """run_name (no .pkl) for every .pkl symlink currently in dest_dir."""
    if not os.path.isdir(dest_dir):
        return set()
    return {os.path.splitext(f)[0] for f in os.listdir(dest_dir) if f.endswith('.pkl')}


def expected_split(
    datasets: List[str], manifest: dict
) -> Tuple[Set[str], Set[str], List[Tuple[str, str, int]]]:
    """
    For the given datasets, read `manifest` and return
    (expected_cal_run_names, expected_train_run_names, missing), where
    `missing` is a list of (dataset, goal, n_runs) for every extracted goal
    that has no manifest entry yet.
    """
    expected_cal, expected_train, missing = set(), set(), []
    for dataset in datasets:
        ds_manifest = manifest.get(dataset, {})
        for goal, names in sorted(extracted_runs_by_goal(dataset).items()):
            split = ds_manifest.get(goal)
            if split is None:
                missing.append((dataset, goal, len(names)))
                continue
            (expected_cal if split == 'cal' else expected_train).update(names)
    return expected_cal, expected_train, missing
