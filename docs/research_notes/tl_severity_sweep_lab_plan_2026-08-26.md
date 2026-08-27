# TL severity sweep — lab session plan (2026-08-26)

Concrete, runnable checklist for the next lab session. Goal: collect the
data for Paper 1's C6 comparison (the severity dose-response curve) and,
as a byproduct, expand the held-out nominal calibration set at goal_007/
012 — closing the exact gap that limited the 2026-08-25 ramp pilot to one
clean goal (goal_026) out of three.

**Read `docs/research_notes/layer1_paper_structure_2026-08-25.md` §C6 and
the project memory's 2026-08-25 TL-severity-pilot entry first** for why
this design looks the way it does (a discrete, fixed-severity design was
chosen specifically because the existing `tl_fault_ramp` data confounds
severity with elapsed-time-in-zone / physical proximity to the
intersection — confirmed directly, not assumed).

## What to run, in order

AWSIM (Terminal 1) and Autoware (Terminal 2) must already be running
before any of this (see `README.md`'s startup section).

```bash
cd /home/kvadner/Desktop/Dissertation/Risk-Aware-Control

# 1. Matched nominal control runs — goal_007/012/026, 3 goals x 2 new
#    trials each = 6 new nominal trials. This is the fault-free reference
#    arm of the matched pair AND directly expands the held-out calibration
#    set (currently only 7 trials total, flagged as load-bearing-weak by
#    three independent sources -- see CLAUDE.md).
./collect.sh nom_v11 --goals goal_007,goal_012,goal_026 --trials 2

# 2. Fixed-severity TL fault sweep — 3 severity levels x 3 goals x 2
#    trials = 18 new fault trials. (Campaign definitions added to
#    collect.sh 2026-08-26 -- see its "Fixed-severity TL confidence
#    tiers" comment block for why these exist alongside tl_fault_ramp,
#    not instead of it.)
./run_fault_campaigns.sh --goals goal_007,goal_012,goal_026 --trials 2 \
    --campaigns "tl_fault_fixed_030 tl_fault_fixed_050 tl_fault_fixed_070"
```

Total new AWSIM trials: 6 (nominal) + 18 (fault) = **24 trials**. Both
commands auto-resume (re-running the exact same command after an
interruption skips whatever's already on disk and picks up the deficit —
no manual bookkeeping needed if a session gets cut short).

If time is short, the fault sweep is the priority (it's the new result);
the nominal control runs are valuable but partially redundant with
existing nom_v11 data — see the verification step below for why they
still matter specifically for goal_007/012.

## After collection: the one verification step that actually matters

**Before treating any of this as usable calibration data**, confirm the
new goal_007/012 nominal trials actually land in the HELD-OUT calibration
set, not the training set. The existing pipeline's train/cal split is a
per-goal 80/20 stratified split over ALL of that goal's trials (existing +
new) — it is NOT guaranteed that a freshly-collected trial ends up in
`CAL_DIR` just because it's new. This is exactly the gap that limited the
2026-08-25 pilot: goal_007/012's EXISTING nom_v11 trials both landed in
`TRAIN_DIR` (the model has seen them), only goal_026's landed in `CAL_DIR`.

```bash
# after running the pipeline (python3 -m st_gat.pipeline.run_pipeline):
ls st_gat/data/h30_30/sequences/calibration/ | xargs -I{} basename {} .pkl
# cross-reference against experiments/data/nom_v11/goal_007/ and goal_012/'s
# run directory names -- if NEITHER new goal_007 trial nor NEITHER new
# goal_012 trial appears, the split didn't do what this experiment needs.
```

If the automatic split doesn't put at least one new goal_007 and one new
goal_012 trial into `CAL_DIR`: manually move that trial's pkl symlink from
`TRAIN_DIR` to `CAL_DIR` (delete the `TRAIN_DIR` symlink, create the
matching one in `CAL_DIR`, pointing at the same
`st_gat/data/h30_30/extracted/nom_v11/<run_name>.pkl`) and **do not use
that trial's runs for anything else that assumes it was held out** (i.e.
don't retrain further using it, or the point becomes moot). This is a
manual, deliberate override for this specific experiment's needs, not a
change to the general pipeline's split logic.

## Analysis (already built, ready to run once data lands)

```bash
source /opt/ros/humble/setup.bash
source /home/kvadner/Desktop/Dissertation/autoware/install/setup.bash
source .venv/bin/activate
python3 -m st_gat.pipeline.run_pipeline --datasets nom_v11   # re-extract with the new trials
python3 experiments/scripts/tl_severity_sweep_analysis.py    # needs a small update -- see below
```

`tl_severity_sweep_analysis.py` currently reconstructs severity from
`tl_confidence_ramp`'s decay formula. For the new FIXED-severity trials,
severity is simply `1 - confidence_scale` for the whole fault-active
window (no reconstruction needed) — reading it straight from each trial's
`fault_log.jsonl` `tl_fault_start` event's `params.confidence_scale`. This
is a small, additive change (a new branch alongside the existing ramp
reconstruction, not a rewrite) — not yet made as of this note, since there
was no fixed-severity data to test it against; do this before running the
analysis on the new data, not after.

## What "done" looks like

A dose-response plot (reusing `tl_severity_sweep_analysis.py`'s existing
plotting code) with 4 real x-axis points instead of the ramp's noisy
reconstructed continuum: severity 0.0 (nominal control), 0.3, 0.5, 0.7 —
per feature, per goal, with the goal_026 curve directly comparable
goal-for-goal against the 2026-08-25 ramp pilot's goal_026 result as a
sanity check that the two methods agree at least directionally.
