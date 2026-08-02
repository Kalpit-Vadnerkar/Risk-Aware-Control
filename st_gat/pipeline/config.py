"""
Pipeline configuration for ST-GAT retraining.

Design decisions documented here:

TRAINING DATA
─────────────
  Train  (80%): baseline_all + nom_v11
  Cal    (20%): held-out split from nom_v11 (single max-velocity operating condition —
                per-speed calibration dropped 2026-07-21, all experiments run at max
                velocity now)
  Test      : obs_recovery, obs_noescape  — NEVER in training.
                Avoidance maneuvers must produce anomalous residuals;
                training on them would teach the model that lane changes are nominal.

LOCALIZATION SOURCE
───────────────────
  Ego position/orientation: /localization/kinematic_state (Autoware EKF output)
  NOT ground truth — RISE must work without it in deployment.

UNCERTAINTY FEATURES (new vs T-ITS paper)
──────────────────────────────────────────
  position_uncertainty (2): x_var, y_var from /localization/pose_with_covariance
  These are model INPUTS so ST-GAT can condition predictions on localization confidence.
  Observation: in nominal runs x_var ≈ y_var ≈ 0.003 m² — expect increases in
  unusual situations (poor localizer convergence, novel map regions).
  NOT used: yaw_var (too small, ~9e-5 rad²) and velocity_var (confounded by speed).

FEATURE VECTOR (per timestep, 10 Hz)
──────────────────────────────────────
  Flat/scalar features (FEATURE_SIZES, concatenated directly — 12 dims):
    position (2): scaled x, y
    velocity (2): longitudinal, lateral
    steering (1): actual tire angle  [/vehicle/status/steering_status]
    acceleration (1): commanded accel [/control/command/control_cmd]
    traffic_light_detected (1): 1 if upcoming lane node has traffic light (map channel)
    position_uncertainty (2): x_var, y_var from EKF covariance
    traffic_light_state (1): perceived color × confidence [0=none/UNKNOWN,
      0.33=green, 0.67=amber, 1.0=red, scaled down toward 0 as confidence drops]
    has_adjacent_lane (1): 1 if lanelet2 routing graph has adjacent lane (HD map)
    traffic_light_discrepancy (1): 1 if traffic_light_detected==1 (map expects
      a TL here) but perception found nothing usable — blackout, all-UNKNOWN,
      or any other detection failure, regardless of which. Added because the
      goal_007 fault campaigns showed the perception-only traffic_light_state
      feature can't distinguish "genuinely no TL nearby" from "TL nearby but
      undetected due to a fault" — exactly the discrepancy this project needs
      to detect, made an explicit first-class signal instead of something the
      model must infer from two other features. Also fixed traffic_light_state
      itself to fold in confidence (previously color-only, so a confidence-
      degradation fault like tl_confidence was invisible to it entirely even
      though color stayed correct).

  Object detection (objects_set/objects_mask, NOT part of FEATURE_SIZES —
  handled separately in model.py via a permutation-invariant set encoder,
  added 2026-08-02, replacing the old object_distance/closest_object_velocity
  scalars that collapsed every tracked object to one "nearest object"
  heuristic before the model ever saw the data): up to MAX_TRACKED_OBJECTS
  objects, each OBJECT_FEATURE_DIM-dim (relative position, speed, a 4-way
  classification group), padded/masked. Input-only — no output head, since
  objects don't have a hard map-grounded prior the way traffic lights do
  (docs/theoretical_framework.md §3.1).

  Total flat dims: 12 (was 14 before object_distance/closest_object_velocity
  were removed 2026-08-02) + the object-set tensor. This, and the position-
  feature coordinate-frame fix from the same session, both invalidate every
  checkpoint/extracted sequence from before 2026-08-02 — retrain against a
  fresh extraction.

SYNCHRONIZATION
───────────────
  Master clock: /perception/object_recognition/tracking/objects (~10 Hz)
  Other topics forward-filled to each objects message timestamp.
  Max staleness: 300 ms — if a topic has no message within 300ms, frame is dropped.

SEQUENCE PARAMETERS (same as T-ITS paper)
──────────────────────────────────────────
  input_seq_len  = 30 timesteps = 3 seconds of history
  output_seq_len = 30 timesteps = 3 seconds of prediction
  stride         = 1 (maximum sequence density — dataset augmentation via overlap)

STOPPED-PERIOD FILTERING
─────────────────────────
  Periods where vehicle speed < 0.05 m/s for > 3 consecutive seconds are removed.
  Short stops (red lights, momentary yielding) are kept.
  This matches the original MessageCleaner.process_velocity_data() logic.
"""

import os
import torch

# ── Paths ──────────────────────────────────────────────────────────────────

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

DATA_ROOT    = os.path.join(REPO_ROOT, 'experiments', 'data')
MAP_FILE     = os.path.normpath(os.path.join(REPO_ROOT, '..', 'Map', 'nishishinjuku_autoware_map', 'lanelet2_map.osm'))
OUTPUT_ROOT  = os.path.join(REPO_ROOT, 'st_gat', 'data')
MODEL_ROOT   = os.path.join(REPO_ROOT, 'st_gat', 'models')

# EXTRACTED_DIR/SEQUENCES_DIR/TRAIN_DIR/CAL_DIR/CHECKPOINT_DIR/model_path are
# defined further down, after HORIZON_TAG (needs INPUT_SEQ_LEN/OUTPUT_SEQ_LEN,
# in "Sequence parameters" below) — see docs/theoretical_framework.md §4 /
# docs/stgat_pipeline_plan.md: a horizon sweep needs each horizon's extracted
# data and checkpoint to live side by side, not silently overwrite each other.

# ── Data split ─────────────────────────────────────────────────────────────

# Datasets to use for training + calibration (nominal behavior only).
# nom_v5/nom_v7 dropped 2026-07-21: all experiments now run at max velocity only,
# so there's a single operating condition instead of per-speed calibration sets.
NOMINAL_DATASETS = ['baseline_all', 'nom_v11']

# Fault campaigns actually collected on this repo's current research direction
# (replaces the stale TEST_DATASETS = ['obs_recovery', 'obs_noescape',
# 'obs_stuck'] — those obs_* scenario campaigns are deprioritized per TODO.md's
# 2026-07-24 reframe and aren't present under experiments/data/ on this
# machine). Used by st_gat/residuals.py for trace/residual computation.
FAULT_DATASETS = [
    'imu_fault_s1', 'imu_fault_s3', 'imu_fault_scale', 'imu_fault_stuck',
    'tl_fault_s2', 'tl_fault_s3', 'tl_fault_s4', 'tl_fault_ramp',
]

# Trials excluded from st_gat/residuals.py's analysis for a confirmed data
# quality issue — NOT deleted (a real AWSIM/Autoware collection run is
# expensive to redo; keeping the raw data lets a rerun be compared against
# it later). Each key is (dataset, goal_id, trial_dirname).
EXCLUDED_TRIALS = {
    ('imu_fault_scale', 'goal_026', 't1_20260728_201708'):
        "ekf_gt_divergence_m in-fault mean (0.247m) is LOWER than "
        "out-of-fault mean (2.572m) — backwards from every other "
        "imu_fault_scale trial and from the fault's expected effect (gyro "
        "bias should corrupt EKF twist during the fault window, raising "
        "in-fault divergence, not lowering it). Consistent with a "
        "2026-07-28 finding (project memory) that this trial's fault effect "
        "looks confounded by the vehicle stopping near an unresolved "
        "traffic light rather than showing the intended IMU-scale-fault "
        "structural effect. Confirmed still true 2026-08-02 via "
        "compare_fault_vs_nominal.py. Flagged then for a rerun that never "
        "happened — exclude until goal_026 is re-collected for this campaign.",
}

# Train/calibration split (per dataset, stratified by goal)
CAL_FRACTION = 0.20

# Speed labels (currently unused — kept for the single remaining operating condition)
SPEED_LABELS = {
    'baseline_all': 'default',
    'nom_v11': '11ms',
}

# ── Bag reading ─────────────────────────────────────────────────────────────

# Topics extracted from each bag
TOPICS = {
    'kinematic_state':   '/localization/kinematic_state',
    'pose_covariance':   '/localization/pose_with_covariance',
    'velocity':          '/vehicle/status/velocity_status',
    'steering':          '/vehicle/status/steering_status',
    'control':           '/control/command/control_cmd',
    'objects':           '/perception/object_recognition/tracking/objects',
    # NOT '/perception/traffic_light_recognition/traffic_signals' (Autoware's
    # real, unmodified recognition output) — behavior_planning actually consumes
    # traffic_signals_faulted (fault_injector.py's relay output, identical to the
    # real signal in passthrough/nominal, but the ONLY place a TL fault is
    # visible). Reading the unmodified topic would make every camera/TL fault
    # invisible to this feature regardless of severity — see
    # docs/research_notes/periodic_fault_strategy.md and README.md item 8.
    'traffic_lights':    '/perception/traffic_light_recognition/traffic_signals_faulted',
}

# objects is the master clock at ~10 Hz; other topics forward-fill
MASTER_CLOCK_TOPIC = 'objects'

# Drop frame if any topic is more than this stale (seconds)
MAX_STALENESS_SEC = 0.30

# ── Traffic-light group scoping (fixes the entity-collapse bug — see
#    docs/stgat_pipeline_plan.md §1.11) ─────────────────────────────────────
# Same file and radius fault_injector.py/compare_fault_vs_nominal.py already
# use to pick the ONE traffic_light_group_id governing the vehicle's current
# lanelet, so a TL fault's effect isn't outvoted by other, unfaulted groups
# in the same perception message. Not every goal has an entry (only goals
# that have had TL fault campaigns computed) — bag_reader falls back to the
# old pooled-across-all-groups behavior when a goal has no zones entry.
TL_ZONES_FILE      = os.path.join(REPO_ROOT, 'experiments', 'configs', 'tl_zones.json')
TL_ZONE_RADIUS_M   = 40.0

# Minimum speed before we consider the vehicle "stopped" (m/s)
STOPPED_THRESHOLD_MS = 0.05

# Maximum consecutive stopped duration to keep (seconds)
MAX_STOPPED_DURATION_SEC = 3.0

# ── Feature scaling ────────────────────────────────────────────────────────
# Scales raw values into [0, 1] for model input

POSITION_SCALING   = 10.0   # Applied on top of graph-normalized position
VELOCITY_SCALING   = 10.0
STEERING_SCALING   = 10.0
ACCEL_SCALING      = 10.0

# Physical ranges for [0,1] clamped normalization
VELOCITY_X_RANGE   = (-0.0,  12.0)   # m/s longitudinal
VELOCITY_Y_RANGE   = (-0.4,   0.4)   # m/s lateral
STEERING_RANGE     = (-0.5,   0.5)   # radians
ACCEL_RANGE        = (-1.0,   1.0)   # m/s²

# Position: real-metre displacement from the window's own last-observed
# frame, scaled to [-1, 1] by this FIXED constant (added 2026-08-02,
# replacing per-window bbox-relative scaling — see sequence_builder.py's
# _scale_position_relative for why the old scheme made the model's own
# position target's units inconsistent across training examples).
# 11.11 m/s (map speed limit) * 6s window ~= 67m one-way; 100m gives headroom
# without wasting most of the range on rare cases.
POSITION_DISPLACEMENT_RANGE_M = 100.0

# Position uncertainty scaling: raw EKF covariance in m²
# Nominal ≈ 0.003 m² → scale so nominal ≈ 0.1 (manageable input magnitude)
# Cap at 0.5 m² (localization has catastrophically failed above this)
UNCERTAINTY_SCALE  = 30.0    # multiply m² values
UNCERTAINTY_CAP    = 0.5     # cap before scaling (m²)

# ── Object detection set representation (added 2026-08-02) ─────────────────
# Replaces the old object_distance/closest_object_velocity scalars, which
# collapsed every tracked object to a single "nearest object" heuristic
# BEFORE the model ever saw the data — exactly the entity-collapse pattern
# docs/stgat_pipeline_plan.md's Stage 0 warns against (already found and
# fixed once for traffic lights, §1.11). Each tracked object is now its own
# feature vector; a permutation-invariant set encoder in model.py (not this
# preprocessing step) learns what to attend to across them. Input-only —
# no output head/residual on this representation (objects don't have a
# hard map-grounded prior the way traffic lights do — see
# docs/theoretical_framework.md §3.1 — so there's no negative-evidence
# argument for scoring a prediction against it, only for conditioning on it).
MAX_TRACKED_OBJECTS = 8      # K: objects beyond this (by distance) are dropped, not collapsed
OBJECT_REL_POS_RANGE  = (-50.0, 50.0)   # m, object position relative to ego (dx, dy)
OBJECT_SPEED_RANGE    = (0.0, 20.0)     # m/s, object speed magnitude (frame-invariant —
                                         # deliberately NOT an ego-relative velocity vector;
                                         # TrackedObjects' twist frame vs. VelocityReport's
                                         # longitudinal/lateral frame isn't confirmed to
                                         # match, and getting that subtraction wrong
                                         # silently would repeat this session's position-
                                         # frame bug in a new, harder-to-notice place)
# Autoware's ObjectClassification label -> a coarse 4-way group (avoids either
# collapsing all classes to one, or a full 12-way one-hot for classes this
# sim rarely produces). 0=unknown, 1=vehicle, 2=vulnerable road user, 3=other.
OBJECT_CLASS_GROUP = {
    0: 0,                    # UNKNOWN
    1: 1, 2: 1, 3: 1, 4: 1,  # CAR, TRUCK, BUS, TRAILER -> vehicle
    5: 2, 6: 2, 7: 2,        # MOTORCYCLE, BICYCLE, PEDESTRIAN -> vulnerable road user
    # 8 (ANIMAL), 9 (HAZARD), 10 (OVER_DRIVABLE), 11 (UNDER_DRIVABLE) -> other (default)
}
OBJECT_CLASS_GROUPS = 4
OBJECT_FEATURE_DIM = 2 + 1 + OBJECT_CLASS_GROUPS   # rel_x, rel_y, speed, class one-hot

# ── Graph parameters (unchanged from T-ITS paper) ─────────────────────────

NODE_FEATURES          = 4       # x, y, traffic_light_node, path_node
MAX_GRAPH_NODES        = 150
MIN_GRAPH_NODES        = 150
MIN_DIST_BETWEEN_NODES = 5       # metres
CONNECTION_THRESHOLD   = 5       # metres

# ── Sequence parameters ────────────────────────────────────────────────────

INPUT_SEQ_LEN  = 30    # 3 seconds of history at 10 Hz
OUTPUT_SEQ_LEN = 30    # 3 seconds of prediction
STRIDE         = 1

# Horizon-tagged paths (added 2026-08-01 — docs/theoretical_framework.md §4):
# the eventual lead-time-vs-horizon sweep (TODO.md P1.3) needs multiple
# INPUT_SEQ_LEN/OUTPUT_SEQ_LEN combinations' extracted sequences and
# checkpoints to coexist without one silently overwriting another's cache.
HORIZON_TAG = f"h{INPUT_SEQ_LEN}_{OUTPUT_SEQ_LEN}"

EXTRACTED_DIR  = os.path.join(OUTPUT_ROOT, HORIZON_TAG, 'extracted')    # Stage 1: per-run 10Hz dicts
SEQUENCES_DIR  = os.path.join(OUTPUT_ROOT, HORIZON_TAG, 'sequences')    # Stage 2: pkl sequence files
TRAIN_DIR      = os.path.join(SEQUENCES_DIR, 'train')
CAL_DIR        = os.path.join(SEQUENCES_DIR, 'calibration')
CHECKPOINT_DIR = os.path.join(REPO_ROOT, 'st_gat', 'checkpoints', HORIZON_TAG)

# REFERENCE_POINTS (the T-ITS reference repo's bag-frame -> ~3500,1800 "local
# frame" affine transform) removed 2026-08-02 — it was calibrated for that
# repo's own guessed-origin map projector (LocalCartesianProjector), a
# different one than this pipeline's MGRSProjector (MapProcessor.py), which
# already loads the map in the same bag/MGRS frame ego and object positions
# from the rosbag are in. Running ego/object positions through it here made
# every position clamp to [0,0] against the graph's bag-frame bounds (and,
# because traffic_light_detected looks up the nearest GRAPH node to that same
# clamped position, made the map-expectation channel uniformly 0 too,
# regardless of real vehicle position) — found live, all sequences extracted
# before this fix have degenerate position/object_distance/has_adjacent_lane/
# traffic_light_detected and must be re-extracted. See sequence_builder.py's
# _process_frame for the fix (use raw bag-frame Point(x, y) directly, no
# conversion — everything is already in the same frame).

# ── Model config (input feature sizes — updated for uncertainty) ───────────

# object_distance/closest_object_velocity removed 2026-08-02 — replaced by
# the per-object set representation (objects_set/objects_mask, handled
# separately in model.py's forward(), not part of this flat-concat dict —
# see OBJECT_FEATURE_DIM/MAX_TRACKED_OBJECTS above).
FEATURE_SIZES = {
    'position':                 2,   # scaled x, y
    'velocity':                 2,   # longitudinal, lateral
    'steering':                 1,
    'acceleration':             1,
    'traffic_light_detected':   1,   # graph node: upcoming lane has traffic light (map)
    'traffic_light_state':      1,   # perception: most restrictive color × confidence
    'traffic_light_discrepancy': 1,  # map expects a TL, perception found none (added 2026-07-23)
    'has_adjacent_lane':        1,   # lanelet2 routing graph: adjacent lane exists
    'uncertainty':              2,   # EKF x_var, y_var
}

MODEL_CONFIG = {
    'graph_sizes':   {'node_features': NODE_FEATURES, 'number_of_nodes': MAX_GRAPH_NODES},
    'feature_sizes': FEATURE_SIZES,
    'object_feature_dim':    OBJECT_FEATURE_DIM,
    'max_tracked_objects':   MAX_TRACKED_OBJECTS,
    'num_epochs':    200,
    'batch_size':    128,
    'hidden_size':   256,
    'num_layers':    2,
    'learning_rate': 0.0004,
    'input_seq_len': INPUT_SEQ_LEN,
    'output_seq_len': OUTPUT_SEQ_LEN,
    'dropout_rate':  0.2,
    'position_scaling_factor':     POSITION_SCALING,
    'velocity_scaling_factor':     VELOCITY_SCALING,
    'steering_scaling_factor':     STEERING_SCALING,
    'acceleration_scaling_factor': ACCEL_SCALING,
    'train_data_folder': TRAIN_DIR,
    'cal_data_folder':   CAL_DIR,
    'model_path':        os.path.join(MODEL_ROOT, HORIZON_TAG, 'st_gat_rise.pth'),
    'device':            torch.device('cuda' if torch.cuda.is_available() else 'cpu'),
}
