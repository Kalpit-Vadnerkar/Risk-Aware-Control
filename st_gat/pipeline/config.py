"""
Pipeline configuration for ST-GAT retraining.

Design decisions documented here:

TRAINING DATA
─────────────
  Train  (80%): baseline_all + nom_v5 + nom_v7 + nom_v10
  Cal    (20%): held-out split from each speed-level dataset
                (used for conformal prediction calibration at each constraint level)
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
  Original 6 features (T-ITS):
    position (2): scaled x, y
    velocity (2): longitudinal, lateral
    steering (1): actual tire angle  [/vehicle/status/steering_status]
    acceleration (1): commanded accel [/control/command/control_cmd]
    object_distance (1): distance to nearest tracked object
    traffic_light_detected (1): 1 if red/amber in ego's upcoming lane nodes
  New 2 features (RISE):
    position_uncertainty (2): x_var, y_var from EKF covariance
  Total: 10 features per timestep

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
MAP_FILE     = os.path.join(REPO_ROOT, 'Shinjuku-Map', 'map', 'lanelet2_map.osm')
OUTPUT_ROOT  = os.path.join(REPO_ROOT, 'st_gat', 'data')

EXTRACTED_DIR  = os.path.join(OUTPUT_ROOT, 'extracted')    # Stage 1: per-run 10Hz dicts
SEQUENCES_DIR  = os.path.join(OUTPUT_ROOT, 'sequences')    # Stage 2: pkl sequence files
TRAIN_DIR      = os.path.join(SEQUENCES_DIR, 'train')
CAL_DIR        = os.path.join(SEQUENCES_DIR, 'calibration')

# ── Data split ─────────────────────────────────────────────────────────────

# Datasets to use for training + calibration (nominal behavior only)
NOMINAL_DATASETS = ['baseline_all', 'nom_v5', 'nom_v7', 'nom_v10']

# Datasets reserved for inference/evaluation only
TEST_DATASETS = ['obs_recovery', 'obs_noescape', 'obs_stuck']

# Train/calibration split (per dataset, stratified by goal)
CAL_FRACTION = 0.20

# Speed labels for calibration stratification
SPEED_LABELS = {
    'baseline_all': 'default',
    'nom_v5': '5ms',
    'nom_v7': '7ms',
    'nom_v10': '10ms',
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
    'traffic_lights':    '/perception/traffic_light_recognition/traffic_signals',
}

# objects is the master clock at ~10 Hz; other topics forward-fill
MASTER_CLOCK_TOPIC = 'objects'

# Drop frame if any topic is more than this stale (seconds)
MAX_STALENESS_SEC = 0.30

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

# Position uncertainty scaling: raw EKF covariance in m²
# Nominal ≈ 0.003 m² → scale so nominal ≈ 0.1 (manageable input magnitude)
# Cap at 0.5 m² (localization has catastrophically failed above this)
UNCERTAINTY_SCALE  = 30.0    # multiply m² values
UNCERTAINTY_CAP    = 0.5     # cap before scaling (m²)

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

# Reference points for coordinate frame conversion (Shinjuku map ↔ local frame)
# Same values as original T-ITS paper config.py
REFERENCE_POINTS = [
    ((81370.40, 49913.81), (3527.96, 1775.78)),
    ((81375.16, 49917.01), (3532.70, 1779.04)),
    ((81371.85, 49911.62), (3529.45, 1773.63)),
    ((81376.60, 49914.82), (3534.15, 1776.87)),
]

# ── Model config (input feature sizes — updated for uncertainty) ───────────

FEATURE_SIZES = {
    'position':             2,   # scaled x, y
    'velocity':             2,   # longitudinal, lateral
    'steering':             1,
    'acceleration':         1,
    'object_distance':      1,
    'traffic_light_detected': 1,
    'uncertainty':          2,   # EKF x_var, y_var (new)
}

MODEL_CONFIG = {
    'graph_sizes':   {'node_features': NODE_FEATURES, 'number_of_nodes': MAX_GRAPH_NODES},
    'feature_sizes': FEATURE_SIZES,
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
    'model_path':        os.path.join(REPO_ROOT, 'st_gat', 'models', 'st_gat_rise.pth'),
    'device':            torch.device('cuda' if torch.cuda.is_available() else 'cpu'),
}
