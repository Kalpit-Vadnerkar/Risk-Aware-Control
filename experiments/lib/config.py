"""
Data-collection configuration: paths, what to record, campaigns, per-trial config.

What to record and why: docs/data_inventory.md. Campaigns:
experiments/configs/campaigns.yaml.
"""

import json
import os
import subprocess
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Dict, List, Optional

import yaml


# Paths
LIB_DIR = os.path.dirname(os.path.abspath(__file__))
EXPERIMENTS_DIR = os.path.dirname(LIB_DIR)
PROJECT_DIR = os.path.dirname(EXPERIMENTS_DIR)
CONFIG_DIR = os.path.join(EXPERIMENTS_DIR, 'configs')
DATA_DIR = os.path.join(EXPERIMENTS_DIR, 'data')
SCRIPTS_DIR = os.path.join(EXPERIMENTS_DIR, 'scripts')
GOALS_FILE = os.path.join(CONFIG_DIR, 'captured_goals.json')
CAMPAIGNS_FILE = os.path.join(CONFIG_DIR, 'campaigns.yaml')

# Bag storage: MCAP (faster than sqlite3 for the large point clouds).
# Readers open bags with storage_id='' so the format is detected.
BAG_STORAGE = 'mcap'


def campaign_meta_dir(campaign: str) -> str:
    """Campaign-level files (the injector's running fault log, batch
    summaries), so data/<campaign>/ only contains goal directories."""
    return os.path.join(DATA_DIR, campaign, '_meta')


# ── What to record ───────────────────────────────────────────────────────────
# Grouped by the four kinds of data the pipeline uses: inputs from the car,
# the stack's belief, its outputs, and simulator truth (docs/data_inventory.md).

_INPUTS = [
    # LiDAR, merged and preprocessed (crop, deskew with the UNFAULTED IMU +
    # wheel speed, outlier filter), base_link frame. Independent of the pose
    # estimate: the pipeline's own scan-to-map check uses it.
    '/sensing/lidar/concatenated/pointcloud',
    # IMU as the stack receives it, and what gyro_odometer -> EKF actually
    # reads (the fault injector's output)
    '/sensing/imu/imu_data',
    '/sensing/imu/imu_data_faulted',
    # Vehicle reports
    '/vehicle/status/velocity_status',
    '/vehicle/status/steering_status',
    '/vehicle/status/gear_status',
    '/vehicle/status/actuation_status',
    '/vehicle/status/control_mode',
    '/tf_static',
]

_BELIEF = [
    # Localization: EKF output
    '/localization/kinematic_state',
    '/localization/pose_with_covariance',
    '/localization/acceleration',
    '/tf',
    # EKF inputs and internals
    '/localization/twist_estimator/twist_with_covariance',      # gyro_odometer: what the EKF fuses
    '/localization/pose_twist_fusion_filter/estimated_yaw_bias',
    # NDT (LiDAR-to-map) before fusion: EKF - NDT measures the pose error
    '/localization/pose_estimator/pose_with_covariance',
    '/localization/pose_estimator/transform_probability',
    '/localization/pose_estimator/nearest_voxel_transformation_likelihood',
    '/localization/pose_estimator/iteration_num',
    '/localization/pose_estimator/exe_time_ms',
    # NDT's correction of the EKF prior (initial pose -> result)
    '/localization/pose_estimator/initial_to_result_relative_pose',
    '/localization/pose_estimator/initial_to_result_distance',
    # The exact scan NDT matched (cropped, downsampled)
    '/localization/util/downsample/pointcloud',
    # Perception: both detectors, tracker, prediction, and the copy planning reads
    '/perception/object_recognition/detection/centerpoint/objects',
    '/perception/object_recognition/detection/clustering/objects',
    '/perception/object_recognition/tracking/objects',
    '/perception/object_recognition/objects',
    '/perception/object_recognition/objects_filtered',
    # Traffic lights: recognition output (unfaulted = the true light for the
    # red-light safe set) and what behavior planning reads (may be faulted)
    '/perception/traffic_light_recognition/traffic_signals',
    '/perception/traffic_light_recognition/traffic_signals_faulted',
    # Planning
    '/planning/mission_planning/route',
    '/planning/scenario_planning/lane_driving/behavior_planning/path',
    '/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id',
    '/planning/trajectory',
    '/planning/scenario_planning/max_velocity',
    '/planning/scenario_planning/current_max_velocity',
    '/planning/planning_factors/behavior_path_planner',
    '/planning/planning_factors/motion_velocity_planner',
    '/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/debug/obstacle_stop/planning_info',
    # System state and health
    '/autoware/state',
    '/api/routing/state',
    '/system/operation_mode/state',
    '/system/fail_safe/mrm_state',
    '/diagnostics',
]

_OUTPUTS = [
    '/control/command/control_cmd',
    '/control/command/actuation_cmd',
]

_TRUTH = [
    '/awsim/ground_truth/localization/kinematic_state',
]

RECORDING_PROFILES: Dict[str, List[str]] = {
    'base': _INPUTS + _BELIEF + _OUTPUTS + _TRUTH,
    # Per-sensor raw clouds as AWSIM publishes them (the ones the stack
    # consumes). For LiDAR-fault campaigns: a fault injected on these reaches
    # NDT and perception through the normal preprocessing chain.
    'lidar_raw': [
        '/sensing/lidar/top/pointcloud_raw_ex',
        '/sensing/lidar/left/pointcloud_raw_ex',
        '/sensing/lidar/right/pointcloud_raw_ex',
    ],
}


def recording_topics(profiles: List[str]) -> List[str]:
    topics: List[str] = []
    for p in profiles:
        if p not in RECORDING_PROFILES:
            raise ValueError(f'unknown recording profile {p!r}; known: {sorted(RECORDING_PROFILES)}')
        topics += [t for t in RECORDING_PROFILES[p] if t not in topics]
    return topics


# ── Goals and campaigns ──────────────────────────────────────────────────────

@dataclass
class GoalConfig:
    id: str
    position: Dict[str, float]      # x, y, z
    orientation: Dict[str, float]   # x, y, z, w
    frame_id: str = 'map'

    @classmethod
    def from_dict(cls, d: dict) -> 'GoalConfig':
        return cls(id=d['id'], position=d['goal']['position'],
                   orientation=d['goal']['orientation'],
                   frame_id=d['goal'].get('frame_id', 'map'))


def load_goals(goals_file: str = GOALS_FILE) -> List[GoalConfig]:
    with open(goals_file) as f:
        return [GoalConfig.from_dict(g) for g in json.load(f)['goals']]


@dataclass
class Campaign:
    name: str
    description: str
    goals: List[str]
    trials: int
    record: List[str]
    stuck_timeout_s: float
    fault: Dict[str, Any] = field(default_factory=dict)   # empty = nominal


def load_campaigns(campaigns_file: str = CAMPAIGNS_FILE,
                   goals_file: str = GOALS_FILE) -> Dict[str, Campaign]:
    with open(campaigns_file) as f:
        spec = yaml.safe_load(f)
    defaults = spec.get('defaults', {})
    all_goals = [g.id for g in load_goals(goals_file)]
    out = {}
    for name, c in spec['campaigns'].items():
        c = {**defaults, **c}
        goals = all_goals if c['goals'] == 'all' else list(c['goals'])
        unknown = [g for g in goals if g not in all_goals]
        if unknown:
            raise ValueError(f'campaign {name}: goals not in {goals_file}: {unknown}')
        out[name] = Campaign(name=name, description=' '.join(str(c.get('description', '')).split()),
                             goals=goals, trials=int(c['trials']), record=list(c['record']),
                             stuck_timeout_s=float(c['stuck_timeout_s']), fault=c.get('fault') or {})
    return out


def git_commit() -> str:
    try:
        sha = subprocess.run(['git', 'rev-parse', '--short', 'HEAD'], cwd=PROJECT_DIR,
                             capture_output=True, text=True, timeout=5).stdout.strip()
        dirty = subprocess.run(['git', 'status', '--porcelain', '--untracked-files=no'], cwd=PROJECT_DIR,
                               capture_output=True, text=True, timeout=5).stdout.strip()
        return sha + ('-dirty' if dirty else '')
    except Exception:
        return 'unknown'


# ── One trial ────────────────────────────────────────────────────────────────

@dataclass
class TrialConfig:
    """data/<campaign>/<goal_id>/t<N>_<timestamp>/ with metadata.json,
    result.json, metrics.json, fault_log.jsonl, rosbag/."""
    campaign: Campaign
    goal: GoalConfig
    trial_num: int
    arm: str = 'A'
    stabilization_delay: float = 5.0
    timestamp: str = field(default_factory=lambda: datetime.now().strftime('%Y%m%d_%H%M%S'))

    def __post_init__(self):
        self.experiment_id = f'{self.campaign.name}_{self.goal.id}_t{self.trial_num}_{self.timestamp}'
        self.data_dir = os.path.join(DATA_DIR, self.campaign.name, self.goal.id,
                                     f't{self.trial_num}_{self.timestamp}')
        self.rosbag_dir = os.path.join(self.data_dir, 'rosbag')
        self.metadata_file = os.path.join(self.data_dir, 'metadata.json')
        self.result_file = os.path.join(self.data_dir, 'result.json')
        self.metrics_file = os.path.join(self.data_dir, 'metrics.json')
        self.fault_log_file = os.path.join(self.data_dir, 'fault_log.jsonl')
        self.recorder_log_file = os.path.join(self.data_dir, 'recorder.log')

    @property
    def stuck_timeout(self) -> float:
        return self.campaign.stuck_timeout_s

    def create_directories(self):
        os.makedirs(self.data_dir, exist_ok=True)

    def save_metadata(self):
        c = self.campaign
        metadata = {
            'experiment_id': self.experiment_id,
            'timestamp': datetime.now().isoformat(),
            'start_wall_time': datetime.now().timestamp(),
            'campaign': c.name,
            'description': c.description,
            'goal_id': self.goal.id,
            'goal_position': self.goal.position,
            'goal_orientation': self.goal.orientation,
            'trial': self.trial_num,
            'fault': c.fault,
            'record': c.record,
            'stuck_timeout_s': c.stuck_timeout_s,
            'stabilization_delay_s': self.stabilization_delay,
            'diagnostic_arm': self.arm,
            'git_commit': git_commit(),
        }
        with open(self.metadata_file, 'w') as f:
            json.dump(metadata, f, indent=2)

    def save_result(self, result: dict):
        result = {**result, 'experiment_id': self.experiment_id, 'timestamp': datetime.now().isoformat()}
        with open(self.result_file, 'w') as f:
            json.dump(result, f, indent=2)
