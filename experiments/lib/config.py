"""
Experiment configuration management.
"""

import os
import json
from dataclasses import dataclass, field, asdict
from typing import List, Optional, Dict, Any
from datetime import datetime


# Paths
LIB_DIR = os.path.dirname(os.path.abspath(__file__))
EXPERIMENTS_DIR = os.path.dirname(LIB_DIR)
PROJECT_DIR = os.path.dirname(EXPERIMENTS_DIR)
CONFIG_DIR = os.path.join(EXPERIMENTS_DIR, 'configs')
DATA_DIR = os.path.join(EXPERIMENTS_DIR, 'data')
SCRIPTS_DIR = os.path.join(EXPERIMENTS_DIR, 'scripts')


# Topics to record during experiments
RECORDING_TOPICS = [
    # Core state & transforms
    '/localization/kinematic_state',
    '/localization/pose_with_covariance',
    '/localization/acceleration',           # EKF-fused acceleration (ST-GAT accel feature)
    '/awsim/ground_truth/localization/kinematic_state',
    '/tf',
    '/tf_static',                           # Static coordinate frames

    # IMU - raw sensor data (acceleration feature source; needed for IMU fault injection)
    '/sensing/imu/imu_data',

    # Vehicle status
    '/vehicle/status/velocity_status',
    '/vehicle/status/steering_status',
    '/vehicle/status/gear_status',
    '/vehicle/status/actuation_status',     # Actual throttle/brake/steer feedback
    '/vehicle/status/control_mode',         # Autonomous / manual / override

    # Control commands
    '/control/command/control_cmd',
    '/control/command/actuation_cmd',       # Low-level actuator commands

    # Perception - objects (original + filtered for interceptor comparison)
    '/perception/object_recognition/objects',
    '/perception/object_recognition/objects_filtered',
    '/perception/object_recognition/tracking/objects',

    # Perception - traffic lights
    # traffic_signals_raw: raw recognition output (pre-fault-injection)
    # traffic_signals: what planning sees (may be faulted by fault_injector)
    '/perception/traffic_light_recognition/traffic_signals_raw',
    '/perception/traffic_light_recognition/traffic_signals',
    '/perception/traffic_light_recognition/traffic_light_states',

    # Planning - trajectories & paths
    '/planning/scenario_planning/trajectory',
    '/planning/mission_planning/route',
    '/planning/scenario_planning/lane_driving/behavior_planning/path',
    '/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id',  # Lane topology

    # Planning - velocity constraints
    '/planning/scenario_planning/max_velocity',           # RISE Phase 4 writes here
    '/planning/scenario_planning/current_max_velocity',   # Actual enforced cap

    # Planning factors - which module is active and why (avoidance decision timestamp)
    '/planning/planning_factors/behavior_path_planner',
    '/planning/planning_factors/motion_velocity_planner',

    # Behavior path planner - avoidance module debug (lateral shift decision)
    '/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/static_obstacle_avoidance',

    # Motion velocity planner - obstacle stop/cruise planning info (TTC / Signal 2)
    '/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/debug/obstacle_stop/planning_info',
    '/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/debug/obstacle_cruise/planning_info',

    # System state
    '/autoware/state',
    '/api/routing/state',
    '/system/fail_safe/mrm_state',
    '/system/operation_mode/state',

    # Mission planning
    '/planning/remaining_distance_time_calculator/output/mission_remaining_distance_time',

    # Diagnostics
    '/diagnostics',
]


@dataclass
class GoalConfig:
    """Configuration for a single goal/destination."""
    id: str
    position: Dict[str, float]  # x, y, z
    orientation: Dict[str, float]  # x, y, z, w
    frame_id: str = 'map'
    estimated_distance: Optional[float] = None

    @classmethod
    def from_dict(cls, data: dict) -> 'GoalConfig':
        return cls(
            id=data['id'],
            position=data['goal']['position'],
            orientation=data['goal']['orientation'],
            frame_id=data['goal'].get('frame_id', 'map'),
            estimated_distance=data.get('estimated_distance')
        )


@dataclass
class ExperimentConfig:
    """Configuration for a single experiment run."""
    experiment_id: str
    goal: GoalConfig
    stuck_timeout: float = 90.0
    stabilization_delay: float = 5.0  # Wait after DRIVING state before recording
    condition: str = 'baseline'  # baseline, fault_xxx, rise_xxx
    fault_params: Dict[str, Any] = field(default_factory=dict)
    rise_enabled: bool = False
    scenario_type: str = 'passthrough'  # passthrough, static_obstacle, cut_in, perception_delay, etc.
    scenario_params: Dict[str, Any] = field(default_factory=dict)
    campaign: str = 'default'  # Subdirectory under data/ for grouping experiments

    # Computed paths
    data_dir: str = field(init=False)
    rosbag_dir: str = field(init=False)
    metadata_file: str = field(init=False)
    result_file: str = field(init=False)
    metrics_file: str = field(init=False)

    def __post_init__(self):
        self.data_dir = os.path.join(DATA_DIR, self.campaign, self.experiment_id)
        self.rosbag_dir = os.path.join(self.data_dir, 'rosbag')
        self.metadata_file = os.path.join(self.data_dir, 'metadata.json')
        self.result_file = os.path.join(self.data_dir, 'result.json')
        self.metrics_file = os.path.join(self.data_dir, 'metrics.json')

    def create_directories(self):
        """Create experiment data directories."""
        os.makedirs(self.data_dir, exist_ok=True)
        # Rosbag directory created by ros2 bag record

    def save_metadata(self):
        """Save experiment metadata."""
        metadata = {
            'experiment_id': self.experiment_id,
            'timestamp': datetime.now().isoformat(),
            'goal_id': self.goal.id,
            'goal_position': self.goal.position,
            'goal_orientation': self.goal.orientation,
            'stuck_timeout': self.stuck_timeout,
            'stabilization_delay': self.stabilization_delay,
            'condition': self.condition,
            'fault_params': self.fault_params,
            'rise_enabled': self.rise_enabled,
            'scenario_type': self.scenario_type,
            'scenario_params': self.scenario_params,
            'campaign': self.campaign,
        }
        with open(self.metadata_file, 'w') as f:
            json.dump(metadata, f, indent=2)

    def save_result(self, result: dict):
        """Save experiment result."""
        result['experiment_id'] = self.experiment_id
        result['timestamp'] = datetime.now().isoformat()
        with open(self.result_file, 'w') as f:
            json.dump(result, f, indent=2)

    def save_metrics(self, metrics: dict):
        """Save computed metrics."""
        metrics['experiment_id'] = self.experiment_id
        metrics['timestamp'] = datetime.now().isoformat()
        with open(self.metrics_file, 'w') as f:
            json.dump(metrics, f, indent=2)


def load_goals(goals_file: Optional[str] = None) -> List[GoalConfig]:
    """Load goals from captured_goals.json."""
    if goals_file is None:
        goals_file = os.path.join(CONFIG_DIR, 'captured_goals.json')

    if not os.path.exists(goals_file):
        raise FileNotFoundError(f"Goals file not found: {goals_file}")

    with open(goals_file, 'r') as f:
        data = json.load(f)

    return [GoalConfig.from_dict(g) for g in data.get('goals', [])]


def get_recording_topics() -> List[str]:
    """Get list of topics to record."""
    return RECORDING_TOPICS.copy()
