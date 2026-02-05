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
    # Core state
    '/localization/kinematic_state',
    '/localization/pose_with_covariance',
    '/awsim/ground_truth/localization/kinematic_state',

    # Vehicle status
    '/vehicle/status/velocity_status',
    '/vehicle/status/steering_status',

    # Perception
    '/perception/object_recognition/objects',
    '/perception/object_recognition/tracking/objects',
    '/perception/traffic_light_recognition/traffic_signals',

    # Planning
    '/planning/scenario_planning/trajectory',
    '/planning/mission_planning/route',

    # Control
    '/control/command/control_cmd',

    # System state
    '/autoware/state',
    '/api/routing/state',
    '/system/fail_safe/mrm_state',

    # Metrics (from planning evaluator)
    '/planning_evaluator/metrics',

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

    # Computed paths
    data_dir: str = field(init=False)
    rosbag_dir: str = field(init=False)
    metadata_file: str = field(init=False)
    result_file: str = field(init=False)
    metrics_file: str = field(init=False)

    def __post_init__(self):
        self.data_dir = os.path.join(DATA_DIR, self.experiment_id)
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
