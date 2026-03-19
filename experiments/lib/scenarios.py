"""
Scenario definitions for RISE validation experiments.

Provides ObjectFactory (builds PredictedObject messages), ObjectPlacer
(computes map-frame positions relative to ego), and YAML scenario loading.
"""

import math
import uuid
import yaml
from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional, Tuple
from enum import Enum

# ROS2 message types
from autoware_perception_msgs.msg import (
    PredictedObjects, PredictedObject, PredictedPath,
    ObjectClassification, Shape
)
from geometry_msgs.msg import (
    Pose, Point, Quaternion, Twist, Vector3,
    PoseWithCovariance, TwistWithCovariance,
    AccelWithCovariance
)
from unique_identifier_msgs.msg import UUID as UUIDMsg
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration


class ScenarioType(str, Enum):
    PASSTHROUGH = 'passthrough'
    STATIC_OBSTACLE = 'static_obstacle'
    CUT_IN = 'cut_in'
    PERCEPTION_DELAY = 'perception_delay'
    PERCEPTION_DROPOUT = 'perception_dropout'
    POSITION_NOISE = 'position_noise'


@dataclass
class ScenarioConfig:
    """Configuration for a single scenario."""
    scenario_type: ScenarioType
    params: Dict[str, Any] = field(default_factory=dict)
    description: str = ''

    # Sweep parameters (for parameter grid expansion)
    sweep_params: Dict[str, List[Any]] = field(default_factory=dict)

    @classmethod
    def from_dict(cls, data: dict) -> 'ScenarioConfig':
        return cls(
            scenario_type=ScenarioType(data['scenario_type']),
            params=data.get('params', {}),
            description=data.get('description', ''),
            sweep_params=data.get('sweep_params', {}),
        )

    def expand_sweep(self) -> List['ScenarioConfig']:
        """Expand sweep_params into a list of concrete ScenarioConfigs."""
        if not self.sweep_params:
            return [self]

        # Build parameter grid
        keys = list(self.sweep_params.keys())
        value_lists = [self.sweep_params[k] for k in keys]

        # Cartesian product
        configs = []
        grid = _cartesian_product(value_lists)
        for combo in grid:
            params = dict(self.params)  # Copy base params
            for key, val in zip(keys, combo):
                params[key] = val
            configs.append(ScenarioConfig(
                scenario_type=self.scenario_type,
                params=params,
                description=self.description,
            ))
        return configs


def _cartesian_product(lists):
    """Simple cartesian product without itertools."""
    if not lists:
        return [()]
    result = [()]
    for pool in lists:
        result = [x + (y,) for x in result for y in pool]
    return result


def load_scenario(yaml_path: str) -> ScenarioConfig:
    """Load a scenario configuration from YAML."""
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    return ScenarioConfig.from_dict(data)


def load_scenario_suite(yaml_path: str) -> List[ScenarioConfig]:
    """Load a scenario suite (multiple scenarios) from YAML."""
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    if 'scenarios' in data:
        return [ScenarioConfig.from_dict(s) for s in data['scenarios']]
    else:
        return [ScenarioConfig.from_dict(data)]


class ObjectFactory:
    """Builds valid autoware_perception_msgs/PredictedObject messages."""

    # Default dimensions by object type (length, width, height) in meters
    DIMENSIONS = {
        ObjectClassification.CAR: (4.5, 2.0, 1.8),
        ObjectClassification.TRUCK: (8.0, 2.5, 3.0),
        ObjectClassification.PEDESTRIAN: (0.5, 0.5, 1.7),
        ObjectClassification.BICYCLE: (1.8, 0.6, 1.5),
        ObjectClassification.MOTORCYCLE: (2.2, 0.8, 1.5),
    }

    @staticmethod
    def create_object(
        x: float, y: float, z: float = 0.0,
        yaw: float = 0.0,
        vx: float = 0.0, vy: float = 0.0,
        obj_type: int = ObjectClassification.CAR,
        existence_probability: float = 1.0,
        dimensions: Optional[Tuple[float, float, float]] = None,
        object_id: Optional[bytes] = None,
    ) -> PredictedObject:
        """Create a PredictedObject with given parameters.

        Args:
            x, y, z: Position in map frame
            yaw: Heading angle (radians)
            vx, vy: Velocity in map frame
            obj_type: ObjectClassification type constant
            existence_probability: Detection confidence
            dimensions: (length, width, height) override
            object_id: 16-byte UUID, auto-generated if None
        """
        obj = PredictedObject()

        # Object ID
        if object_id is None:
            object_id = uuid.uuid4().bytes
        obj.object_id = UUIDMsg(uuid=list(object_id))

        # Existence probability
        obj.existence_probability = existence_probability

        # Classification
        classification = ObjectClassification()
        classification.label = obj_type
        classification.probability = 1.0
        obj.classification = [classification]

        # Kinematics
        obj.kinematics.initial_pose_with_covariance.pose.position.x = x
        obj.kinematics.initial_pose_with_covariance.pose.position.y = y
        obj.kinematics.initial_pose_with_covariance.pose.position.z = z

        # Quaternion from yaw
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        obj.kinematics.initial_pose_with_covariance.pose.orientation.z = qz
        obj.kinematics.initial_pose_with_covariance.pose.orientation.w = qw

        # Velocity
        obj.kinematics.initial_twist_with_covariance.twist.linear.x = vx
        obj.kinematics.initial_twist_with_covariance.twist.linear.y = vy

        # Shape
        if dimensions is None:
            dimensions = ObjectFactory.DIMENSIONS.get(
                obj_type, (4.5, 2.0, 1.8)
            )
        obj.shape.type = Shape.BOUNDING_BOX
        obj.shape.dimensions.x = dimensions[0]  # length
        obj.shape.dimensions.y = dimensions[1]  # width
        obj.shape.dimensions.z = dimensions[2]  # height

        # Predicted path (stationary object: single point, 3s horizon)
        path = PredictedPath()
        path.confidence = 1.0
        path.time_step = Duration(sec=3, nanosec=0)
        predicted_pose = Pose()
        predicted_pose.position.x = x
        predicted_pose.position.y = y
        predicted_pose.position.z = z
        predicted_pose.orientation.z = qz
        predicted_pose.orientation.w = qw
        path.path = [predicted_pose]
        obj.kinematics.predicted_paths = [path]

        return obj


class ObjectPlacer:
    """Computes map-frame positions relative to ego pose and heading."""

    @staticmethod
    def ahead_of_ego(
        ego_x: float, ego_y: float, ego_z: float, ego_heading: float,
        distance: float, lateral_offset: float = 0.0,
    ) -> Tuple[float, float, float]:
        """Compute position ahead of ego along heading direction.

        Args:
            ego_x, ego_y, ego_z: Ego position in map frame
            ego_heading: Ego yaw in radians
            distance: Longitudinal distance ahead (meters)
            lateral_offset: Lateral offset from ego centerline (positive = left)

        Returns:
            (x, y, z) in map frame
        """
        obs_x = ego_x + distance * math.cos(ego_heading) - lateral_offset * math.sin(ego_heading)
        obs_y = ego_y + distance * math.sin(ego_heading) + lateral_offset * math.cos(ego_heading)
        obs_z = ego_z
        return obs_x, obs_y, obs_z

    @staticmethod
    def cut_in_position(
        ego_x: float, ego_y: float, ego_z: float, ego_heading: float,
        longitudinal_distance: float,
        lateral_start: float,
        lateral_end: float,
        progress: float,
    ) -> Tuple[float, float, float]:
        """Compute position for a cut-in object at given progress.

        Args:
            ego_x, ego_y, ego_z: Ego position in map frame
            ego_heading: Ego yaw in radians
            longitudinal_distance: Distance ahead of ego
            lateral_start: Initial lateral offset (e.g., 3.5 for adjacent lane)
            lateral_end: Final lateral offset (e.g., 0.0 for ego lane center)
            progress: 0.0 to 1.0 (fraction of cut-in completed)

        Returns:
            (x, y, z) in map frame
        """
        progress = max(0.0, min(1.0, progress))
        lateral = lateral_start + (lateral_end - lateral_start) * progress
        return ObjectPlacer.ahead_of_ego(
            ego_x, ego_y, ego_z, ego_heading,
            longitudinal_distance, lateral
        )
