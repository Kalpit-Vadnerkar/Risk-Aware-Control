# Data Pipeline Plan for RISE

**Based on:** Graph-Scene-Representation-and-Prediction (T-ITS paper) codebase analysis

## Overview

The T-ITS paper uses a 7-topic recording strategy with 10Hz synchronization. We need to adapt this for the new Autoware/AWSIM version and add RISE-specific topics.

## Topics to Record

### Core Topics (from T-ITS paper)

| Topic | Type | Rate | Purpose |
|-------|------|------|---------|
| `/vehicle/status/steering_status` | SteeringReport | 30Hz | Steering angle |
| `/vehicle/status/velocity_status` | VelocityReport | 30Hz | Longitudinal/lateral velocity, yaw rate |
| `/tf` | TFMessage | 100Hz | Vehicle pose in map frame |
| `/perception/object_recognition/tracking/objects` | TrackedObjects | 10Hz | Object tracking (position, velocity, class) |
| `/perception/traffic_light_recognition/traffic_signals` | TrafficSignalArray | 10Hz | Traffic light states |
| `/planning/mission_planning/route` | LaneletRoute | Once | Route lane IDs |

### Additional Topics for RISE

| Topic | Type | Rate | Purpose |
|-------|------|------|---------|
| `/system/fail_safe/mrm_state` | MrmState | 10Hz | MRM state for regime detection |
| `/planning/scenario_planning/trajectory` | Trajectory | 10Hz | Planned trajectory for deviation calc |
| `/localization/kinematic_state` | Odometry | 30Hz | Localization state |
| `/awsim/ground_truth/localization/kinematic_state` | Odometry | 60Hz | Ground truth |
| `/control/command/control_cmd` | Control | 30Hz | Control commands |
| `/planning_evaluator/metrics` | MetricArray | 10Hz | TTC, obstacle distance metrics |
| `/diagnostics` | DiagnosticArray | Variable | System health |

### Total: 13 Topics

## Data Pipeline Stages

### Stage 1: Recording (ROS2 Bag)

```
Experiment → ros2 bag record → SQLite3 database
```

**Key Requirements:**
- Start recording AFTER vehicle enters DRIVING state (skip initialization)
- Wait 5-10 seconds after DRIVING for system stabilization
- Record until goal reached OR timeout

### Stage 2: Extraction (JSON per topic)

```
SQLite3 → MessageExtractor → JSON files (one message per line)
```

**Output Structure:**
```
extracted_data/
├── experiment_001/
│   ├── steering_status.json
│   ├── velocity_status.json
│   ├── tf.json
│   ├── tracked_objects.json
│   ├── traffic_signals.json
│   ├── route.json
│   ├── mrm_state.json
│   ├── trajectory.json
│   ├── kinematic_state.json
│   ├── ground_truth.json
│   ├── control_cmd.json
│   ├── planning_metrics.json
│   └── metadata.json
└── experiment_002/
    └── ...
```

### Stage 3: Synchronization & Cleaning

```
JSON files → Synchronizer → Cleaned aligned data
```

**Process:**
1. Find common timestamps across all topics
2. Resample to 10Hz (100ms intervals)
3. Remove initialization transients (first 5s)
4. Remove stopped periods (velocity < 0.001 m/s for > 3s)
5. Interpolate missing values

**Output:** Synchronized JSON files at 10Hz

### Stage 4: Feature Extraction

```
Synchronized data → FeatureExtractor → Per-timestep features
```

**Features per timestep:**
```python
{
    'timestamp': float,
    'ego': {
        'position': [x, y],           # From tf
        'velocity': [vx, vy],         # From velocity_status
        'steering': float,            # From steering_status
        'acceleration': float,        # Computed from velocity
        'yaw_rate': float,           # From velocity_status
    },
    'objects': [{
        'position': [x, y, z],
        'velocity': [vx, vy, vz],
        'classification': int,
    }],
    'traffic_light': int,            # Closest light state
    'mrm_state': int,                # 0=NORMAL, 1=MRM_OPERATING
    'trajectory_deviation': {
        'lateral': float,
        'longitudinal': float,
    },
    'metrics': {
        'ttc': float,
        'obstacle_distance': float,
    }
}
```

### Stage 5: Sequence Generation

```
Per-timestep features → SequenceGenerator → Training sequences
```

**Configuration:**
- Past window: 30 timesteps (3 seconds)
- Future window: 30 timesteps (3 seconds)
- Stride: 1 (overlapping sequences)
- Min sequence length: 60 timesteps

### Stage 6: Graph Building

```
HD Map + Route + Position → GraphBuilder → Scene graph
```

**Graph Properties:**
- 150 nodes (fixed)
- Node features: [x, y, traffic_light_node, path_node]
- Edge features: Lane connectivity
- Coordinate transform: Map → Vehicle-local

### Stage 7: Dataset Packaging

```
Sequences + Graphs → DatasetBuilder → Pickle files
```

**Output:** `sequence_dataset/*.pkl`
- Max 1000 sequences per file
- Split into train/validation/test

## Key Differences from T-ITS Paper

| Aspect | T-ITS Paper | RISE |
|--------|-------------|------|
| **Topics** | 7 core | 13 (core + RISE-specific) |
| **MRM State** | Not recorded | Included as feature |
| **Trajectory** | Not recorded | For deviation calculation |
| **Ground Truth** | Not recorded | For evaluation |
| **Collision Metrics** | Not recorded | TTC, obstacle distance |
| **Recording Start** | Manual | Automated after DRIVING state |

## Implementation Modules

```
ros2_ws/src/rise_data_pipeline/
├── rise_recorder/           # Recording node
│   └── recorder_node.py
├── rise_extractor/          # Bag → JSON extraction
│   ├── extractor.py
│   └── message_handlers/
├── rise_synchronizer/       # Timestamp alignment
│   └── synchronizer.py
├── rise_feature_extractor/  # Feature computation
│   └── feature_extractor.py
├── rise_sequence_generator/ # Sequence creation
│   └── sequence_generator.py
├── rise_graph_builder/      # Scene graph
│   └── graph_builder.py
└── rise_dataset_builder/    # Final packaging
    └── dataset_builder.py
```

## Timeline for Implementation

1. **Phase 1:** Recording infrastructure (modify existing scripts)
2. **Phase 2:** Extraction scripts (adapt from T-ITS)
3. **Phase 3:** Synchronization and feature extraction
4. **Phase 4:** Sequence and graph generation
5. **Phase 5:** Integration testing with ST-GAT training

## Notes

- The T-ITS paper uses `/perception/object_recognition/tracking/objects` not `/perception/object_recognition/objects` - need to verify which topic AWSIM publishes
- Route is published once with transient local QoS - must record before or immediately after goal setting
- Traffic light topic may differ in current Autoware version - verify
