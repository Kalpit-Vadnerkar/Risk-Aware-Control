# Risk-Aware Control for Autonomous Vehicles

Extension of the Digital Twin framework from passive fault detection to active risk-aware control using CVaR-based constraint tightening (RISE - Residual-Informed Safety Envelopes).

**Principal Investigator:** Kalpit Vadnerkar
**Institution:** Clemson University, Department of Electrical and Computer Engineering
**Advisor:** Pierluigi Pisu, PhD

## Quick Start

```bash
# Terminal 1: Launch AWSIM
./Run_AWSIM.sh

# Terminal 2: Launch Autoware
./Run_Autoware.sh

# Terminal 3: Run experiments
cd experiments/scripts
python3 run_batch_experiments.py --goals goal_001
```

## Environment Configuration

This project uses modified versions of Autoware and AWSIM. Since the Autoware source code is pre-built and not included in git, the following changes were made manually:

### Autoware Modifications

| File | Parameter | Original | Modified | Reason |
|------|-----------|----------|----------|--------|
| `config/planning/scenario_planning/common/common.param.yaml` | `max_vel` | 4.17 m/s (15 km/h) | 11.11 m/s (40 km/h) | Enable higher-speed driving scenarios for research |

**Full path:**
```
autoware/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/common/common.param.yaml
```

**If rebuilding Autoware**, ensure this change is applied:
```yaml
max_vel: 11.11  # max velocity limit [m/s] (40 km/h, changed from 4.17/15kmh for research)
```

### AWSIM Configuration

The ego vehicle spawn position is configured in `experiments/configs/baseline.json`:

```json
{
  "mapConfiguration": {
    "mapName": "Shinjuku",
    "useShadows": false
  },
  "egoConfiguration": {
    "egoPosition": {
      "x": 81384.60,
      "y": 49922.00,
      "z": 41.28
    },
    "egoEulerAngles": {
      "x": 0.0,
      "y": 0.0,
      "z": 33.5
    }
  }
}
```

This spawn point is at the start of the Shinjuku map, facing the correct direction for the captured goal routes.

### Version Information

- **Autoware:** Universe (Humble branch)
- **AWSIM:** Labs v1.6.1
- **ROS2:** Humble
- **Map:** Shinjuku

## Repository Structure

```
Risk-Aware-Control/
├── autoware/                    # Pre-built Autoware (not in git)
├── awsim_labs_v1.6.1/          # AWSIM binary (not in git)
├── Shinjuku-Map/               # HD map data
├── experiments/
│   ├── configs/                # Experiment configurations
│   │   ├── baseline.json       # AWSIM spawn config
│   │   └── captured_goals.json # Validated goal positions
│   ├── scripts/                # Automation scripts
│   │   ├── run_batch_experiments.py
│   │   ├── experiment_watchdog.py
│   │   └── ...
│   └── data/                   # Experiment results (rosbags, JSON)
├── docs/
│   └── research_notes/         # Decision documentation
├── ros2_ws/                    # (Future) Custom ROS2 packages
├── st_gat/                     # (Future) ST-GAT model port
├── Run_AWSIM.sh
├── Run_Autoware.sh
└── TODO.md                     # Task tracking
```

## Recorded Topics

During experiments, the following ROS2 topics are recorded:

| Topic | Type | Rate | Purpose |
|-------|------|------|---------|
| `/localization/kinematic_state` | Odometry | 30 Hz | Vehicle state estimation |
| `/awsim/ground_truth/localization/kinematic_state` | Odometry | 60 Hz | Ground truth |
| `/perception/object_recognition/objects` | PredictedObjects | 10 Hz | Detected objects |
| `/control/command/control_cmd` | Control | 30 Hz | Control commands |
| `/vehicle/status/velocity_status` | VelocityReport | 30 Hz | Actual velocity |
| `/vehicle/status/steering_status` | SteeringReport | 30 Hz | Actual steering |
| `/planning/scenario_planning/trajectory` | Trajectory | 10 Hz | Planned trajectory |
| `/diagnostics` | DiagnosticArray | ~800/s | System health |
| `/autoware/state` | AutowareState | 10 Hz | System state machine |
| `/system/fail_safe/mrm_state` | MrmState | 10 Hz | MRM (safety) state |

### Topics to Add

```
/planning/mission_planning/route                    # Route geometry
/perception/traffic_light_recognition/traffic_signals
```

## Related Work

This project extends:
- **IEEE T-ITS 2025 Paper:** "Digital Twins as Predictive Models for Real-Time Probabilistic Risk Assessment of Autonomous Vehicles"
- **GitHub:** [Graph-Scene-Representation-and-Prediction](https://github.com/Kalpit-Vadnerkar/Graph-Scene-Representation-and-Prediction)

## Contact

- **Email:** kvadner@clemson.edu
- **Lab:** Center for Connected Multimodal Mobility (C2M2)
