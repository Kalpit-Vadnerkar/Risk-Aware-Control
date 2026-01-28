# Phase 0: Exploration Summary

**Completed:** 2026-01-22

---

## 1. Environment Verification

- AWSIM Labs v1.6.1 launches successfully
- Autoware rebuilt after directory move (453 packages)
- Waypoint demo completed in Shinjuku map

---

## 2. Key Findings

### AWSIM Capabilities
- **Fault injection:** None native - must implement at ROS2 level
- **Traffic:** NPC vehicles enabled via `useTraffic: true`
- **Sensors:** LiDAR (3), Camera (1), IMU, GNSS

### Autoware Parameters (Safety-Critical)

| Parameter | Default | Location | Effect |
|-----------|---------|----------|--------|
| `safe_distance_margin` | 5.0 m | obstacle_cruise | Min distance to obstacles |
| `stop_margin` | 5.0 m | obstacle_stop | Stopping distance buffer |
| `max_lateral_deviation` | 2.0 m | lane_departure_checker | Lane keeping tolerance |
| `longitudinal_offset_margin` | 1.0 m | AEB | Emergency braking margin |

### Controllable at Runtime
- **Via topic:** `/planning/scenario_planning/max_velocity` (VelocityLimit msg)
- **Via param service:** Most parameters require `ros2 param set`

### ROS2 Topics (788 total, key ones)

| Category | Key Topics |
|----------|-----------|
| Localization (with covariance) | `/localization/pose_with_covariance`, `/localization/twist_estimator/twist_with_covariance` |
| Perception | `/perception/object_recognition/objects` (has existence_probability) |
| Ground truth | `/awsim/ground_truth/localization/kinematic_state` |
| Control | `/control/command/control_cmd` |
| Velocity limit | `/planning/scenario_planning/max_velocity` |

---

## 3. Architecture Understanding

### Constraint Application Flow
```
Behavior Planner → Motion Velocity Planner → Control
                        ↓
              Plugin modules apply:
              - Stop points (v=0)
              - Slowdown intervals (v≤target)
```

**Key insight:** Most constraints affect PLANNING, not CONTROL. The lane_departure_checker is a monitor, not an enforcer.

### For RISE Integration
Best insertion point: Velocity planning plugin or trajectory modifier that adjusts constraints based on uncertainty.

---

## 4. Fault Injection Strategy

Since AWSIM has no native fault injection, implement at ROS2 level:

```
AWSIM → [Fault Injector Node] → Autoware
         ↓
    Modify topics:
    - /sensing/gnss/pose_with_covariance (add position noise)
    - /sensing/imu/tamagawa/imu_raw (add gyro bias)
    - /sensing/lidar/*/pointcloud_raw (point dropout)
```

### Priority Faults (from literature)
| Fault Type | Impact | Priority |
|------------|--------|----------|
| LiDAR dropout | Critical | High |
| IMU gyro bias | Critical | High |
| GNSS drift | Moderate | Medium |
| IMU accel noise | Low | Low |

---

## 5. Files Consolidated From

This document replaces:
- `phase0_exploration/ros2_topics.md`
- `phase0_exploration/awsim_capabilities.md`
- `phase0_exploration/autoware_parameters.md`
- `phase0_exploration/constraint_identification.md`
