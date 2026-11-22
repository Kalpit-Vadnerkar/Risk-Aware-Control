# What to record, and why

2026-11-21. Derived from what the research needs (architecture.md), checked
against the Autoware / AWSIM source and launch files. Rates and sizes are
from a one-off measurement on the live stack (`experiments/scripts/topic_inventory.py`,
`experiments/analysis/topic_inventory/idle_2026-11-21.json`), used only for
the storage budget.

## Principle

The pipeline monitors the stack as a black box from four kinds of data:

| Kind | What | Why |
|---|---|---|
| Inputs | what the car sends the stack | independent evidence, not computed by the stack |
| Belief | the stack's state estimate, perceived objects and lights, its plan | the thing being judged |
| Outputs | commands back to the car | Layer 3 / closed loop |
| Truth | simulator only | labels and validation, never a pipeline input |

The circularity the Layer 2 results exposed: LiDAR health and the "δ = 0
when LiDAR is healthy" update came from Autoware's own NDT, the same source
the estimate is fused from. The fix is an input-level LiDAR measurement the
pipeline evaluates itself (scan against the map at the estimated pose). That
is the one large new stream.

## Need → stream

| Research need | Streams | Status |
|---|---|---|
| Layer 1 inputs (ST-GAT, every schema) | EKF kinematic state, acceleration, pose covariance; velocity / steering report; tracked + predicted objects; TL signals; route, trajectory, path; `/diagnostics` | recorded |
| 2b step gaps (how much, which way) | EKF kinematic state + velocity report | recorded |
| 2b absolute reference, own (removes the circularity) | `/sensing/lidar/concatenated/pointcloud` + map PCD (on disk) + `/tf_static` | **add** |
| 2b absolute reference, Autoware's (comparison) | NDT pose, transform probability, NVTL | added 2026-10-23, not yet in any bag |
| NDT's correction of the EKF prior (a direct δ measurement before fusion) | `/localization/pose_estimator/initial_to_result_relative_pose`, `initial_to_result_distance`, `iteration_num` | **add** |
| Same input NDT used (replay / compare our matcher on identical data) | `/localization/util/downsample/pointcloud` (117 kB/s) | **add** |
| IMU fault as the EKF saw it | `/sensing/imu/imu_data`, `imu_data_faulted`; `/localization/twist_estimator/twist_with_covariance` (gyro_odometer output, what the EKF fuses); EKF `estimated_yaw_bias` | first two recorded; **add** the other two |
| TL faults (injected after recognition) | `traffic_signals` (unfaulted = true light for the red-light safe set), `traffic_signals_faulted` | recorded |
| Object-conflict safe set | tracked objects; the two detectors (CenterPoint, clustering) if detector disagreement is used | detectors: **add** (30 kB/s) |
| Road-departure / speed safe sets, labels | AWSIM truth kinematic state; lanelet map (on disk) | recorded |
| Layer 3 / closed loop | control_cmd, actuation_cmd, actuation status, max velocity | recorded |
| Run validity | autoware state, MRM, operation mode, routing state | recorded |

Remove from `RECORDING_TOPICS`: `/perception/traffic_light_recognition/traffic_light_states`
and `/planning/remaining_distance_time_calculator/output/mission_remaining_distance_time`
(neither exists on this stack).

## Deliberately not recorded

- **Camera image** (62 MB/s). It feeds only traffic-light recognition
  (object detection is LiDAR-only), and the TL faults are injected after
  recognition, so the image adds nothing to any experiment planned.
- **GNSS.** In AWSIM it is ground truth: zero noise, zero covariance, a
  constant offset equal to the antenna mount (`base_link → gnss_link`,
  0.80 m forward, 1.80 m up), 1 Hz. Autoware uses it only to initialise the
  pose; the EKF never fuses it. It carries nothing that the truth topic
  does not, and using it as a "sensor" would leak truth. (Likely the
  earlier "GNSS = same data" finding: same as truth, not as the IMU.)
- **Raw IMU** (`/sensing/imu/tamagawa/imu_raw`). `imu_corrector` only
  applies configured static offsets; `/sensing/imu/imu_data` is what every
  consumer and the fault injector use.
- **Per-sensor raw LiDAR** (`/sensing/lidar/{top,left,right}/pointcloud_raw_ex`,
  23 MB/s) on nominal / IMU / TL campaigns. LiDAR faults are in scope
  (decided 2026-11-21, not implemented yet): a LiDAR-fault campaign records
  the `lidar_raw` profile too, and the fault is injected on these topics so
  it reaches NDT and perception through the normal preprocessing chain.
- Intermediate cloud stages, debug markers, occupancy grid: re-derivable or
  unused.

## Why the concatenated cloud is an independent input

Source: `awsim_labs_sensor_kit_launch` / `common_awsim_labs_sensor_launch`.
Chain per sensor: `pointcloud_raw_ex` → self / mirror crop → distortion
corrector → ring outlier filter → concatenate (base_link). The distortion
corrector uses `/sensing/imu/imu_data` (unfaulted) and the wheel-speed
twist; the IMU fault is injected on `imu_data_faulted`, which only
`gyro_odometer` (→ EKF) reads. Nothing in the chain uses the pose estimate,
so the concatenated cloud is unaffected by the IMU faults and by the
localization error. NDT then uses a cropped, downsampled copy of it.

## Needs a Unity rebuild (not planned for now)

AWSIM's source has publishers that are not in the built scene:
`PerceptionResultRos2Publisher` (truth for other agents,
`/awsim/ground_truth/perception/object_recognition/detection/objects`,
up to 200 m) and `OnCollisionRos2Publisher` (`awsim/ground_truth/on_collision`).
Without them, object-conflict labels can only come from perception.

## Storage

Measured on 20 s recordings (MCAP, zstd_fast; point clouds barely compress):

| Set | Rate | Per trial (195 s median) | 100 trials |
|---|---|---|---|
| Current list | 1.1 MB/s | 0.2 GB | 21 GB |
| + concatenated cloud + NDT input | 7.5 MB/s | 1.5 GB | 150 GB |
| (per-sensor raw LiDAR, if chosen) | 23 MB/s | 4.5 GB | 450 GB |

718 GB free.

## Decisions (2026-11-21)

- Recording profiles in `experiments/lib/config.py`: `base` on every
  campaign (~1.5 GB per trial), `lidar_raw` added for LiDAR-fault
  campaigns. Storage MCAP.
- No camera, no GNSS, no raw IMU (above).
- Detector topics recorded (cheap), so detector disagreement stays
  available.
- No AWSIM rebuild for now.
