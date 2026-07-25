"""
FaultInjector — ROS2 relay node for RISE sensor fault experiments.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
1. TRAFFIC LIGHT CAMERA FAULT
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  IN:  /perception/traffic_light_recognition/traffic_signals          (Autoware's real final output)
  OUT: /perception/traffic_light_recognition/traffic_signals_faulted  (requires an Autoware-side
                                                                        patch — see README.md — to
                                                                        repoint behavior_planning's
                                                                        input_traffic_light_topic_name
                                                                        here; mirrors the
                                                                        objects → objects_filtered
                                                                        pattern)

  BUG FIXED 2026-07-22: this previously subscribed to a "_raw" topic that
  Autoware never publishes (there is no raw/final split in this Autoware
  install's TL pipeline) — the relay received zero messages and every TL
  fault silently never fired. Confirmed via msg_count_tl=0 in fault_log.jsonl
  across all 4 tl_fault_s1..s4 campaigns' first real trials, and zero
  behavioral difference from nominal via compare_fault_vs_nominal.py.

  Timing model (zone-triggered periodic, revised 2026-07-24):
    0. Vehicle must travel >= --fault-min-runway-m (straight-line, from
       ground truth) from its trial-start position AND be moving
       (>= 0.5 m/s) before anything below can arm. All 26 goals in this repo
       spawn at the same intersection (captured_goals.json), so without this
       gate, --fault-delay alone (a wall-clock timer) reliably armed faults
       while the vehicle was still at/near that first intersection — this
       step exists specifically to eliminate that confound. A ground-truth
       position jump > 3m mid-batch is treated as a new trial (teleport
       reset between trials) and re-arms this gate fresh for that trial.
    1. Once the runway clears, the vehicle drives nominally for a further
       --fault-delay seconds (originally meant to let EKF/localization
       settle after spawn — now anchored to runway-clear instead of process
       start, so it still means that even mid-batch).
    2. On EVERY entry into a TL detection zone after that delay, the fault
       activates (tl_fault_start logged, cycle counter incremented).
    3. The fault deactivates (tl_fault_end logged) on whichever comes first:
       --fault-duration seconds elapsed (per-zone duration cap), or the
       vehicle leaving the detection zone (a fault that outlasts the
       intersection isn't meaningful, and this bounds exposure at long
       dwells e.g. stopped at a red light).
    4. A --tl-recovery-gap second nominal window follows so residuals have
       a clean segment to return to, then the injector re-arms and repeats
       from step 2 at the NEXT detection zone entered — for every
       intersection the route passes (optionally capped via
       --max-tl-cycles, default unbounded).

    This gives repeated nominal → fault → recovery cycles across a single
    trial — one per TL intersection encountered (more if a single dwell is
    long enough to hit the duration cap more than once) — instead of one
    cycle per trial. Each cycle is anchored to a real TL intersection via
    the map-based zone gating below.

  TL zone gating (always active) — REDESIGNED 2026-07-24, map-position-based:
    Faults are only applied when ego ground-truth position is within
    --tl-zone-radius-m (default 80m — see _DEFAULT_TL_ZONE_RADIUS_M's
    comment for the braking-distance grounding) of a real
    traffic-light position, loaded once at startup from the Lanelet2 map,
    scoped to --zone-goals' routes (default goal_007/012/026 — the fixed
    production goal set; see _load_tl_zone_points).
    ORIGINAL design (message-content heuristic: >=30% of the last 30 raw
    /traffic_signals messages non-empty) is REMOVED, not just retuned — live
    ground-truth data on 2026-07-24 showed it staying "in zone" continuously
    for 60-445m of real driving per activation (Autoware's TL recognition
    evidently keeps the topic non-empty for most of a route's planning
    lookahead, not just current proximity), so faults were firing across
    multi-block stretches of open road, not "at every intersection" as the
    design always claimed. This was true of every TL fault trial ever
    collected under the zone-triggered redesign (2026-07-22 onward), not
    just runs affected by the other 2026-07-24 bugs.

  Fault modes:
    tl_confidence   Multiply each element's confidence by confidence_scale.
                    Realistic model for gradual occlusion (rain, glare).
                    Params: {"confidence_scale": 0.5}

    tl_oscillate    Alternate between forced-GREEN and the true signal on a
                    fixed period.  During the GREEN half-cycle the vehicle
                    accelerates; during the RED half-cycle it brakes — producing
                    velocity oscillations that CUSUM can detect.
                    Params: {"period_s": 5.0}  (half GREEN, half original)

    tl_unknown      Replace all element colors with UNKNOWN, zero confidence.
                    Vehicle falls back to conservative stop behaviour.
                    Params: {}

    tl_blackout     Publish empty TrafficLightGroupArray (complete signal loss).
                    Params: {}

  Severity tiers (revised 2026-06-28):
    S1  tl_confidence {"confidence_scale": 0.5}  — mild degradation (light fog)
    S2  tl_oscillate  {"period_s": 5.0}           — intermittent GREEN/original cycling
    S3  tl_unknown    {}                           — classification failure
    S4  tl_blackout   {}                           — complete camera loss

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
2. IMU BIAS FAULT (periodic)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  IN:  /sensing/imu/imu_data          (imu_corrector output, BEST_EFFORT)
  OUT: /sensing/imu/imu_data_faulted  (gyro_odometer reads this, RELIABLE)

  Signal chain: fault_injector → gyro_odometer → EKF twist →
                x_var/y_var → ST-GAT input features

  NOTE: AEB reads /sensing/imu/imu_data directly and is NOT faulted.
  This is intentional — verified in Autoware source (autonomous_emergency_
  braking/src/node.cpp): AEB independently computes ego_imu_path via
  generateEgoPath(current_v, angular_velocity_ptr_->z) — a simple constant-
  curvature extrapolation from raw speed+yaw-rate, entirely independent of
  ego_mpc_path (the main planning stack's trajectory) — then merges both
  paths' collision checks. This is deliberate diverse redundancy: AEB is
  built specifically to not share a single point of failure with the primary
  planning/localization computation, so faulting its IMU copy too would
  defeat the mechanism meant to catch exactly this kind of failure.
  CAVEAT (2026-07-24): this framing only holds if AEB and gyro_odometer read
  physically diverse sensors on the real target vehicle. If they'd share one
  physical IMU chip in reality, this setup models "the copy of IMU data
  reaching localization is corrupted" (a consumption-path/driver fault), not
  "the IMU sensor itself failed" (a true hardware fault, which would hit
  every consumer identically, AEB included). Worth being explicit in the
  dissertation about which of the two is the actual claim.

  NOTE: gyro_odometer uses only angular_velocity from the IMU message.
  The accel_bias_ms2 parameter biases linear_acceleration.x but this does
  NOT propagate to the EKF in the current Autoware pipeline. Only
  gyro_bias_rads (corrupts angular_velocity.z) has observable effect on
  ST-GAT residuals via EKF twist uncertainty.

  Fault mode: imu_bias
    Periodic constant bias on angular velocity (and optionally acceleration).
    Params: {
        "gyro_bias_rads":  0.1,   # effective: corrupts EKF twist uncertainty
        "accel_bias_ms2":  0.0,   # no-op in current pipeline (gyro_odometer
                                  # does not use linear_acceleration)
        "on_seconds":      30,
        "off_seconds":     30,    # 0 = sustained
    }

  Fault mode: imu_bias_ramp (added 2026-07-24)
    One-shot linear ramp: gyro bias starts at 0 at activation and grows at a
    constant rate until it hits a cap, where it holds. Unlike imu_bias, this
    is not periodic — a single trial is meant to walk from clearly-fused
    (sub Mahalanobis-gate) through the crossing point into clearly-rejected
    (supra-gate, cascading into NDT/planning/MRM failure — see
    docs/design_decisions.md item 7) exactly once, giving a well-defined
    "when did this become unsafe" timestamp for lead-time measurement
    against Autoware's own MRM trigger (the two-arm design's Arm B — see
    docs/theoretical_framework.md §4). The S1-S4 tiers above were
    deliberately redesigned to stay well clear of that boundary; this mode
    exists specifically to find and cross it, in one continuous trial rather
    than guessing a step value.
    Params: {
        "gyro_bias_rate_rads_per_s": 0.003,  # rad/s of bias added per second
                                              # of exposure — a first guess,
                                              # not a validated value: how
                                              # much bias the EKF tolerates
                                              # under slow ramping (vs. an
                                              # instant step) is exactly the
                                              # open empirical question.
        "max_gyro_bias_rads":       0.4,     # cap once reached (safety bound
                                              # in case the gate is never
                                              # crossed at this rate) —
                                              # reached at ~133s at the
                                              # default rate.
        "ramp_log_interval_s":      5.0,     # how often imu_ramp_level
                                              # checkpoints are logged.
    }
    Logs: imu_ramp_started (rate/max at activation) | imu_ramp_level
    (periodic bias-value checkpoints — enough to reconstruct bias(t) exactly,
    since it's linear from a logged start time) | imu_ramp_reached_max.

  Fault mode: imu_scale_factor (added 2026-07-24)
    Periodic on/off multiplicative gain error: angular_velocity.z *= factor,
    instead of imu_bias's additive offset. This is a genuinely different
    error SHAPE, not just a different magnitude — ekf_localizer's yaw_bias
    state (enable_yaw_bias_estimation: true) explicitly models and cancels a
    constant additive discrepancy between gyro-integrated and pose-measured
    yaw; it has no way to represent an error that scales with the true
    signal. Also naturally state-dependent: near-zero error on straight
    roads (true rate ~0) regardless of factor, growing specifically during
    turns/intersections — a single trial can show both an invisible regime
    and a caught regime without an artificial ramp.
    Params: {"gyro_scale_factor": 1.8, "on_seconds": 20, "off_seconds": 30}
    Reuses the same periodic on/off loop as imu_bias (see _imu_bias_loop) —
    only the transform applied in _on_imu differs.

  Fault mode: imu_stuck_at (added 2026-07-24)
    Periodic on/off frozen sensor: angular_velocity.z is held at whatever it
    read the instant the "on" phase activated (or an explicit
    stuck_value_rads), ignoring true motion for the rest of the window. A
    real "sensor stopped updating" failure (bus fault, ADC saturation, driver
    crash), not a constant offset from truth — same reasoning as
    imu_scale_factor for why yaw_bias can't compensate it, and likely to
    cross the Mahalanobis gate the moment the vehicle actually turns (a
    frozen "going straight" reading vs. a real turn is a large, sudden
    discrepancy) — same "unambiguous ground-truth event" spirit as
    tl_blackout.
    Params: {"on_seconds": 20, "off_seconds": 30, "stuck_value_rads": null}

  Severity tiers (gyro_bias_rads is the active parameter) — REDESIGNED
  2026-07-23 after old S2 (0.15 rad/s, on=30s) broke localization within ~1s
  of activation (see docs/design_decisions.md item 7 for the root-cause
  writeup — Mahalanobis twist-gate rejection cascading into NDT/planning
  failure, not just an overly-cautious flag). Bounded to keep peak accumulated heading
  error (gyro_bias_rads × on_seconds) <= 1.2 rad, comfortably under old S1's
  proven-safe 1.5 rad. Authoritative values live in collect.sh's
  imu_fault_s1..s4 cases — this block is kept in sync with them, not a
  second source of truth:
    S1  gyro=0.03 rad/s, on=20s, off=30s  — accumulated 0.6 rad
    S2  gyro=0.05 rad/s, on=20s, off=30s  — accumulated 1.0 rad
    S3  gyro=0.08 rad/s, on=15s, off=30s  — accumulated 1.2 rad
    S4  gyro=0.12 rad/s, on=10s, off=30s  — accumulated 1.2 rad (higher rate, shorter dwell)
  off=30s is uniform across all four tiers (recovery gap, not part of the
  fault window) — lengthened from the old 10-20s range to give NDT/EKF more
  time to reconverge between cycles.

  These S1-S4 tiers are deliberately conservative (all comfortably below the
  gate that broke old S2) and NOT part of the standard campaign suite as of
  2026-07-24 (see collect.sh) — mostly demonstrate ekf_localizer's own
  yaw_bias mitigation working, not this system's detection reach. Kept here
  for ad-hoc reference/comparison only. See imu_bias_ramp, imu_scale_factor,
  and imu_stuck_at above for the modes that are actually in the suite.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
LOGGING
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  All state transitions are written as JSON lines to --log-file.
  Shared:     startup | runway_cleared | trial_reset_detected | shutdown
  TL events:  tl_fault_start | tl_fault_end |
              tl_window_entered | tl_window_exited | tl_fault_applied
  IMU events: imu_fault_activated | imu_bias_on | imu_bias_off

Usage:
  # Passthrough (nominal):
  python3 fault_injector.py

  # S1 — confidence degradation, repeats at every TL zone (45s cap/cycle):
  python3 fault_injector.py --tl-fault tl_confidence \\
      --tl-params '{"confidence_scale":0.5}' --fault-delay 30 --fault-duration 45

  # S2 — oscillating GREEN/original (intermittent), repeats at every TL zone:
  python3 fault_injector.py --tl-fault tl_oscillate \
      --tl-params '{"period_s":5.0}' --fault-delay 30 --fault-duration 45

  # S3 — UNKNOWN classification, repeats at every TL zone:
  python3 fault_injector.py --tl-fault tl_unknown --fault-delay 30 --fault-duration 45

  # S4 — blackout, repeats at every TL zone:
  python3 fault_injector.py --tl-fault tl_blackout --fault-delay 30 --fault-duration 45
"""

import argparse
import json
import math
import os
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from autoware_perception_msgs.msg import TrafficLightGroupArray, TrafficLightElement
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

# Lanelet2 map loading — same pattern as experiments/scripts/plot_routes.py
# and explore_map.py (the reference-correct examples, see CLAUDE.md). Import
# is optional at module scope: a machine that has sourced ROS/Autoware but
# not the venv (or vice versa) can still run passthrough/non-TL-zone work;
# map-zone loading degrades to the flat-distance runway fallback if this
# fails (see _load_tl_zone_points).
try:
    import lanelet2
    from autoware_lanelet2_extension_python.projection import MGRSProjector
    _LANELET2_AVAILABLE = True
except ImportError:
    _LANELET2_AVAILABLE = False


_PERCEPTION_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

# AWSIM's raw ground-truth publisher is BEST_EFFORT (confirmed live via
# `ros2 topic info /awsim/ground_truth/localization/kinematic_state
# --verbose`, 2026-07-24) — a RELIABLE subscriber never connects to a
# BEST_EFFORT publisher (ROS2/DDS QoS compatibility rule), silently, no
# error either side. This was the actual root cause of every fault_log.jsonl
# 'gt_pose_never_received' warning: _gt_sub was requesting _PERCEPTION_QOS
# (RELIABLE) against this topic. Confirmed live: only rosbag2_recorder
# (BEST_EFFORT) was actually receiving messages; fault_injector and
# ros_utils.py's experiment_state_monitor were BOTH requesting RELIABLE and
# both silently never connected.
_GT_SUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

# imu_corrector publishes sensor data as BEST_EFFORT; match it for subscription.
_IMU_SUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# gyro_odometer subscribes with RELIABLE; the fault injector must publish RELIABLE.
_IMU_PUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# Ground-truth pose is used only to gate WHEN a fault may arm (runway/teleport
# tracking) — never fed into the fault itself. This is deliberately separate
# from Autoware's own /localization/kinematic_state so that a fault, once
# active, corrupting Autoware's own estimate can't also corrupt the signal
# used to detect the next trial boundary.
_GT_POSE_TOPIC = '/awsim/ground_truth/localization/kinematic_state'

# A single-step ground-truth position jump bigger than this is a trial-boundary
# teleport/reset, not real driving — same convention/threshold used by
# experiments/scripts/analyze_mrm_diagnostics.py's teleport check.
_TELEPORT_JUMP_THRESHOLD_M = 3.0

# Runway is only considered "cleared" once the vehicle is actually moving,
# not just sitting somewhere >= fault_min_runway_m from spawn (e.g. stopped
# at the very first light within the runway radius).
_RUNWAY_MIN_SPEED_MPS = 0.5

# ── Map-based TL zone detection (added 2026-07-24) ──────────────────────────
# REPLACES the original message-content heuristic (ring buffer of raw
# /traffic_signals non-empty-ness, ~30 msgs, >=30% enter / <15% exit): live
# data showed that heuristic staying "in zone" continuously for 60-445m of
# real driving per activation (confirmed via ground-truth position at the
# fault_log's own tl_window_entered/exited timestamps in the 2026-07-24
# goal_012 run) — Autoware's TL recognition evidently keeps
# /perception/traffic_light_recognition/traffic_signals non-empty for most of
# a route's planning lookahead, not just "currently at an intersection," so
# the old heuristic was never actually a location signal despite being
# designed and documented as one (periodic_fault_strategy.md's own "goal_012
# has 3 zone entries" prediction used this exact same heuristic offline, so
# it's very likely similarly inflated, not just today's live run).
#
# This uses real map data instead: every traffic-light head position
# (Lanelet2 regulatory element, subtype='traffic_light') within a bounding
# box around each of a small set of known goal routes, and gates on ego
# ground-truth proximity to the nearest one. Same technique
# experiments/scripts/plot_routes.py already uses to draw TL markers
# (traffic_light_points()) — ported here, not reimplemented from scratch.
#
# Radius grounded in Autoware's own config, REVISED 2026-07-24 (was 150.0):
# the first version of this reasoning used DETECTION range (how far away
# perception can technically see a light — Autoware's own
# max_detection_range: 200.0, cross-checked against published TL-recognition
# literature at ~120-170m reliable range) — but detection range and REACTION
# range are different things. A fault is only behaviorally meaningful while
# it's active over the span the vehicle would actually be DECIDING/EXECUTING
# a stop, not just technically perceiving a light far ahead with nothing yet
# committed. That's a braking-distance question, not a perception-range one.
#
# Autoware's own planning config (common.param.yaml): max_vel: 11.11 m/s,
# normal.min_acc: -1.0 m/s^2 (the COMFORTABLE deceleration actually used for
# routine stops, not limit.min_acc: -2.5, which is the emergency/urgent
# bound). Comfortable full-speed stopping distance: v^2/(2*|a|) =
# 11.11^2/(2*1.0) ~= 61.7m — the distance from a stop line at which
# Autoware's planner would normally start committing to decelerate for a red
# light. ~30% margin (so the fault is active for the FULL decision/braking
# window, not just barely clipping its start) rounds to 80m. Comfortably
# inside both the old 200m detection ceiling and the ~120-170m literature
# range, so detection was never the bottleneck — reaction was.
_DEFAULT_TL_ZONE_RADIUS_M = 80.0

_SCRIPT_DIR      = os.path.dirname(os.path.abspath(__file__))
_REPO_DIR        = os.path.dirname(os.path.dirname(_SCRIPT_DIR))
_WORKSPACE_DIR   = os.path.dirname(_REPO_DIR)
_DEFAULT_MAP_FILE = os.path.join(_WORKSPACE_DIR, 'Map', 'nishishinjuku_autoware_map', 'lanelet2_map.osm')
_DEFAULT_GOALS_FILE = os.path.join(_REPO_DIR, 'experiments', 'configs', 'captured_goals.json')
# Fixed production goal set (docs/research_notes/periodic_fault_strategy.md
# §4/§5, run_fault_campaigns.sh's default GOALS) — not the whole 383-TL-point
# map, since fault campaigns only ever run on these three routes. Bounding
# box per goal (start_position to goal.position, with margin), not one bbox
# spanning all three, to avoid pulling in irrelevant TLs between routes that
# aren't actually near any of them.
_DEFAULT_ZONE_GOALS = ['goal_007', 'goal_012', 'goal_026']
_ZONE_BBOX_MARGIN_M = 100.0


def _ll_attr(ll, key, default=''):
    try:
        return ll.attributes[key]
    except Exception:
        return default


def _traffic_light_points_in_bbox(map_data, xmin, xmax, ymin, ymax) -> List[Tuple[float, float]]:
    """Midpoint of each traffic-light-head linestring ('refers' role of
    traffic_light regulatory elements) within the bbox, deduped by regulatory
    element id. Ported from experiments/scripts/plot_routes.py's
    traffic_light_points() — same technique, not reimplemented.
    """
    seen_reg = set()
    pts = []
    for ll in map_data.laneletLayer:
        for reg in ll.regulatoryElements:
            if _ll_attr(reg, 'subtype') != 'traffic_light' or reg.id in seen_reg:
                continue
            seen_reg.add(reg.id)
            try:
                refers = reg.parameters['refers']
            except Exception:
                continue
            for ls in refers:
                pts_xy = [(p.x, p.y) for p in ls]
                if not pts_xy:
                    continue
                mx = sum(p[0] for p in pts_xy) / len(pts_xy)
                my = sum(p[1] for p in pts_xy) / len(pts_xy)
                if xmin <= mx <= xmax and ymin <= my <= ymax:
                    pts.append((mx, my))
    return pts


def _load_tl_zone_points(
    goal_ids: List[str],
    map_file: str = _DEFAULT_MAP_FILE,
    goals_file: str = _DEFAULT_GOALS_FILE,
    bbox_margin_m: float = _ZONE_BBOX_MARGIN_M,
) -> List[Tuple[float, float]]:
    """Real traffic-light positions near the given goals' routes, unioned
    across per-goal bounding boxes. Returns [] (not an exception) on any
    failure — callers fall back to the flat-distance runway / no map-zone
    TL gating, degraded but not crashed, since a map/goals-file problem
    shouldn't take down fault injection entirely.
    """
    if not _LANELET2_AVAILABLE:
        return []
    try:
        with open(goals_file) as f:
            goals_data = json.load(f)
        goals_by_id = {g['id']: g for g in goals_data['goals']}

        projector = MGRSProjector(lanelet2.io.Origin(0.0, 0.0))
        map_data, _errs = lanelet2.io.loadRobust(map_file, projector)

        all_points: List[Tuple[float, float]] = []
        seen_xy = set()
        for gid in goal_ids:
            g = goals_by_id.get(gid)
            if g is None:
                continue
            sx, sy = g['goal']['start_position']['x'], g['goal']['start_position']['y']
            gx, gy = g['goal']['position']['x'], g['goal']['position']['y']
            xmin, xmax = min(sx, gx) - bbox_margin_m, max(sx, gx) + bbox_margin_m
            ymin, ymax = min(sy, gy) - bbox_margin_m, max(sy, gy) + bbox_margin_m
            for pt in _traffic_light_points_in_bbox(map_data, xmin, xmax, ymin, ymax):
                key = (round(pt[0], 1), round(pt[1], 1))
                if key not in seen_xy:
                    seen_xy.add(key)
                    all_points.append(pt)
        return all_points
    except Exception:
        return []


class FaultInjector(Node):

    def __init__(
        self,
        tl_fault:          Optional[str],
        tl_params:         Dict[str, Any],
        imu_fault:         Optional[str],
        imu_params:        Dict[str, Any],
        fault_delay:       float,
        fault_duration:    float,
        tl_recovery_gap:   float,
        max_tl_cycles:     int,
        fault_min_runway_m: float,
        log_file:          Optional[str],
        tl_zone_radius_m:  float = _DEFAULT_TL_ZONE_RADIUS_M,
        zone_goals:        Optional[List[str]] = None,
    ):
        super().__init__(
            'fault_injector',
            parameter_overrides=[
                rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True),
            ],
        )

        self._tl_fault_config: Optional[str] = tl_fault   # what to apply when active
        self._tl_params                       = tl_params
        self._imu_fault_config: Optional[str] = imu_fault  # what's configured for this run
        self._imu_fault_type: Optional[str]   = None       # None until armed for this trial
        self._imu_params                      = imu_params
        self._fault_delay                     = fault_delay
        self._fault_duration                  = fault_duration
        self._tl_recovery_gap                 = tl_recovery_gap
        self._max_tl_cycles                   = max_tl_cycles  # 0 = unbounded
        self._fault_min_runway_m              = fault_min_runway_m

        # Map-based TL zone points — loaded once, synchronously, at startup
        # (a few hundred ms at most for 3 goal bboxes; acceptable one-time
        # cost vs. the complexity of doing this async before any subscription
        # is safe to act on). Empty list (map/goals-file unavailable, or
        # lanelet2 not importable) degrades to the pre-2026-07-24 flat-
        # distance-only runway and disables map-based TL fault gating — see
        # _on_gt_pose and _on_tl.
        self._tl_zone_radius_m = tl_zone_radius_m
        self._tl_zone_points: List[Tuple[float, float]] = _load_tl_zone_points(
            zone_goals or _DEFAULT_ZONE_GOALS
        )
        if self._tl_zone_points:
            self.get_logger().info(
                f'Loaded {len(self._tl_zone_points)} map TL points for zone '
                f'detection (radius={tl_zone_radius_m:.0f}m, goals='
                f'{zone_goals or _DEFAULT_ZONE_GOALS})'
            )
        else:
            self.get_logger().warn(
                'No map TL points loaded — falling back to flat-distance-only '
                'runway (no map-based "cleared the first intersection" check), '
                'and TL fault zone gating is disabled (tl_fault will never arm '
                'if configured). Check lanelet2 is importable and '
                f'{_DEFAULT_MAP_FILE} / {_DEFAULT_GOALS_FILE} exist.'
            )
        # Sticky, one-shot runway tracking (distinct from the continuous
        # _in_detection_zone the TL fault state machine cycles on all trial
        # long) — "cleared the first real intersection" for THIS trial only.
        self._runway_seen_first_zone: bool = False
        self._runway_position_ok:     bool = False
        self._log_file                        = log_file
        # Opened once and kept open for the node's lifetime (main() already
        # truncated/created it with the startup record) — avoids an open+close
        # syscall pair on every logged event, just a buffered write + flush.
        self._log_fh = open(log_file, 'a') if log_file else None
        self._lock                            = threading.Lock()

        # TL fault state machine — zone-triggered periodic, gated by a runway
        # (revised 2026-07-24 — see _on_gt_pose):
        # Phases: 'waiting_runway' → 'waiting_delay' → 'waiting_zone' →
        #         'fault_active' → 'recovering' → 'waiting_zone' → ...
        #         (repeats at every TL zone encountered for the rest of the
        #         trial). 'done' is a terminal passthrough state, only entered
        #         if no TL fault is configured, or --max-tl-cycles is reached
        #         for the CURRENT trial (a teleport/new-trial reset re-arms
        #         from 'waiting_runway', giving each trial its own cycle
        #         budget — see _on_gt_pose's reset_detected branch).
        #
        # 'waiting_runway' replaces the old design where --fault-delay was a
        # single wall-clock timer counted from THIS PROCESS's own startup —
        # since the fault injector is one long-lived relay for an entire
        # campaign batch (run_experiments.py starts it once, before the first
        # trial's engage), that old delay only ever meant something for the
        # batch's first trial, and had no relationship to where the vehicle
        # physically was. Every collected goal in this repo spawns at the
        # SAME intersection (verified against captured_goals.json — all 26
        # goals share one start_position), so a delay with no runway
        # component reliably armed the fault while the vehicle was still at
        # or near that first intersection. --fault-delay is now a floor
        # measured from when the runway clears (its original intended
        # purpose — letting EKF/localization settle — still applies, just
        # anchored to a meaningful moment instead of process start).
        self._tl_phase: str             = 'waiting_runway' if tl_fault else 'done'
        self._tl_fault_active: bool     = False   # True only during a fault window
        self._tl_fault_start_time: float = 0.0    # used by tl_confidence_ramp
        self._tl_fault_end_time: float  = 0.0
        self._tl_recovering_end_time: float = 0.0
        self._tl_cycle_count: int       = 0
        self._tl_runway_wait_started: bool = False  # guards spawning the delay-then-arm thread once

        # Runway / trial-boundary tracking, driven by ground-truth pose only
        # (never by Autoware's own /localization/kinematic_state, which a
        # fault — once active — may itself corrupt; see _GT_POSE_TOPIC).
        self._gt_ready: bool                       = False
        self._trial_origin_xy: Optional[Tuple[float, float]] = None
        self._last_gt_xy: Optional[Tuple[float, float]]      = None
        # Wall-clock time of the last GT message, used to estimate speed from
        # position deltas rather than trusting msg.twist — AWSIM's raw ground-
        # truth publisher's twist field has never been verified populated
        # anywhere in this codebase (ros_utils.py's own GT subscription only
        # ever reads .pose.pose.position, never .twist); if it's always 0.0,
        # a twist-based speed gate can never clear regardless of actual motion
        # (suspected root cause of goal_012 tl_fault_s1..s4/imu_fault_s1
        # 2026-07-24 never arming any fault — see TODO.md).
        self._last_gt_time: float                   = 0.0
        self._runway_cleared: bool                  = False
        # Shared by TL and IMU arming: bumped on every detected trial reset so
        # a delay-then-arm thread spawned for a superseded trial can recognize
        # it's stale and no-op instead of firing late into the new trial (see
        # _delay_then_arm_tl / _arm_imu_for_trial).
        self._trial_generation: int                 = 0

        # TL zone membership — set by _on_gt_pose (map-position-based), read
        # by _on_tl. See _DEFAULT_TL_ZONE_RADIUS_M's comment for why this is
        # position-based now rather than the original message-content ring-
        # buffer heuristic.
        self._in_detection_zone: bool = False

        # Counters
        self._msg_count_tl        = 0
        self._msg_count_imu       = 0
        self._tl_fault_applied    = 0
        self._imu_bias_on_cycles  = 0

        # ── Traffic light relay ───────────────────────────────────────────────
        # Subscribes to Autoware's REAL final TL output (unchanged on the
        # Autoware side) and publishes a new "_faulted" topic — behavior_planning
        # must be repointed to consume it (Autoware-side patch, see README.md),
        # mirroring the objects → objects_filtered pattern.
        self._tl_pub = self.create_publisher(
            TrafficLightGroupArray,
            '/perception/traffic_light_recognition/traffic_signals_faulted',
            _PERCEPTION_QOS,
        )
        self._tl_sub = self.create_subscription(
            TrafficLightGroupArray,
            '/perception/traffic_light_recognition/traffic_signals',
            self._on_tl,
            _PERCEPTION_QOS,
        )

        # ── IMU relay ────────────────────────────────────────────────────────
        self._imu_pub = self.create_publisher(
            Imu,
            '/sensing/imu/imu_data_faulted',
            _IMU_PUB_QOS,
        )
        self._imu_sub = self.create_subscription(
            Imu,
            '/sensing/imu/imu_data',
            self._on_imu,
            _IMU_SUB_QOS,
        )

        self._imu_bias_active = False

        # imu_bias_ramp state — set once at activation in _arm_imu_for_trial,
        # read (never written) per-message in _on_imu for maximum timing
        # precision. Zeroed here and on trial reset so a stale ramp can never
        # be computed if _imu_bias_active somehow lagged behind a reset.
        self._imu_ramp_start_time: float = 0.0
        self._imu_ramp_rate:       float = 0.0
        self._imu_ramp_max:        float = 0.0

        # imu_stuck_at state — the frozen value for the CURRENT "on" cycle,
        # captured lazily on first message once active, cleared on the
        # following "off" phase so the next cycle captures its own fresh value.
        self._imu_stuck_value: Optional[float] = None

        # ── Ground-truth pose relay (runway/teleport tracking only) ────────────
        self._gt_sub = self.create_subscription(
            Odometry,
            _GT_POSE_TOPIC,
            self._on_gt_pose,
            _GT_SUB_QOS,
        )

        # Diagnostic: fail LOUDLY, not silently, if ground truth never arrives
        # or goes stale mid-trial — a runway that never clears otherwise looks
        # exactly like a quiet, "successful" trial with the fault simply never
        # firing (confirmed happening 2026-07-24: goal_012 tl_fault_s1..s4 all
        # completed goal_reached=True with zero tl_fault_start/runway_cleared
        # events). Checked every 15s; only ever logs, never blocks anything.
        self._gt_last_msg_wall_time: Optional[float] = None
        self._gt_health_timer = self.create_timer(15.0, self._check_gt_health)

        # Fault arming is normally entirely driven by _on_gt_pose (runway
        # cleared → start the --fault-delay floor → arm) — see
        # _delay_then_arm_tl (TL) and _arm_imu_for_trial (IMU).
        #
        # FALLBACK (added 2026-07-24): --fault-min-runway-m <= 0 disables the
        # runway requirement entirely and arms directly from process start,
        # matching the pre-runway-gating behavior exactly (same functions,
        # just triggered here instead of from a runway-clear event). This
        # exists because the runway gate depends on _on_gt_pose firing at
        # all, and in the 2026-07-24 goal_012 smoke test it never did across
        # 9/9 fresh trials (gt_pose_never_received fired every ~15s for the
        # full duration of every trial) — root cause not yet found (topic
        # name, QoS reliability/durability, and message type all match
        # ros_utils.py's proven-working live subscription to the same topic;
        # see docs/design_decisions.md and TODO.md). Use this to keep
        # collecting real fault data while that's chased down live — it
        # reintroduces the original "may arm near the start intersection"
        # limitation the runway gate was built to remove, nothing else.
        if fault_min_runway_m <= 0:
            self.get_logger().warn(
                'fault_min_runway_m <= 0 — runway gating DISABLED, falling back '
                'to delay-from-process-start arming (pre-2026-07-24 behavior). '
                'Faults may arm near the shared start intersection again.'
            )
            if tl_fault:
                t = threading.Thread(
                    target=self._delay_then_arm_tl,
                    args=(fault_delay, self._trial_generation),
                    daemon=True,
                )
                t.start()
            if imu_fault:
                t = threading.Thread(
                    target=self._arm_imu_for_trial,
                    args=(imu_fault, fault_delay, self._trial_generation),
                    daemon=True,
                )
                t.start()

        self.get_logger().info(
            f'FaultInjector ready — tl={tl_fault or "passthrough"}, '
            f'imu={imu_fault or "passthrough"}, '
            f'min_runway={fault_min_runway_m:.0f}m, delay={fault_delay:.0f}s '
            f'(floor after runway clears), duration={fault_duration:.0f}s'
        )

    # ── Runway / trial-boundary tracking ────────────────────────────────────────

    def _on_gt_pose(self, msg: Odometry):
        """Track ground-truth position to (1) gate fault arming on a post-
        spawn runway, (2) detect trial-boundary teleports so each trial gets
        its own runway + delay + (for TL) cycle budget, and (3) — as of
        2026-07-24 — drive TL zone membership from real map positions
        instead of the message-content heuristic _on_tl used to compute (see
        _DEFAULT_TL_ZONE_RADIUS_M's comment for why that heuristic was
        replaced, not just tuned).

        Speed is estimated from consecutive position deltas, NOT read from
        msg.twist — AWSIM's raw ground-truth publisher's twist field has never
        been verified populated anywhere in this codebase (ros_utils.py's own
        GT subscription only ever reads .pose.pose.position). If twist is
        always 0.0 there, a twist-based speed gate can never clear regardless
        of actual motion — the suspected cause of every tl_fault_s1..s4 /
        imu_fault_s1 trial in the goal_012 smoke test (2026-07-24) completing
        goal_reached=True with zero runway_cleared/fault events ever logged.
        """
        now = time.time()
        self._gt_last_msg_wall_time = now

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Pure function of position — computed outside the lock, cheap even
        # with the full goal-scoped TL point set (tens of points, not 383).
        in_map_zone = any(
            math.hypot(x - px, y - py) <= self._tl_zone_radius_m
            for px, py in self._tl_zone_points
        )

        newly_cleared  = False
        reset_detected = False
        zone_entered   = False
        zone_exited    = False
        gen            = None  # this trial's generation, captured under lock below
        speed          = 0.0

        with self._lock:
            if not self._gt_ready:
                self._gt_ready = True
                self._trial_origin_xy = (x, y)
                self._last_gt_xy = (x, y)
                self._last_gt_time = now
            else:
                last_x, last_y = self._last_gt_xy
                dt = max(now - self._last_gt_time, 1e-3)
                jump = math.hypot(x - last_x, y - last_y)
                speed = jump / dt
                self._last_gt_xy = (x, y)
                self._last_gt_time = now
                if jump > _TELEPORT_JUMP_THRESHOLD_M:
                    reset_detected = True
                    self._trial_origin_xy = (x, y)
                    self._runway_cleared = False
                    self._runway_seen_first_zone = False
                    self._runway_position_ok = False
                    self._tl_runway_wait_started = False
                    if self._tl_fault_config:
                        self._tl_phase = 'waiting_runway'
                        self._tl_cycle_count = 0
                    self._trial_generation += 1
                    self._imu_fault_type = None
                    self._imu_bias_active = False
                    self._imu_ramp_start_time = 0.0
                    self._imu_stuck_value = None

            # ── TL zone membership (map-position-based) ─────────────────────
            was_in_zone = self._in_detection_zone
            self._in_detection_zone = in_map_zone
            zone_entered = in_map_zone and not was_in_zone
            zone_exited  = (not in_map_zone) and was_in_zone

            # ── Runway ───────────────────────────────────────────────────────
            if not self._runway_cleared:
                if self._tl_zone_points:
                    # Map-based: "cleared the first real intersection" — sticky
                    # once the position criterion is met, so it doesn't require
                    # the exact zone-exit message to also have speed>=threshold.
                    if in_map_zone:
                        self._runway_seen_first_zone = True
                    if self._runway_seen_first_zone and zone_exited:
                        self._runway_position_ok = True
                else:
                    # Fallback (no map points loaded): flat straight-line
                    # distance from trial start, pre-2026-07-24 behavior.
                    if self._trial_origin_xy is not None:
                        ox, oy = self._trial_origin_xy
                        if math.hypot(x - ox, y - oy) >= self._fault_min_runway_m:
                            self._runway_position_ok = True

                if self._runway_position_ok and speed >= _RUNWAY_MIN_SPEED_MPS:
                    self._runway_cleared = True
                    newly_cleared = True

            start_tl_wait = (
                newly_cleared and self._tl_fault_config
                and self._tl_phase == 'waiting_runway'
                and not self._tl_runway_wait_started
            )
            if start_tl_wait:
                self._tl_runway_wait_started = True

            start_imu_wait = newly_cleared and self._imu_fault_config

            if start_tl_wait or start_imu_wait:
                gen = self._trial_generation  # capture once, used by both below

        if zone_entered:
            self._log_event('tl_window_entered', {'method': 'map_position'})
        elif zone_exited:
            self._log_event('tl_window_exited', {'method': 'map_position'})

        if reset_detected:
            self._log_event('trial_reset_detected', {'jump_m': round(jump, 2)})
            self.get_logger().info(
                f'Trial boundary detected (ground-truth jump={jump:.1f}m) — '
                f're-arming runway gate for the new trial'
            )

        if newly_cleared:
            self._log_event('runway_cleared', {
                'method':       'map_zone' if self._tl_zone_points else 'flat_distance',
                'min_runway_m': self._fault_min_runway_m,
                'speed_mps':    round(speed, 2),
            })
            self.get_logger().info(
                f'Runway cleared (speed={speed:.1f}m/s) — starting fault-delay floor'
            )

        if start_tl_wait:
            t = threading.Thread(
                target=self._delay_then_arm_tl,
                args=(self._fault_delay, gen),
                daemon=True,
            )
            t.start()

        if start_imu_wait:
            t = threading.Thread(
                target=self._arm_imu_for_trial,
                args=(self._imu_fault_config, self._fault_delay, gen),
                daemon=True,
            )
            t.start()

    def _check_gt_health(self):
        """Periodic watchdog (every 15s) — logs loudly if ground truth has
        never arrived, or has gone stale mid-trial, instead of letting a fault
        campaign silently run as an unfaulted passthrough for its entire
        duration with no visible sign anything is wrong.
        """
        now = time.time()
        if self._gt_last_msg_wall_time is None:
            self._log_event('gt_pose_never_received', {'topic': _GT_POSE_TOPIC})
            self.get_logger().warn(
                f'No ground-truth message received on {_GT_POSE_TOPIC} since '
                f'startup — fault_min_runway_m gating cannot clear, so NO fault '
                f'will ever arm this trial. Check `ros2 topic info {_GT_POSE_TOPIC} '
                f'--verbose` (publisher count/QoS) and `ros2 topic hz {_GT_POSE_TOPIC}`.'
            )
        else:
            age = now - self._gt_last_msg_wall_time
            if age > 15.0:
                self._log_event('gt_pose_stale', {'age_sec': round(age, 1)})
                self.get_logger().warn(
                    f'Ground-truth message on {_GT_POSE_TOPIC} is {age:.0f}s '
                    f'stale — runway/teleport tracking is blind until it resumes.'
                )

    # ── Phase transitions ─────────────────────────────────────────────────────

    def _delay_then_arm_tl(self, delay: float, gen: int):
        """After the runway-cleared delay floor, move waiting_runway → waiting_zone
        — unless a later trial reset (gen mismatch) has already superseded
        this wait, which would otherwise let a stale thread from a previous
        trial arm the fault early into a new one.
        """
        time.sleep(delay)
        with self._lock:
            if gen != self._trial_generation:
                return  # a teleport reset happened during the delay; stale
            if self._tl_phase == 'waiting_runway':
                self._tl_phase = 'waiting_zone'
        self.get_logger().info(
            f'TL fault armed — waiting for first detection zone entry'
        )

    def _arm_imu_for_trial(self, imu_fault: str, delay: float, gen: int):
        """Wait out the post-runway delay floor, then activate the IMU fault
        for this trial generation — unless superseded by a newer trial reset
        (gen mismatch) in the meantime. Dispatches to the periodic on/off loop
        (imu_bias) or the one-shot ramp (imu_bias_ramp) based on fault type.
        """
        time.sleep(delay)
        with self._lock:
            if gen != self._trial_generation:
                return  # a teleport reset happened during the delay; stale
            self._imu_fault_type = imu_fault
            if imu_fault == 'imu_bias_ramp':
                self._imu_ramp_start_time = time.time()
                self._imu_ramp_rate = float(self._imu_params.get('gyro_bias_rate_rads_per_s', 0.003))
                self._imu_ramp_max  = float(self._imu_params.get('max_gyro_bias_rads', 0.4))
                self._imu_bias_active = True  # ramp is "active" for its entire trial, no on/off toggling
        self._log_event('imu_fault_activated', {
            'fault_type': imu_fault, 'params': self._imu_params, 'generation': gen,
        })
        if imu_fault == 'imu_bias_ramp':
            t = threading.Thread(target=self._imu_ramp_log_loop, args=(gen,), daemon=True)
        else:
            t = threading.Thread(target=self._imu_bias_loop, args=(gen,), daemon=True)
        t.start()

    def _imu_ramp_log_loop(self, gen: int):
        """Periodically checkpoints the ramp's bias-vs-time to the fault log
        for traceability (bias(t) is fully reconstructible from
        imu_ramp_started's rate + start time alone, but explicit checkpoints
        make post-hoc correlation with /diagnostics trivial without redoing
        that arithmetic). Applying the bias itself happens in _on_imu, computed
        fresh per-message from self._imu_ramp_start_time/_rate/_max — this
        loop never touches the applied value, only logs it.
        """
        log_interval = float(self._imu_params.get('ramp_log_interval_s', 5.0))
        with self._lock:
            rate, max_bias, start = self._imu_ramp_rate, self._imu_ramp_max, self._imu_ramp_start_time
        self._log_event('imu_ramp_started', {
            'gyro_bias_rate_rads_per_s': rate, 'max_gyro_bias_rads': max_bias, 'generation': gen,
        })
        while True:
            with self._lock:
                if gen != self._trial_generation:
                    return
            elapsed = time.time() - start
            bias = min(max_bias, rate * elapsed)
            self._log_event('imu_ramp_level', {
                'elapsed_s': round(elapsed, 1), 'gyro_bias_rads': round(bias, 4), 'generation': gen,
            })
            if bias >= max_bias:
                self._log_event('imu_ramp_reached_max', {
                    'gyro_bias_rads': max_bias, 'elapsed_s': round(elapsed, 1), 'generation': gen,
                })
                return
            time.sleep(log_interval)

    def _start_tl_fault_window(self):
        """Called when a detection zone is entered while in waiting_zone phase."""
        start_t = time.time()
        end_t = start_t + self._fault_duration
        with self._lock:
            self._tl_cycle_count  += 1
            cycle                  = self._tl_cycle_count
            self._tl_phase         = 'fault_active'
            self._tl_fault_active  = True
            self._tl_fault_start_time = start_t
            self._tl_fault_end_time = end_t
        self._log_event('tl_fault_start', {
            'fault_type':      self._tl_fault_config,
            'params':          self._tl_params,
            'duration_cap_sec': self._fault_duration,
            'cycle':           cycle,
        })
        self.get_logger().info(
            f'TL fault ACTIVE (cycle {cycle}): {self._tl_fault_config}, '
            f'duration_cap={self._fault_duration:.0f}s'
        )

    def _end_tl_fault_window(self, reason: str):
        """Called when the fault duration cap elapses or the zone is exited."""
        with self._lock:
            self._tl_phase              = 'recovering'
            self._tl_fault_active        = False
            self._tl_recovering_end_time = time.time() + self._tl_recovery_gap
            cycle                        = self._tl_cycle_count
        self._log_event('tl_fault_end', {
            'fault_type':    self._tl_fault_config,
            'applied_count': self._tl_fault_applied,
            'cycle':         cycle,
            'reason':        reason,
        })
        self.get_logger().info(
            f'TL fault ENDED (cycle {cycle}, reason={reason}) — '
            f'recovering for {self._tl_recovery_gap:.0f}s '
            f'({self._tl_fault_applied} messages faulted so far)'
        )

    def _rearm_tl_zone_wait(self):
        """Called when the post-fault recovery gap elapses — arms for the next zone."""
        with self._lock:
            self._tl_phase = 'waiting_zone'

    def _exhaust_tl_cycles(self):
        """Called once --max-tl-cycles is reached — permanent passthrough."""
        with self._lock:
            self._tl_phase = 'done'
        self._log_event('tl_cycles_exhausted', {'cycles': self._tl_cycle_count})
        self.get_logger().info(
            f'TL fault cycles exhausted ({self._tl_cycle_count}/{self._max_tl_cycles}) '
            f'— passthrough for remainder of trial'
        )

    # ── Traffic light relay ───────────────────────────────────────────────────

    def _on_tl(self, msg: TrafficLightGroupArray):
        self._msg_count_tl += 1

        # Zone membership is owned by _on_gt_pose (map-position-based) as of
        # 2026-07-24 — this callback only reads it. The original design had
        # this callback compute "in zone" from the raw message content itself
        # (ring buffer of non-empty groups); replaced because that heuristic
        # turned out not to track physical intersection proximity at all —
        # see _DEFAULT_TL_ZONE_RADIUS_M's comment for the live evidence.
        with self._lock:
            phase      = self._tl_phase
            in_zone    = self._in_detection_zone
            fault_end  = self._tl_fault_end_time
            rec_end    = self._tl_recovering_end_time

        # ── Phase transitions (outside lock to avoid blocking) ────────────────
        # Loop: waiting_zone → fault_active → recovering → waiting_zone → ...
        # repeats at every TL zone encountered, so a single trial accumulates
        # one (reaction, recovery) sample per intersection instead of one
        # per trial. See periodic_fault_strategy.md.
        #
        # BUG FIXED 2026-07-24: these were sequential `if`s, not `elif`s, so a
        # phase transition triggered by the first block (e.g. waiting_zone →
        # fault_active) fell through into the next block's check IN THE SAME
        # CALLBACK — but `fault_end`/`rec_end` above were snapshotted from
        # self._tl_fault_end_time/self._tl_recovering_end_time BEFORE that
        # transition ran, so the fall-through check compared `now` against a
        # STALE end-time left over from the previous cycle (or the 0.0 init
        # default, for the very first cycle). That stale value was almost
        # always already in the past, so the fault ended itself immediately
        # (0-duration) roughly every other cycle — confirmed in every
        # tl_fault_s1..s4 trial collected 2026-07-22 (fault_windows alternate
        # a real ~15s window with a 0-duration phantom one). Using `elif`
        # means only the phase active at THIS callback's entry can transition
        # once; a freshly-entered phase is only evaluated on the NEXT
        # callback, by which point the top-of-function snapshot is fresh.
        if phase == 'waiting_zone' and in_zone:
            if self._max_tl_cycles and self._tl_cycle_count >= self._max_tl_cycles:
                self._exhaust_tl_cycles()
                phase = 'done'
            else:
                self._start_tl_fault_window()
                phase = 'fault_active'

        elif phase == 'fault_active':
            now = time.time()
            if now >= fault_end:
                self._end_tl_fault_window('duration_cap')
                phase = 'recovering'
            elif not in_zone:
                self._end_tl_fault_window('zone_exited')
                phase = 'recovering'

        elif phase == 'recovering' and time.time() >= rec_end:
            self._rearm_tl_zone_wait()
            phase = 'waiting_zone'

        # ── Apply or passthrough ──────────────────────────────────────────────
        with self._lock:
            active      = self._tl_fault_active and self._in_detection_zone
            fault_start = self._tl_fault_start_time

        if not active:
            self._tl_pub.publish(msg)
            return

        fault = self._tl_fault_config

        if fault == 'tl_blackout':
            empty       = TrafficLightGroupArray()
            empty.stamp = msg.stamp
            self._tl_pub.publish(empty)

        elif fault == 'tl_confidence':
            # Mutated in place, not deepcopy'd: rclpy hands this callback a
            # freshly-deserialized message with no other owner, so there's
            # nothing to preserve by copying it first (removes an allocation
            # + full nested-object copy on every message while the fault is
            # active, which otherwise runs at TL topic rate — ~20Hz observed).
            scale = self._tl_params.get('confidence_scale', 0.5)
            for group in msg.traffic_light_groups:
                for elem in group.elements:
                    elem.confidence = max(0.0, min(1.0, elem.confidence * scale))
            self._tl_pub.publish(msg)

        elif fault == 'tl_confidence_ramp':
            # Same effect as tl_confidence, but confidence_scale decays
            # linearly from 1.0 (no degradation) down to min_confidence_scale
            # over the CURRENT fault-active window (elapsed since this
            # cycle's own tl_fault_start, not the whole trial — mirrors
            # imu_bias_ramp's one-shot-per-activation shape, just re-armed
            # fresh at every TL zone like the rest of the periodic TL faults,
            # instead of once for the whole trial). Gives a richer,
            # continuously-varying signal (a real approach-to-intersection
            # glare/occlusion getting gradually worse) instead of a step
            # function — see module docstring.
            rate      = self._tl_params.get('confidence_ramp_rate_per_s', 0.1)
            min_scale = self._tl_params.get('min_confidence_scale', 0.0)
            elapsed   = time.time() - fault_start
            scale     = max(min_scale, 1.0 - rate * elapsed)
            for group in msg.traffic_light_groups:
                for elem in group.elements:
                    elem.confidence = max(0.0, min(1.0, elem.confidence * scale))
            self._tl_pub.publish(msg)

        elif fault == 'tl_oscillate':
            # Alternate between forced-GREEN and the original signal on a fixed
            # period.  This produces velocity oscillations at intersections that
            # CUSUM can accumulate — the vehicle repeatedly starts/stops as the
            # signal toggles between GREEN (go) and the true RED (stop).
            period = self._tl_params.get('period_s', 5.0)
            if time.time() % period < period / 2:
                for group in msg.traffic_light_groups:
                    for elem in group.elements:
                        elem.color      = TrafficLightElement.GREEN
                        elem.confidence = 1.0
            self._tl_pub.publish(msg)

        elif fault == 'tl_unknown':
            for group in msg.traffic_light_groups:
                for elem in group.elements:
                    elem.color      = TrafficLightElement.UNKNOWN
                    elem.confidence = 0.0
            self._tl_pub.publish(msg)

        else:
            self.get_logger().warn(
                f'Unknown TL fault type: {fault}',
                throttle_duration_sec=5.0,
            )
            self._tl_pub.publish(msg)
            return

        self._tl_fault_applied += 1
        if self._tl_fault_applied % 50 == 1:
            self._log_event('tl_fault_applied', {
                'fault': fault, 'count': self._tl_fault_applied,
            })

    # ── IMU relay ────────────────────────────────────────────────────────────

    def _on_imu(self, msg: Imu):
        self._msg_count_imu += 1
        with self._lock:
            fault      = self._imu_fault_type
            active     = self._imu_bias_active
            ramp_start = self._imu_ramp_start_time
            ramp_rate  = self._imu_ramp_rate
            ramp_max   = self._imu_ramp_max

        if fault == 'imu_bias' and active:
            msg.linear_acceleration.x += self._imu_params.get('accel_bias_ms2', 0.0)
            msg.angular_velocity.z    += self._imu_params.get('gyro_bias_rads', 0.0)

        elif fault == 'imu_bias_ramp' and active:
            # Computed fresh per-message (not from the log loop's coarse
            # checkpoints) for maximum timing precision — this is what
            # actually reaches the EKF, so it must be exact, not a 5s-stale
            # approximation.
            elapsed = time.time() - ramp_start
            msg.angular_velocity.z += min(ramp_max, ramp_rate * elapsed)

        elif fault == 'imu_scale_factor' and active:
            # Multiplicative, not additive — structurally different from
            # imu_bias: error is proportional to the TRUE rate, so it's near
            # zero on straight roads and grows specifically during turns.
            # yaw_bias (an additive-constant EKF state) cannot represent this.
            scale = self._imu_params.get('gyro_scale_factor', 1.0)
            msg.angular_velocity.z *= scale

        elif fault == 'imu_stuck_at' and active:
            # Freeze at whatever the sensor read the instant this "on" cycle
            # activated (or an explicit stuck_value_rads if given), ignoring
            # true motion for the rest of the window — a real "sensor stopped
            # updating" failure mode, not a constant offset from truth, so
            # yaw_bias's compensation (built for a constant discrepancy) has
            # no fixed target to converge to here either.
            stuck_value = self._imu_params.get('stuck_value_rads')
            if stuck_value is None:
                with self._lock:
                    if self._imu_stuck_value is None:
                        self._imu_stuck_value = msg.angular_velocity.z
                    stuck_value = self._imu_stuck_value
            msg.angular_velocity.z = stuck_value

        elif fault == 'imu_stuck_at' and not active:
            # Off phase (or not yet armed) — clear so the NEXT "on" cycle
            # re-captures a fresh frozen value instead of reusing this one.
            if self._imu_stuck_value is not None:
                with self._lock:
                    self._imu_stuck_value = None

        self._imu_pub.publish(msg)

    def _imu_bias_loop(self, gen: int):
        """Runs the on/off bias cycle for trial generation `gen`. Bails out
        without touching state if a teleport (_on_gt_pose) has already bumped
        `self._trial_generation` past `gen` — that handler already force-set
        _imu_bias_active=False synchronously, so the only failure mode this
        guards against is a stale loop iteration re-enabling the bias for a
        trial that has already ended.
        """
        on_sec  = float(self._imu_params.get('on_seconds',  30))
        off_sec = float(self._imu_params.get('off_seconds', 30))

        while True:
            with self._lock:
                if gen != self._trial_generation:
                    return
                self._imu_bias_active = True
                self._imu_bias_on_cycles += 1
                cycle = self._imu_bias_on_cycles
            self._log_event('imu_bias_on', {
                'cycle':      cycle,
                'accel_bias': self._imu_params.get('accel_bias_ms2'),
                'gyro_bias':  self._imu_params.get('gyro_bias_rads'),
                'on_seconds': on_sec,
                'generation': gen,
            })
            time.sleep(on_sec)

            if off_sec <= 0:
                return  # sustained fault (off_seconds=0) — stays active, no toggle-off

            with self._lock:
                if gen != self._trial_generation:
                    return
                self._imu_bias_active = False

            self._log_event('imu_bias_off', {
                'cycle':       cycle,
                'off_seconds': off_sec,
                'generation':  gen,
            })
            time.sleep(off_sec)

    # ── Logging ──────────────────────────────────────────────────────────────

    def _log_event(self, event: str, extra: dict | None = None):
        try:
            sim_sec = self.get_clock().now().nanoseconds * 1e-9
        except Exception:
            sim_sec = 0.0

        record = {
            'event':        event,
            'wall_time':    time.time(),
            'sim_time_sec': sim_sec,
        }
        if extra:
            record.update(extra)

        if self._log_fh:
            try:
                self._log_fh.write(json.dumps(record) + '\n')
                self._log_fh.flush()
            except Exception as exc:
                self.get_logger().warn(f'Could not write fault log: {exc}')

        self.get_logger().info(f'[fault_log] {json.dumps(record)}')

    def shutdown_log(self):
        try:
            sim_sec = self.get_clock().now().nanoseconds * 1e-9
        except Exception:
            sim_sec = 0.0

        record = {
            'event':             'shutdown',
            'wall_time':         time.time(),
            'sim_time_sec':      sim_sec,
            'tl_phase_at_exit':  self._tl_phase,
            'msg_count_tl':      self._msg_count_tl,
            'msg_count_imu':     self._msg_count_imu,
            'tl_fault_applied':  self._tl_fault_applied,
            'imu_bias_cycles':   self._imu_bias_on_cycles,
        }

        if self._log_fh:
            try:
                self._log_fh.write(json.dumps(record) + '\n')
                self._log_fh.flush()
                self._log_fh.close()
            except Exception:
                pass

        self.get_logger().info(
            f'Shutdown — TL phase={self._tl_phase}, '
            f'TL: {self._msg_count_tl} msgs, {self._tl_fault_applied} faulted; '
            f'IMU: {self._msg_count_imu} msgs, {self._imu_bias_on_cycles} bias cycles'
        )


def main():
    parser = argparse.ArgumentParser(description='Sensor fault injector for RISE experiments')
    parser.add_argument(
        '--tl-fault', type=str, default=None,
        choices=['tl_confidence', 'tl_confidence_ramp', 'tl_oscillate', 'tl_unknown', 'tl_blackout'],
        help=(
            'Traffic light fault mode.\n'
            '  tl_confidence:      multiply element confidence by confidence_scale param.\n'
            '  tl_confidence_ramp: confidence_scale decays linearly from 1.0 to\n'
            '                      min_confidence_scale over each fault-active window\n'
            '                      (elapsed since that cycle\'s own activation) — a\n'
            '                      richer, continuously-varying degradation instead of\n'
            '                      tl_confidence\'s step function.\n'
            '  tl_oscillate:       alternate GREEN / original on period_s cycle (intermittent).\n'
            '  tl_unknown:         set all elements to UNKNOWN (classification failure).\n'
            '  tl_blackout:        suppress message entirely during detection windows.'
        ),
    )
    parser.add_argument(
        '--tl-params', type=str, default='{}',
        help=(
            'JSON params for TL fault.\n'
            '  tl_confidence:      {"confidence_scale": 0.5}\n'
            '  tl_confidence_ramp: {"confidence_ramp_rate_per_s": 0.1, "min_confidence_scale": 0.0}'
        ),
    )
    parser.add_argument(
        '--imu-fault', type=str, default=None,
        choices=['imu_bias', 'imu_bias_ramp', 'imu_scale_factor', 'imu_stuck_at'],
        help=(
            'IMU fault mode.\n'
            '  imu_bias:         periodic on/off constant gyro bias. NOT part of the\n'
            '                    standard campaign suite as of 2026-07-24 — largely\n'
            '                    absorbed by ekf_localizer\'s yaw_bias estimation below\n'
            '                    the Mahalanobis gate; kept for ad-hoc use only.\n'
            '  imu_bias_ramp:    one-shot linear ramp to a cap — see module docstring\n'
            '                    for the crossing-point / lead-time rationale.\n'
            '  imu_scale_factor: periodic on/off multiplicative gain error — see\n'
            '                    module docstring, structurally unabsorbable by\n'
            '                    yaw_bias (additive-constant state).\n'
            '  imu_stuck_at:     periodic on/off frozen gyro output — same.'
        ),
    )
    parser.add_argument(
        '--imu-params', type=str, default='{}',
        help=(
            'JSON params.\n'
            '  imu_bias:         {"accel_bias_ms2":0.5, "gyro_bias_rads":0.1, "on_seconds":30, "off_seconds":30}\n'
            '  imu_bias_ramp:    {"gyro_bias_rate_rads_per_s":0.003, "max_gyro_bias_rads":0.4, "ramp_log_interval_s":5.0}\n'
            '  imu_scale_factor: {"gyro_scale_factor":1.8, "on_seconds":20, "off_seconds":30}\n'
            '  imu_stuck_at:     {"on_seconds":20, "off_seconds":30} — optionally\n'
            '                    "stuck_value_rads":<float> to freeze at a fixed value\n'
            '                    instead of whatever was read at activation.'
        ),
    )
    parser.add_argument(
        '--fault-delay', type=float, default=30.0,
        help=(
            'Seconds after the runway clears (see --fault-min-runway-m) before '
            'the fault arms (default: 30) — a floor for EKF/localization to '
            'settle, no longer counted from process start.'
        ),
    )
    parser.add_argument(
        '--fault-min-runway-m', type=float, default=150.0,
        help=(
            'FALLBACK ONLY as of 2026-07-24 (used when --zone-goals\' map TL '
            'points fail to load): straight-line distance in metres the vehicle '
            'must travel from its trial-start position (ground truth) before any '
            'fault may arm. When map points ARE loaded (the normal case), the '
            'runway instead requires clearing the first real TL zone (see '
            '--tl-zone-radius-m) — more precise, since it is tied to a real map '
            'feature rather than a distance guess. Every goal in this repo spawns '
            'at the same intersection, so without ONE of these two mechanisms, '
            '--fault-delay alone reliably armed faults while still at/near it. A '
            'ground-truth position jump > 3m mid-trial is treated as a new trial '
            '(teleport reset) and re-arms the runway gate either way.'
        ),
    )
    parser.add_argument(
        '--tl-zone-radius-m', type=float, default=_DEFAULT_TL_ZONE_RADIUS_M,
        help=(
            f'Metres from a real map TL position (see --zone-goals) within which '
            f'ego is considered "in a TL zone" — gates both TL fault application '
            f'and (when map points load) the runway. Default {_DEFAULT_TL_ZONE_RADIUS_M:.0f}, '
            f'grounded in Autoware\'s own traffic_light_map_based_detector '
            f'max_detection_range (200m) and published literature (~120-170m '
            f'typical reliable detection range) — see _DEFAULT_TL_ZONE_RADIUS_M\'s '
            f'comment for the full citation trail.'
        ),
    )
    parser.add_argument(
        '--zone-goals', type=str, default=','.join(_DEFAULT_ZONE_GOALS),
        help=(
            'Comma-separated goal IDs (from experiments/configs/captured_goals.json) '
            f'whose routes bound the map TL point search (default: '
            f'{",".join(_DEFAULT_ZONE_GOALS)} — the fixed production goal set; see '
            'run_fault_campaigns.sh). Fault campaigns only ever run on these routes, '
            'so scoping to their bounding boxes (not the whole 383-TL-point map) '
            'avoids false zone hits from unrelated intersections.'
        ),
    )
    parser.add_argument(
        '--fault-duration', type=float, default=45.0,
        help=(
            'Per-zone duration CAP in seconds (default: 45): the fault deactivates '
            'after this many seconds OR when the vehicle leaves the detection zone, '
            'whichever comes first, then re-arms for the next TL zone encountered '
            '(repeats for every intersection on the route).'
        ),
    )
    parser.add_argument(
        '--tl-recovery-gap', type=float, default=8.0,
        help=(
            'Seconds of guaranteed nominal passthrough after a TL fault cycle ends '
            'before re-arming for the next zone (default: 8), giving residuals a '
            'clean segment to recover into before the next cycle.'
        ),
    )
    parser.add_argument(
        '--max-tl-cycles', type=int, default=0,
        help='Cap on TL fault cycles per trial (default: 0 = unbounded, i.e. every '
             'TL zone on the route).',
    )
    parser.add_argument(
        '--log-file', type=str, default='/tmp/rise_fault_log.jsonl',
        help='Path to write JSON-lines fault event log.',
    )

    args        = parser.parse_args()
    tl_p        = json.loads(args.tl_params)
    imu_p       = json.loads(args.imu_params)
    zone_goals  = [g.strip() for g in args.zone_goals.split(',') if g.strip()]

    try:
        with open(args.log_file, 'w') as f:
            f.write(json.dumps({
                'event':          'startup',
                'tl_fault':       args.tl_fault,
                'tl_params':      tl_p,
                'imu_fault':      args.imu_fault,
                'imu_params':     imu_p,
                'fault_delay':     args.fault_delay,
                'fault_duration':  args.fault_duration,
                'tl_recovery_gap': args.tl_recovery_gap,
                'max_tl_cycles':   args.max_tl_cycles,
                'fault_min_runway_m': args.fault_min_runway_m,
                'tl_zone_radius_m':   args.tl_zone_radius_m,
                'zone_goals':         zone_goals,
                'wall_time':       time.time(),
            }) + '\n')
    except Exception:
        pass

    rclpy.init()
    node = FaultInjector(
        tl_fault           = args.tl_fault,
        tl_params          = tl_p,
        imu_fault          = args.imu_fault,
        imu_params         = imu_p,
        fault_delay        = args.fault_delay,
        fault_duration     = args.fault_duration,
        tl_recovery_gap    = args.tl_recovery_gap,
        max_tl_cycles      = args.max_tl_cycles,
        fault_min_runway_m = args.fault_min_runway_m,
        log_file           = args.log_file,
        tl_zone_radius_m   = args.tl_zone_radius_m,
        zone_goals         = zone_goals,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_log()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
