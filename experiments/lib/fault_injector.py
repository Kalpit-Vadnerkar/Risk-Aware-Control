"""
FaultInjector — ROS2 relay node for RISE sensor fault experiments.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
1. TRAFFIC LIGHT CAMERA FAULT
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  IN:  /perception/traffic_light_recognition/traffic_signals_raw
  OUT: /perception/traffic_light_recognition/traffic_signals

  Timing model:
    1. Vehicle drives nominally for --fault-delay seconds.
    2. On the FIRST entry into a TL detection zone after that delay,
       the fault activates (tl_fault_start logged).
    3. After --fault-duration seconds the fault deactivates (tl_fault_end
       logged) and the node passes through raw signals unchanged for the
       remainder of the trial.

    This gives three clean segments per trial: nominal → fault → recovery.
    The fault is therefore always anchored to a real TL intersection, and
    the vehicle can complete the route after the fault window closes.

  Detection-window gating (always active):
    Faults are only applied when the vehicle is in an active TL detection
    zone (≥30% of the last 30 raw messages contained non-empty groups).
    This prevents injecting meaningless faults in open road segments.

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
  This is intentional — AEB safety is preserved while we fault localization.

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

  Severity tiers (gyro_bias_rads is the active parameter):
    S1  gyro=0.05 rad/s, on=30s, off=30s  — low-grade MEMS drift (~2.9°/s)
    S2  gyro=0.15 rad/s, on=30s, off=20s  — moderate drift (~8.6°/s), more dwell
    S3  gyro=0.30 rad/s, on=20s, off=10s  — significant drift (~17.2°/s), fast cycle
    S4  gyro=0.60 rad/s, on=∞,  off=0    — severe sustained drift (~34.4°/s)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
LOGGING
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  All state transitions are written as JSON lines to --log-file.
  TL events: startup | tl_fault_start | tl_fault_end |
             tl_window_entered | tl_window_exited | tl_fault_applied |
             shutdown
  IMU events: imu_fault_activated | imu_bias_on | imu_bias_off

Usage:
  # Passthrough (nominal):
  python3 fault_injector.py

  # S1 — confidence degradation, 45s window starting at first TL zone:
  python3 fault_injector.py --tl-fault tl_confidence \\
      --tl-params '{"confidence_scale":0.5}' --fault-delay 30 --fault-duration 45

  # S2 — oscillating GREEN/original (intermittent), 45s window:
  python3 fault_injector.py --tl-fault tl_oscillate \
      --tl-params '{"period_s":5.0}' --fault-delay 30 --fault-duration 45

  # S3 — UNKNOWN classification, 45s window:
  python3 fault_injector.py --tl-fault tl_unknown --fault-delay 30 --fault-duration 45

  # S4 — blackout, 45s window:
  python3 fault_injector.py --tl-fault tl_blackout --fault-delay 30 --fault-duration 45
"""

import argparse
import copy
import json
import threading
import time
from collections import deque
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from autoware_perception_msgs.msg import TrafficLightGroupArray, TrafficLightElement
from sensor_msgs.msg import Imu


_PERCEPTION_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
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

# How many recent raw messages to track for detection-window gating (~3s at 10Hz).
_DETECTION_WINDOW_SIZE = 30
_DETECTION_THRESHOLD   = 0.3   # ≥30% non-empty in lookback = active zone


class FaultInjector(Node):

    def __init__(
        self,
        tl_fault:       Optional[str],
        tl_params:      Dict[str, Any],
        imu_fault:      Optional[str],
        imu_params:     Dict[str, Any],
        fault_delay:    float,
        fault_duration: float,
        log_file:       Optional[str],
    ):
        super().__init__(
            'fault_injector',
            parameter_overrides=[
                rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True),
            ],
        )

        self._tl_fault_config: Optional[str] = tl_fault   # what to apply when active
        self._tl_params                       = tl_params
        self._imu_fault_type: Optional[str]   = None
        self._imu_params                      = imu_params
        self._fault_duration                  = fault_duration
        self._log_file                        = log_file
        self._lock                            = threading.Lock()

        # TL fault state machine
        # Phases: 'waiting_delay' → 'waiting_zone' → 'fault_active' → 'done'
        self._tl_phase: str             = 'waiting_delay' if tl_fault else 'done'
        self._tl_fault_active: bool     = False   # True only during fault window
        self._tl_fault_end_time: float  = 0.0

        # Detection-window tracking
        self._detection_ring: deque   = deque(maxlen=_DETECTION_WINDOW_SIZE)
        self._in_detection_zone: bool = False

        # Counters
        self._msg_count_tl        = 0
        self._msg_count_imu       = 0
        self._tl_fault_applied    = 0
        self._imu_bias_on_cycles  = 0

        # ── Traffic light relay ───────────────────────────────────────────────
        self._tl_pub = self.create_publisher(
            TrafficLightGroupArray,
            '/perception/traffic_light_recognition/traffic_signals',
            _PERCEPTION_QOS,
        )
        self._tl_sub = self.create_subscription(
            TrafficLightGroupArray,
            '/perception/traffic_light_recognition/traffic_signals_raw',
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

        # ── Deferred fault setup ─────────────────────────────────────────────
        if tl_fault and fault_delay > 0:
            self.get_logger().info(
                f'TL fault "{tl_fault}" armed: delay={fault_delay:.0f}s, '
                f'duration={fault_duration:.0f}s'
            )
            t = threading.Thread(
                target=self._delay_then_arm_tl,
                args=(fault_delay,),
                daemon=True,
            )
            t.start()
        elif tl_fault:
            with self._lock:
                self._tl_phase = 'waiting_zone'

        if imu_fault:
            if fault_delay > 0:
                t = threading.Thread(
                    target=self._delay_then_activate_imu,
                    args=(imu_fault, fault_delay),
                    daemon=True,
                )
                t.start()
            else:
                self._activate_imu_fault(imu_fault)

        self.get_logger().info(
            f'FaultInjector ready — tl={tl_fault or "passthrough"}, '
            f'imu={imu_fault or "passthrough"}, '
            f'delay={fault_delay:.0f}s, duration={fault_duration:.0f}s'
        )

    # ── Phase transitions ─────────────────────────────────────────────────────

    def _delay_then_arm_tl(self, delay: float):
        """After the initial delay, move from waiting_delay → waiting_zone."""
        time.sleep(delay)
        with self._lock:
            if self._tl_phase == 'waiting_delay':
                self._tl_phase = 'waiting_zone'
        self.get_logger().info(
            f'TL fault armed — waiting for first detection zone entry'
        )

    def _delay_then_activate_imu(self, imu_fault: str, delay: float):
        time.sleep(delay)
        self._activate_imu_fault(imu_fault)

    def _activate_imu_fault(self, imu_fault: str):
        with self._lock:
            self._imu_fault_type = imu_fault
        self._log_event('imu_fault_activated', {
            'fault_type': imu_fault, 'params': self._imu_params,
        })
        t = threading.Thread(target=self._imu_bias_loop, daemon=True)
        t.start()

    def _start_tl_fault_window(self):
        """Called when detection zone entered while in waiting_zone phase."""
        end_t = time.time() + self._fault_duration
        with self._lock:
            self._tl_phase         = 'fault_active'
            self._tl_fault_active  = True
            self._tl_fault_end_time = end_t
        self._log_event('tl_fault_start', {
            'fault_type':    self._tl_fault_config,
            'params':        self._tl_params,
            'duration_sec':  self._fault_duration,
        })
        self.get_logger().info(
            f'TL fault ACTIVE: {self._tl_fault_config}, '
            f'window={self._fault_duration:.0f}s'
        )

    def _end_tl_fault_window(self):
        """Called when fault duration expires."""
        with self._lock:
            self._tl_phase        = 'done'
            self._tl_fault_active = False
        self._log_event('tl_fault_end', {
            'fault_type':   self._tl_fault_config,
            'applied_count': self._tl_fault_applied,
        })
        self.get_logger().info(
            f'TL fault ENDED — passthrough restored '
            f'({self._tl_fault_applied} messages faulted)'
        )

    # ── Traffic light relay ───────────────────────────────────────────────────

    def _on_tl(self, msg: TrafficLightGroupArray):
        self._msg_count_tl += 1
        is_nonempty = len(msg.traffic_light_groups) > 0

        with self._lock:
            # Update detection ring
            self._detection_ring.append(1 if is_nonempty else 0)
            if len(self._detection_ring) >= _DETECTION_WINDOW_SIZE // 3:
                ratio       = sum(self._detection_ring) / len(self._detection_ring)
                was_in_zone = self._in_detection_zone
                if ratio >= _DETECTION_THRESHOLD:
                    self._in_detection_zone = True
                elif ratio < _DETECTION_THRESHOLD / 2:
                    self._in_detection_zone = False

                if self._in_detection_zone and not was_in_zone:
                    self._log_event('tl_window_entered', {'ratio': round(ratio, 3)})
                elif not self._in_detection_zone and was_in_zone:
                    self._log_event('tl_window_exited', {'ratio': round(ratio, 3)})

            phase      = self._tl_phase
            in_zone    = self._in_detection_zone
            fault_end  = self._tl_fault_end_time

        # ── Phase transitions (outside lock to avoid blocking) ────────────────
        if phase == 'waiting_zone' and in_zone:
            self._start_tl_fault_window()
            phase = 'fault_active'

        if phase == 'fault_active' and time.time() >= fault_end:
            self._end_tl_fault_window()
            phase = 'done'

        # ── Apply or passthrough ──────────────────────────────────────────────
        with self._lock:
            active = self._tl_fault_active and self._in_detection_zone

        if not active:
            self._tl_pub.publish(msg)
            return

        fault = self._tl_fault_config

        if fault == 'tl_blackout':
            empty       = TrafficLightGroupArray()
            empty.stamp = msg.stamp
            self._tl_pub.publish(empty)

        elif fault == 'tl_confidence':
            scale = self._tl_params.get('confidence_scale', 0.5)
            out   = copy.deepcopy(msg)
            for group in out.traffic_light_groups:
                for elem in group.elements:
                    elem.confidence = max(0.0, min(1.0, elem.confidence * scale))
            self._tl_pub.publish(out)

        elif fault == 'tl_oscillate':
            # Alternate between forced-GREEN and the original signal on a fixed
            # period.  This produces velocity oscillations at intersections that
            # CUSUM can accumulate — the vehicle repeatedly starts/stops as the
            # signal toggles between GREEN (go) and the true RED (stop).
            period = self._tl_params.get('period_s', 5.0)
            if time.time() % period < period / 2:
                out = copy.deepcopy(msg)
                for group in out.traffic_light_groups:
                    for elem in group.elements:
                        elem.color      = TrafficLightElement.GREEN
                        elem.confidence = 1.0
                self._tl_pub.publish(out)
            else:
                self._tl_pub.publish(msg)

        elif fault == 'tl_unknown':
            out = copy.deepcopy(msg)
            for group in out.traffic_light_groups:
                for elem in group.elements:
                    elem.color      = TrafficLightElement.UNKNOWN
                    elem.confidence = 0.0
            self._tl_pub.publish(out)

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
            fault  = self._imu_fault_type
            active = self._imu_bias_active

        if fault == 'imu_bias' and active:
            msg.linear_acceleration.x += self._imu_params.get('accel_bias_ms2', 0.0)
            msg.angular_velocity.z    += self._imu_params.get('gyro_bias_rads', 0.0)

        self._imu_pub.publish(msg)

    def _imu_bias_loop(self):
        on_sec  = float(self._imu_params.get('on_seconds',  30))
        off_sec = float(self._imu_params.get('off_seconds', 30))

        while True:
            with self._lock:
                self._imu_bias_active = True
                self._imu_bias_on_cycles += 1
            self._log_event('imu_bias_on', {
                'cycle':      self._imu_bias_on_cycles,
                'accel_bias': self._imu_params.get('accel_bias_ms2'),
                'gyro_bias':  self._imu_params.get('gyro_bias_rads'),
                'on_seconds': on_sec,
            })
            time.sleep(on_sec)

            if off_sec <= 0:
                break

            with self._lock:
                self._imu_bias_active = False
            self._log_event('imu_bias_off', {
                'cycle':       self._imu_bias_on_cycles,
                'off_seconds': off_sec,
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

        if self._log_file:
            try:
                with open(self._log_file, 'a') as f:
                    f.write(json.dumps(record) + '\n')
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

        if self._log_file:
            try:
                with open(self._log_file, 'a') as f:
                    f.write(json.dumps(record) + '\n')
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
        choices=['tl_confidence', 'tl_oscillate', 'tl_unknown', 'tl_blackout'],
        help=(
            'Traffic light fault mode.\n'
            '  tl_confidence: multiply element confidence by confidence_scale param.\n'
            '  tl_oscillate:  alternate GREEN / original on period_s cycle (intermittent).\n'
            '  tl_unknown:    set all elements to UNKNOWN (classification failure).\n'
            '  tl_blackout:   suppress message entirely during detection windows.'
        ),
    )
    parser.add_argument(
        '--tl-params', type=str, default='{}',
        help='JSON params for TL fault. tl_confidence: {"confidence_scale": 0.5}',
    )
    parser.add_argument(
        '--imu-fault', type=str, default=None,
        choices=['imu_bias'],
        help='IMU fault mode.',
    )
    parser.add_argument(
        '--imu-params', type=str, default='{}',
        help=(
            'JSON params for imu_bias.\n'
            '  {"accel_bias_ms2":0.5, "gyro_bias_rads":0.1, "on_seconds":30, "off_seconds":30}'
        ),
    )
    parser.add_argument(
        '--fault-delay', type=float, default=30.0,
        help='Seconds after start before the fault arms (default: 30).',
    )
    parser.add_argument(
        '--fault-duration', type=float, default=45.0,
        help=(
            'Seconds the TL fault is active once the first detection zone is entered '
            'after the delay (default: 45). After this window the node passes through '
            'raw signals unchanged, allowing the vehicle to recover and complete the route.'
        ),
    )
    parser.add_argument(
        '--log-file', type=str, default='/tmp/rise_fault_log.jsonl',
        help='Path to write JSON-lines fault event log.',
    )

    args   = parser.parse_args()
    tl_p   = json.loads(args.tl_params)
    imu_p  = json.loads(args.imu_params)

    try:
        with open(args.log_file, 'w') as f:
            f.write(json.dumps({
                'event':          'startup',
                'tl_fault':       args.tl_fault,
                'tl_params':      tl_p,
                'imu_fault':      args.imu_fault,
                'imu_params':     imu_p,
                'fault_delay':    args.fault_delay,
                'fault_duration': args.fault_duration,
                'wall_time':      time.time(),
            }) + '\n')
    except Exception:
        pass

    rclpy.init()
    node = FaultInjector(
        tl_fault       = args.tl_fault,
        tl_params      = tl_p,
        imu_fault      = args.imu_fault,
        imu_params     = imu_p,
        fault_delay    = args.fault_delay,
        fault_duration = args.fault_duration,
        log_file       = args.log_file,
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
