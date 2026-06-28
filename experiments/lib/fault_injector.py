"""
FaultInjector — ROS2 relay node for RISE sensor fault experiments.

Implements two independent relay-based faults:

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
1. TRAFFIC LIGHT CAMERA FAULT
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  IN:  /perception/traffic_light_recognition/traffic_signals_raw
  OUT: /perception/traffic_light_recognition/traffic_signals

  traffic_light.launch.xml is modified so the recognition pipeline publishes
  to traffic_signals_raw; this node is always in the chain.

  Detection-window gating (always active):
    The node tracks whether the vehicle is in a "TL detection zone" by monitoring
    how many of the last N raw messages contained non-empty groups.  Faults are
    ONLY applied during active detection windows (ratio > detection_threshold).
    This avoids injecting meaningless faults when there are no lights in view.

  Fault modes:
    tl_confidence  Multiply each element's confidence by confidence_scale [0,1].
                   Low confidence → arbiter treats signal as less trusted.
                   Realistic model for gradual occlusion (rain, glare, partial cover).
                   Params: {"confidence_scale": 0.3}

    tl_unknown     Replace all element colors with UNKNOWN (classification failure).
                   Confidence is also zeroed.  Planning falls back to conservative.
                   Realistic for severe lens contamination or strong backlight.
                   Params: {}  (no extra params)

    tl_blackout    Publish an empty TrafficLightGroupArray (complete signal loss).
                   Only suppresses during active detection windows — empty messages
                   outside detection zones pass through unchanged.
                   Params: {}

  Severity tiers (Deng et al. 2023; ISO 21448 SOTIF):
    S1  tl_confidence {"confidence_scale": 0.5}   — mild degradation (light fog)
    S2  tl_confidence {"confidence_scale": 0.1}   — severe degradation (heavy rain)
    S3  tl_unknown    {}                            — classification failure
    S4  tl_blackout   {}                            — complete camera loss

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
2. IMU BIAS FAULT (periodic)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  IN:  /sensing/imu/imu_data          (imu_corrector output)
  OUT: /sensing/imu/imu_data_faulted  (gyro_odometer reads this)

  gyro_odometer.launch.xml passes input_imu_topic=/sensing/imu/imu_data_faulted,
  so this node is always in the localization chain.

  Fault mode: imu_bias
    Adds constant accelerometer bias (x-axis) and gyroscope bias (z-axis) for
    `on_seconds`, then pauses for `off_seconds`, and repeats.  The periodic
    pattern creates clear on/off transitions in ST-GAT residuals.
    S4 uses off_seconds=0 (sustained, no recovery).

    The bias accumulates in the EKF: position covariance (x_var, y_var) rises
    during bias-on periods and recovers during bias-off — both effects appear
    in the model's `uncertainty` feature.

    Params: {
        "accel_bias_ms2":   0.5,   # m/s² constant added to linear_acceleration.x
        "gyro_bias_rads":   0.1,   # rad/s constant added to angular_velocity.z
        "on_seconds":       30,    # bias active duration per cycle
        "off_seconds":      30,    # bias inactive duration per cycle (0 = sustained)
    }

  Severity tiers (Woodman 2007, UCAM-CL-TR-696; Abdel-Hafez et al. 2021):
    S1  accel=0.1 m/s², gyro=0.05 rad/s, on=30s, off=30s  — low-grade MEMS drift
    S2  accel=0.5 m/s², gyro=0.10 rad/s, on=30s, off=30s  — moderate bias
    S3  accel=1.0 m/s², gyro=0.30 rad/s, on=20s, off=10s  — high bias, fast cycling
    S4  accel=2.0 m/s², gyro=0.50 rad/s, on=∞,  off=0     — sustained failure

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
LOGGING
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  All state transitions are written as JSON lines to --log-file.
  Events: startup | tl_fault_activated | tl_window_entered | tl_window_exited |
          tl_fault_applied | imu_fault_activated | imu_bias_on | imu_bias_off |
          shutdown

Usage:
  # Both relays in passthrough (nominal experiments):
  python3 fault_injector.py

  # TL severity 1 — confidence degradation:
  python3 fault_injector.py --tl-fault tl_confidence \\
      --tl-params '{"confidence_scale":0.5}' --fault-delay 30

  # TL severity 4 — blackout:
  python3 fault_injector.py --tl-fault tl_blackout --fault-delay 30

  # IMU severity 2 — periodic bias:
  python3 fault_injector.py --imu-fault imu_bias \\
      --imu-params '{"accel_bias_ms2":0.5,"gyro_bias_rads":0.1,"on_seconds":30,"off_seconds":30}' \\
      --fault-delay 30

  # Combined:
  python3 fault_injector.py \\
      --tl-fault tl_unknown \\
      --imu-fault imu_bias --imu-params '{"accel_bias_ms2":1.0,"gyro_bias_rads":0.3,"on_seconds":20,"off_seconds":10}' \\
      --fault-delay 30
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

_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# How many recent raw messages to track for detection-window gating.
# At ~10 Hz recognition rate, this is a 3-second lookback.
_DETECTION_WINDOW_SIZE = 30
# Fraction of recent messages that must have been non-empty to declare
# an active detection zone.
_DETECTION_THRESHOLD   = 0.3   # at least 30% non-empty in lookback window


class FaultInjector(Node):

    def __init__(
        self,
        tl_fault:    Optional[str],
        tl_params:   Dict[str, Any],
        imu_fault:   Optional[str],
        imu_params:  Dict[str, Any],
        fault_delay: float,
        log_file:    Optional[str],
    ):
        super().__init__('fault_injector')

        self._tl_fault_type:   Optional[str] = None   # set after delay
        self._tl_params                       = tl_params
        self._imu_fault_type:  Optional[str] = None   # set after delay
        self._imu_params                      = imu_params
        self._log_file                        = log_file
        self._lock                            = threading.Lock()

        # ── Detection-window tracking ────────────────────────────────────────
        # Stores 1/0 (non-empty / empty) for each recent raw message.
        self._detection_ring: deque = deque(maxlen=_DETECTION_WINDOW_SIZE)
        self._in_detection_zone: bool = False

        # ── Counters ─────────────────────────────────────────────────────────
        self._msg_count_tl         = 0
        self._msg_count_imu        = 0
        self._tl_fault_applied     = 0
        self._imu_bias_on_cycles   = 0

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
            _SENSOR_QOS,
        )
        self._imu_sub = self.create_subscription(
            Imu,
            '/sensing/imu/imu_data',
            self._on_imu,
            _SENSOR_QOS,
        )

        # ── IMU bias state ───────────────────────────────────────────────────
        self._imu_bias_active = False

        # ── Deferred fault activation ────────────────────────────────────────
        if fault_delay > 0 and (tl_fault or imu_fault):
            self.get_logger().info(
                f'Faults deferred {fault_delay:.0f}s: tl={tl_fault}, imu={imu_fault}'
            )
            t = threading.Thread(
                target=self._deferred_activate,
                args=(tl_fault, imu_fault, fault_delay),
                daemon=True,
            )
            t.start()
        else:
            self._activate_faults(tl_fault, imu_fault)

        self.get_logger().info(
            f'FaultInjector ready — tl={tl_fault or "passthrough"}, '
            f'imu={imu_fault or "passthrough"}, delay={fault_delay:.0f}s'
        )

    # ── Fault activation ─────────────────────────────────────────────────────

    def _deferred_activate(self, tl_fault, imu_fault, delay):
        time.sleep(delay)
        self._activate_faults(tl_fault, imu_fault)
        self.get_logger().info(f'Faults activated — tl={tl_fault}, imu={imu_fault}')

    def _activate_faults(self, tl_fault, imu_fault):
        with self._lock:
            if tl_fault:
                self._tl_fault_type = tl_fault
                self._log_event('tl_fault_activated', {
                    'fault_type': tl_fault, 'params': self._tl_params,
                })
            if imu_fault:
                self._imu_fault_type = imu_fault
                self._log_event('imu_fault_activated', {
                    'fault_type': imu_fault, 'params': self._imu_params,
                })
                # Start the periodic IMU bias thread
                t = threading.Thread(target=self._imu_bias_loop, daemon=True)
                t.start()

    # ── Traffic light relay ───────────────────────────────────────────────────

    def _on_tl(self, msg: TrafficLightGroupArray):
        self._msg_count_tl += 1
        is_nonempty = len(msg.traffic_light_groups) > 0

        with self._lock:
            # Update detection-window ring
            self._detection_ring.append(1 if is_nonempty else 0)
            if len(self._detection_ring) >= _DETECTION_WINDOW_SIZE // 3:
                ratio = sum(self._detection_ring) / len(self._detection_ring)
                newly_in_zone = ratio >= _DETECTION_THRESHOLD
                if newly_in_zone and not self._in_detection_zone:
                    self._in_detection_zone = True
                    self._log_event('tl_window_entered', {'ratio': round(ratio, 3)})
                elif not newly_in_zone and self._in_detection_zone:
                    self._in_detection_zone = True   # keep True until ratio drops to 0
                    # Hysteresis: only leave zone if ratio drops below half the threshold
                    if ratio < _DETECTION_THRESHOLD / 2:
                        self._in_detection_zone = False
                        self._log_event('tl_window_exited', {'ratio': round(ratio, 3)})

            fault      = self._tl_fault_type
            in_zone    = self._in_detection_zone

        # --- Apply fault -------------------------------------------------
        if fault is None or not in_zone:
            # Passthrough: no fault, or outside detection zone
            self._tl_pub.publish(msg)
            return

        if fault == 'tl_blackout':
            # Suppress the message entirely during detection windows
            empty = TrafficLightGroupArray()
            empty.stamp = msg.stamp
            self._tl_pub.publish(empty)
            self._tl_fault_applied += 1
            if self._tl_fault_applied % 50 == 1:
                self._log_event('tl_fault_applied', {
                    'fault': fault, 'count': self._tl_fault_applied,
                })

        elif fault == 'tl_confidence':
            scale = self._tl_params.get('confidence_scale', 0.5)
            out = copy.deepcopy(msg)
            for group in out.traffic_light_groups:
                for elem in group.elements:
                    elem.confidence = max(0.0, min(1.0, elem.confidence * scale))
            self._tl_pub.publish(out)
            self._tl_fault_applied += 1
            if self._tl_fault_applied % 50 == 1:
                self._log_event('tl_fault_applied', {
                    'fault': fault, 'scale': scale, 'count': self._tl_fault_applied,
                })

        elif fault == 'tl_unknown':
            out = copy.deepcopy(msg)
            for group in out.traffic_light_groups:
                for elem in group.elements:
                    elem.color      = TrafficLightElement.UNKNOWN
                    elem.confidence = 0.0
            self._tl_pub.publish(out)
            self._tl_fault_applied += 1
            if self._tl_fault_applied % 50 == 1:
                self._log_event('tl_fault_applied', {
                    'fault': fault, 'count': self._tl_fault_applied,
                })

        else:
            self.get_logger().warn(
                f'Unknown TL fault type: {fault}',
                throttle_duration_sec=5.0,
            )
            self._tl_pub.publish(msg)

    # ── IMU relay ────────────────────────────────────────────────────────────

    def _on_imu(self, msg: Imu):
        self._msg_count_imu += 1
        with self._lock:
            fault  = self._imu_fault_type
            active = self._imu_bias_active

        if fault == 'imu_bias' and active:
            accel_bias = self._imu_params.get('accel_bias_ms2', 0.0)
            gyro_bias  = self._imu_params.get('gyro_bias_rads', 0.0)
            msg.linear_acceleration.x += accel_bias
            msg.angular_velocity.z    += gyro_bias

        self._imu_pub.publish(msg)

    def _imu_bias_loop(self):
        """
        Cycles bias on/off according to on_seconds/off_seconds params.
        off_seconds=0 → sustained bias (no off period).
        """
        on_sec  = float(self._imu_params.get('on_seconds',  30))
        off_sec = float(self._imu_params.get('off_seconds', 30))

        while True:
            # ── Bias ON ──────────────────────────────────────────────────────
            with self._lock:
                self._imu_bias_active = True
                self._imu_bias_on_cycles += 1
            self._log_event('imu_bias_on', {
                'cycle':        self._imu_bias_on_cycles,
                'accel_bias':   self._imu_params.get('accel_bias_ms2'),
                'gyro_bias':    self._imu_params.get('gyro_bias_rads'),
                'on_seconds':   on_sec,
            })
            time.sleep(on_sec)

            # ── Bias OFF (skip if sustained) ──────────────────────────────────
            if off_sec <= 0:
                break   # sustained: leave bias on forever, exit thread

            with self._lock:
                self._imu_bias_active = False
            self._log_event('imu_bias_off', {
                'cycle':      self._imu_bias_on_cycles,
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
            f'Shutdown — TL: {self._msg_count_tl} msgs, {self._tl_fault_applied} faulted; '
            f'IMU: {self._msg_count_imu} msgs, {self._imu_bias_on_cycles} bias cycles'
        )


def main():
    parser = argparse.ArgumentParser(description='Sensor fault injector for RISE experiments')
    parser.add_argument(
        '--tl-fault', type=str, default=None,
        choices=['tl_confidence', 'tl_unknown', 'tl_blackout'],
        help=(
            'Traffic light fault mode.\n'
            '  tl_confidence: multiply element confidence by confidence_scale param.\n'
            '  tl_unknown:    set all elements to UNKNOWN (classification failure).\n'
            '  tl_blackout:   suppress message entirely during detection windows.'
        ),
    )
    parser.add_argument(
        '--tl-params', type=str, default='{}',
        help=(
            'JSON params for TL fault.\n'
            '  tl_confidence: {"confidence_scale": 0.5}  (0.0 = zero, 1.0 = no change)'
        ),
    )
    parser.add_argument(
        '--imu-fault', type=str, default=None,
        choices=['imu_bias'],
        help='IMU fault mode. imu_bias: periodic constant bias injection.',
    )
    parser.add_argument(
        '--imu-params', type=str, default='{}',
        help=(
            'JSON params for imu_bias.\n'
            '  {"accel_bias_ms2":0.5, "gyro_bias_rads":0.1, "on_seconds":30, "off_seconds":30}\n'
            '  Set off_seconds=0 for sustained (no off period).'
        ),
    )
    parser.add_argument(
        '--fault-delay', type=float, default=30.0,
        help=(
            'Seconds to wait before activating faults (default: 30).\n'
            'Allows localization to converge before IMU bias begins.'
        ),
    )
    parser.add_argument(
        '--log-file', type=str, default='/tmp/rise_fault_log.jsonl',
        help='Path to write JSON-lines fault event log.',
    )

    args = parser.parse_args()

    tl_params  = json.loads(args.tl_params)
    imu_params = json.loads(args.imu_params)

    # Write startup record (clears previous log for this run)
    try:
        with open(args.log_file, 'w') as f:
            f.write(json.dumps({
                'event':       'startup',
                'tl_fault':    args.tl_fault,
                'tl_params':   tl_params,
                'imu_fault':   args.imu_fault,
                'imu_params':  imu_params,
                'fault_delay': args.fault_delay,
                'wall_time':   time.time(),
            }) + '\n')
    except Exception:
        pass

    rclpy.init()
    node = FaultInjector(
        tl_fault    = args.tl_fault,
        tl_params   = tl_params,
        imu_fault   = args.imu_fault,
        imu_params  = imu_params,
        fault_delay = args.fault_delay,
        log_file    = args.log_file,
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
