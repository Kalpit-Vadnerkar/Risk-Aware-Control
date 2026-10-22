"""Per-step consistency gaps for the 2b pose-error filter, from raw bag topics.

gap = the estimate's own pose increment between consecutive Layer 1 windows
      minus the vehicle report integrated over the same interval
      (yaw: heading change - int r dt; position: displacement in the
      estimate's mid-interval heading frame - int (v, v_l) dt)

Two corrections against the pipeline's window gaps
(sequence_builder._add_pose_consistency), both measured against AWSIM truth
in pose_error_derivative_check.py (docs/findings.md section 4):
  - lever arm: AWSIM's velocity report is the rigidbody (centre-of-mass)
    velocity, the pose is base_link, so v_l,report = v_l,base_link + L_CM * r
  - no window lag: report and pose cover the same interval (the pipeline
    compares a 1 s backward pose difference with the instantaneous report).
Summed over windows the pose increments telescope, so the forward-fill jitter
that needs 1 s windows for a rate cancels in the integral.
"""
import math
import os

import numpy as np

GT_TOPIC = '/awsim/ground_truth/localization/kinematic_state'
EST_TOPIC = '/localization/kinematic_state'
VEL_TOPIC = '/vehicle/status/velocity_status'
# centre of mass ahead of base_link (m); fitted on every IMU and nominal trial
# checked (residual 0.10 -> 0.003 m/s RMS)
L_CM = 1.18
MAX_STEP_S = 0.3         # increments across a longer gap are dropped (as the filter clips dt)


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def read_bag(bag_dir, clock='receive'):
    """Est pose, AWSIM truth (t, x, y, yaw, vx, vy, wz) and the vehicle report
    (t, v, v_l, r; heading_rate sign-flipped to ROS). clock='receive' = the bag
    clock of the Layer 1 window times; 'header' = sim time (fault_log
    sim_time_sec)."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=''), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    topics = [GT_TOPIC, EST_TOPIC, VEL_TOPIC]
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    mt = {k: get_message(types[k]) for k in topics}
    out = {k: [] for k in topics}
    while r.has_next():
        k, data, ts = r.read_next()
        m = deserialize_message(data, mt[k])
        if clock == 'header':
            ts = m.header.stamp.sec * 1e9 + m.header.stamp.nanosec
        if k == VEL_TOPIC:
            # AWSIM heading_rate has the opposite sign to ROS (findings.md 2)
            out[k].append((ts * 1e-9, m.longitudinal_velocity, m.lateral_velocity, -m.heading_rate))
        else:
            p, q, tw = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
            out[k].append((ts * 1e-9, p.x, p.y, yaw, tw.linear.x, tw.linear.y, tw.angular.z))
    return {k: np.array(v) for k, v in out.items()}


def step_gaps(B, ta, lever=L_CM):
    """Per-step gaps (yaw rad/s, along m/s, lateral m/s) at absolute window
    times ta (bag receive clock), and the true pose error (psi, x, y) =
    estimate - AWSIM truth in the estimate's heading frame."""
    est, vel, gt = B[EST_TOPIC], B[VEL_TOPIC], B[GT_TOPIC]
    exy = np.stack([np.interp(ta, est[:, 0], est[:, c]) for c in (1, 2)], -1)
    eyaw = np.interp(ta, est[:, 0], np.unwrap(est[:, 3]))
    rep = vel[:, 1:4].copy()
    rep[:, 1] -= lever * rep[:, 2]
    cum = np.concatenate([np.zeros((1, 3)), np.cumsum(0.5 * (rep[1:] + rep[:-1]) * np.diff(vel[:, 0])[:, None], 0)])
    C = np.stack([np.interp(ta, vel[:, 0], cum[:, c]) for c in range(3)], -1)
    dt = np.diff(ta)
    hm = 0.5 * (eyaw[1:] + eyaw[:-1])
    d, dC = np.diff(exy, axis=0), np.diff(C, axis=0)
    inc = np.stack([np.diff(eyaw) - dC[:, 2],
                    d[:, 0] * np.cos(hm) + d[:, 1] * np.sin(hm) - dC[:, 0],
                    -d[:, 0] * np.sin(hm) + d[:, 1] * np.cos(hm) - dC[:, 1]], -1)
    g = np.zeros((len(ta), 3))
    ok = (dt > 1e-3) & (dt <= MAX_STEP_S)
    g[1:][ok] = inc[ok] / dt[ok, None]
    gyaw = np.interp(ta, gt[:, 0], np.unwrap(gt[:, 3]))
    dd = exy - np.stack([np.interp(ta, gt[:, 0], gt[:, c]) for c in (1, 2)], -1)
    true = np.stack([wrap(eyaw - gyaw),
                     dd[:, 0] * np.cos(eyaw) + dd[:, 1] * np.sin(eyaw),
                     -dd[:, 0] * np.sin(eyaw) + dd[:, 1] * np.cos(eyaw)], -1)
    return g, true


def nominal_window_times(run_dir):
    """Absolute window times of a trial, as the sequence extraction builds
    them (one per Layer 1 window)."""
    from inspect_fault_predictions import compute_window_times
    from st_gat.pipeline.bag_reader import read_bag as read_frames
    from st_gat.pipeline.sequence_builder import SequenceBuilder
    frames = read_frames(os.path.join(run_dir, 'rosbag'), goal_id=os.path.basename(os.path.dirname(run_dir)), verbose=False)
    return np.array(compute_window_times(frames, SequenceBuilder._MAX_FRAME_GAP_SEC))
