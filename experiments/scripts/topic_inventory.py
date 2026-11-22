#!/usr/bin/env python3
"""Inventory of every live ROS 2 topic: type, publishers, subscribers, QoS,
measured rate and bandwidth over a fixed window.

Subscribes to all topics at once with raw (serialized) callbacks, so the
measurement covers every stream, including the large sensor ones, without
deserialization cost. Best-effort + volatile subscribers match every
publisher; latched (transient-local) topics are flagged from the publisher
QoS, since a volatile subscriber only sees their later updates.

Usage:
  python3 topic_inventory.py --duration 60 --out logs/inventory/driving.json
"""
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rosidl_runtime_py.utilities import get_message


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=60.0)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    rclpy.init()
    node = Node('topic_inventory_probe')
    time.sleep(3.0)                       # discovery
    topics = node.get_topic_names_and_types()
    stats, subs = {}, []
    qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50,
                     reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    for name, types in topics:
        pubs = node.get_publishers_info_by_topic(name)
        s = {'type': types[0], 'n': 0, 'bytes': 0, 'max_bytes': 0, 't_first': None, 't_last': None,
             'publishers': sorted({f'{p.node_namespace.rstrip("/")}/{p.node_name}' for p in pubs}),
             'subscribers': sorted({f'{p.node_namespace.rstrip("/")}/{p.node_name}'
                                    for p in node.get_subscriptions_info_by_topic(name)}),
             'reliability': sorted({p.qos_profile.reliability.name for p in pubs}),
             'durability': sorted({p.qos_profile.durability.name for p in pubs})}
        stats[name] = s
        try:
            mt = get_message(types[0])
        except Exception as e:            # type not in this workspace
            s['error'] = f'type unavailable: {e}'
            continue

        def cb(msg, s=s):
            t = time.monotonic()
            s['n'] += 1
            s['bytes'] += len(msg)
            s['max_bytes'] = max(s['max_bytes'], len(msg))
            s['t_first'] = s['t_first'] or t
            s['t_last'] = t
        subs.append(node.create_subscription(mt, name, cb, qos, raw=True))

    t0 = time.monotonic()
    while time.monotonic() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.05)
    T = time.monotonic() - t0
    for s in stats.values():
        s['hz'] = s['n'] / T
        s['bytes_per_s'] = s['bytes'] / T
        s['mean_bytes'] = s['bytes'] / s['n'] if s['n'] else 0
        s.pop('t_first'); s.pop('t_last')
    json.dump({'duration_s': T, 'n_topics': len(stats), 'topics': stats}, open(args.out, 'w'), indent=1)
    node.get_logger().info(f'{len(stats)} topics, {sum(s["n"] > 0 for s in stats.values())} active -> {args.out}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
