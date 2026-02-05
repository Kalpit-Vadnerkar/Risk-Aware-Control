#!/usr/bin/env python3
"""
Analyze MRM triggers and diagnostics from rosbag.

This script reads a rosbag and correlates diagnostic failures with MRM state changes.
Requires sourcing Autoware workspace for message types.

Usage:
  source /opt/ros/humble/setup.bash
  source autoware/install/setup.bash
  python3 analyze_mrm_diagnostics.py <rosbag_path>

Example:
  python3 analyze_mrm_diagnostics.py ../data/goal_001/rosbag/
"""

import sys
import os
import json
from collections import defaultdict
from datetime import datetime

# ROS2 imports
import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

# Message types (requires sourcing Autoware)
from autoware_adapi_v1_msgs.msg import MrmState
from diagnostic_msgs.msg import DiagnosticArray


def get_message_type(topic_type_str):
    """Get message class from type string."""
    type_map = {
        'autoware_adapi_v1_msgs/msg/MrmState': MrmState,
        'diagnostic_msgs/msg/DiagnosticArray': DiagnosticArray,
    }
    return type_map.get(topic_type_str)


def analyze_rosbag(bag_path):
    """Analyze MRM triggers and diagnostics from rosbag."""

    # Storage for data
    mrm_messages = []  # (timestamp_ns, state, behavior)
    diagnostics_messages = []  # (timestamp_ns, [(name, level, message), ...])

    # MRM state/behavior names
    MRM_STATE_NAMES = {
        MrmState.UNKNOWN: 'UNKNOWN',
        MrmState.NORMAL: 'NORMAL',
        MrmState.MRM_OPERATING: 'MRM_OPERATING',
        MrmState.MRM_SUCCEEDED: 'MRM_SUCCEEDED',
        MrmState.MRM_FAILED: 'MRM_FAILED'
    }
    MRM_BEHAVIOR_NAMES = {
        MrmState.UNKNOWN: 'UNKNOWN',
        MrmState.NONE: 'NONE',
        MrmState.EMERGENCY_STOP: 'EMERGENCY_STOP',
        MrmState.COMFORTABLE_STOP: 'COMFORTABLE_STOP',
        MrmState.PULL_OVER: 'PULL_OVER'
    }
    DIAG_LEVEL_NAMES = {0: 'OK', 1: 'WARN', 2: 'ERROR', 3: 'STALE'}

    print(f"Reading rosbag: {bag_path}")

    # Setup reader
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Get topic info
    topic_types = {info.name: info.type for info in reader.get_all_topics_and_types()}
    print(f"Topics in bag: {list(topic_types.keys())}")

    # Read messages
    msg_count = 0
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_count += 1

        if topic == '/system/fail_safe/mrm_state':
            msg_type = get_message_type(topic_types[topic])
            if msg_type:
                msg = deserialize_message(data, msg_type)
                mrm_messages.append({
                    'timestamp_ns': timestamp,
                    'state': msg.state,
                    'behavior': msg.behavior,
                    'state_name': MRM_STATE_NAMES.get(msg.state, f'?{msg.state}'),
                    'behavior_name': MRM_BEHAVIOR_NAMES.get(msg.behavior, f'?{msg.behavior}')
                })

        elif topic == '/diagnostics':
            msg_type = get_message_type(topic_types[topic])
            if msg_type:
                msg = deserialize_message(data, msg_type)
                diag_list = []
                for status in msg.status:
                    if status.level > 0:  # Not OK
                        diag_list.append({
                            'name': status.name,
                            'level': status.level,
                            'level_name': DIAG_LEVEL_NAMES.get(status.level, f'?{status.level}'),
                            'message': status.message[:200] if status.message else ''
                        })
                if diag_list:
                    diagnostics_messages.append({
                        'timestamp_ns': timestamp,
                        'diagnostics': diag_list
                    })

    print(f"Read {msg_count} total messages")
    print(f"Found {len(mrm_messages)} MRM state messages")
    print(f"Found {len(diagnostics_messages)} diagnostic messages with issues")

    # Find start time for relative timestamps
    if mrm_messages:
        start_time_ns = mrm_messages[0]['timestamp_ns']
    elif diagnostics_messages:
        start_time_ns = diagnostics_messages[0]['timestamp_ns']
    else:
        print("No relevant messages found")
        return None

    # Identify MRM triggers (transitions from NORMAL to MRM_OPERATING)
    mrm_triggers = []
    prev_state = MrmState.NORMAL

    for msg in mrm_messages:
        if prev_state == MrmState.NORMAL and msg['state'] == MrmState.MRM_OPERATING:
            msg['time_sec'] = (msg['timestamp_ns'] - start_time_ns) / 1e9
            mrm_triggers.append(msg)
        prev_state = msg['state']

    print(f"\nFound {len(mrm_triggers)} MRM triggers (NORMAL → MRM_OPERATING)")

    # Analyze diagnostics around each MRM trigger
    print("\n" + "="*80)
    print("MRM TRIGGER ANALYSIS")
    print("="*80)

    diag_at_mrm = defaultdict(int)  # Count diagnostics at MRM triggers
    diag_level_at_mrm = defaultdict(lambda: defaultdict(int))  # Diagnostic -> level counts

    for i, trigger in enumerate(mrm_triggers[:30]):  # Analyze first 30
        trigger_time_ns = trigger['timestamp_ns']
        trigger_time_sec = trigger['time_sec']

        print(f"\n--- MRM Trigger #{i+1} at t={trigger_time_sec:.2f}s ({trigger['behavior_name']}) ---")

        # Find diagnostics within 500ms window before trigger
        window_start_ns = trigger_time_ns - 500_000_000  # 500ms before
        window_end_ns = trigger_time_ns + 100_000_000    # 100ms after

        nearby_diags = []
        for diag_msg in diagnostics_messages:
            if window_start_ns <= diag_msg['timestamp_ns'] <= window_end_ns:
                for d in diag_msg['diagnostics']:
                    nearby_diags.append(d)
                    diag_at_mrm[d['name']] += 1
                    diag_level_at_mrm[d['name']][d['level_name']] += 1

        # Show unique diagnostic names
        unique_diags = {}
        for d in nearby_diags:
            if d['name'] not in unique_diags:
                unique_diags[d['name']] = d

        if unique_diags:
            print(f"  Diagnostics with issues ({len(unique_diags)} unique):")
            # Sort by level (ERROR first) then name
            sorted_diags = sorted(unique_diags.values(),
                                  key=lambda x: (-x['level'], x['name']))
            for d in sorted_diags[:15]:  # Show top 15
                print(f"    [{d['level_name']:5s}] {d['name']}")
                if d['message'] and d['level'] >= 2:  # Show message for ERROR+
                    print(f"            └─ {d['message'][:80]}")
        else:
            print("  No diagnostic issues in window")

    # Summary of most common diagnostics at MRM triggers
    print("\n" + "="*80)
    print("MOST COMMON DIAGNOSTICS AT MRM TRIGGERS")
    print("="*80)

    sorted_diags = sorted(diag_at_mrm.items(), key=lambda x: -x[1])
    for name, count in sorted_diags[:40]:
        pct = 100 * count / len(mrm_triggers) if mrm_triggers else 0
        levels = diag_level_at_mrm[name]
        level_str = ", ".join(f"{l}:{c}" for l, c in sorted(levels.items()))
        print(f"  {count:3d} ({pct:5.1f}%) | [{level_str:20s}] | {name}")

    # Categorize by subsystem
    print("\n" + "="*80)
    print("DIAGNOSTICS BY SUBSYSTEM")
    print("="*80)

    subsystems = defaultdict(lambda: {'total': 0, 'diags': defaultdict(int)})
    for name, count in sorted_diags:
        # Parse subsystem from diagnostic name
        parts = name.split(':')
        if len(parts) >= 2:
            subsystem = parts[0]
        elif '/' in name:
            subsystem = name.split('/')[0]
        else:
            subsystem = 'other'
        subsystems[subsystem]['total'] += count
        subsystems[subsystem]['diags'][name] = count

    # Sort subsystems by total count
    for subsystem in sorted(subsystems.keys(), key=lambda x: -subsystems[x]['total']):
        data = subsystems[subsystem]
        print(f"\n{subsystem}: {data['total']} total occurrences")
        for name, count in sorted(data['diags'].items(), key=lambda x: -x[1])[:5]:
            print(f"    {count:3d} | {name}")

    # MRM behavior distribution
    print("\n" + "="*80)
    print("MRM BEHAVIOR DISTRIBUTION")
    print("="*80)

    behavior_counts = defaultdict(int)
    for trigger in mrm_triggers:
        behavior_counts[trigger['behavior_name']] += 1

    for behavior, count in sorted(behavior_counts.items(), key=lambda x: -x[1]):
        pct = 100 * count / len(mrm_triggers) if mrm_triggers else 0
        print(f"  {behavior}: {count} ({pct:.1f}%)")

    # Timeline analysis
    print("\n" + "="*80)
    print("MRM TRIGGER TIMELINE")
    print("="*80)

    if mrm_triggers:
        # Bucket into 10-second intervals
        max_time = max(t['time_sec'] for t in mrm_triggers)
        bucket_size = 10  # seconds
        num_buckets = int(max_time / bucket_size) + 1

        buckets = defaultdict(lambda: {'count': 0, 'emergency': 0, 'comfortable': 0})
        for trigger in mrm_triggers:
            bucket = int(trigger['time_sec'] / bucket_size)
            buckets[bucket]['count'] += 1
            if trigger['behavior_name'] == 'EMERGENCY_STOP':
                buckets[bucket]['emergency'] += 1
            elif trigger['behavior_name'] == 'COMFORTABLE_STOP':
                buckets[bucket]['comfortable'] += 1

        print(f"  Time Range | Triggers | Emergency | Comfortable")
        print(f"  {'-'*50}")
        for i in range(num_buckets):
            b = buckets[i]
            start = i * bucket_size
            end = (i + 1) * bucket_size
            print(f"  {start:3d}-{end:3d}s   | {b['count']:8d} | {b['emergency']:9d} | {b['comfortable']:11d}")

    # Save results
    results = {
        'analysis_time': datetime.now().isoformat(),
        'bag_path': bag_path,
        'total_mrm_triggers': len(mrm_triggers),
        'behavior_distribution': dict(behavior_counts),
        'top_diagnostics': sorted_diags[:30],
        'triggers': [
            {
                'time_sec': t['time_sec'],
                'state': t['state_name'],
                'behavior': t['behavior_name']
            }
            for t in mrm_triggers
        ]
    }

    output_file = os.path.join(os.path.dirname(bag_path), 'mrm_analysis.json')
    with open(output_file, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"\nResults saved to: {output_file}")

    return results


def main():
    if len(sys.argv) < 2:
        # Default to most recent experiment
        script_dir = os.path.dirname(os.path.abspath(__file__))
        data_dir = os.path.join(script_dir, '..', 'data')

        # Find first goal directory with rosbag
        for item in sorted(os.listdir(data_dir)):
            bag_path = os.path.join(data_dir, item, 'rosbag')
            if os.path.isdir(bag_path):
                break
        else:
            print("ERROR: No rosbag found in data directory")
            print("Usage: python3 analyze_mrm_diagnostics.py <rosbag_path>")
            return 1
    else:
        bag_path = sys.argv[1]

    if not os.path.exists(bag_path):
        print(f"ERROR: Rosbag not found at {bag_path}")
        return 1

    results = analyze_rosbag(bag_path)

    if results:
        print("\n" + "="*80)
        print("SUMMARY")
        print("="*80)
        print(f"Total MRM triggers: {results['total_mrm_triggers']}")
        print(f"Behavior distribution: {results['behavior_distribution']}")
        print(f"\nTop 5 diagnostic issues at MRM triggers:")
        for name, count in results['top_diagnostics'][:5]:
            print(f"  - {name}: {count} occurrences")

    return 0


if __name__ == '__main__':
    sys.exit(main())
