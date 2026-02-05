#!/usr/bin/env python3
"""
Compute metrics for all experiments and display summary table.

Usage:
  python3 compute_metrics.py                    # Process experiments/data
  python3 compute_metrics.py /path/to/data      # Process custom directory
  python3 compute_metrics.py --recompute        # Force recompute all metrics

Output:
  - Computes metrics.json for each experiment
  - Displays summary table
  - Saves summary to experiments/data/summary.txt
"""

import os
import sys
import argparse

# Add lib to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'lib'))

from metrics import compute_all_metrics, compute_metrics_from_bag, save_metrics, load_metrics, generate_summary_table


def main():
    parser = argparse.ArgumentParser(description='Compute metrics for all experiments')
    parser.add_argument('data_dir', nargs='?', default=None,
                       help='Directory containing experiment data (default: experiments/data)')
    parser.add_argument('--recompute', action='store_true',
                       help='Force recompute all metrics even if metrics.json exists')
    parser.add_argument('--output', '-o', default=None,
                       help='Output file for summary (default: data_dir/summary.txt)')

    args = parser.parse_args()

    # Determine data directory
    if args.data_dir:
        data_dir = args.data_dir
    else:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        data_dir = os.path.join(script_dir, '..', 'data')

    if not os.path.exists(data_dir):
        print(f"ERROR: Data directory not found: {data_dir}")
        return 1

    # Output file
    output_file = args.output or os.path.join(data_dir, 'summary.txt')

    print("=" * 60)
    print("COMPUTING EXPERIMENT METRICS")
    print("=" * 60)
    print(f"Data directory: {data_dir}")
    print(f"Recompute: {args.recompute}")
    print()

    # If recompute, delete existing metrics files
    if args.recompute:
        exp_dirs = [d for d in os.listdir(data_dir)
                   if os.path.isdir(os.path.join(data_dir, d)) and d.startswith('goal_')]
        for exp_dir in exp_dirs:
            metrics_file = os.path.join(data_dir, exp_dir, 'metrics.json')
            if os.path.exists(metrics_file):
                os.remove(metrics_file)
                print(f"Removed: {metrics_file}")
        print()

    # Compute metrics
    metrics_list, summary = compute_all_metrics(data_dir, output_file)

    # Display summary
    print()
    print(summary)

    return 0


if __name__ == '__main__':
    sys.exit(main())
