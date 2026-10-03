"""
Shared helper for classifying a fault campaign's kind (tl / imu / nominal)
from its own recorded ground truth, instead of guessing from the campaign
directory name.

Added 2026-10-03: `inspect_fault_predictions.py` and
`compare_fault_vs_nominal.py` used to each guess independently via
`campaign.startswith(...)`, with inconsistent fallback defaults — a new
fault-family campaign name that didn't match either prefix would silently
misclassify in at least one of them. Every trial's `fault_log.jsonl`
startup event already records the real fault type directly (`tl_fault`/
`imu_fault`, exactly one non-null per campaign, both null only for a
nominal campaign where fault_injector.py never runs) — this reads that
instead of parsing the name.
"""
import glob
import json
import os


def campaign_fault_kind(campaign_dir: str) -> str:
    """
    Returns 'tl', 'imu', or 'nominal' for the fault campaign at
    `campaign_dir` (e.g. `.../experiments/data/tl_fault_s2`), read from the
    first trial's fault_log.jsonl startup event found under it. All trials
    in one campaign share the same fault type, so any one trial suffices.
    """
    for log_path in sorted(glob.glob(os.path.join(campaign_dir, 'goal_*', 't*', 'fault_log.jsonl'))):
        try:
            with open(log_path) as f:
                first_line = f.readline()
            if not first_line:
                continue
            entry = json.loads(first_line)
        except (OSError, json.JSONDecodeError):
            continue
        if entry.get('event') != 'startup':
            continue
        if entry.get('tl_fault'):
            return 'tl'
        if entry.get('imu_fault'):
            return 'imu'
        return 'nominal'
    return 'nominal'
