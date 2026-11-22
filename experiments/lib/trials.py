"""
Finding trials on disk, for analysis scripts. No ROS needed.

Layout (experiments/lib/config.py TrialConfig):
    experiments/data/<campaign>/<goal_id>/t<N>_<timestamp>/
        metadata.json  result.json  metrics.json  fault_log.jsonl  rosbag/
    experiments/data/<campaign>/_meta/      runner / injector logs
    experiments/data/<campaign>/_invalid/   trials the runner rejected

Campaign names and kinds come from experiments/configs/campaigns.yaml; a
trial's fault parameters from its own metadata.json, never from the name.
"""

import glob
import json
import os
from typing import Dict, List, Optional

from config import DATA_DIR, load_campaigns


def campaigns(kind: Optional[str] = None) -> List[str]:
    """Campaign names in file order. kind: 'nominal', 'imu', 'tl' or None (all)."""
    out = []
    for name, c in load_campaigns().items():
        k = fault_kind_of(c.fault)
        if kind is None or k == kind:
            out.append(name)
    return out


def fault_campaigns() -> List[str]:
    return [c for c in campaigns() if c not in campaigns('nominal')]


def fault_kind_of(fault: dict) -> str:
    """'tl', 'imu' or 'nominal' for a metadata.json / campaigns.yaml fault dict."""
    if not fault:
        return 'nominal'
    if fault.get('tl_fault'):
        return 'tl'
    if fault.get('imu_fault'):
        return 'imu'
    raise ValueError(f'fault with neither tl_fault nor imu_fault: {fault}')


def campaign_kind(campaign: str) -> str:
    """Kind of a campaign directory name (an Arm B '<name>_armB' maps to its base)."""
    spec = load_campaigns()
    base = campaign[:-len('_armB')] if campaign.endswith('_armB') else campaign
    if base not in spec:
        raise KeyError(f'{campaign!r} is not in campaigns.yaml')
    return fault_kind_of(spec[base].fault)


def zone_kind(campaign: str) -> str:
    """The route zone a campaign's fault arms on (fault_injector.py): 'tl'
    (traffic-light zones), 'imu_leadin' (gyro bias, bias lead-in zones) or
    'imu_turn' (scale / stuck, turn zones)."""
    spec = load_campaigns()
    base = campaign[:-len('_armB')] if campaign.endswith('_armB') else campaign
    fault = spec[base].fault
    kind = fault_kind_of(fault)
    if kind == 'nominal':
        raise ValueError(f'{campaign} has no fault')
    if kind == 'tl':
        return 'tl'
    return 'imu_leadin' if fault['imu_fault'] == 'imu_bias' else 'imu_turn'


def _load(path: str) -> dict:
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, json.JSONDecodeError):
        return {}


def trial_dirs(campaign: str, goals: Optional[List[str]] = None,
               valid_only: bool = True, data_dir: str = DATA_DIR) -> List[str]:
    """Trial directories of one campaign, sorted by goal then trial name.
    valid_only: the runner's own verdict (result.json 'valid'), which also
    requires a complete bag; _invalid/ and _meta/ are never included."""
    out = []
    for d in sorted(glob.glob(os.path.join(data_dir, campaign, 'goal_*', 't*'))):
        if goals and goal_of(d) not in goals:
            continue
        if not os.path.isdir(os.path.join(d, 'rosbag')):
            continue
        if valid_only and not _load(os.path.join(d, 'result.json')).get('valid'):
            continue
        out.append(d)
    return out


def goal_of(trial_dir: str) -> str:
    return os.path.basename(os.path.dirname(os.path.normpath(trial_dir)))


def campaign_of(trial_dir: str) -> str:
    return os.path.basename(os.path.dirname(os.path.dirname(os.path.normpath(trial_dir))))


def metadata(trial_dir: str) -> dict:
    return _load(os.path.join(trial_dir, 'metadata.json'))


def result(trial_dir: str) -> dict:
    return _load(os.path.join(trial_dir, 'result.json'))


def fault_log(trial_dir: str) -> List[dict]:
    """The trial's fault_log.jsonl events (startup line first)."""
    out = []
    try:
        with open(os.path.join(trial_dir, 'fault_log.jsonl')) as f:
            for line in f:
                line = line.strip()
                if line:
                    out.append(json.loads(line))
    except OSError:
        pass
    return out


def find_nominal_run(name: str, data_dir: str = DATA_DIR) -> Optional[str]:
    """Trial directory of an extracted sequence file 't<N>_<timestamp>'
    (st_gat EXTRACTED_DIR / TRAIN_DIR / CAL_DIR name) in a nominal campaign."""
    for c in campaigns('nominal'):
        m = glob.glob(os.path.join(data_dir, c, 'goal_*', name))
        if m:
            return m[0]
    return None


def trials_by_campaign(names: Optional[List[str]] = None) -> Dict[str, List[str]]:
    """{campaign: valid trial dirs} for the given (default: every fault) campaign."""
    return {c: trial_dirs(c) for c in (names or fault_campaigns())}
