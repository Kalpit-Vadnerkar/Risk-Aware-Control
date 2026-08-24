"""
Shared scenario-zone geometry (2026-08-24): loads the SAME zone files
`experiments/lib/fault_injector.py` gates fault injection on
(`turn_zones.json`'s turn_zones/bias_leadin_zones, `tl_zones.json`'s
tl_zones), pools them across all goals (fixed physical map locations, not
goal-specific), and computes per-window continuous "difficulty" weights
for training-time oversampling.

Extracted from `experiments/scripts/audit_minority_scenarios.py` (which
now imports from here) so `st_gat/train.py` can reuse the exact same zone
geometry for sample weighting without duplicating it -- one definition of
"near a turn/intersection zone," used consistently for both diagnosing the
turn-anticipation gap and fixing it in training.

Why weight by CONTINUOUS turn severity rather than a binary "near a
turn/leadin zone" flag: the zone-proximity radius (15-20m) covers a much
broader corridor than the seconds where a window is actually mid-turn --
`diagnose_turn_learning.py` found genuine active-turning (>15deg heading
change within a specific 3s window) is only ~10% of windows, while
"within 15m of SOME turn/leadin zone point" is closer to half the dataset
(overlapping categories). Boosting half the dataset 3x would swamp the
loss with easy near-zone-but-not-actually-turning windows and barely
change the actual turn-heavy minority's representation. Weighting
proportional to the window's OWN actual turn severity targets the real
minority directly. TL-zone proximity stays binary (no equivalent
continuous "how much of an intersection is this" signal exists) since the
audit found tl_zones under-covers regardless of whether the ego is
currently turning there.
"""

import json
import os

import numpy as np
from scipy.spatial import cKDTree

_LIB_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(os.path.dirname(_LIB_DIR))

TURN_ZONES_FILE = os.path.join(REPO_DIR, 'experiments', 'configs', 'turn_zones.json')
TL_ZONES_FILE   = os.path.join(REPO_DIR, 'experiments', 'configs', 'tl_zones.json')

RADIUS_M = {
    'turn_zones':          15.0,
    'bias_leadin_zones':   15.0,
    'lane_change_zones':   15.0,
    'curved_road_zones':   15.0,
    'tl_zones':            20.0,   # matches fault_injector.py's established 20-30m stable TL-zone range
}

MIN_SEG_M = 0.5
_K = 5   # steps averaged at each end for the denoised heading-change estimate


def _heading(vec):
    return np.arctan2(vec[1], vec[0])


def _wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def robust_turn_deg(xy_m, k=_K, min_seg_m=MIN_SEG_M):
    """Denoised future heading-change magnitude in degrees, averaging over
    k steps at each end rather than a single-step finite difference (see
    diagnose_turn_learning.py's original derivation -- canonical copy now
    lives here). xy_m: (T, 2) array of real-metre positions."""
    start_vec = xy_m[k] - xy_m[0]
    end_vec   = xy_m[-1] - xy_m[-1 - k]
    if np.linalg.norm(start_vec) < min_seg_m or np.linalg.norm(end_vec) < min_seg_m:
        return 0.0
    return float(np.degrees(abs(_wrap(_heading(end_vec) - _heading(start_vec)))))


def _pooled_turn_zone_points(turn_zones_data, category):
    pts = []
    for g in turn_zones_data['goals'].values():
        for z in g[category]:
            pts.append((z['x'], z['y']))
            if 'end_x' in z:
                pts.append((z['end_x'], z['end_y']))
    return np.array(pts) if pts else np.zeros((0, 2))


def _pooled_tl_points(tl_zones_data):
    pts = []
    for g in tl_zones_data['goals'].values():
        for z in g['tl_zones']:
            pts.append((z['x'], z['y']))
    return np.array(pts) if pts else np.zeros((0, 2))


def load_zone_trees():
    """Returns {category: (cKDTree or None, n_points)} for all 5 categories."""
    with open(TURN_ZONES_FILE) as f:
        turn_zones_data = json.load(f)
    with open(TL_ZONES_FILE) as f:
        tl_zones_data = json.load(f)

    trees = {}
    for cat in ['turn_zones', 'bias_leadin_zones', 'lane_change_zones', 'curved_road_zones']:
        pts = _pooled_turn_zone_points(turn_zones_data, cat)
        trees[cat] = (cKDTree(pts) if len(pts) else None, len(pts))
    tl_pts = _pooled_tl_points(tl_zones_data)
    trees['tl_zones'] = (cKDTree(tl_pts) if len(tl_pts) else None, len(tl_pts))
    return trees


def zone_membership(xy, trees=None):
    """xy: (N, 2) real-metre positions -> {category: (N,) bool array}."""
    if trees is None:
        trees = load_zone_trees()
    membership = {}
    for cat, (tree, _n) in trees.items():
        if tree is None:
            membership[cat] = np.zeros(len(xy), dtype=bool)
            continue
        dist, _ = tree.query(xy, k=1)
        membership[cat] = dist <= RADIUS_M[cat]
    return membership


def window_start_xy(dataset, cfg):
    """Real-metre (x, y) at each window's start (last observed frame) --
    the moment fault_injector.py's own zone-arming logic checks against.
    `dataset` is a TrajectoryDataset; `cfg` is st_gat.pipeline.config."""
    n = len(dataset)
    xy = np.zeros((n, 2))
    for idx in range(n):
        seq = dataset.sequences[idx]
        ref_x, ref_y = seq['position_ref']
        last_past = seq['past'][-1]['position']
        xy[idx, 0] = ref_x + last_past[0] * cfg.POSITION_DISPLACEMENT_RANGE_M
        xy[idx, 1] = ref_y + last_past[1] * cfg.POSITION_DISPLACEMENT_RANGE_M
    return xy


def compute_train_sample_weights(dataset, cfg, turn_boost=3.0, tl_boost=2.0, turn_cap_deg=20.0):
    """Per-window training sample weight (2026-08-24, for
    st_gat/train.py's --zone-weighted-sampling): 1.0 baseline, +turn_boost
    scaled by the window's OWN actual future turn severity (capped at
    turn_cap_deg, so a 20+ degree turn gets the full boost and severity
    beyond that doesn't runaway-dominate the sampler), +tl_boost flat if
    the window starts within a real TL/intersection zone (see module
    docstring for why TL stays binary while turn severity is continuous).
    Returns an (N,) float array suitable for
    torch.utils.data.WeightedRandomSampler."""
    n = len(dataset)
    turn_deg = np.zeros(n)
    for idx in range(n):
        future_pos = np.array([f['position'] for f in dataset.sequences[idx]['future']]) * cfg.POSITION_DISPLACEMENT_RANGE_M
        turn_deg[idx] = robust_turn_deg(future_pos)

    xy = window_start_xy(dataset, cfg)
    trees = load_zone_trees()
    membership = zone_membership(xy, trees)

    weights = np.ones(n)
    weights += turn_boost * np.clip(turn_deg / turn_cap_deg, 0.0, 1.0)
    weights += tl_boost * membership['tl_zones'].astype(float)
    return weights
