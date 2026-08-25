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


def compute_train_sample_weights(dataset, cfg, n_turn_bins=10, weight_cap_percentile=95.0, verbose=True):
    """Per-window training sample weight, DATA-DRIVEN (2026-08-25, replaces
    the 2026-08-24 version's fixed turn_boost=3.0/tl_boost=2.0/
    turn_cap_deg=20.0 constants per Kalpit's explicit direction: "I dont
    want to hard-code values as we plan to get more data and so our
    pipeline should be ready for that"). Classic inverse-class-frequency
    reweighting instead of hand-picked multipliers:

    1. Bin windows by (turn-severity decile, tl-zone membership) -- bin
       EDGES are the CURRENT dataset's own quantiles (rank-based, not
       fixed degree thresholds), so as more/different data is collected
       the bins re-derive themselves automatically, no manual retuning.
    2. weight_i = 1 / frequency(bin_i), i.e. rarer combined scenarios get
       proportionally more weight -- automatically tracks whatever the
       actual imbalance ratio in the current dataset is, rather than
       asserting a fixed multiplier that may over- or under-correct as
       the data distribution shifts.
    3. Capped at the `weight_cap_percentile`-th percentile of the weights
       THEMSELVES (self-referential, not an external hardcoded ceiling)
       to prevent one near-empty bin from dominating the sampler, then
       renormalized to mean 1.0 so the effective dataset size stays
       stable regardless of how skewed the underlying distribution is.

    n_turn_bins/weight_cap_percentile are binning-RESOLUTION choices, not
    tuned "how much should turns matter" constants -- reasonable to leave
    at their defaults across retrains; expose them for the sweep in
    experiments/scripts/sweep_zone_weighting.py rather than the boost
    magnitudes the old version exposed.

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
    tl_flag = membership['tl_zones'].astype(int)

    # Rank-based (not value-based) quantile binning of the nonzero turn-
    # severity tail: most windows are ~0 deg (straight driving), so a
    # plain value-quantile split would collapse most bin edges at 0.
    # Ranking only the nonzero subset spreads the tail into n_turn_bins-1
    # genuinely distinct bins; bin 0 is reserved for ~straight driving.
    nonzero = turn_deg > 0.01
    turn_bin = np.zeros(n, dtype=int)
    if nonzero.sum() >= n_turn_bins:
        order = np.argsort(np.argsort(turn_deg[nonzero]))
        ranks = order / max(1, nonzero.sum() - 1)
        turn_bin[nonzero] = 1 + np.clip((ranks * (n_turn_bins - 1)).astype(int), 0, n_turn_bins - 2)

    category = turn_bin * 2 + tl_flag
    cats, counts = np.unique(category, return_counts=True)
    freq = dict(zip(cats.tolist(), (counts / n).tolist()))

    raw_weight = np.array([1.0 / freq[c] for c in category])
    cap = float(np.percentile(raw_weight, weight_cap_percentile))
    weight = np.clip(raw_weight, None, cap)
    weight = weight / weight.mean()

    if verbose:
        print(f"[scenario_zones] auto sample weights: {len(cats)} combined (turn-severity-bin x "
              f"tl-zone) cells over {n} windows; raw inverse-frequency range "
              f"[{raw_weight.min():.2f}, {raw_weight.max():.2f}], capped at "
              f"p{weight_cap_percentile:.0f}={cap:.2f}, final normalized range "
              f"[{weight.min():.2f}, {weight.max():.2f}] (mean 1.0)")

    return weight
