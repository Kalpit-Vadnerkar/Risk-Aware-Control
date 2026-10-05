#!/usr/bin/env python3
"""
Layer 2 v2 (2026-10-04): a priori (2a) + a posteriori (2b) consequence
estimation, run on real fault trials against AWSIM ground truth.

Pipeline, per analysis window (time t, the window's "now"):

  1. Point prediction mu(t) of the ego's next 3s (ST-GAT, position head).
  2. 2a -- a priori tube shaping: K counterfactual trajectories
     mu(t) + s(t) * r_j, where r_j are WHOLE residual trajectories
     bootstrapped from Layer 1's nominal calibration pool. The pool is
     one of: `pooled` (whole calibration set -- the v1 behaviour, the
     control), `mondrian` (same zone group, assign_groups()), or
     `embedding` (k nearest calibration windows in h_last).
  3. Layer 1 -> Layer 2 coupling, s(t): `static` = 1 always (v1 behaviour);
     `adaptive` = how far outside its nominal calibration the model's
     prediction made 1s ago has turned out to be, now that its first second
     has been observed (causal). Per kinematic series f, ratio_f = mean over
     the first 10 steps of |residual| / that step's 90% conformal quantile;
     A = max_f ratio_f; s = clip(A / q90(A on nominal), 1, --scale-max).
     So s = 1 for >= 90% of nominal windows by construction, and the tube
     only inflates when Layer 1 has real evidence the ego's motion is off
     nominal. Without this, the bootstrap tube is made purely of NOMINAL
     residuals and cannot widen under a fault at all -- the wiring plan
     (docs/research_notes/layer2b_wiring_plan_2026-10-04.md) had no channel
     from Layer 1's anomaly signal into Layer 2.
  4. 2b -- a posteriori trim: drop counterfactuals that are kinematically
     infeasible from the current state (|a_long| or |a_lat| > 8 m/s^2,
     measured over 1.0s segments, including continuity with the observed
     current velocity). This removes what CAN'T happen.
  5. 2b -- scoring against the safe set: what SHOULDN'T happen. A
     counterfactual violates if the ego footprint leaves the drivable area
     (margin.DrivableArea: union of road lanelets, signed distance field)
     or overlaps a tracked object's constant-velocity footprint
     (margin.object_clearance_v2). P(violation within H) = violating /
     feasible.

Trim vs. score is deliberately split: lane/road containment cannot be a
trim criterion, because trimming every counterfactual that leaves the road
defines the risk away. Physics trims; the map scores.

Ground truth: fault trials read AWSIM's own ground-truth pose
(/awsim/ground_truth/localization/kinematic_state, recorded in every bag)
-- NOT Autoware's estimate, which is what the ST-GAT pipeline sees. Under
IMU faults those diverge by 10-20m and the real vehicle leaves the road
while the estimate never does (see --mode check-safe-set's companion
findings in docs/research_notes/layer2b_results_2026-10-04.md).

Modes:
  check-safe-set  acceptance test for the safe-set + feasibility
                  definitions: the REALIZED nominal future must (almost)
                  never violate / be infeasible. v1's margin fails this
                  (flags ~71% of real nominal driving); v2 must pass.
  nominal         leave-one-trial-out P(violation) floor on CAL_DIR, all 6
                  variants; sets each variant's alarm threshold at the
                  --far false-alarm budget (matched FAR).
  faults          every window of every fault trial on disk, all 6
                  variants + a Layer-1-only anomaly alarm at the same FAR;
                  lead time against ground-truth road departure; featured
                  plots (P(violation) over time + envelope cascade) for
                  trials with a real departure.
  all             (default) all three, in order.

v1 history (2026-08-21): bootstrap -> margin -> P(violation) on nominal
data only, with margin.py's v1 lane-boundary + worst-case object margin.
Its one run reported mean P(violation) = 0.84 on nominal driving; that
number measured the margin definition, not risk (see margin.py's v2 note).

Usage (ROS + repo venv sourced -- needs lanelet2 and rosbag2_py):
  source /opt/ros/humble/setup.bash
  source /home/kvadner/Desktop/Dissertation/autoware/install/setup.bash
  source .venv/bin/activate
  python3 experiments/scripts/layer2_consequence_estimation.py [--mode all]
"""

import argparse
import glob
import json
import math
import os
import pickle
import sys

import numpy as np
import torch
from torch.utils.data import DataLoader

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
sys.path.insert(0, REPO_DIR)
sys.path.insert(0, os.path.join(REPO_DIR, 'experiments', 'lib'))
sys.path.insert(0, SCRIPT_DIR)

import margin as margin_lib  # noqa: E402
from st_gat.pipeline import config as cfg  # noqa: E402
from st_gat.model import STGAT, TrajectoryDataset  # noqa: E402
from conformal_horizon_calibration import (  # noqa: E402
    _trial_ids_for, _FEATURES, _extract_series, conformal_quantile)
from conformal_mondrian_calibration import assign_groups, MIN_FOLD_GROUP_N  # noqa: E402

DEFAULT_OUTPUT_DIR = os.path.join(cfg.ANALYSIS_ROOT, 'layer2_consequence_estimation')
DATA_DIR = os.path.join(REPO_DIR, 'experiments', 'data')
GT_TOPIC = '/awsim/ground_truth/localization/kinematic_state'
EST_TOPIC = '/localization/kinematic_state'

DT = 0.1
R_POS = cfg.POSITION_DISPLACEMENT_RANGE_M
# Series that feed the adaptive coupling: the ones describing the ego's own
# motion (chosen a priori, not by looking at fault results). TL color/
# confidence are excluded -- their nominal 90% quantile is ~0 (quasi-
# categorical), so any ratio against it is unbounded, and a TL anomaly only
# matters for ego risk once it shows up in the ego's motion anyway.
KINEMATIC_SERIES = ['position', 'velocity_longitudinal', 'velocity_lateral', 'steering', 'acceleration']
POOLS = ['pooled', 'mondrian', 'embedding']
COUPLINGS = ['static', 'adaptive']
VARIANTS = [f'{p}_{c}' for c in COUPLINGS for p in POOLS]
A_LONG_MAX = 8.0   # m/s^2, ~0.8g: beyond hard braking on dry asphalt
A_LAT_MAX = 8.0    # m/s^2, ~friction limit
FEAS_SEG = 10      # feasibility measured over 1.0s segments: pose is forward-filled onto the 10Hz
                   # objects clock, so single-frame timing slips (~1.1m at 11 m/s) make 0.1-0.3s
                   # differences read as ~9 m/s^2 phantom acceleration on REAL nominal trajectories


# ── Model / inference ───────────────────────────────────────────────────────

class _SeqList(TrajectoryDataset):
    """TrajectoryDataset over an in-memory sequence list (fault trials are
    built straight from the bag by process_trial(), never extracted to pkl)."""
    def __init__(self, sequences):   # noqa: super().__init__ deliberately skipped (it loads a folder)
        self.sequences = sequences


def _load_model(model_path, device):
    model = STGAT(cfg.build_inference_model_cfg(device)).to(device)
    model.load_state_dict(torch.load(model_path, map_location=device, weights_only=True))
    model.eval()
    print(f"Loaded model: {model_path} ({model.count_parameters():,} params)")
    return model


def infer(model, device, ds, batch):
    """One pass. Returns pos_mean (N,T,2) and pos_err (N,T,2) [normalized],
    series {report_key: (N,T) residual magnitude}, h (N,D)."""
    loader = DataLoader(ds, batch_size=batch, shuffle=False, num_workers=0)
    means, errs, hs = [], {k: [] for k in _FEATURES}, []
    with torch.no_grad():
        for past, future, graph, _b in loader:
            past = {k: v.to(device) for k, v in past.items()}
            graph = {k: v.to(device) for k, v in graph.items()}
            hs.append(model.encode_scene(past, graph).cpu().numpy())
            preds = model(past, graph)
            for key, dims in _FEATURES.items():
                mean = preds[f'{key}_mean'].cpu()
                actual = future[key]
                if dims == 1 and actual.dim() == 3 and actual.size(-1) == 1:
                    actual = actual.squeeze(-1)
                errs[key].append((actual - mean).numpy())
            means.append(preds['position_mean'].cpu().numpy())
    err = {k: np.concatenate(v) for k, v in errs.items()}
    return {'pos_mean': np.concatenate(means), 'pos_err': err['position'],
            'series': _extract_series(err), 'h': np.concatenate(hs)}


# ── Window geometry ─────────────────────────────────────────────────────────

def _ref(seq):
    return np.asarray(seq['position_ref'], dtype=float)


def _past_xy(seq):
    return _ref(seq) + np.array([f['position'] for f in seq['past']]) * R_POS


def _future_xy(seq):
    return _ref(seq) + np.array([f['position'] for f in seq['future']]) * R_POS


def _frame_objects(frame, ego_xy):
    """Real-metre absolute (x, y, speed, class_group) for every unclamped
    tracked object in one frame (objects beyond OBJECT_REL_POS_RANGE are
    stored clamped to the edge -- position unknown, skipped)."""
    feats = np.asarray(frame['objects_set'])
    mask = np.asarray(frame['objects_mask'])
    lo, hi = cfg.OBJECT_REL_POS_RANGE
    s_lo, s_hi = cfg.OBJECT_SPEED_RANGE
    out = []
    for k in range(len(mask)):
        if mask[k] < 0.5:
            continue
        dx = feats[k, 0] * (hi - lo) + lo
        dy = feats[k, 1] * (hi - lo) + lo
        if max(abs(dx), abs(dy)) >= hi - 0.05:
            continue
        speed = feats[k, 2] * (s_hi - s_lo) + s_lo
        cls = int(np.argmax(feats[k, 3:3 + cfg.OBJECT_CLASS_GROUPS]))
        out.append((ego_xy[0] + dx, ego_xy[1] + dy, speed, cls))
    return out


def window_objects(seq, map_data, T):
    """Constant-velocity forecast of every tracked object over T steps.
    Velocity from nearest-neighbour association between the current frame
    and 0.5s earlier (same class, gated by observed speed); falls back to
    the lanelet's direction of travel at the observed speed when there's no
    match. Returns (obj_xy (M,T,2), obj_heading (M,T), obj_class (M,))."""
    past_xy = _past_xy(seq)
    now = _frame_objects(seq['past'][-1], past_xy[-1])
    prev = _frame_objects(seq['past'][-6], past_xy[-6])
    used = set()
    t = np.arange(1, T + 1) * DT
    xy, hd, cl = [], [], []
    for (x, y, sp, c) in now:
        best, bj = sp * 0.5 + 2.0, None
        for j, (px, py, _ps, pc) in enumerate(prev):
            d = math.hypot(x - px, y - py)
            if pc == c and j not in used and d < best:
                best, bj = d, j
        v = None
        if bj is not None:
            used.add(bj)
            vx, vy = (x - prev[bj][0]) / 0.5, (y - prev[bj][1]) / 0.5
            if math.hypot(vx, vy) > 1.0:
                v = np.array([vx, vy]) * (sp / max(math.hypot(vx, vy), 1e-6))   # direction from motion, magnitude as observed
        if v is None:
            h = margin_lib.lanelet_heading(x, y, map_data)
            v = np.array([math.cos(h), math.sin(h)]) * (sp if sp > 1.0 else 0.0)
            heading = h
        else:
            heading = math.atan2(v[1], v[0])
        xy.append(np.array([x, y]) + t[:, None] * v[None, :])
        hd.append(np.full(T, heading))
        cl.append(c)
    if not xy:
        return np.zeros((0, T, 2)), np.zeros((0, T)), np.zeros(0, dtype=int)
    return np.array(xy), np.array(hd), np.array(cl)


def _past_heading_and_velocity(seq, map_data):
    p = _past_xy(seq)
    d = p[-1] - p[-1 - FEAS_SEG]
    v0 = d / (FEAS_SEG * DT)
    if np.hypot(*(p[-1] - p[-6])) > 0.3:
        h = math.atan2(*(p[-1] - p[-6])[::-1])
    else:
        h = margin_lib.lanelet_heading(p[-1][0], p[-1][1], map_data)
    return h, v0


def ego_headings(traj, ref, h_past):
    """traj (K,T,2) real -> (K,T) heading of motion; h_past where nearly stopped."""
    pts = np.concatenate([np.broadcast_to(ref, (traj.shape[0], 1, 2)), traj], axis=1)
    d = np.diff(pts, axis=1)
    h = np.arctan2(d[..., 1], d[..., 0])
    return np.where(np.linalg.norm(d, axis=-1) < 0.05, h_past, h)


def _rot(err, h):
    """Rotate (..., T, 2) vectors by angle(s) h (broadcast over leading dims)."""
    c, s_ = np.cos(h)[..., None], np.sin(h)[..., None]
    return np.stack([c * err[..., 0] - s_ * err[..., 1], s_ * err[..., 0] + c * err[..., 1]], axis=-1)


def feasible_mask(traj, ref, v0):
    """traj (K,T,2) real. True where the counterfactual stays within
    A_LONG_MAX / A_LAT_MAX given the observed current velocity v0."""
    K = traj.shape[0]
    pts = np.concatenate([np.broadcast_to(ref, (K, 1, 2)), traj], axis=1)[:, ::FEAS_SEG]
    v = np.diff(pts, axis=1) / (FEAS_SEG * DT)                       # (K, S, 2)
    v = np.concatenate([np.broadcast_to(v0, (K, 1, 2)), v], axis=1)  # (K, S+1, 2)
    dv = np.diff(v, axis=1) / (FEAS_SEG * DT)                        # (K, S, 2)
    vm = 0.5 * (v[:, 1:] + v[:, :-1])
    sp = np.linalg.norm(vm, axis=-1)
    tang = vm / np.maximum(sp, 1e-6)[..., None]
    a_long = np.abs((dv * tang).sum(-1))
    a_lat = np.abs(dv[..., 0] * tang[..., 1] - dv[..., 1] * tang[..., 0])
    a_lat = np.where(sp > 2.0, a_lat, 0.0)   # heading is noise below walking pace
    a_long = np.where(sp > 0.5, a_long, np.abs(dv).max(-1))
    return (a_long.max(1) <= A_LONG_MAX) & (a_lat.max(1) <= A_LAT_MAX)


def score(traj, ref, h_past, objs, drivable):
    """traj (K,T,2) real -> (road_viol (K,), obj_viol (K,), min road margin (K,))."""
    rm = margin_lib.road_margin(drivable.signed_distance(traj[..., 0], traj[..., 1]))
    road_min = rm.min(1)
    obj_xy, obj_hd, obj_cl = objs
    if len(obj_cl):
        oc = margin_lib.object_clearance_v2(traj, ego_headings(traj, ref, h_past), obj_xy, obj_hd, obj_cl)
        obj_viol = oc.min(1) < 0
    else:
        obj_viol = np.zeros(traj.shape[0], dtype=bool)
    return road_min < 0, obj_viol, road_min


# ── Calibration pool + Layer 1 anomaly score ────────────────────────────────

def _ratio(series, q, idx):
    """Per-window kinematic nonconformity: max over KINEMATIC_SERIES of mean
    over the first L steps of |residual| / step quantile. idx may contain -1
    (no causal predecessor) -> nan."""
    L = len(next(iter(q.values())))
    r = np.stack([(series[f][np.maximum(idx, 0), :L] / q[f][None, :]).mean(1) for f in KINEMATIC_SERIES], 1)
    out = r.max(1)
    out[idx < 0] = np.nan
    return out


def build_cal_pool(model, device, map_data, args):
    # Keyed on graph schema + checkpoint: a different model must never reuse
    # another model's cached calibration-pool inference.
    key = f"{cfg.GRAPH_FRAME}_{os.path.basename(os.path.dirname(os.path.abspath(args.model)))}"
    cache = os.path.join(args.output_dir, f'cal_pool_cache_{key}.npz')
    ds = TrajectoryDataset(cfg.CAL_DIR)
    trial_id, pkl_files = _trial_ids_for(cfg.CAL_DIR)
    if os.path.exists(cache) and not args.no_cache:
        z = np.load(cache, allow_pickle=True)
        pool = {k: z[k] for k in ['pos_mean', 'pos_err', 'h']}
        pool['series'] = z['series'].item()
        print(f"Loaded cached calibration-pool inference ({cache})")
    else:
        print(f"Calibration pool: one model pass over {len(ds)} windows...")
        pool = infer(model, device, ds, args.batch)
        np.savez(cache, pos_mean=pool['pos_mean'], pos_err=pool['pos_err'], h=pool['h'],
                 series=np.array(pool['series'], dtype=object))
    pool['ds'] = ds
    pool['trial'] = trial_id
    pool['pkl_files'] = pkl_files
    pool['ref'] = np.array([_ref(s) for s in ds.sequences])
    pool['group'] = assign_groups(pool['ref'])
    # Residuals are bootstrapped in the EGO HEADING FRAME (along-track,
    # cross-track), not the map frame: a map-frame residual from a window
    # heading north, pasted onto a window heading east, turns an along-track
    # speed error (p90 ~5.4m at 3s) into a lateral one and drives the
    # counterfactual off the road. v1 had this bug.
    hcache = os.path.join(args.output_dir, f'cal_pool_heading_cache_{cfg.GRAPH_FRAME}.npy')
    if os.path.exists(hcache) and not args.no_cache and len(np.load(hcache)) == len(ds):
        pool['heading'] = np.load(hcache)
    else:
        pool['heading'] = np.array([_past_heading_and_velocity(s, map_data)[0] for s in ds.sequences])
        np.save(hcache, pool['heading'])
    pool['err_local'] = _rot(pool['pos_err'], -pool['heading'])
    L = int(round(args.anomaly_lag_s / DT))
    pool['q'] = {f: np.maximum([conformal_quantile(pool['series'][f][:, s], 0.1) for s in range(L)], 1e-6)
                 for f in KINEMATIC_SERIES}
    # Causal predecessor: the window L positions earlier in the SAME trial
    # (CAL pkls carry no timestamps; windows are stride-1 at 10Hz, with rare
    # frame-gap rejections that make this approximate).
    n = len(trial_id)
    prev = np.arange(n) - L
    prev[(prev < 0) | (trial_id[np.maximum(prev, 0)] != trial_id)] = -1
    A = _ratio(pool['series'], pool['q'], prev)
    pool['A'] = A
    pool['qA'] = conformal_quantile(A[~np.isnan(A)], 0.1)
    pool['scale'] = np.clip(np.nan_to_num(A / pool['qA'], nan=1.0), 1.0, args.scale_max)
    print(f"  {n} windows, {len(pkl_files)} trials; groups: "
          + ", ".join(f"{g}={int((pool['group'] == g).sum())}" for g in np.unique(pool['group']))
          + f"; q90(A) = {pool['qA']:.3f}")
    return pool


def pool_indices(pool, h, group, exclude_trial, k_neighbors):
    ok = pool['trial'] != exclude_trial if exclude_trial is not None else np.ones(len(pool['trial']), bool)
    base = np.where(ok)[0]
    g = base[pool['group'][base] == group]
    mond = g if len(g) >= MIN_FOLD_GROUP_N else base
    d = np.linalg.norm(pool['h'][base] - h[None, :], axis=1)
    emb = base[np.argpartition(d, k_neighbors)[:k_neighbors]]
    return {'pooled': base, 'mondrian': mond, 'embedding': emb}


# ── Per-window evaluation ───────────────────────────────────────────────────

def evaluate_window(seq, mu_norm, h, group, scale, pool, map_data, drivable, rng, args,
                    exclude_trial=None, keep_draws=False):
    ref = _ref(seq)
    h_past, v0 = _past_heading_and_velocity(seq, map_data)
    objs = window_objects(seq, map_data, cfg.OUTPUT_SEQ_LEN)
    pidx = pool_indices(pool, h, group, exclude_trial, args.k_neighbors)
    u = rng.random(args.k_draws)   # common random numbers across variants
    out, draws = {}, {}
    for p in POOLS:
        j = pidx[p][(u * len(pidx[p])).astype(int)]
        for c in COUPLINGS:
            s = scale if c == 'adaptive' else 1.0
            traj = ref + (mu_norm[None] + s * _rot(pool['err_local'][j], h_past)) * R_POS
            feas = feasible_mask(traj, ref, v0)
            road_v, obj_v, road_min = score(traj, ref, h_past, objs, drivable)
            viol = road_v | obj_v
            v = f'{p}_{c}'
            nf = int(feas.sum())
            out[v] = {
                'p': float(viol[feas].mean()) if nf else float('nan'),
                'p_untrimmed': float(viol.mean()),
                'p_road': float(road_v[feas].mean()) if nf else float('nan'),
                'p_obj': float(obj_v[feas].mean()) if nf else float('nan'),
                'frac_trimmed': 1.0 - nf / args.k_draws,
            }
            if keep_draws:
                draws[v] = {'traj': traj, 'feas': feas, 'viol': viol}
    return out, draws, objs


# ── Mode: check-safe-set ────────────────────────────────────────────────────

def mode_check_safe_set(pool, map_data, drivable, args):
    """Acceptance test: the REALIZED nominal future is, by definition, a
    non-violating and physically feasible trajectory. Count how often each
    definition says otherwise."""
    ds = pool['ds']
    idx = np.arange(0, len(ds), args.nominal_stride)
    v1_viol = v2_road = v2_obj = infeas = 0
    road_mins = []
    for i in idx:
        seq = ds.sequences[i]
        ref = _ref(seq)
        fut = _future_xy(seq)
        h_past, v0 = _past_heading_and_velocity(seq, map_data)
        rv, _ov, rmin = score(fut[None], ref, h_past, (np.zeros((0, 30, 2)), np.zeros((0, 30)), np.zeros(0, int)), drivable)
        v2_road += int(rv[0])
        road_mins.append(float(rmin[0]))
        # realized objects at every 3rd future frame vs realized ego footprint
        ego_h = ego_headings(fut[None], ref, h_past)[0]
        hit = False
        for t in range(2, 30, 3):
            objs = _frame_objects(seq['future'][t], fut[t])
            if not objs:
                continue
            oxy = np.array([[o[0], o[1]] for o in objs])[:, None, :]
            ohd = np.array([[margin_lib.lanelet_heading(o[0], o[1], map_data)] for o in objs])
            ocl = np.array([o[3] for o in objs])
            if margin_lib.object_clearance_v2(fut[None, t:t + 1], ego_h[None, t:t + 1], oxy, ohd, ocl).min() < 0:
                hit = True
                break
        v2_obj += int(hit)
        infeas += int(~feasible_mask(fut[None], ref, v0)[0])
        # v1 margin, same windows (lane threshold 1.0m, worst-case objects), for the before/after
        objs_v1 = []
        for (x, y, sp, _c) in _frame_objects(seq['past'][-1], ref):
            objs_v1.append((x - ref[0], y - ref[1], sp))
        m = []
        for t in range(0, 30, 3):
            x, y = fut[t]
            ld = margin_lib.lane_boundary_distance(x, y, map_data)
            disp = float(np.hypot(*(fut[t] - fut[0])))
            m.append(margin_lib.margin(ld, [margin_lib.object_clearance(dx, dy, sp, t * DT, disp) for dx, dy, sp in objs_v1]))
        v1_viol += int(min(m) < 1.0)
    n = len(idx)
    res = {
        'n_windows': n,
        'v1_margin_realized_violation_rate': v1_viol / n,
        'v2_road_realized_violation_rate': v2_road / n,
        'v2_object_realized_violation_rate': v2_obj / n,
        'v2_realized_infeasible_rate': infeas / n,
        'v2_realized_road_margin_pct_1_5_50': np.percentile(road_mins, [1, 5, 50]).tolist(),
    }
    print("\n=== Safe-set acceptance test (realized NOMINAL futures, should all be ~0) ===")
    for k, v in res.items():
        print(f"  {k:<42} {v}")
    with open(os.path.join(args.output_dir, 'safe_set_check.json'), 'w') as f:
        json.dump(res, f, indent=2)
    return res


# ── Mode: nominal (LOO floor, sets matched-FAR thresholds) ─────────────────

def mode_nominal(pool, map_data, drivable, args):
    ds = pool['ds']
    idx = np.arange(0, len(ds), args.nominal_stride)
    rng = np.random.default_rng(args.seed)
    P = {v: np.full(len(idx), np.nan) for v in VARIANTS}
    trim = {v: np.full(len(idx), np.nan) for v in VARIANTS}
    print(f"\nNominal LOO floor: {len(idx)} windows, K={args.k_draws}...")
    for n_done, (r, i) in enumerate(zip(range(len(idx)), idx)):
        out, _, _ = evaluate_window(ds.sequences[i], pool['pos_mean'][i], pool['h'][i], pool['group'][i],
                                    pool['scale'][i], pool, map_data, drivable, rng, args,
                                    exclude_trial=pool['trial'][i])
        for v in VARIANTS:
            P[v][r] = out[v]['p']
            trim[v][r] = out[v]['frac_trimmed']
        if (n_done + 1) % 500 == 0:
            print(f"  {n_done + 1}/{len(idx)}")
    thr = {v: float(np.nanquantile(P[v], 1 - args.far)) for v in VARIANTS}
    scale_thr = float(np.quantile(pool['scale'], 1 - args.far))
    res = {'n_windows': int(len(idx)), 'far_budget': args.far, 'thresholds': thr,
           'layer1_scale_threshold': scale_thr, 'variants': {}}
    print(f"\n=== Nominal P(violation within {cfg.OUTPUT_SEQ_LEN * DT:.0f}s), leave-one-trial-out ===")
    print(f"{'variant':<22}{'mean':>8}{'p95':>8}{'p99':>8}{'max':>8}{'frac>0.1':>10}{'trimmed':>9}  threshold@FAR")
    for v in VARIANTS:
        a = P[v][~np.isnan(P[v])]
        row = {'mean': float(a.mean()), 'p95': float(np.quantile(a, .95)), 'p99': float(np.quantile(a, .99)),
               'max': float(a.max()), 'frac_above_0.1': float((a > 0.1).mean()),
               'mean_frac_trimmed': float(np.nanmean(trim[v]))}
        res['variants'][v] = row
        print(f"{v:<22}{row['mean']:8.4f}{row['p95']:8.3f}{row['p99']:8.3f}{row['max']:8.3f}"
              f"{row['frac_above_0.1']:10.4f}{row['mean_frac_trimmed']:9.3f}  {thr[v]:.3f}")
    with open(os.path.join(args.output_dir, 'nominal_floor.json'), 'w') as f:
        json.dump(res, f, indent=2)
    np.savez(os.path.join(args.output_dir, 'nominal_floor.npz'), idx=idx, trial=pool['trial'][idx],
             **{f'P_{v}': P[v] for v in VARIANTS})

    fig, ax = plt.subplots(figsize=(8, 4.5))
    for v, ls in zip(VARIANTS, ['-'] * 3 + ['--'] * 3):
        a = np.sort(P[v][~np.isnan(P[v])])
        ax.plot(a, 1 - np.arange(len(a)) / len(a), ls, label=v)
    ax.set_yscale('log')
    ax.set_xlabel('P(violation within 3s)')
    ax.set_ylabel('fraction of nominal windows with P >= x')
    ax.axhline(args.far, color='grey', lw=0.8)
    ax.set_title('Nominal false-alarm floor (leave-one-trial-out)')
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(args.output_dir, 'nominal_floor.png'), dpi=140)
    plt.close(fig)
    return res


# ── Mode: faults ────────────────────────────────────────────────────────────

def read_bag_xy(bag_dir, topic):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=''), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    if topic not in types:
        return None
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    T = get_message(types[topic])
    out = []
    while r.has_next():
        _, data, ts = r.read_next()
        p = deserialize_message(data, T).pose.pose.position
        out.append((ts * 1e-9, p.x, p.y))
    return np.array(out) if out else None


def _fault_trials(campaigns):
    out = []
    for c in campaigns:
        for run_dir in sorted(glob.glob(os.path.join(DATA_DIR, c, 'goal_*', 't*'))):
            if os.path.isdir(os.path.join(run_dir, 'rosbag')):
                out.append(run_dir)
    return out


def process_fault_trial(run_dir, model, device, pool, map_data, drivable, shared_builder, args):
    from inspect_fault_predictions import process_trial
    from fault_log import campaign_fault_kind
    campaign = run_dir.split(os.sep)[-3]
    kind = campaign_fault_kind(os.path.join(DATA_DIR, campaign))
    tr = process_trial(run_dir, campaign, shared_builder, kind)
    if tr is None:
        return None
    seqs, t_rel, t0 = tr['sequences'], tr['t_rel'], tr['t0']
    inf = infer(model, device, _SeqList(seqs), args.batch)
    refs = np.array([_ref(s) for s in seqs])
    groups = assign_groups(refs)
    # Causal Layer-1 anomaly: the window whose "now" was anomaly_lag_s ago
    j = np.searchsorted(t_rel, t_rel - args.anomaly_lag_s, side='right') - 1
    j[(j < 0) | (np.abs(t_rel[np.maximum(j, 0)] - (t_rel - args.anomaly_lag_s)) > 0.25)] = -1
    A = _ratio(inf['series'], pool['q'], j)
    scale = np.clip(np.nan_to_num(A / pool['qA'], nan=1.0), 1.0, args.scale_max)

    bag = os.path.join(run_dir, 'rosbag')
    gt = read_bag_xy(bag, GT_TOPIC)
    est = read_bag_xy(bag, EST_TOPIC)
    gt_rm = None
    if gt is not None:
        gt_t = gt[:, 0] - t0
        gt_rm = margin_lib.road_margin(drivable.signed_distance(gt[:, 1], gt[:, 2]))
    idx = np.arange(0, len(seqs), args.fault_stride)
    rng = np.random.default_rng(args.seed)
    rows = []
    for i in idx:
        out, _, _ = evaluate_window(seqs[i], inf['pos_mean'][i], inf['h'][i], groups[i], scale[i],
                                    pool, map_data, drivable, rng, args)
        row = {'t': float(t_rel[i]), 'scale': float(scale[i]), 'A': float(A[i]) if not np.isnan(A[i]) else None,
               'group': str(groups[i])}
        row.update({f'P_{v}': out[v]['p'] for v in VARIANTS})
        row.update({f'Proad_{v}': out[v]['p_road'] for v in VARIANTS})
        row.update({f'Puntrim_{v}': out[v]['p_untrimmed'] for v in VARIANTS})
        row.update({f'trim_{v}': out[v]['frac_trimmed'] for v in VARIANTS})
        # realized ground truth within H: GT road departure in (t, t+H]
        if gt_rm is not None:
            sel = (gt_t > t_rel[i]) & (gt_t <= t_rel[i] + cfg.OUTPUT_SEQ_LEN * DT)
            row['gt_viol_H'] = bool(sel.any() and gt_rm[sel].min() < 0)
        est_fut = _future_xy(seqs[i])
        row['est_viol_H'] = bool(margin_lib.road_margin(drivable.signed_distance(est_fut[:, 0], est_fut[:, 1])).min() < 0)
        rows.append(row)
    return {'run_dir': run_dir, 'campaign': campaign, 'kind': kind, 't_rel': t_rel, 't0': t0,
            'fault_windows': tr['fault_windows'], 'rows': rows, 'gt': gt, 'est': est, 'gt_rm': gt_rm,
            'seqs': seqs, 'inf': inf, 'groups': groups, 'scale': scale}


def summarize_trial(tr, thr, scale_thr, args):
    rows = tr['rows']
    t = np.array([r['t'] for r in rows])
    fw = tr['fault_windows']
    fault_start = min((w['start'] for w in fw), default=None)
    gt_dep = None
    if tr['gt_rm'] is not None and (tr['gt_rm'] < 0).any():
        gt_t = tr['gt'][:, 0] - tr['t0']
        gt_dep = float(gt_t[np.argmax(tr['gt_rm'] < 0)])
    in_fault = np.array([any(w['start'] <= x <= w['end'] for w in fw) for x in t])
    pre = t < (fault_start if fault_start is not None else np.inf)
    s = {'trial': os.path.relpath(tr['run_dir'], DATA_DIR), 'campaign': tr['campaign'], 'kind': tr['kind'],
         'fault_start': fault_start, 'gt_departure': gt_dep,
         'est_ever_departs': bool(any(r['est_viol_H'] for r in rows)), 'n_windows': len(rows)}
    alarms = {v: np.array([r[f'P_{v}'] > thr[v] for r in rows]) for v in VARIANTS}
    alarms['layer1_only'] = np.array([r['scale'] > scale_thr for r in rows])
    for name, a in alarms.items():
        d = {'pre_fault_alarm_rate': float(a[pre].mean()) if pre.any() else None,
             'in_fault_alarm_rate': float(a[in_fault].mean()) if in_fault.any() else None}
        if gt_dep is not None and fault_start is not None:
            win = (t >= fault_start) & (t <= gt_dep)
            first = t[win & a]
            d['first_alarm'] = float(first[0]) if len(first) else None
            d['lead_time_s'] = float(gt_dep - first[0]) if len(first) else None
        s[name] = d
    return s


def plot_featured(tr, summ, thr, scale_thr, args):
    rows = tr['rows']
    t = np.array([r['t'] for r in rows])
    name = summ['trial'].replace('/', '__')
    fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True,
                             gridspec_kw={'height_ratios': [3, 1.3, 1.3]})
    colors = {'pooled': '#7f7f7f', 'mondrian': '#2ca02c', 'embedding': '#9467bd'}
    ax = axes[0]
    for v in VARIANTS:
        p, c = v.split('_')
        ax.plot(t, [r[f'P_{v}'] for r in rows], color=colors[p], ls='-' if c == 'adaptive' else ':',
                lw=1.6 if c == 'adaptive' else 1.0, label=v)
    ax.plot(t, [float(r.get('gt_viol_H', False)) for r in rows], color='#d62728', lw=0, marker='|', ms=10,
            label='ground truth: real road departure within 3s')
    ax.set_ylabel('P(violation within 3s)')
    ax.set_ylim(-0.03, 1.05)
    ax.legend(fontsize=7, ncol=3, loc='upper left')
    axes[1].plot(t, [r['scale'] for r in rows], color='#1f77b4')
    axes[1].axhline(scale_thr, color='#1f77b4', ls='--', lw=0.8)
    axes[1].set_ylabel('Layer 1 anomaly\nscale s(t)')
    if tr['gt'] is not None and tr['est'] is not None:
        gt, est = tr['gt'], tr['est']
        gi = np.searchsorted(gt[:, 0], est[:, 0]).clip(0, len(gt) - 1)
        axes[2].plot(est[:, 0] - tr['t0'], np.hypot(est[:, 1] - gt[gi, 1], est[:, 2] - gt[gi, 2]), 'k',
                     label='|Autoware estimate - ground truth| (m)')
        axes[2].plot(gt[:, 0] - tr['t0'], tr['gt_rm'], color='#d62728', label='ground-truth road margin (m)')
        axes[2].axhline(0, color='grey', lw=0.7)
        axes[2].set_ylim(-8, 22)
        axes[2].legend(fontsize=7, loc='upper left')
    for a in axes:
        for w in tr['fault_windows']:
            a.axvspan(w['start'], w['end'], color='#ff7f0e', alpha=0.12)
        if summ['gt_departure'] is not None:
            a.axvline(summ['gt_departure'], color='#d62728', ls='--', lw=1.2)
    axes[2].set_xlabel('seconds since trial start  (orange = fault active, red dashed = real road departure)')
    lt = summ['embedding_adaptive'].get('lead_time_s')
    fig.suptitle(f"{summ['trial']} — Layer 2 P(violation) vs ground truth"
                 + (f"  (embedding_adaptive lead time {lt:.1f}s)" if lt is not None else ''), fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    path = os.path.join(args.output_dir, f'p_violation_{name}.png')
    fig.savefig(path, dpi=130)
    plt.close(fig)
    print(f"  Saved {path}")


def plot_cascade(tr, summ, pool, map_data, drivable, args):
    """Envelope cascade at one frame: the same K draws, pooled -> Mondrian
    -> embedding (2a) -> embedding + Layer-1 coupling, with the 2b trim
    (grey) and safe-set verdict (red/green) drawn per counterfactual."""
    rows = tr['rows']
    t = np.array([r['t'] for r in rows])
    target = summ['gt_departure'] - 1.5 if summ['gt_departure'] is not None else t[int(np.argmax([r['P_embedding_adaptive'] for r in rows]))]
    i = int(np.searchsorted(tr['t_rel'], target))
    i = min(i, len(tr['seqs']) - 1)
    seq = tr['seqs'][i]
    rng = np.random.default_rng(args.seed)
    out, draws, objs = evaluate_window(seq, tr['inf']['pos_mean'][i], tr['inf']['h'][i], tr['groups'][i],
                                       tr['scale'][i], pool, map_data, drivable, rng, args, keep_draws=True)
    ref = _ref(seq)
    past = _past_xy(seq)
    mu = ref + tr['inf']['pos_mean'][i] * R_POS
    panels = ['pooled_static', 'mondrian_static', 'embedding_static', 'embedding_adaptive']
    titles = ['raw pooled tube (v1)', '2a: Mondrian pool', '2a: embedding k-NN pool',
              f'+ Layer-1 coupling (s={tr["scale"][i]:.2f}) + 2b trim']
    allpts = np.concatenate([draws[v]['traj'][draws[v]['feas']].reshape(-1, 2) for v in panels] + [past, mu])
    lo, hi = np.percentile(allpts, 1, axis=0) - 12, np.percentile(allpts, 99, axis=0) + 12
    if tr['gt'] is not None:
        tg = tr['gt'][:, 0] - tr['t0']
        gsel = (tg >= tr['t_rel'][i] - 3) & (tg <= tr['t_rel'][i] + 3)
    fig, axes = plt.subplots(1, 4, figsize=(22, 6.2))
    i0, j0 = int((lo[0] - drivable.x0) / drivable.res), int((lo[1] - drivable.y0) / drivable.res)
    i1, j1 = int((hi[0] - drivable.x0) / drivable.res), int((hi[1] - drivable.y0) / drivable.res)
    for ax, v, ttl in zip(axes, panels, titles):
        ax.imshow(drivable.mask[j0:j1, i0:i1], origin='lower', extent=[lo[0], hi[0], lo[1], hi[1]],
                  cmap='Greys', vmin=0, vmax=2.5)
        d = draws[v]
        for k in range(len(d['feas'])):
            col = '#bbbbbb' if not d['feas'][k] else ('#d62728' if d['viol'][k] else '#2ca02c')
            ax.plot(d['traj'][k, :, 0], d['traj'][k, :, 1], color=col, lw=0.6, alpha=0.5 if d['feas'][k] else 0.3)
        ax.plot(past[:, 0], past[:, 1], 'k-', lw=2, label='past (Autoware estimate)')
        ax.plot(mu[:, 0], mu[:, 1], 'b-', lw=2, label='ST-GAT point prediction')
        if tr['gt'] is not None:
            ax.plot(tr['gt'][gsel, 1], tr['gt'][gsel, 2], color='#ff7f0e', lw=2, ls='--', label='ground truth ±3s')
        for m in range(len(objs[2])):
            ax.plot(objs[0][m, :, 0], objs[0][m, :, 1], 'm-', lw=1)
        o = out[v]
        ax.set_title(f"{ttl}\nP(viol)={o['p']:.2f}   trimmed={o['frac_trimmed']:.0%}", fontsize=10)
        ax.set_xlim(lo[0], hi[0])
        ax.set_ylim(lo[1], hi[1])
        ax.set_aspect('equal')
        ax.tick_params(labelsize=7)
    axes[0].legend(fontsize=7, loc='lower left')
    fig.suptitle(f"{summ['trial']} at t={tr['t_rel'][i]:.1f}s  — green: feasible & safe, red: feasible & violating, "
                 f"grey: trimmed (kinematically infeasible)", fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.93])
    path = os.path.join(args.output_dir, f"cascade_{summ['trial'].replace('/', '__')}.png")
    fig.savefig(path, dpi=120)
    plt.close(fig)
    print(f"  Saved {path}")


def mode_faults(model, device, pool, map_data, drivable, nominal_res, args):
    from st_gat.pipeline.State_Estimator.MapProcessor import MapProcessor
    from st_gat.pipeline.sequence_builder import SequenceBuilder
    thr = nominal_res['thresholds']
    scale_thr = nominal_res['layer1_scale_threshold']
    shared_builder = SequenceBuilder(MapProcessor(cfg.MAP_FILE).map_data, route=[])
    trial_dirs = _fault_trials(args.campaigns)
    if args.trials:
        trial_dirs = [d for d in trial_dirs if any(os.path.relpath(d, DATA_DIR).startswith(x) for x in args.trials)]
    print(f"\nFault trials: {len(trial_dirs)} across {len(args.campaigns)} campaigns")
    summ_path = os.path.join(args.output_dir, 'fault_summary.json')
    summaries = []
    if os.path.exists(summ_path) and not args.no_cache:
        with open(summ_path) as f:
            summaries = json.load(f)
    done = {s['trial'] for s in summaries}
    for run_dir in trial_dirs:
        rel = os.path.relpath(run_dir, DATA_DIR)
        if rel in done:
            print(f"[{rel}] cached, skipping")
            continue
        print(f"\n[{rel}]")
        tr = process_fault_trial(run_dir, model, device, pool, map_data, drivable, shared_builder, args)
        if tr is None:
            summaries.append({'trial': rel, 'campaign': rel.split('/')[0], 'skipped': True})
            continue
        s = summarize_trial(tr, thr, scale_thr, args)
        summaries.append(s)
        print(f"  fault_start={s['fault_start']}  gt_departure={s['gt_departure']}  "
              f"est_ever_departs={s['est_ever_departs']}")
        for v in VARIANTS + ['layer1_only']:
            print(f"    {v:<22} {s[v]}")
        with open(os.path.join(args.output_dir, f"trial_{rel.replace('/', '__')}.json"), 'w') as f:
            json.dump({'summary': s, 'fault_windows': tr['fault_windows'], 'rows': tr['rows']}, f)
        if s['gt_departure'] is not None or rel.startswith(tuple(args.featured)):
            plot_featured(tr, s, thr, scale_thr, args)
            plot_cascade(tr, s, pool, map_data, drivable, args)
        with open(summ_path, 'w') as f:
            json.dump(summaries, f, indent=2)
        del tr
    print_fault_table(summaries)
    return summaries


def print_fault_table(summaries):
    print(f"\n{'=' * 100}\nFAULT TRIALS — alarm at matched nominal FAR; lead time vs REAL (ground-truth) road departure\n{'=' * 100}")
    print(f"{'trial':<46}{'GT dep':>8}{'L1-only lead':>13}{'emb_stat lead':>14}{'emb_adapt lead':>15}{'pool_adapt lead':>16}")
    for s in summaries:
        if s.get('skipped'):
            print(f"{s['trial']:<46}  (skipped: could not process)")
            continue
        def lt(v):
            x = s[v].get('lead_time_s')
            return f"{x:.1f}" if x is not None else ('miss' if s['gt_departure'] is not None else '-')
        dep = f"{s['gt_departure']:.1f}" if s['gt_departure'] is not None else '-'
        print(f"{s['trial']:<46}{dep:>8}{lt('layer1_only'):>13}{lt('embedding_static'):>14}"
              f"{lt('embedding_adaptive'):>15}{lt('pooled_adaptive'):>16}")
    print(f"\nIn-fault alarm rate (fraction of fault-active windows alarmed) / pre-fault alarm rate:")
    print(f"{'trial':<46}" + ''.join(f"{v:>20}" for v in ['layer1_only', 'embedding_static', 'embedding_adaptive']))
    for s in summaries:
        if s.get('skipped'):
            continue
        def r(v):
            a, b = s[v]['in_fault_alarm_rate'], s[v]['pre_fault_alarm_rate']
            return f"{(a if a is not None else float('nan')):.2f} / {(b if b is not None else float('nan')):.2f}"
        print(f"{s['trial']:<46}" + ''.join(f"{r(v):>20}" for v in ['layer1_only', 'embedding_static', 'embedding_adaptive']))


# ── main ────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--mode', choices=['check-safe-set', 'nominal', 'faults', 'all'], default='all')
    ap.add_argument('--model', default=cfg.MODEL_CONFIG['model_path'])
    ap.add_argument('--batch', type=int, default=256)
    ap.add_argument('--k-draws', type=int, default=200, help='bootstrap counterfactuals per window')
    ap.add_argument('--k-neighbors', type=int, default=150, help='embedding pool size (as conformal_embedding_calibration.py)')
    ap.add_argument('--nominal-stride', type=int, default=5, help='analyze every Nth nominal CAL window')
    ap.add_argument('--fault-stride', type=int, default=2, help='analyze every Nth fault-trial window')
    ap.add_argument('--anomaly-lag-s', type=float, default=1.0,
                    help='Layer-1 coupling looks at the prediction made this long ago (its first lag_s are observed now)')
    ap.add_argument('--scale-max', type=float, default=10.0)
    ap.add_argument('--far', type=float, default=0.01, help='nominal window-level false-alarm budget for thresholds')
    ap.add_argument('--campaigns', nargs='+', default=None, help='default: every fault campaign on disk')
    ap.add_argument('--trials', nargs='*', default=None, help='restrict to trials whose campaign/goal/run starts with these')
    ap.add_argument('--featured', nargs='*', default=[], help='also plot these trials even without a GT departure')
    ap.add_argument('--no-cache', action='store_true')
    ap.add_argument('--seed', type=int, default=20261004)
    ap.add_argument('--output-dir', default=DEFAULT_OUTPUT_DIR)
    args = ap.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)
    if args.campaigns is None:
        args.campaigns = sorted(c for c in os.listdir(DATA_DIR)
                                if os.path.isdir(os.path.join(DATA_DIR, c)) and not c.startswith('nom'))

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = _load_model(args.model, device)
    map_data = margin_lib.load_map(cfg.MAP_FILE)
    drivable = margin_lib.DrivableArea(map_data)
    pool = build_cal_pool(model, device, map_data, args)

    if args.mode in ('check-safe-set', 'all'):
        mode_check_safe_set(pool, map_data, drivable, args)
    nominal_res = None
    if args.mode in ('nominal', 'all'):
        nominal_res = mode_nominal(pool, map_data, drivable, args)
    if args.mode in ('faults', 'all'):
        if nominal_res is None:
            with open(os.path.join(args.output_dir, 'nominal_floor.json')) as f:
                nominal_res = json.load(f)
        mode_faults(model, device, pool, map_data, drivable, nominal_res, args)


if __name__ == '__main__':
    main()
