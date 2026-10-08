#!/usr/bin/env python3
"""
Layer 2 — the safety envelope as specified in docs/architecture.md
(2026-10-07). Needs a multi-source model (STGAT_GRAPH_FRAME=ego_v4).

Per window (time t, 10 Hz):

  Layer 1 evidence (causal: only already-observed steps of earlier forecasts)
    behavioural   L_recent / H_recent: signed lateral / heading residuals of
                  the most recent observed step of forecasts made in the last
                  1 s (residual_triangle_check.py)        -> WHEN
    consistency   e(t) = observed - predicted cross-source gap, most recent
                  observed step of forecasts made in the last 0.5 s:
                  [EKF - vehicle yaw rate (rad/s), EKF - wheel speed (m/s)]
                                                          -> HOW MUCH, WHICH WAY
    onset         every channel (consistency, lateral, heading, along-track,
                  speed, traffic-light) -> conformal p-value vs nominal;
                  Bonferroni-combined into a bounded score u; capped CUSUM on
                  u (h: 1% of nominal windows distrusted, cap 2h); distrust ends
                  when the last 2 s look nominal (scan-count recovery test)
    carried error the pose error integrated from the change-point is kept
                  after distrust ends, until it is back inside the nominal
                  drift band of the integrator (distribution-free spread)

  2a nominal envelope   K whole-trajectory counterfactuals = forecast + one
                  recorded error trajectory (ego frame), drawn from the 150
                  calibration windows nearest in the model's scene embedding
                  (other trials only). [lane-keeping prior: not yet]

  2b evidence growth    while distrusted, integrate from onset:
                  Psi = int e_yaw dt            (estimated - true heading)
                  X   = int e_speed dt          (estimated - true along-track)
                  Y   = int v sin(Psi) dt       (estimated - true lateral)
                  spread = integrated calibrated band (sigma_e * tau);
                  every counterfactual is rigidly moved to a sampled true
                  pose and keeps drifting at the current measured rate.
                  P(violation) therefore grows the longer a fault lasts
                  unmitigated; slowing down slows the lateral growth.

  2c safe sets  road departure (drivable-area SDF, footprint) + tracked
                objects (moved by the same pose correction); P = share of
                counterfactuals that violate.

Variants reported: nominal (2a only), evidence (2a+2b, the method),
deadreckon (hand-built bicycle-model consistency, baseline), layer1 (the
distrust flag alone, no map). Thresholds at a 1% nominal false-alarm rate
(leave-one-trial-out on CAL); lead time = onset of the alarm episode still
active at the real (AWSIM ground-truth) road departure.

Outputs (cfg.ANALYSIS_ROOT/layer2_envelope/): nominal_floor.json,
fault_summary.json, per-trial timeseries + filmstrip PNGs, one GIF.

Usage (ROS + venv):
  STGAT_GRAPH_FRAME=ego_v4 python3 experiments/scripts/layer2_envelope.py [--mode all]
"""

import argparse
import json
import math
import os
import sys

import numpy as np
import torch

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
sys.path[:0] = [REPO_DIR, SCRIPT_DIR, os.path.join(REPO_DIR, 'experiments', 'lib')]

import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

from st_gat.pipeline import config as cfg  # noqa: E402
from st_gat.model import TrajectoryDataset  # noqa: E402
import layer2_consequence_estimation as L2  # noqa: E402  (shared, tested helpers)
import residual_triangle_check as RTC  # noqa: E402
from conformal_horizon_calibration import _trial_ids_for  # noqa: E402

DT = 0.1
R_POS = cfg.POSITION_DISPLACEMENT_RANGE_M
K_RECENT_CONS = 5        # consistency evidence: forecasts made in the last 0.5 s
VARIANTS = ['nominal', 'evidence', 'deadreckon']
ALARMS = VARIANTS + ['evidence_dP']      # evidence_dP: P(evidence) - P(nominal), the risk attributable to the evidence
OUT_DIR = os.path.join(cfg.ANALYSIS_ROOT, 'layer2_envelope')
FEATURED = ['imu_fault_scale/goal_007', 'imu_fault_stuck/goal_007', 'imu_fault_s3/goal_007',
            'imu_fault_stuck/goal_012', 'imu_fault_s1/goal_007', 'imu_fault_s3/goal_012',
            'tl_fault_fixed_070/goal_007/t1']
GIF_TRIAL = 'imu_fault_scale/goal_007'


# ── Layer 1 evidence ────────────────────────────────────────────────────────

def recent_consistency(cons_obs, cons_pred, prev_ok):
    """(N,2) signed consistency residual: mean over forecasts made 1..K steps
    ago of the residual at their most recent observed step (step k-1)."""
    rc = cons_obs - cons_pred
    N, C = rc.shape[0], rc.shape[-1]
    acc, cnt = np.zeros((N, C)), np.zeros(N)
    for k in range(1, K_RECENT_CONS + 1):
        ok = prev_ok[k]
        src = np.arange(N) - k
        acc[ok] += rc[src[ok], k - 1]
        cnt[ok] += 1
    out = acc / np.maximum(cnt, 1)[:, None]
    out[cnt == 0] = np.nan
    return out


def window_speed(seqs):
    lo, hi = cfg.VELOCITY_X_RANGE
    return np.array([s['past'][-1]['velocity'][0] * (hi - lo) + lo for s in seqs])


CHANNEL_NAMES = ['cons_ekf_yaw', 'cons_ekf_speed', 'cons_pose_yaw', 'cons_pose_along', 'cons_pose_lat',
                 'lateral', 'heading', 'along', 'speed', 'tl_color', 'tl_confidence']
K_RECENT_BEH = 10        # behavioural evidence: forecasts made in the last 1 s
RECOVERY_W_S = 2.0       # recovery test window
POS_WINDOW_S = 3.0       # along/lateral pose error: sliding-window integral (see sliding_position_error)
ALPHA_FLAG = 0.01        # a window is "evidence" when the Bonferroni-combined p < this


def recent_at(X, prev_ok, K):
    """(N,T) per-forecast, per-step array -> (N,) mean over forecasts made
    1..K steps ago of their most recent observed step (k-1). NaN if none."""
    N = X.shape[0]
    acc, cnt = np.zeros(N), np.zeros(N)
    for k in range(1, K + 1):
        ok = prev_ok[k]
        src = np.arange(N) - k
        v = X[src[ok], k - 1]
        good = ~np.isnan(v)
        idx = np.where(ok)[0][good]
        acc[idx] += v[good]
        cnt[idx] += 1
    out = acc / np.maximum(cnt, 1)
    out[cnt == 0] = np.nan
    return out


def evidence_channels(e, comp, series, prev_ok):
    """(N, C) absolute evidence per channel: consistency (recent, signed ->
    abs), behavioural lateral / heading / along-track (recent signed -> abs),
    and the longitudinal-speed and traffic-light heads (recent residual
    magnitude). Channels the model doesn't have are NaN (p = 1)."""
    N = e.shape[0]
    cols = [np.abs(e[:, c]) if c < e.shape[1] else np.full(N, np.nan) for c in range(5)]
    for key in ('lat', 'head', 'along'):
        cols.append(np.abs(recent_at(comp[key], prev_ok, K_RECENT_BEH)))
    for key in ('velocity_longitudinal', 'traffic_light_color', 'traffic_light_confidence'):
        cols.append(recent_at(series[key], prev_ok, K_RECENT_BEH) if key in series else np.full(N, np.nan))
    return np.stack(cols, 1)


def pvalues(X, nominal_sorted):
    """Conformal p-value per window and channel: (1 + #nominal >= x) / (n + 1).
    NaN -> 1 (no evidence)."""
    P = np.ones_like(X)
    for c, ref in enumerate(nominal_sorted):
        if ref is None or not len(ref):
            continue
        x = X[:, c]
        ok = ~np.isnan(x)
        ge = len(ref) - np.searchsorted(ref, x[ok], side='left')
        P[ok, c] = (1 + ge) / (len(ref) + 1)
    return P


def combined_score(P):
    """Bonferroni across channels (valid under any dependence) -> bounded
    score u = -log(min(1, C * min_c p)) in [0, log(n+1)]; also the combined p."""
    pc = np.minimum(1.0, P.shape[1] * P.min(1))
    return -np.log(pc), pc


def capped_cusum(t, u, kappa, cap, reset_gap=1.0):
    S = np.zeros(len(t))
    for i in range(1, len(t)):
        S[i] = 0.0 if t[i] - t[i - 1] > reset_gap else min(cap, max(0.0, S[i - 1] + u[i] - kappa))
    return S


def scan_count(t, flag, w=RECOVERY_W_S):
    """Number of flagged windows in the last w seconds (causal)."""
    out = np.zeros(len(t))
    j = 0
    run = 0
    for i in range(len(t)):
        run += flag[i]
        while t[i] - t[j] > w:
            run -= flag[j]
            j += 1
        out[i] = run
    return out


def _window_xy(t, e_along, e_lat, v, psi_hist, i, w=POS_WINDOW_S):
    """sliding_position_error evaluated at window i only (causal)."""
    j = i
    while j > 0 and t[i] - t[j - 1] < w:
        j -= 1
    dt = np.r_[DT, np.clip(np.diff(t[j:i + 1]), 0, 0.3)] if i > j else np.array([DT])
    X = float(np.sum(e_along[j:i + 1] * dt))
    Y = float(np.sum((e_lat[j:i + 1] + v[j:i + 1] * np.sin(psi_hist[j:i + 1])) * dt))
    return X, Y


def drift_band(bands, tau, key):
    """Nominal 95% band of the integrator at time-since-onset tau (linear
    extrapolation past the calibrated range)."""
    taus, vals = bands['tau'], bands[key]
    if tau <= taus[-1]:
        return float(np.interp(tau, taus, vals))
    return float(vals[-1] * tau / taus[-1])


def run_integrator(e_int, v, t):
    """Plain integration of the pose-consistency channels from the first
    window: (Psi, X, Y) per window -- used for the nominal drift bands.
    e_int columns: yaw gap, along gap[, lateral gap]; without a measured
    lateral gap, Y falls back to int v sin(Psi)."""
    n = len(t)
    Psi = np.zeros(n); X = np.zeros(n); Y = np.zeros(n)
    for i in range(1, n):
        dt = min(t[i] - t[i - 1], 0.3)
        Psi[i] = Psi[i - 1] + np.nan_to_num(e_int[i, 0]) * dt
        X[i] = X[i - 1] + np.nan_to_num(e_int[i, 1]) * dt
        Y[i] = Y[i - 1] + (np.nan_to_num(e_int[i, 2]) if e_int.shape[1] > 2 else v[i] * math.sin(Psi[i])) * dt
    if e_int.shape[1] > 2:
        X, Y = sliding_position_error(t, np.nan_to_num(e_int[:, 1]), np.nan_to_num(e_int[:, 2]), v, Psi)
    return Psi, X, Y


def sliding_position_error(t, e_along, e_lat, v, Psi, w=POS_WINDOW_S):
    """Along-track / lateral pose error from the measured pose-velocity gaps,
    as a sliding w-second integral (the error accumulated recently).
    Lateral rate = measured lateral gap + v sin(Psi): the LiDAR-corrected
    position tracks the true one, but its displacement is projected on the
    (wrong) estimated heading, so a pure heading error shows up as a lateral
    gap of -v sin(Psi); adding it back leaves the position error. Integrating
    since onset drifts (velocity biases of ~0.1 m/s become metres while LiDAR
    keeps the true position error bounded); a sliding window does not.
    Checked against AWSIM truth (2026-10-08): imu_fault_s1 stays within
    +/-1.3 m (true ~0); stuck +8.9 m (true +7.1), scale -3.2 (-4.2), s3 +1.7 (+3.0)."""
    n = len(t)
    dt = np.r_[DT, np.clip(np.diff(t), 0, 0.3)]
    ra = e_along * dt
    rl = (e_lat + v * np.sin(Psi)) * dt
    X, Y = np.zeros(n), np.zeros(n)
    j = 0
    sa = sl = 0.0
    for i in range(n):
        sa += ra[i]; sl += rl[i]
        while t[i] - t[j] >= w:
            sa -= ra[j]; sl -= rl[j]
            j += 1
        X[i], Y[i] = sa, sl
    return X, Y


def evidence_state(t, u, flag, e, v, cal):
    """Layer-1 WHEN + 2b carried pose error over one trial's windows, causal.
    - distrust ON when the capped CUSUM S of the bounded score u exceeds h;
      onset (change-point) = last time S was 0
    - distrust OFF when the last RECOVERY_W_S look nominal (scan count of
      flagged windows <= n_off) and S < h; S is reset to 0
    - the pose error (Psi, X, Y) is integrated from the onset with the
      pose-consistency channels and CARRIED after distrust ends, until it is
      back inside the nominal drift band (a LiDAR correction shows up in that
      channel with the opposite sign, so the integral comes back by itself)
    - spread = the nominal drift band at time-since-onset (distribution-free)
    The envelope is moved while distrusted or carrying."""
    n = len(t)
    o = {k: np.zeros(n) for k in ('D', 'S', 'N', 'active', 'tau', 'Psi', 'X', 'Y',
                                  'sPsi', 'sX', 'sY', 'rate_yaw', 'rate_spd')}
    iy, isp = cal['int_ch'][:2]
    ey, es = np.nan_to_num(e[:, iy]), np.nan_to_num(e[:, isp])
    elat = np.nan_to_num(e[:, cal['int_ch'][2]]) if len(cal['int_ch']) > 2 else None
    N = scan_count(t, flag)
    o['N'] = N
    S = np.zeros(n)
    distrusted, carrying = False, False
    psi_hist = np.zeros(n)
    last_zero = t[0]
    onset = None
    Psi = X = Y = 0.0
    for i in range(n):
        if i and t[i] - t[i - 1] > 1.0:
            distrusted = carrying = False
            Psi = X = Y = 0.0
        if i:
            S[i] = 0.0 if t[i] - t[i - 1] > 1.0 else min(cal['cap'], max(0.0, S[i - 1] + u[i] - cal['kappa']))
        o['S'][i] = S[i]
        if S[i] <= 0:
            last_zero = t[i]
        if not distrusted and S[i] > cal['h']:
            distrusted = True
            if not carrying:
                onset, Psi, X, Y = last_zero, 0.0, 0.0, 0.0
                # back-fill the integral from the change-point to now
                for j in range(i, -1, -1):
                    if t[j] < last_zero:
                        break
                    dtj = min(t[j] - t[j - 1], 0.3) if j else DT
                    Psi += ey[j] * dtj
                    X += es[j] * dtj
                    if elat is not None:
                        Y += elat[j] * dtj
                if elat is None:
                    Y = v[i] * math.sin(Psi) * (t[i] - onset) / 2.0
            carrying = True
        elif distrusted and N[i] <= cal['n_off'] and S[i] < cal['h']:
            distrusted = False
            S[i] = 0.0          # explicit reset on recovery
        if carrying and i:
            dt = min(t[i] - t[i - 1], 0.3)
            Psi += ey[i] * dt
            X += es[i] * dt
            Y += (elat[i] if elat is not None else v[i] * math.sin(Psi)) * dt
        psi_hist[i] = Psi if carrying else 0.0
        if carrying and elat is not None:
            X, Y = _window_xy(t, es, elat, v, psi_hist, i)
        if carrying:
            tau = t[i] - onset + DT
            bP, bY = drift_band(cal['bands'], tau, 'Psi'), drift_band(cal['bands'], tau, 'Y')
            if not distrusted and abs(Psi) <= bP and abs(Y) <= bY:
                carrying = False      # back to nominal: release the carried error
                Psi = X = Y = 0.0
                continue
            o['active'][i] = 1.0
            o['D'][i] = float(distrusted)
            o['tau'][i] = tau
            o['Psi'][i], o['X'][i], o['Y'][i] = Psi, X, Y
            o['sPsi'][i] = bP / 1.96
            o['sY'][i] = bY / 1.96
            o['sX'][i] = drift_band(cal['bands'], tau, 'X') / 1.96
            o['rate_yaw'][i], o['rate_spd'][i] = ey[i], es[i]
    return o


# ── 2a + 2b envelope ────────────────────────────────────────────────────────

def nominal_bundle(seq, mu_norm, err_local_draws):
    ref = L2._ref(seq)
    yaw = seq['past'][-1]['yaw']
    return ref + (mu_norm[None] + L2._rot(err_local_draws, yaw)) * R_POS


def evidence_bundle(base, ref, yaw, ev, i, se_rate, rng):
    """2b: rigid move of every counterfactual to a sampled true pose
    (estimated - true = Psi, X, Y), continuing to drift at the measured
    rates. tau == 0 -> unchanged."""
    K, T = base.shape[:2]
    if ev['tau'][i] <= 0:
        return base, np.zeros((K, 2)), np.zeros(K)
    psi0 = ev['Psi'][i] + rng.normal(0, max(ev['sPsi'][i], 1e-6), K)
    rate = ev['rate_yaw'][i] + rng.normal(0, se_rate[0], K)
    lat = ev['Y'][i] + rng.normal(0, max(ev['sY'][i], 1e-6), K)
    alo = ev['X'][i] + rng.normal(0, max(ev['sX'][i], 1e-6), K)
    srate = ev['rate_spd'][i] + rng.normal(0, se_rate[1], K)
    t = (np.arange(1, T + 1) * DT)[None, :]
    dpsi = -(psi0[:, None] + rate[:, None] * t)
    u = np.array([math.cos(yaw), math.sin(yaw)])
    n = np.array([-math.sin(yaw), math.cos(yaw)])
    shift = -lat[:, None] * n[None, :] - alo[:, None] * u[None, :]                 # (K,2)
    drift = -(srate[:, None] * t)[..., None] * u[None, None, :]                   # (K,T,2)
    rel = base - ref
    c, s_ = np.cos(dpsi), np.sin(dpsi)
    rot = np.stack([c * rel[..., 0] - s_ * rel[..., 1], s_ * rel[..., 0] + c * rel[..., 1]], -1)
    return ref + shift[:, None, :] + drift + rot, shift, -psi0


def evaluate(seq, mu_norm, h, pool, ev, i, drivable, map_data, rng, args, exclude_trial=None, keep=False):
    ref = L2._ref(seq)
    yaw = seq['past'][-1]['yaw']
    h_past, _v0 = L2._past_heading_and_velocity(seq, map_data)
    objs = L2.window_objects(seq, map_data, cfg.OUTPUT_SEQ_LEN)
    ok = pool['trial'] != exclude_trial if exclude_trial is not None else np.ones(len(pool['trial']), bool)
    base_idx = np.where(ok)[0]
    d = np.linalg.norm(pool['h'][base_idx] - h[None, :], axis=1)
    nn = base_idx[np.argpartition(d, args.k_neighbors)[:args.k_neighbors]]
    j = nn[rng.integers(0, len(nn), args.k_draws)]
    seed = int(rng.integers(1 << 31))
    base = nominal_bundle(seq, mu_norm, pool['err_local'][j])
    bundles = {'nominal': (base, objs)}
    tr, sh, dp = evidence_bundle(base, ref, yaw, ev, i, pool['se_rate'], np.random.default_rng(seed))
    bundles['evidence'] = (tr, L2._move_objects(objs, ref, sh, dp) if ev['tau'][i] > 0 else objs)
    pe = L2.pose_error(seq)
    tr, sh, dp = L2.pose_counterfactuals(base, ref, yaw, pe, pool['pose_sigma'], np.random.default_rng(seed + 1))
    bundles['deadreckon'] = (tr, L2._move_objects(objs, ref, sh, dp))
    out, draws = {}, {}
    for name, (traj, ob) in bundles.items():
        road_v, obj_v, _ = L2.score(traj, ref, h_past, ob, drivable)
        viol = road_v | obj_v
        out[name] = float(viol.mean())
        out[f'{name}_road'] = float(road_v.mean())
        if keep:
            draws[name] = (traj, viol)
    out['psi_dr'] = pe[0]
    return out, draws, objs


# ── calibration pool ────────────────────────────────────────────────────────

def _layer1_raw(inf, seqs, prev_ok, tri_q=None):
    """Raw Layer-1 evidence for a window sequence: signed consistency
    residuals e (N,C) and the absolute channel matrix (N, len(CHANNEL_NAMES))."""
    e = recent_consistency(inf['cons_obs'], inf['cons_pred'], prev_ok)
    lat, along, head = RTC.residual_components(seqs, inf['pos_mean'])
    comp = {'lat': lat, 'along': along, 'head': head}
    series = {k[len('series_'):]: v for k, v in inf.items() if k.startswith('series_')}
    return e, evidence_channels(e, comp, series, prev_ok)


def _flatten_series(inf):
    for k, v in inf.pop('series', {}).items():
        inf[f'series_{k}'] = v
    return inf


def build_pool(model, device, args):
    cache = os.path.join(args.output_dir, f"cal_pool_{int(os.path.getmtime(args.model))}.npz")   # keyed on the checkpoint
    ds = TrajectoryDataset(cfg.CAL_DIR)
    trial, _ = _trial_ids_for(cfg.CAL_DIR)
    if os.path.exists(cache) and not args.no_cache:
        z = np.load(cache, allow_pickle=True)
        inf = {k: z[k] for k in z.files}
    else:
        inf = _flatten_series(L2.infer(model, device, ds, args.batch))
        np.savez(cache, **inf)
    assert 'cons_pred' in inf, "model has no consistency head -- run with STGAT_GRAPH_FRAME=ego_v4/ego_v5"
    seqs = ds.sequences
    yaw = np.array([s['past'][-1]['yaw'] for s in seqs])
    pool = {'trial': trial, 'h': inf['h'], 'pos_mean': inf['pos_mean'],
            'err_local': L2._rot(inf['pos_err'], -yaw), 'seqs': seqs}
    prev_ok = RTC.prev_ok_from_trial(trial)
    e, Xch = _layer1_raw(inf, seqs, prev_ok)
    t_all = np.zeros(len(trial))
    for tid in np.unique(trial):
        m = np.where(trial == tid)[0]
        t_all[m] = np.arange(len(m)) * DT
    # per-channel nominal references (for fault trials) and leave-one-trial-out
    # p-values for the calibration windows themselves
    ref_all = [np.sort(Xch[~np.isnan(Xch[:, c]), c]) for c in range(Xch.shape[1])]
    Pn = np.ones_like(Xch)
    for tid in np.unique(trial):
        m = trial == tid
        ref_loo = [np.sort(Xch[(~m) & ~np.isnan(Xch[:, c]), c]) for c in range(Xch.shape[1])]
        Pn[m] = pvalues(Xch[m], ref_loo)
    u, pc = combined_score(Pn)
    flag = (pc < ALPHA_FLAG).astype(float)
    int_ch = (2, 3, 4) if e.shape[1] >= 5 else ((2, 3) if e.shape[1] >= 4 else (0, 1))
    cal = {'ref': ref_all, 'int_ch': int_ch,
           'q90': np.nanquantile(np.abs(e), 0.90, axis=0),
           'q99': np.nanquantile(np.abs(e), 0.99, axis=0),
           'kappa': float(np.quantile(u, args.kappa_q))}
    # h: capped CUSUM above h on args.distrust_rate of nominal windows (the cap
    # = 2h only acts above h, so it doesn't change that share)
    S_unc = np.concatenate([capped_cusum(t_all[trial == tid], u[trial == tid], cal['kappa'], np.inf)
                            for tid in np.unique(trial)])
    cal['h'] = max(float(np.quantile(S_unc, 1 - args.distrust_rate)), 1e-6)
    cal['cap'] = 2.0 * cal['h']
    Nall = np.concatenate([scan_count(t_all[trial == tid], flag[trial == tid]) for tid in np.unique(trial)])
    cal['n_off'] = float(np.quantile(Nall, 0.95))
    # nominal drift bands of the 2b integrator: integrate the pose channels
    # from start times every 10 s, up to 60 s, and take the 95% |.| per tau
    v_all = window_speed(seqs)
    grid = np.arange(1, 601) * DT
    acc = {k: [[] for _ in grid] for k in ('Psi', 'X', 'Y')}
    for tid in np.unique(trial):
        m = np.where(trial == tid)[0]
        for s0 in range(0, len(m), 100):
            mm = m[s0:s0 + 600]
            Psi, Xi, Yi = run_integrator(e[mm][:, list(int_ch)], v_all[mm], t_all[mm] - t_all[mm][0])
            for g in range(min(len(mm), len(grid))):
                acc['Psi'][g].append(abs(Psi[g])); acc['X'][g].append(abs(Xi[g])); acc['Y'][g].append(abs(Yi[g]))
    bands = {'tau': grid}
    for k in acc:
        bands[k] = np.array([np.quantile(a, 0.95) if len(a) else np.nan for a in acc[k]])
        bands[k] = np.maximum.accumulate(np.nan_to_num(bands[k], nan=np.nanmax(bands[k])))
    cal['bands'] = bands
    pool['cal'] = cal
    pool['se_rate'] = cal['q90'][list(int_ch[:2])] / 1.645
    pe = np.array([L2.pose_error(s) for s in seqs[::5]])
    pool['pose_sigma'] = pe.std(0)
    ev = None
    for tid in np.unique(trial):
        m = np.where(trial == tid)[0]
        sub = evidence_state(t_all[m], u[m], flag[m], e[m], v_all[m], cal)
        if ev is None:
            ev = {k: np.zeros(len(trial)) for k in sub}
        for k in sub:
            ev[k][m] = sub[k]
    ev['u'] = u
    pool['ev'] = ev
    eps = sum(len(L2._alarm_episodes(t_all[trial == tid], ev['D'][trial == tid] > 0, 1.0)) for tid in np.unique(trial))
    hours = len(trial) * DT / 3600
    print(f"Calibration: {len(trial)} windows ({hours:.2f} h), {len(np.unique(trial))} trials; integrating channels {int_ch}")
    print(f"  bounded score u: kappa {cal['kappa']:.2f}, h {cal['h']:.2f}, cap {cal['cap']:.2f}, recovery n_off {cal['n_off']:.0f} "
          f"in {RECOVERY_W_S:.0f} s")
    print(f"  nominal (LOO): distrusted {ev['D'].mean():.2%}, envelope active {ev['active'].mean():.2%}, "
          f"distrust episodes {eps} = {eps / hours:.1f} per hour")
    print(f"  nominal 95% drift band at 3 s / 10 s: heading {np.degrees(drift_band(bands, 3, 'Psi')):.1f} / "
          f"{np.degrees(drift_band(bands, 10, 'Psi')):.1f} deg, lateral {drift_band(bands, 3, 'Y'):.2f} / "
          f"{drift_band(bands, 10, 'Y'):.2f} m")
    return pool


# ── nominal floor ───────────────────────────────────────────────────────────

def mode_nominal(pool, drivable, map_data, args):
    rng = np.random.default_rng(args.seed)
    idx = np.arange(0, len(pool['trial']), args.nominal_stride)
    P = {v: np.zeros(len(idx)) for v in VARIANTS}
    for r, i in enumerate(idx):
        out, _, _ = evaluate(pool['seqs'][i], pool['pos_mean'][i], pool['h'][i], pool, pool['ev'], i,
                             drivable, map_data, rng, args, exclude_trial=pool['trial'][i])
        for v in VARIANTS:
            P[v][r] = out[v]
        if (r + 1) % 1000 == 0:
            print(f"  nominal {r + 1}/{len(idx)}")
    P['evidence_dP'] = P['evidence'] - P['nominal']
    thr = {v: max(float(np.quantile(P[v], 1 - args.far)), 0.05 if v == 'evidence_dP' else 0.0) for v in ALARMS}
    res = {'n_windows': int(len(idx)), 'far': args.far, 'thresholds': thr,
           'layer1_flag_rate': float(pool['ev']['D'][idx].mean()),
           'variants': {v: {'mean': float(P[v].mean()), 'p95': float(np.quantile(P[v], .95)),
                            'p99': float(np.quantile(P[v], .99)), 'frac_gt_0.1': float((P[v] > 0.1).mean())}
                        for v in ALARMS}}
    print(f"\n=== Nominal P(violation within 3 s), leave-one-trial-out ({len(idx)} windows) ===")
    for v in ALARMS:
        x = res['variants'][v]
        print(f"  {v:<11} mean {x['mean']:.4f}  p95 {x['p95']:.3f}  p99 {x['p99']:.3f}  threshold@1% {thr[v]:.3f}")
    print(f"  layer1 flag on nominal windows: {res['layer1_flag_rate']:.2%}")
    with open(os.path.join(args.output_dir, 'nominal_floor.json'), 'w') as f:
        json.dump(res, f, indent=2)
    return res


# ── fault trials ────────────────────────────────────────────────────────────

def process_trial(run_dir, model, device, pool, builder, args):
    from inspect_fault_predictions import process_trial as pt
    from fault_log import campaign_fault_kind
    camp = run_dir.split(os.sep)[-3]
    tr = pt(run_dir, camp, builder, campaign_fault_kind(os.path.join(L2.DATA_DIR, camp)))
    if tr is None:
        return None
    seqs, t = tr['sequences'], tr['t_rel']
    inf = _flatten_series(L2.infer(model, device, L2._SeqList(seqs), args.batch))
    prev_ok = RTC.prev_ok_from_time(t)
    e, Xch = _layer1_raw(inf, seqs, prev_ok)
    P = pvalues(Xch, pool['cal']['ref'])
    u, pc = combined_score(P)
    v = window_speed(seqs)
    ev = evidence_state(t, u, (pc < ALPHA_FLAG).astype(float), e, v, pool['cal'])
    ev['u'] = u
    ev['pchan'] = P
    bag = os.path.join(run_dir, 'rosbag')
    gt, est = RTC.read_pose_yaw(bag, L2.GT_TOPIC), RTC.read_pose_yaw(bag, L2.EST_TOPIC)
    return {'rel': os.path.relpath(run_dir, L2.DATA_DIR), 'seqs': seqs, 't': t, 't0': tr['t0'],
            'fault_windows': tr['fault_windows'], 'inf': inf, 'e': e, 'Xch': Xch,
            'ev': ev, 'v': v, 'gt': gt, 'est': est}


def ground_truth(tr, drivable):
    gt, est, t0 = tr['gt'], tr['est'], tr['t0']
    rm = L2.margin_lib.road_margin(drivable.signed_distance(gt[:, 1], gt[:, 2]))
    dep = float(gt[np.argmax(rm < 0), 0] - t0) if (rm < 0).any() else None
    ta = tr['t'] + t0
    gi = np.searchsorted(gt[:, 0], ta).clip(0, len(gt) - 1)
    ei = np.searchsorted(est[:, 0], ta).clip(0, len(est) - 1)
    d = est[ei, 1:3] - gt[gi, 1:3]
    n = np.stack([-np.sin(est[ei, 3]), np.cos(est[ei, 3])], -1)
    return {'rm': rm, 'gt_t': gt[:, 0] - t0, 'departure': dep,
            'true_psi': RTC.wrap(est[ei, 3] - gt[gi, 3]),     # estimated - true heading
            'true_lat': (d * n).sum(-1)}                       # estimated - true lateral


def summarize(rel, t, P, D, fault_windows, dep, thr, gap):
    fs = min((w['start'] for w in fault_windows), default=None)
    pre = t < (fs if fs is not None else np.inf)
    inf_ = np.array([any(w['start'] <= x <= w['end'] for w in fault_windows) for x in t])
    s = {'trial': rel, 'fault_start': fs, 'departure': dep}
    alarms = {v: P[v] > thr[v] for v in ALARMS}
    alarms['layer1'] = D > 0
    for name, a in alarms.items():
        eps = L2._alarm_episodes(t, a, gap)
        d = {'pre_fault_alarm_rate': float(a[pre].mean()) if pre.any() else None,
             'in_fault_alarm_rate': float(a[inf_].mean()) if inf_.any() else None}
        if dep is not None and fs is not None:
            live = [(a0, a1) for (a0, a1) in eps if a0 <= dep and a1 >= dep - gap]
            d['lead_time_s'] = float(dep - live[0][0]) if live else None
            d['alarm_start_after_fault_s'] = float(live[0][0] - fs) if live else None
        s[name] = d
    return s


def mode_faults(model, device, pool, drivable, map_data, nominal, args):
    from st_gat.pipeline.State_Estimator.MapProcessor import MapProcessor
    from st_gat.pipeline.sequence_builder import SequenceBuilder
    builder = SequenceBuilder(MapProcessor(cfg.MAP_FILE).map_data, route=[])
    thr = nominal['thresholds']
    camps = sorted(c for c in os.listdir(L2.DATA_DIR) if not c.startswith('nom'))
    summaries = []
    for run_dir in L2._fault_trials(camps):
        rel = os.path.relpath(run_dir, L2.DATA_DIR)
        if args.trials and not any(rel.startswith(x) for x in args.trials):
            continue
        print(f"\n[{rel}]")
        tr = process_trial(run_dir, model, device, pool, builder, args)
        if tr is None:
            summaries.append({'trial': rel, 'skipped': True})
            continue
        g = ground_truth(tr, drivable)
        rng = np.random.default_rng(args.seed)
        idx = np.arange(0, len(tr['seqs']), args.fault_stride)
        P = {v: np.zeros(len(idx)) for v in VARIANTS}
        for r, i in enumerate(idx):
            out, _, _ = evaluate(tr['seqs'][i], tr['inf']['pos_mean'][i], tr['inf']['h'][i], pool, tr['ev'], i,
                                 drivable, map_data, rng, args)
            for v in VARIANTS:
                P[v][r] = out[v]
        P['evidence_dP'] = P['evidence'] - P['nominal']
        t = tr['t'][idx]
        s = summarize(rel, t, P, tr['ev']['D'][idx], tr['fault_windows'], g['departure'], thr, args.alarm_gap_s)
        summaries.append(s)
        print("  " + "  ".join(f"{k}: {s[k].get('lead_time_s')}" if g['departure'] else
                               f"{k}: {s[k]['in_fault_alarm_rate']}" for k in ALARMS + ['layer1']))
        if any(rel.startswith(x) for x in FEATURED):
            plot_timeseries(tr, g, t, idx, P, thr, args)
            plot_filmstrip(tr, g, pool, drivable, map_data, args)
            if rel.startswith(GIF_TRIAL):
                make_gif(tr, g, pool, drivable, map_data, args)
        with open(os.path.join(args.output_dir, 'fault_summary.json'), 'w') as f:
            json.dump(summaries, f, indent=2)
    print_table(summaries)
    return summaries


def print_table(summaries):
    cols = ['layer1', 'nominal', 'evidence', 'evidence_dP', 'deadreckon']
    print(f"\n{'trial':<46}{'GT dep':>8}" + ''.join(f"{c:>18}" for c in cols))
    for s in summaries:
        if s.get('skipped'):
            print(f"{s['trial']:<46}  skipped")
            continue
        def cell(c):
            d = s[c]
            if s['departure'] is not None:
                if d.get('lead_time_s') is None:
                    return 'MISS'
                return f"{d['lead_time_s']:.1f}s (f+{d['alarm_start_after_fault_s']:.1f})"
            return f"{d['in_fault_alarm_rate'] or 0:.2f}/{d['pre_fault_alarm_rate'] or 0:.2f}"
        dep = f"{s['departure']:.1f}" if s['departure'] is not None else '-'
        print(f"{s['trial'][:45]:<46}{dep:>8}" + ''.join(f"{cell(c):>18}" for c in cols))
    print("(departure trials: lead time before the real departure (alarm start, s after fault onset);"
          " others: in-fault / pre-fault alarm rate)")


# ── plots ───────────────────────────────────────────────────────────────────

def _shade(ax, tr, g):
    for w in tr['fault_windows']:
        ax.axvspan(w['start'], w['end'], color='#ff7f0e', alpha=0.10, lw=0)
    if g['departure'] is not None:
        ax.axvline(g['departure'], color='#d62728', ls='--', lw=1.2)


def plot_timeseries(tr, g, t, idx, P, thr, args):
    """Predictions' evidence, the integrated pose error vs ground truth, and
    P(violation), on one time axis."""
    ev, T = tr['ev'], tr['t']
    cal = args._pool_cal
    fig, ax = plt.subplots(5, 1, figsize=(13, 14), sharex=True,
                           gridspec_kw={'height_ratios': [1.1, 1.1, 1.2, 1.2, 1.6]})
    a = ax[0]
    a.plot(T, ev['u'], color='#7f7f7f', lw=0.7, label='bounded evidence score u (all channels, Bonferroni)')
    a.plot(T, ev['S'] / cal['h'], color='k', lw=1.6, label='capped CUSUM S / threshold h')
    a.fill_between(T, 0, 12, where=ev['D'] > 0, color='#d62728', alpha=0.10, step='mid', label='distrusted')
    a.fill_between(T, 0, 12, where=(ev['active'] > 0) & (ev['D'] == 0), color='#9467bd', alpha=0.10, step='mid',
                   label='carrying pose error')
    a.axhline(1, color='grey', ls=':', lw=0.8)
    a.set_ylim(0, 12)
    a.set_ylabel('evidence')
    a.legend(fontsize=7, loc='upper left', ncol=2)
    a.set_title('Layer 1: WHEN (bounded score, capped CUSUM, recovery test)', fontsize=9, loc='left')
    a = ax[1]
    iy = cal['int_ch'][0]
    if iy != 0:
        a.plot(T, np.degrees(tr['e'][:, 0]), color='#bcbd22', lw=0.8, label='EKF - vehicle yaw-rate residual (deg/s, WHEN only)')
    a.plot(T, np.degrees(tr['e'][:, iy]), color='#9467bd', lw=1.2,
           label=('pose' if iy else 'EKF') + ' - vehicle yaw-rate residual (deg/s, integrated by 2b)')
    a.fill_between(T, -np.degrees(cal['q99'][iy]), np.degrees(cal['q99'][iy]), color='#9467bd', alpha=0.12,
                   label='nominal 99% band')
    a.set_ylim(-60, 60)
    a.set_ylabel('consistency\nresidual')
    a.legend(fontsize=7, loc='upper left')
    a.set_title('Layer 1, consistency head: HOW MUCH and WHICH WAY', fontsize=9, loc='left')
    a = ax[2]
    a.plot(T, np.degrees(ev['Psi']), color='#9467bd', lw=1.6, label='2b integrated heading error Psi')
    a.fill_between(T, np.degrees(ev['Psi'] - ev['sPsi']), np.degrees(ev['Psi'] + ev['sPsi']), color='#9467bd',
                   alpha=0.15, label='+/- integrated band')
    a.plot(T, np.degrees(g['true_psi']), color='k', lw=1, label='TRUE: estimated - AWSIM heading')
    a.set_ylim(-60, 60)
    a.set_ylabel('heading\nerror (deg)')
    a.legend(fontsize=7, loc='upper left')
    a.set_title('2b: integrated evidence vs ground truth', fontsize=9, loc='left')
    a = ax[3]
    a.plot(T, ev['Y'], color='#9467bd', lw=1.6, label='2b integrated lateral offset Y')
    a.plot(T, g['true_lat'], color='k', lw=1, label='TRUE: estimated - AWSIM lateral (m)')
    a.plot(g['gt_t'], g['rm'], color='#d62728', lw=0.8, alpha=0.7, label='true road margin (m)')
    a.axhline(0, color='grey', lw=0.6)
    a.set_ylim(-8, 8)
    a.set_ylabel('metres')
    a.legend(fontsize=7, loc='upper left')
    a = ax[4]
    col = {'nominal': '#7f7f7f', 'evidence': '#9467bd', 'deadreckon': '#e377c2'}
    lab = {'nominal': '2a nominal only', 'evidence': '2a + 2b evidence (method)', 'deadreckon': 'dead-reckoning baseline'}
    for v in VARIANTS:
        a.plot(t, P[v], color=col[v], lw=1.8 if v == 'evidence' else 1, ls='-' if v != 'deadreckon' else '--',
               label=f"{lab[v]} (alarm > {thr[v]:.2f})")
    a.plot(t, P['evidence_dP'], color='#17becf', lw=1.2, label=f"dP = evidence - nominal (alarm > {thr['evidence_dP']:.2f})")
    a.fill_between(T, 0, 1.05, where=ev['active'] > 0, color='#9467bd', alpha=0.06, step='mid', label='envelope moved')
    a.set_ylim(-0.02, 1.05)
    a.set_ylabel('P(violation\nwithin 3 s)')
    a.legend(fontsize=7, loc='upper left')
    a.set_xlabel('seconds since trial start  (orange = fault active, red dashed = real road departure)')
    for x in ax:
        _shade(x, tr, g)
    fig.suptitle(f"{tr['rel']} — evidence, envelope growth and P(violation)", fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.98])
    fig.savefig(os.path.join(args.output_dir, f"timeseries_{tr['rel'].replace('/', '__')}.png"), dpi=110)
    plt.close(fig)


def _frame_panel(ax, tr, g, i, pool, drivable, map_data, args, title=True):
    seq = tr['seqs'][i]
    rng = np.random.default_rng(args.seed + i)
    out, draws, objs = evaluate(seq, tr['inf']['pos_mean'][i], tr['inf']['h'][i], pool, tr['ev'], i,
                                drivable, map_data, rng, args, keep=True)
    ref = L2._ref(seq)
    past = L2._past_xy(seq)
    fut = L2._future_xy(seq)
    mu = ref + tr['inf']['pos_mean'][i] * R_POS
    half = 28
    x0, y0 = ref[0] - half, ref[1] - half
    i0, j0 = int((x0 - drivable.x0) / drivable.res), int((y0 - drivable.y0) / drivable.res)
    npx = int(2 * half / drivable.res)
    ax.imshow(drivable.mask[j0:j0 + npx, i0:i0 + npx], origin='lower', extent=[x0, x0 + 2 * half, y0, y0 + 2 * half],
              cmap='Greys', vmin=0, vmax=2.5)
    for traj in draws['nominal'][0][::4]:
        ax.plot(traj[:, 0], traj[:, 1], color='#8c8c8c', lw=0.5, alpha=0.35)
    traj_e, viol_e = draws['evidence']
    for k in range(0, len(viol_e), 2):
        ax.plot(traj_e[k, :, 0], traj_e[k, :, 1], color='#d62728' if viol_e[k] else '#2ca02c', lw=0.6, alpha=0.45)
    ax.plot(past[:, 0], past[:, 1], 'k-', lw=2)
    ax.plot(mu[:, 0], mu[:, 1], color='#1f77b4', lw=2)
    ax.plot(fut[:, 0], fut[:, 1], color='#1f77b4', lw=1, ls=':')
    t_abs = tr['t'][i] + tr['t0']
    gs = (tr['gt'][:, 0] >= t_abs - 3) & (tr['gt'][:, 0] <= t_abs + 3)
    ax.plot(tr['gt'][gs, 1], tr['gt'][gs, 2], color='#ff7f0e', lw=2, ls='--')
    for m in range(len(objs[2])):
        ax.plot(objs[0][m, :, 0], objs[0][m, :, 1], 'm-', lw=1)
    ax.set_xlim(x0, x0 + 2 * half)
    ax.set_ylim(y0, y0 + 2 * half)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    if title:
        ev = tr['ev']
        ax.set_title(f"t = {tr['t'][i]:.1f} s   distrusted {ev['tau'][i]:.1f} s\n"
                     f"Psi {np.degrees(ev['Psi'][i]):+.1f} deg (true {np.degrees(g['true_psi'][i]):+.1f})\n"
                     f"P: nominal {out['nominal']:.2f} | evidence {out['evidence']:.2f}", fontsize=8.5)
    return out


def _snapshot_times(tr, g):
    t = tr['t']
    fs = min((w['start'] for w in tr['fault_windows']), default=t[len(t) // 2])
    on = t[(tr['ev']['tau'] > 0) & (t >= fs - 1)]
    onset = on[0] if len(on) else fs + 2
    if g['departure'] is not None:
        dep = g['departure']
        times = [fs - 2, onset, (onset + dep) / 2, dep - 2, dep - 1, dep - 0.2]
    else:
        times = [fs - 2, onset, onset + 2, onset + 4, onset + 6, onset + 8]
    out = []
    for x in times:
        i = int(np.argmin(np.abs(t - x)))
        if i not in out:
            out.append(i)
    return sorted(out)


def plot_filmstrip(tr, g, pool, drivable, map_data, args):
    idxs = _snapshot_times(tr, g)
    fig, axes = plt.subplots(1, len(idxs), figsize=(4.2 * len(idxs), 5.2))
    axes = np.atleast_1d(axes)
    for ax, i in zip(axes, idxs):
        _frame_panel(ax, tr, g, i, pool, drivable, map_data, args)
    fig.suptitle(f"{tr['rel']} — envelope over time.  black: past (estimate) · blue: forecast (dotted: what the "
                 f"estimate did) · orange dashed: TRUE path (AWSIM) · grey: 2a nominal envelope · "
                 f"green/red: 2a+2b envelope, safe / violating", fontsize=9)
    fig.tight_layout(rect=[0, 0, 1, 0.93])
    fig.savefig(os.path.join(args.output_dir, f"filmstrip_{tr['rel'].replace('/', '__')}.png"), dpi=110)
    plt.close(fig)


def make_gif(tr, g, pool, drivable, map_data, args):
    from matplotlib.animation import FuncAnimation, PillowWriter
    t = tr['t']
    fs = min((w['start'] for w in tr['fault_windows']), default=t[0])
    end = (g['departure'] + 1.5) if g['departure'] is not None else fs + 12
    frames = [int(np.argmin(np.abs(t - x))) for x in np.arange(fs - 3, end, 0.3)]
    fig, ax = plt.subplots(figsize=(6, 6.4))

    def draw(i):
        ax.clear()
        _frame_panel(ax, tr, g, i, pool, drivable, map_data, args)
    anim = FuncAnimation(fig, draw, frames=frames, interval=150)
    anim.save(os.path.join(args.output_dir, f"envelope_{tr['rel'].replace('/', '__')}.gif"), writer=PillowWriter(fps=6))
    plt.close(fig)


# ── main ────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--mode', choices=['nominal', 'faults', 'all'], default='all')
    ap.add_argument('--model', default=cfg.MODEL_CONFIG['model_path'])
    ap.add_argument('--batch', type=int, default=256)
    ap.add_argument('--k-draws', type=int, default=200)
    ap.add_argument('--k-neighbors', type=int, default=150)
    ap.add_argument('--nominal-stride', type=int, default=5)
    ap.add_argument('--fault-stride', type=int, default=2)
    ap.add_argument('--kappa-q', type=float, default=0.99,
                    help='CUSUM reference kappa = this nominal quantile of the combined evidence strength')
    ap.add_argument('--distrust-rate', type=float, default=0.01, help='CUSUM threshold set so this share of nominal windows is distrusted')
    ap.add_argument('--far', type=float, default=0.01)
    ap.add_argument('--alarm-gap-s', type=float, default=1.0)
    ap.add_argument('--trials', nargs='*', default=None)
    ap.add_argument('--no-cache', action='store_true')
    ap.add_argument('--seed', type=int, default=20261007)
    ap.add_argument('--output-dir', default=OUT_DIR)
    args = ap.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = L2._load_model(args.model, device)
    map_data = L2.margin_lib.load_map(cfg.MAP_FILE)
    drivable = L2.margin_lib.DrivableArea(map_data)
    pool = build_pool(model, device, args)
    args._pool_cal = pool['cal']
    if args.mode in ('nominal', 'all'):
        nominal = mode_nominal(pool, drivable, map_data, args)
    else:
        with open(os.path.join(args.output_dir, 'nominal_floor.json')) as f:
            nominal = json.load(f)
    if args.mode in ('faults', 'all'):
        mode_faults(model, device, pool, drivable, map_data, nominal, args)


if __name__ == '__main__':
    main()
