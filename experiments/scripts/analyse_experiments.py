#!/usr/bin/env python3
"""
Comprehensive experiment analysis script for RISE validation.

Reads all experiment data (result.json + metrics.json + metadata.json) from
one or more campaign directories and produces:
  1. Baseline summary: per-goal reliability, MRM rates, safety clearances
  2. Scenario sweep analysis: per-distance/condition breakdown, obstacle placement
     verification, lane-change vs stop behaviour
  3. Sanity checks: obstacle placement consistency, UUID tracking coverage,
     velocity plausibility

Usage:
  # Analyse a single campaign
  python3 analyse_experiments.py --campaign baseline_all
  python3 analyse_experiments.py --campaign static_obstacle

  # Analyse and compare baseline vs scenario
  python3 analyse_experiments.py --campaign baseline_all --campaign static_obstacle

  # Save output to a file
  python3 analyse_experiments.py --campaign static_obstacle --out report.txt

  # Full auto: analyse all campaigns in the data directory
  python3 analyse_experiments.py --all
"""

import argparse
import json
import math
import os
import sys
from collections import defaultdict
from typing import Dict, List, Optional, Tuple

# --------------------------------------------------------------------------- #
# Paths
# --------------------------------------------------------------------------- #
SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
LIB_DIR     = os.path.join(SCRIPT_DIR, '..', 'lib')
DATA_DIR    = os.path.join(SCRIPT_DIR, '..', 'data')
sys.path.insert(0, LIB_DIR)


# --------------------------------------------------------------------------- #
# Data loading
# --------------------------------------------------------------------------- #

def load_experiment(exp_dir: str) -> Optional[dict]:
    """Load result + metrics + metadata for one experiment directory."""
    rf  = os.path.join(exp_dir, 'result.json')
    mf  = os.path.join(exp_dir, 'metrics.json')
    mdf = os.path.join(exp_dir, 'metadata.json')
    if not all(os.path.exists(f) for f in [rf, mf, mdf]):
        return None
    r   = json.load(open(rf))
    m   = json.load(open(mf))
    md  = json.load(open(mdf))
    return {'result': r, 'metrics': m, 'metadata': md, 'dir': exp_dir}


def load_campaign(campaign_name: str) -> List[dict]:
    campaign_dir = os.path.join(DATA_DIR, campaign_name)
    if not os.path.isdir(campaign_dir):
        print(f'[WARN] Campaign directory not found: {campaign_dir}')
        return []
    exps = []
    for name in sorted(os.listdir(campaign_dir)):
        if not name.startswith('goal_'):
            continue
        e = load_experiment(os.path.join(campaign_dir, name))
        if e:
            e['name'] = name
            e['campaign'] = campaign_name
            exps.append(e)
    return exps


# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #

def _dist(x1, y1, x2, y2):
    return math.sqrt((x2-x1)**2 + (y2-y1)**2)


def _safe(d, *keys, default=None):
    """Safe nested dict get."""
    for k in keys:
        if not isinstance(d, dict):
            return default
        d = d.get(k, None)
        if d is None:
            return default
    return d


def _fmt(v, fmt='.2f', na='—'):
    if v is None or (isinstance(v, float) and (math.isinf(v) or math.isnan(v))):
        return na
    return format(v, fmt)


def _goal_id(name: str) -> str:
    parts = name.split('_')
    return f'{parts[0]}_{parts[1]}'


def _scenario_params(md: dict) -> dict:
    return md.get('scenario_params', {})


def _obs_pos(md: dict) -> Optional[Tuple[float,float]]:
    obs = md.get('injected_obstacle', {})
    x, y = obs.get('world_x'), obs.get('world_y')
    return (x, y) if x is not None and y is not None else None


# --------------------------------------------------------------------------- #
# Section 1 – Baseline analysis
# --------------------------------------------------------------------------- #

def analyse_baseline(exps: List[dict], out: List[str]):
    out.append('\n' + '='*90)
    out.append('SECTION 1 — BASELINE: Per-Goal Summary')
    out.append('='*90)

    success_count = 0
    goals_data = []

    for e in exps:
        r   = e['result']
        m   = e['metrics']
        md  = e['metadata']
        goal = _goal_id(e['name'])

        status     = r.get('status', '?')
        success    = status == 'goal_reached'
        if success: success_count += 1

        dt         = _safe(m, 'reliability', 'driving_time', default=0)
        dist_km    = _safe(m, 'reliability', 'distance_km', default=0)
        mean_vel   = _safe(m, 'reliability', 'mean_velocity', default=0)
        mrm_cnt    = _safe(m, 'fail_operational', 'mrm_trigger_count', default=0)
        mrm_rate   = _safe(m, 'fail_operational', 'mrm_rate', default=0)
        mrm_dur    = _safe(m, 'fail_operational', 'avg_mrm_duration', default=0)
        min_obj    = _safe(m, 'safety', 'min_object_distance', default=float('inf'))
        clr_p5     = _safe(m, 'safety', 'clearance_p5', default=float('inf'))
        est_dist   = md.get('goal', {}).get('estimated_distance', None) if 'goal' not in md else None
        # try alternative path
        if est_dist is None:
            est_dist = _safe(md, 'goal', 'estimated_distance')

        goals_data.append({
            'goal': goal, 'status': status, 'success': success,
            'dt': dt, 'dist_km': dist_km, 'mean_vel': mean_vel,
            'mrm_cnt': mrm_cnt, 'mrm_rate': mrm_rate, 'mrm_dur': mrm_dur,
            'min_obj': min_obj, 'clr_p5': clr_p5, 'est_dist': est_dist,
        })

    total = len(goals_data)
    out.append(f'\nTotal goals: {total}  |  Successful: {success_count}/{total}  '
               f'({100*success_count/total:.0f}%)')

    hdr = (f"{'Goal':<12} {'Status':<22} {'Dist(km)':>9} {'Vel(m/s)':>9} "
           f"{'Time(s)':>8} {'MRM#':>5} {'MRM/km':>7} {'AvgMRM(ms)':>11} "
           f"{'MinObj(m)':>10} {'Clr_P5(m)':>10}")
    out.append('\n' + hdr)
    out.append('-'*105)

    for g in goals_data:
        vel_str  = _fmt(g['mean_vel'])
        mrm_r    = _fmt(g['mrm_rate'], '.0f')
        mrm_d    = _fmt(g['mrm_dur']*1000 if g['mrm_dur'] else None, '.0f')
        min_o    = _fmt(g['min_obj'] if g['min_obj'] < 1e6 else None)
        clr      = _fmt(g['clr_p5'] if g['clr_p5'] < 1e6 else None)
        flag     = ' ← FAIL' if not g['success'] else ''
        out.append(
            f"{g['goal']:<12} {g['status']:<22} {g['dist_km']:>9.3f} {vel_str:>9} "
            f"{g['dt']:>8.0f} {g['mrm_cnt']:>5} {mrm_r:>7} {mrm_d:>11} "
            f"{min_o:>10} {clr:>10}{flag}"
        )

    # Aggregate stats over successful runs
    ok = [g for g in goals_data if g['success']]
    if ok:
        out.append('\nAggregate (successful runs only):')
        out.append(f"  Mean velocity:      {sum(g['mean_vel'] for g in ok)/len(ok):.2f} m/s")
        out.append(f"  Mean MRM/km:        {sum(g['mrm_rate'] for g in ok)/len(ok):.1f}")
        mrm_durs = [g['mrm_dur']*1000 for g in ok if g['mrm_dur']]
        if mrm_durs:
            out.append(f"  Mean MRM duration:  {sum(mrm_durs)/len(mrm_durs):.0f} ms (avg transient)")
        min_objs = [g['min_obj'] for g in ok if g['min_obj'] < 1e6]
        if min_objs:
            out.append(f"  Min object dist:    {min(min_objs):.2f} m (worst case, incl. NPC traffic)")

    # Flag problematic goals
    fails = [g for g in goals_data if not g['success']]
    if fails:
        out.append('\nFailed goals:')
        for g in fails:
            out.append(f"  {g['goal']}: {g['status']}")

    # Short routes that may be unsuitable for scenario experiments
    short = [g for g in goals_data if g['success'] and g['dist_km'] < 0.4]
    if short:
        out.append('\nShort routes (< 400 m, unsuitable for obstacle scenarios):')
        for g in short:
            out.append(f"  {g['goal']}: {g['dist_km']*1000:.0f} m")

    out.append('')


# --------------------------------------------------------------------------- #
# Section 2 – Static obstacle sweep analysis
# --------------------------------------------------------------------------- #

def analyse_static_obstacle(exps: List[dict], out: List[str]):
    out.append('\n' + '='*90)
    out.append('SECTION 2 — STATIC OBSTACLE SWEEP')
    out.append('='*90)

    # Group by distance
    by_dist: Dict[float, List[dict]] = defaultdict(list)
    for e in exps:
        sp = _scenario_params(e['metadata'])
        d  = sp.get('distance')
        if d is not None:
            by_dist[d].append(e)

    if not by_dist:
        out.append('  (no static_obstacle experiments found)')
        return

    # ── 2a. Placement sanity ─────────────────────────────────────────────────
    out.append('\n── 2a. Obstacle Placement Verification ──')
    out.append(f"{'Dist':>6}  {'N':>3}  {'UniquePos':>9}  {'AvgObsY':>10}  "
               f"{'AvgDistEgo→Obs':>15}  {'PlacementOK'}")
    out.append('-'*70)

    placement_ok = {}
    for dist in sorted(by_dist.keys()):
        entries = by_dist[dist]
        positions = []
        ego_to_obs_distances = []

        for e in entries:
            obs = _obs_pos(e['metadata'])
            if obs is None:
                continue
            positions.append(obs)

        unique_pos = len(set((round(x, 0), round(y, 0)) for x, y in positions))
        avg_y = sum(p[1] for p in positions) / len(positions) if positions else 0

        # Good placement = multiple unique positions (not all the same terminal point)
        ok = unique_pos >= max(2, len(entries) // 4)
        placement_ok[dist] = ok
        flag = '✓' if ok else '✗ TERMINAL-CLUSTERING'

        out.append(f'{dist:>6.0f}  {len(entries):>3}  {unique_pos:>9}  '
                   f'{avg_y:>10.1f}  {"see detail":>15}  {flag}')

    # ── 2b. Per-distance outcome breakdown ───────────────────────────────────
    out.append('\n── 2b. Outcomes by Distance ──')
    out.append(f"{'Dist':>6}  {'N':>3}  {'LaneChange':>10}  {'Stuck':>6}  "
               f"{'MRM_S':>6}  {'Fail':>5}  {'AvgDist(m)':>11}  {'AvgMRMs':>8}")
    out.append('-'*75)

    for dist in sorted(by_dist.keys()):
        entries = by_dist[dist]
        n  = len(entries)
        gr = sum(1 for e in entries if e['result']['status'] == 'goal_reached')
        st = sum(1 for e in entries if e['result']['status'] == 'stuck')
        ms = sum(1 for e in entries if 'mrm_mrm' in e['result']['status'])
        fa = sum(1 for e in entries if e['result']['status'] not in
                 ['goal_reached', 'stuck'] and 'mrm_mrm' not in e['result']['status'])
        avg_dist = sum(_safe(e['metrics'], 'reliability', 'distance_km', default=0)
                       * 1000 for e in entries) / n
        avg_mrm  = sum(_safe(e['metrics'], 'fail_operational', 'mrm_trigger_count', default=0)
                       for e in entries) / n
        out.append(f'{dist:>6.0f}  {n:>3}  {gr:>10}  {st:>6}  '
                   f'{ms:>6}  {fa:>5}  {avg_dist:>11.1f}  {avg_mrm:>8.0f}')

    # ── 2c. Lane-change vs stop detail (goal_reached or properly stuck) ──────
    out.append('\n── 2c. Obstacle Approach — Lane-Change vs Stop ──')
    out.append(f"{'Goal':<10} {'ObjType':<8} {'Trial':<6} {'Dist':>5} "
               f"{'Status':<22} {'TravM':>7} {'MinInj(m)':>10} {'MinCT(s)':>9} {'MRMs':>5}")
    out.append('-'*90)

    for dist in sorted(by_dist.keys()):
        for e in sorted(by_dist[dist],
                        key=lambda x: (_goal_id(x['name']), x['metadata'].get('scenario_params',{}).get('obj_type',''), x['name'])):
            r   = e['result']
            m   = e['metrics']
            sp  = _scenario_params(e['metadata'])
            goal = _goal_id(e['name'])
            trial = [p for p in e['name'].split('_')
                     if p.startswith('t') and p[1:].isdigit() and len(p) <= 3]
            trial = trial[0] if trial else '?'
            obj_type = sp.get('obj_type', '?')

            status   = r['status']
            trav_m   = _safe(m, 'reliability', 'distance_km', default=0) * 1000
            min_inj  = _safe(m, 'safety', 'min_injected_obstacle_distance', default=float('inf'))
            min_ct   = _safe(m, 'safety', 'min_closing_time', default=float('inf'))
            mrm_cnt  = _safe(m, 'fail_operational', 'mrm_trigger_count', default=0)

            inj_str = _fmt(min_inj) if min_inj < 999 else '—(UUID miss)'
            ct_str  = _fmt(min_ct) if min_ct < 999 else '—'

            # Highlight lane-change successes and close approaches
            flag = ''
            if status == 'goal_reached':
                flag = ' ← LANE-CHANGE'
            elif min_inj < 5 and min_inj < 999:
                flag = f' ← CLOSE ({min_inj:.1f}m)'

            out.append(f"{goal:<10} {obj_type:<8} {trial:<6} {dist:>5.0f} "
                       f"{status:<22} {trav_m:>7.0f} {inj_str:>10} {ct_str:>9} {mrm_cnt:>5}{flag}")

    # ── 2d. Summary statistics by distance ───────────────────────────────────
    out.append('\n── 2d. Stopping Margin Statistics (stuck runs, valid UUID only) ──')
    out.append(f"{'Dist':>6}  {'N_stuck':>8}  {'N_UUIDok':>9}  "
               f"{'Min_inj(m)':>11}  {'Mean_inj(m)':>12}  {'Max_inj(m)':>12}")
    out.append('-'*70)
    for dist in sorted(by_dist.keys()):
        stuck = [e for e in by_dist[dist] if e['result']['status'] == 'stuck']
        valid = [_safe(e['metrics'], 'safety', 'min_injected_obstacle_distance', default=float('inf'))
                 for e in stuck]
        valid = [v for v in valid if v < 999]
        if not valid:
            out.append(f'{dist:>6.0f}  {len(stuck):>8}  {0:>9}  {"—":>11}  {"—":>12}  {"—":>12}')
        else:
            out.append(f'{dist:>6.0f}  {len(stuck):>8}  {len(valid):>9}  '
                       f'{min(valid):>11.2f}  {sum(valid)/len(valid):>12.2f}  '
                       f'{max(valid):>12.2f}')

    # ── 2e. Lane-change summary ───────────────────────────────────────────────
    out.append('\n── 2e. Passing Distance (goal_reached = lane-change succeeded) ──')
    out.append(f"{'Dist':>6}  {'LaneChanges':>12}  {'AvgPassDist(m)':>15}  "
               f"{'MinPassDist(m)':>15}  Notes")
    out.append('-'*75)
    for dist in sorted(by_dist.keys()):
        gr_entries = [e for e in by_dist[dist] if e['result']['status'] == 'goal_reached']
        valid_inj = [_safe(e['metrics'], 'safety', 'min_injected_obstacle_distance', default=float('inf'))
                     for e in gr_entries]
        valid_inj = [v for v in valid_inj if v < 999]
        if not gr_entries:
            out.append(f'{dist:>6.0f}  {0:>12}  {"—":>15}  {"—":>15}  no lane-changes')
        elif not valid_inj:
            out.append(f'{dist:>6.0f}  {len(gr_entries):>12}  {"—":>15}  {"—":>15}  UUID miss')
        else:
            out.append(f'{dist:>6.0f}  {len(gr_entries):>12}  '
                       f'{sum(valid_inj)/len(valid_inj):>15.3f}  '
                       f'{min(valid_inj):>15.3f}  virtual obstacle (no collision physics)')

    out.append('')


# --------------------------------------------------------------------------- #
# Section 3 – Sanity checks
# --------------------------------------------------------------------------- #

def sanity_checks(all_exps: List[dict], out: List[str]):
    out.append('\n' + '='*90)
    out.append('SECTION 3 — SANITY CHECKS')
    out.append('='*90)

    issues = []

    for e in all_exps:
        name   = e['name']
        r      = e['result']
        m      = e['metrics']
        md     = e['metadata']
        goal   = _goal_id(name)

        dist_km  = _safe(m, 'reliability', 'distance_km', default=0)
        mean_vel = _safe(m, 'reliability', 'mean_velocity', default=0)
        dt       = _safe(m, 'reliability', 'driving_time', default=0)
        mrm_cnt  = _safe(m, 'fail_operational', 'mrm_trigger_count', default=0)
        status   = r.get('status', '?')

        # Check 1: stationary vehicle masquerading as distance (localization drift)
        # Signature: dist_km < 0.05 but driving_time > 100s  → real movement = 0
        if dist_km * 1000 < 50 and dt > 100:
            issues.append(f'  DRIFT: {goal} ({name[-15:]}) — '
                          f'dist={dist_km*1000:.0f}m in {dt:.0f}s (={mean_vel:.2f}m/s) → '
                          f'localization drift only, vehicle never moved')

        # Check 2: implausible velocity for a "successful" run
        if status == 'goal_reached' and mean_vel < 0.5:
            issues.append(f'  VEL:   {goal} — goal_reached but mean_vel={mean_vel:.2f}m/s (suspicious)')

        # Check 3: scenario experiment but no obstacle placed (obs_pos missing)
        sp = _scenario_params(md)
        if sp.get('distance') is not None:
            obs = _obs_pos(md)
            if obs is None:
                issues.append(f'  OBS:   {goal} ({name[-15:]}) — '
                               f'distance={sp.get("distance")} but no obstacle placement recorded')

        # Check 4: obstacle placement not moving with distance
        # (same position for all distances = terminal clustering)
        # — handled per-campaign below

        # Check 5: UUID tracking failure on a run where vehicle clearly reached obstacle
        # (stuck, dist > 100m, but min_inj = inf)
        min_inj = _safe(m, 'safety', 'min_injected_obstacle_distance', default=float('inf'))
        if status == 'stuck' and dist_km * 1000 > 100 and min_inj >= 999:
            issues.append(f'  UUID:  {goal} ({name[-15:]}) — stuck at {dist_km*1000:.0f}m '
                          f'but min_inj=inf (UUID tracking miss)')

    # Check 6: terminal-clustering across obstacle campaigns
    campaigns = defaultdict(list)
    for e in all_exps:
        campaigns[e['campaign']].append(e)

    for camp, exps in campaigns.items():
        sp_exps = [e for e in exps if _scenario_params(e['metadata']).get('distance') is not None]
        if not sp_exps:
            continue
        by_dist = defaultdict(list)
        for e in sp_exps:
            d = _scenario_params(e['metadata']).get('distance')
            obs = _obs_pos(e['metadata'])
            if obs:
                by_dist[d].append(obs)

        if len(by_dist) < 2:
            continue

        # Get representative (median) obs_y per distance
        rep_y = {}
        for d, poses in by_dist.items():
            ys = sorted(p[1] for p in poses)
            rep_y[d] = ys[len(ys)//2]

        dists = sorted(rep_y.keys())
        for i in range(1, len(dists)):
            dy = abs(rep_y[dists[i]] - rep_y[dists[i-1]])
            expected_min_dy = (dists[i] - dists[i-1]) * 0.3  # expect at least 30% of distance diff
            if dy < expected_min_dy:
                issues.append(
                    f'  CLUSTER ({camp}): distance {dists[i-1]}m→{dists[i]}m '
                    f'obs_y diff={dy:.1f}m (expected ≥{expected_min_dy:.0f}m) '
                    f'→ trajectory-terminal clustering still present'
                )

    if issues:
        out.append('\nIssues found:')
        for iss in issues:
            out.append(iss)
    else:
        out.append('\nNo issues found. All sanity checks passed.')

    out.append('')


# --------------------------------------------------------------------------- #
# Section 4 – Cross-campaign comparison
# --------------------------------------------------------------------------- #

def compare_campaigns(campaigns: Dict[str, List[dict]], out: List[str]):
    if len(campaigns) < 2:
        return

    out.append('\n' + '='*90)
    out.append('SECTION 4 — CROSS-CAMPAIGN COMPARISON')
    out.append('='*90)

    for name, exps in campaigns.items():
        ok = sum(1 for e in exps if e['result']['status'] == 'goal_reached')
        n  = len(exps)
        avg_mrm = (sum(_safe(e['metrics'],'fail_operational','mrm_trigger_count',default=0)
                       for e in exps) / n) if n else 0
        avg_vel = (sum(_safe(e['metrics'],'reliability','mean_velocity',default=0)
                       for e in exps) / n) if n else 0
        out.append(f'\n  {name}:  {ok}/{n} goal_reached  '
                   f'avg_mrm={avg_mrm:.0f}  avg_vel={avg_vel:.2f}m/s')

    out.append('')


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #

def main():
    parser = argparse.ArgumentParser(description='RISE experiment analyser')
    parser.add_argument('--campaign', action='append', dest='campaigns',
                        metavar='NAME', help='Campaign directory name (repeatable)')
    parser.add_argument('--all', action='store_true',
                        help='Analyse all campaigns in the data directory')
    parser.add_argument('--out', type=str, default=None,
                        help='Write report to this file instead of stdout')
    args = parser.parse_args()

    if args.all:
        if os.path.isdir(DATA_DIR):
            campaign_names = sorted(
                d for d in os.listdir(DATA_DIR)
                if os.path.isdir(os.path.join(DATA_DIR, d))
            )
        else:
            campaign_names = []
    elif args.campaigns:
        campaign_names = args.campaigns
    else:
        parser.print_help()
        sys.exit(1)

    if not campaign_names:
        print('No campaigns found.')
        sys.exit(1)

    # Load all data
    loaded: Dict[str, List[dict]] = {}
    for camp in campaign_names:
        exps = load_campaign(camp)
        if exps:
            loaded[camp] = exps
            print(f'Loaded {len(exps)} experiments from "{camp}"')

    if not loaded:
        print('No experiment data found.')
        sys.exit(1)

    all_exps = [e for exps in loaded.values() for e in exps]

    lines = []
    lines.append('=' * 90)
    lines.append('RISE EXPERIMENT ANALYSIS REPORT')
    lines.append(f'Campaigns: {", ".join(loaded.keys())}')
    lines.append(f'Total experiments: {len(all_exps)}')
    lines.append('=' * 90)

    # Run analyses based on what's loaded
    for camp, exps in loaded.items():
        # Detect campaign type by checking scenario_params
        sp_exps = [e for e in exps if _scenario_params(e['metadata']).get('distance') is not None]
        baseline_exps = [e for e in exps if not _scenario_params(e['metadata'])]

        if baseline_exps:
            lines.append(f'\n>>> Campaign: {camp}  ({len(baseline_exps)} baseline experiments)')
            analyse_baseline(baseline_exps, lines)
        if sp_exps:
            lines.append(f'\n>>> Campaign: {camp}  ({len(sp_exps)} scenario experiments)')
            analyse_static_obstacle(sp_exps, lines)

    sanity_checks(all_exps, lines)
    compare_campaigns(loaded, lines)

    report = '\n'.join(lines)

    if args.out:
        with open(args.out, 'w') as f:
            f.write(report)
        print(f'\nReport saved to: {args.out}')
    else:
        print(report)


if __name__ == '__main__':
    main()
