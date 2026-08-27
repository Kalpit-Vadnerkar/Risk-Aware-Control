#!/bin/bash
# =============================================================================
# Data Collection Script
# =============================================================================
# Collects experiment data for each campaign.
# AWSIM (Terminal 1) and Autoware (Terminal 2) must already be running.
#
# Usage:
#   ./collect.sh <campaign> [--trials N] [--goals GOALS] [--dry-run]
#
# Campaigns:
#   nom_v5              Nominal driving, 5 m/s velocity cap
#   nom_v7              Nominal driving, 7 m/s velocity cap
#   nom_v11             Nominal driving, 11.11 m/s (Autoware map speed limit)
#   obs_stuck           30m obstacle, Autoware stops (avoidance=manual)
#   obs_recovery        30m obstacle, Autoware swerves (avoidance=auto)
#   obs_noescape        30m obstacle in single-lane (LL 241), no path forward
#   obs_singlelane      30m obstacle, no adjacent lane — Signal 1 validation
#   obs_tooclosetoreact 6m obstacle, multi-lane — Signal 2 (TTC) validation
#   tl_fault_s2         TL oscillating GREEN/original (5s period) — repeats at every TL zone
#   tl_fault_s3         TL classification → UNKNOWN — repeats at every TL zone
#   tl_fault_s4         TL full blackout — repeats at every TL zone
#   tl_fault_ramp       TL confidence decays 1.0->0.0 over each 15s cap/cycle
#                       (gradual, not a step) — repeats at every TL zone
#   tl_fault_fixed_030  TL confidence FIXED at confidence_scale=0.7 (severity 0.3)
#   tl_fault_fixed_050  TL confidence FIXED at confidence_scale=0.5 (severity 0.5)
#   tl_fault_fixed_070  TL confidence FIXED at confidence_scale=0.3 (severity 0.7)
#                       (added 2026-08-26 for a clean severity dose-response —
#                       see the case block below for why the ramp doesn't serve
#                       this specific purpose despite superseding tl_fault_s1)
#   tl_fault_s1 (fixed confidence x0.5) REMOVED 2026-07-24 — tl_fault_ramp's
#   decay passes through and beyond S1's exact severity level within a single
#   trial, so it subsumes what S1 could show. Mirrors the imu_bias tiers'
#   removal in favor of imu_fault_ramp. The underlying tl_confidence mode is
#   still available for ad-hoc use (run_experiments.py --tl-fault
#   tl_confidence --tl-params '{"confidence_scale":0.5}'), e.g. if a fixed-
#   severity comparison point against the ramp is ever wanted again.
#   All TL/IMU faults: must clear the first real TL zone (map-position-based,
#   20m radius from a real Lanelet2 traffic light — revised 2026-07-25 from
#   an earlier 80m reaction-range-based radius, which was still wide enough
#   to merge several physically distinct real intersections into one
#   continuous zone; 20m was validated empirically against real driven
#   trajectories, not guessed — see fault_injector.py's
#   _DEFAULT_TL_ZONE_RADIUS_M), goal-scoped to whatever
#   --goals this run uses — see fault_injector.py's _on_gt_pose and
#   _load_tl_zone_points; all goals share one start intersection, so this
#   replaces an earlier flat-150m-distance runway), then arms IMMEDIATELY
#   (no wall-clock delay — REMOVED 2026-07-25, see the TL campaign comments
#   below). TL then re-fires at EVERY such zone entered for the rest
#   of the trial (15s cap/cycle, 8s recovery gap between cycles) — one
#   (reaction, recovery) sample per real intersection on the route. REDESIGNED
#   2026-07-24: the original zone detector used raw TL-message content
#   (>=30% of last 30 msgs non-empty), which live data showed staying "in
#   zone" continuously for 60-445m of real driving — not a location signal
#   at all despite being designed as one. See fault_injector.py's module
#   docstring for the full writeup.
#   Constant-bias IMU tiers (old imu_fault_s1..s4) REMOVED 2026-07-24 — Autoware's
#   ekf_localizer explicitly estimates and cancels a constant gyro bias
#   (enable_yaw_bias_estimation: true), so sub-gate constant-bias tiers mostly
#   just demonstrate Autoware's own mitigation working, not our detector's
#   reach. See docs/design_decisions.md item 7.
#   imu_fault_s1        REINSTATED 2026-07-25 as a control condition, not a
#                       reversal of the above: constant bias 0.03 rad/s, 20s
#                       on/30s off (0.6 rad accumulated) — deliberately inside
#                       the "gets absorbed" regime, to show the boundary
#                       ramp/scale/stuck sit outside of, not just assume it.
#   imu_fault_s3        ADDED 2026-07-26 as S1's above-threshold pair: fixed
#                       0.08 rad/s, 15s on/30s off (1.2 rad accumulated) — a
#                       step, not a sweep, so onset is unambiguous and lead
#                       time isn't confounded with elapsed time the way
#                       imu_fault_ramp's continuous growth is. Not yet
#                       validated as cleanly above threshold vs. marginal —
#                       that's this campaign's own open question.
#   imu_fault_ramp      One-shot linear ramp 0 -> 0.4 rad/s (0.003 rad/s/s) —
#                       sweeps through absorbed -> gate-rejection cliff in one
#                       trial. Demoted from the default production suite
#                       (run_fault_campaigns.sh) 2026-07-26 in favor of the
#                       s1/s3 fixed-tier pair above — kept available for
#                       ad-hoc use (e.g. re-probing the boundary if s1/s3
#                       data ever suggests 0.08 was mis-sized).
#   imu_fault_scale     Multiplicative gain error (gyro x1.8) — structurally
#                       unabsorbable by yaw_bias's additive-constant model.
#   imu_fault_stuck     Gyro frozen at its activation-time value — same.
#
# Examples:
#   ./collect.sh nom_v7
#   ./collect.sh obs_recovery --trials 3
#   ./collect.sh obs_stuck --goals "goal_007,goal_011" --dry-run
# =============================================================================

set -eo pipefail

# ── Colors ────────────────────────────────────────────────────────────────────
GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m';  RED='\033[0;31m'; NC='\033[0m'

# ── Paths ─────────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
AUTOWARE_DIR="$WORKSPACE_DIR/autoware"
EXPERIMENTS_DIR="$SCRIPT_DIR/experiments"
RUNNER="$EXPERIMENTS_DIR/scripts/run_experiments.py"
SCENARIOS_DIR="$EXPERIMENTS_DIR/configs/scenarios"
AVOIDANCE_CFG="$AUTOWARE_DIR/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/static_obstacle_avoidance.param.yaml"

# ── Defaults ──────────────────────────────────────────────────────────────────
TRIALS=6
GOALS="goal_007,goal_011,goal_021"
GOALS_FILE=""
DRY_RUN=""
YES=""
FAULT_MIN_RUNWAY=""   # empty = use fault_injector.py's own default (150m, GT-gated)
ARM="A"   # A = safety features disabled (science condition, this repo's long-standing
          # default). B = stock/full diagnostic gate (ground-truth oracle) — requires
          # switch_diagnostic_arm.sh B + an Autoware restart FIRST; this flag only
          # labels the data, it does not itself change what Autoware is running.

# ── Parse arguments ───────────────────────────────────────────────────────────
if [[ $# -eq 0 ]]; then
    echo -e "${RED}Usage: ./collect.sh <campaign> [--trials N] [--goals GOALS] [--goals-file FILE] [--fault-min-runway-m M] [--arm A|B] [--yes] [--dry-run]${NC}"
    echo ""
    echo "Campaigns: nom_v5  nom_v7  nom_v11  obs_stuck  obs_recovery  obs_noescape  obs_singlelane  obs_tooclosetoreact"
    echo "           tl_fault_s2..s4  tl_fault_ramp  tl_fault_fixed_030/050/070  imu_fault_s1  imu_fault_s3  imu_fault_ramp  imu_fault_scale  imu_fault_stuck"
    echo ""
    echo "--fault-min-runway-m M: fallback-only straight-line runway distance,"
    echo "used only if map TL zone points fail to load (normally they do, and"
    echo "the zone-based runway is used instead). The old 'M<=0 disables all"
    echo "gating' escape hatch was removed 2026-07-26 — see fault_injector.py's"
    echo "__init__ comment for why (it had a real gap with IMU zone-gating)."
    echo ""
    echo "--arm A|B (default A): which diagnostic-gate configuration Autoware is"
    echo "currently running under — see experiments/scripts/switch_diagnostic_arm.sh."
    echo "Only labels the collected data (campaign dir gets an _armB suffix, plus"
    echo "an 'arm' field in metadata.json/fault_log.jsonl) — switch the actual"
    echo "config and restart Autoware yourself BEFORE running with --arm B."
    exit 1
fi

CAMPAIGN="$1"; shift

while [[ $# -gt 0 ]]; do
    case "$1" in
        --trials)             TRIALS="$2";           shift 2 ;;
        --goals)              GOALS="$2";             shift 2 ;;
        --goals-file)         GOALS_FILE="$2";        shift 2 ;;
        --fault-min-runway-m) FAULT_MIN_RUNWAY="$2";  shift 2 ;;
        --arm)                ARM="$2";               shift 2 ;;
        --yes|-y)      YES="--yes";     shift ;;
        --dry-run)     DRY_RUN="--dry-run"; shift ;;
        *) echo -e "${RED}Unknown argument: $1${NC}"; exit 1 ;;
    esac
done

if [[ "$ARM" != "A" && "$ARM" != "B" ]]; then
    echo -e "${RED}--arm must be A or B, got: ${ARM}${NC}"; exit 1
fi

# ── Source environment ────────────────────────────────────────────────────────
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Data Collection — campaign: ${CAMPAIGN}${NC}"
echo -e "${GREEN}========================================${NC}"

echo -e "${BLUE}Sourcing ROS2 Humble...${NC}"
source /opt/ros/humble/setup.bash

echo -e "${BLUE}Sourcing Autoware install...${NC}"
source "$AUTOWARE_DIR/install/setup.bash"

# ── Prerequisites check ───────────────────────────────────────────────────────
echo -e "${BLUE}Checking prerequisites...${NC}"
if ! ros2 topic list 2>/dev/null | grep -q "/sensing/lidar/top/pointcloud_raw"; then
    echo -e "${RED}ERROR: AWSIM not running. Start it first: ./Run_AWSIM.sh${NC}"; exit 1
fi
echo -e "  AWSIM:     ${GREEN}OK${NC}"
echo -e "  Autoware:  (readiness checked by the experiment runner)"
ACTIVE_ARM_MARKER="$AUTOWARE_DIR/src/launcher/autoware_launch/autoware_launch/config/system/diagnostics/.active_arm"
if [[ -f "$ACTIVE_ARM_MARKER" ]]; then
    ACTIVE_ARM="$(cat "$ACTIVE_ARM_MARKER")"
    if [[ "$ACTIVE_ARM" != "$ARM" ]]; then
        echo -e "${RED}ERROR: --arm ${ARM} requested, but the diagnostic gate config on disk is${NC}"
        echo -e "${RED}       currently set to Arm ${ACTIVE_ARM} (per switch_diagnostic_arm.sh's marker).${NC}"
        echo -e "${RED}       Run experiments/scripts/switch_diagnostic_arm.sh ${ARM} and restart${NC}"
        echo -e "${RED}       Autoware first — otherwise the collected data will be mislabeled.${NC}"
        exit 1
    fi
    echo -e "  Diagnostic arm: ${GREEN}${ACTIVE_ARM}${NC} (matches --arm ${ARM})"
else
    echo -e "  ${YELLOW}Diagnostic arm: unknown (no switch_diagnostic_arm.sh marker found — assuming this matches --arm ${ARM})${NC}"
fi
echo ""

# ── Helper: run the Python experiment runner ──────────────────────────────────
run() {
    local campaign="$1"; local condition="$2"
    shift 2  # remaining args passed through (scenario, velocity-limit, etc.)
    if [[ "$ARM" == "B" ]]; then
        campaign="${campaign}_armB"
        condition="${condition}_armB"
    fi
    local goals_file_arg=()
    [[ -n "$GOALS_FILE" ]] && goals_file_arg=(--goals-file "$GOALS_FILE")
    local runway_arg=()
    # Placed last so it overrides any --fault-min-runway-m a case block might
    # hardcode (none currently do — this is the only source today).
    [[ -n "$FAULT_MIN_RUNWAY" ]] && runway_arg=(--fault-min-runway-m "$FAULT_MIN_RUNWAY")
    python3 "$RUNNER" \
        --campaign "$campaign" \
        --condition "$condition" \
        --goals "$GOALS" \
        --trials "$TRIALS" \
        --stuck-timeout 100 \
        --arm "$ARM" \
        "${goals_file_arg[@]}" \
        $DRY_RUN \
        $YES \
        "$@" \
        "${runway_arg[@]}"
}

# ── Helper: verify/edit avoidance policy ─────────────────────────────────────
avoidance_set() {
    local target="$1"  # "auto" or "manual"
    if [[ ! -f "$AVOIDANCE_CFG" ]]; then
        echo -e "${RED}ERROR: Avoidance config not found:${NC}"; echo "  $AVOIDANCE_CFG"; exit 1
    fi
    local current
    current=$(grep -m1 '^ *policy:' "$AVOIDANCE_CFG" | awk '{print $2}' | tr -d '"')
    if [[ "$current" == "$target" ]]; then
        echo -e "  Avoidance policy already \"${target}\" — no change needed."
        return
    fi
    if [[ -z "$DRY_RUN" ]]; then
        cp "$AVOIDANCE_CFG" "${AVOIDANCE_CFG}.backup"
        sed -i "s/^          policy: \"manual\"/          policy: \"${target}\"/" "$AVOIDANCE_CFG"
        echo -e "  Avoidance policy set to ${GREEN}\"${target}\"${NC} (backup: *.backup)"
    else
        echo -e "  ${YELLOW}[DRY-RUN] Would set avoidance policy to \"${target}\"${NC}"
    fi
}

avoidance_verify() {
    local expected="$1"
    local current
    current=$(grep -m1 '^ *policy:' "$AVOIDANCE_CFG" | awk '{print $2}' | tr -d '"')
    if [[ "$current" != "$expected" ]]; then
        echo -e "${RED}ERROR: Avoidance policy is \"${current}\", expected \"${expected}\".${NC}"
        echo -e "       Run:  sed -i 's/policy: \"${current}\"/policy: \"${expected}\"/' \\"
        echo -e "             $AVOIDANCE_CFG"
        exit 1
    fi
    echo -e "  Avoidance policy: ${GREEN}\"${current}\"${NC} — correct"
}

# ── Dispatch campaign ─────────────────────────────────────────────────────────
case "$CAMPAIGN" in

    nom_v5)
        echo -e "${BLUE}Nominal driving — 5 m/s velocity cap${NC}"
        trap 'echo -e "\n${YELLOW}Restoring velocity limit to default (11.11 m/s)...${NC}"; ros2 param set /planning/scenario_planning/external_velocity_limit_selector max_vel 11.11 2>/dev/null || true' EXIT
        run nom_v5 nom_v5 --velocity-limit 5.0
        ;;

    nom_v7)
        echo -e "${BLUE}Nominal driving — 7 m/s velocity cap${NC}"
        trap 'echo -e "\n${YELLOW}Restoring velocity limit to default (11.11 m/s)...${NC}"; ros2 param set /planning/scenario_planning/external_velocity_limit_selector max_vel 11.11 2>/dev/null || true' EXIT
        run nom_v7 nom_v7 --velocity-limit 7.0
        ;;

    nom_v11)
        echo -e "${BLUE}Nominal driving — 11.11 m/s (Autoware map speed limit, no external cap)${NC}"
        run nom_v11 nom_v11
        ;;

    obs_stuck)
        echo -e "${BLUE}Static obstacle — Autoware stops (avoidance=manual)${NC}"
        avoidance_verify "manual"
        run obs_stuck obs_stuck \
            --scenario "$SCENARIOS_DIR/obs_stuck.yaml"
        ;;

    obs_recovery)
        echo -e "${BLUE}Static obstacle — Autoware recovers (avoidance=auto)${NC}"
        echo -e "${YELLOW}NOTE: This will temporarily set avoidance policy to \"auto\".${NC}"
        echo -e "${YELLOW}      It will be restored to \"manual\" when the run completes.${NC}"
        echo ""
        avoidance_set "auto"
        # Trap ensures we restore even on Ctrl+C or error
        trap 'echo -e "\n${YELLOW}Restoring avoidance policy to manual...${NC}"; avoidance_set "manual"' EXIT
        run obs_recovery obs_recovery \
            --scenario "$SCENARIOS_DIR/obs_recovery.yaml"
        ;;

    obs_noescape)
        echo -e "${BLUE}Static obstacle — single-lane segment (no escape path)${NC}"
        avoidance_verify "manual"
        run obs_noescape obs_noescape \
            --scenario "$SCENARIOS_DIR/obs_noescape.yaml"
        ;;

    obs_singlelane)
        echo -e "${BLUE}Static obstacle — single-lane section, Signal 1 validation${NC}"
        avoidance_verify "manual"
        run obs_singlelane obs_singlelane \
            --scenario "$SCENARIOS_DIR/obs_singlelane.yaml"
        ;;

    obs_tooclosetoreact)
        echo -e "${BLUE}Static obstacle — 6m ahead, Signal 2 (TTC) validation${NC}"
        echo -e "${YELLOW}NOTE: Temporarily sets avoidance policy to \"auto\".${NC}"
        avoidance_set "auto"
        trap 'echo -e "\n${YELLOW}Restoring avoidance policy to manual...${NC}"; avoidance_set "manual"' EXIT
        run obs_tooclosetoreact obs_tooclosetoreact \
            --scenario "$SCENARIOS_DIR/obs_tooclosetoreact.yaml"
        ;;

    # ── Traffic light fault campaigns ─────────────────────────────────────────
    # Fault arming (revised 2026-07-24 — see fault_injector.py's module
    # docstring and _on_gt_pose for the full mechanism):
    #   0. Vehicle must first travel >= --fault-min-runway-m (default 150m,
    #      ground-truth straight-line distance, all goals share one start
    #      intersection) from trial start AND be moving. Without this, a
    #      wall-clock timer has no relationship to vehicle position, and
    #      reliably armed while still at/near the start intersection
    #      (confirmed in the tl_fault_s1..s4 data collected 2026-07-22 —
    #      every trial's first TL zone entry was the START intersection, not
    #      a downstream one).
    #   1. TL then arms IMMEDIATELY (no --fault-delay wait — REMOVED from
    #      this path 2026-07-25; it raced against how long the vehicle
    #      happened to dwell in the next zone, which is traffic-light-phase-
    #      dependent and varies trial to trial — a 30s fixed floor silently
    #      skipped a ~6s zone crossing in 3 of 4 goal_012 trials that
    #      session, only ever arming at whatever zone came after. See
    #      fault_injector.py's TL state-machine comment for the full
    #      writeup). --fault-delay itself was removed entirely 2026-07-26 —
    #      the same reasoning turned out to apply to IMU faults too once
    #      they gained their own zone-gating (see imu_fault_scale below).
    # TL fault design (revised 2026-07-22 — zone-triggered periodic):
    #   Each fault activates at EVERY TL detection zone entered after the
    #   runway above, stays active for up to 15s (or until the zone is
    #   exited, whichever is first), then an 8s recovery gap before re-arming
    #   for the NEXT zone — repeating for every intersection on the route.
    #   tl_fault_start / tl_fault_end timestamps (one pair per cycle) are
    #   written to the per-campaign fault_log.jsonl for CUSUM alignment; see
    #   docs/research_notes/periodic_fault_strategy.md for the design rationale.
    #
    #   S1: mild confidence degradation — may produce little behavioral change
    #       (Autoware still trusts the classification), establishing a baseline
    #       for what a mild sensor fault looks like to ST-GAT.
    #   S2: oscillating GREEN/RED — vehicle repeatedly starts/stops at each
    #       intersection as the signal toggles every 2.5s (5s period). Produces
    #       distinctive high-frequency velocity residuals that CUSUM accumulates.
    #   S3: UNKNOWN classification — over-caution, vehicle stops at intersections.
    #   S4: complete blackout — over-caution, no TL signal at all.

    tl_fault_s2)
        echo -e "${BLUE}TL fault S2: oscillating GREEN/RED (5s period) — 15s cap/cycle${NC}"
        run tl_fault_s2 tl_fault_s2 \
            --tl-fault tl_oscillate \
            --tl-params '{"period_s":5.0}' \
            --fault-duration 15
        ;;

    tl_fault_s3)
        echo -e "${BLUE}TL fault S3: UNKNOWN classification (over-caution) — 15s cap/cycle${NC}"
        run tl_fault_s3 tl_fault_s3 \
            --tl-fault tl_unknown \
            --fault-duration 15
        ;;

    tl_fault_s4)
        echo -e "${BLUE}TL fault S4: full blackout (no signal) — 15s cap/cycle${NC}"
        run tl_fault_s4 tl_fault_s4 \
            --tl-fault tl_blackout \
            --fault-duration 15
        ;;

    # Gradual-degradation counterpart to tl_confidence (added 2026-07-24) —
    # confidence_scale decays linearly from 1.0 to 0.0 over each 15s
    # fault-active window instead of stepping straight to 0.5, mirroring
    # imu_bias_ramp's "richer representation of the stack's behavior" idea
    # for the TL channel. Re-arms fresh at every TL zone like the S1-S4
    # tiers, not once for the whole trial.
    tl_fault_ramp)
        echo -e "${BLUE}TL fault RAMP: confidence 1.0 -> 0.0 over each 15s cap/cycle${NC}"
        run tl_fault_ramp tl_fault_ramp \
            --tl-fault tl_confidence_ramp \
            --tl-params '{"confidence_ramp_rate_per_s":0.1,"min_confidence_scale":0.0}' \
            --fault-duration 15
        ;;

    # ── Fixed-severity TL confidence tiers (added 2026-08-26) ───────────────────
    # Deliberately REINTRODUCES what tl_fault_s1 (fixed confidence_scale=0.5)
    # was removed for on 2026-07-24, for a specific, scoped reason the removal
    # didn't need to consider. That removal's own reasoning was correct for its
    # purpose (fault-signal verification / discriminability: the ramp visits
    # every severity level within one trial, so a fixed tier adds nothing
    # THERE). This campaign exists for a DIFFERENT purpose: a clean residual
    # dose-response curve for Layer 1 calibration validation
    # (docs/research_notes/tl_severity_sweep_lab_plan_2026-08-26.md). The ramp
    # is unsuitable for THAT purpose specifically because it confounds severity
    # with elapsed-time-since-fault-onset, which itself correlates with
    # physical proximity to the actual intersection (early-cycle = still
    # approaching, easy; late-cycle = at the decision point, hard) — confirmed
    # directly by reconstructing confidence_scale from the existing tl_fault_ramp
    # data and finding a non-monotonic dip-then-spike residual curve instead of
    # a clean dose-response (see experiments/scripts/tl_severity_sweep_analysis.py's
    # 2026-08-25 result). Holding severity FIXED for a whole zone dwell removes
    # that confound. Three levels chosen to bracket the ramp pilot's finding
    # that traffic_light_confidence residual becomes clearly elevated above
    # ~0.5 severity (1 - confidence_scale): a below-threshold point (0.3), a
    # near-threshold point (0.5), and an above-threshold point (0.7).
    tl_fault_fixed_030)
        echo -e "${BLUE}TL fault FIXED severity 0.3 (confidence_scale=0.7) — 15s cap/cycle${NC}"
        run tl_fault_fixed_030 tl_fault_fixed_030 \
            --tl-fault tl_confidence \
            --tl-params '{"confidence_scale":0.7}' \
            --fault-duration 15
        ;;

    tl_fault_fixed_050)
        echo -e "${BLUE}TL fault FIXED severity 0.5 (confidence_scale=0.5) — 15s cap/cycle${NC}"
        run tl_fault_fixed_050 tl_fault_fixed_050 \
            --tl-fault tl_confidence \
            --tl-params '{"confidence_scale":0.5}' \
            --fault-duration 15
        ;;

    tl_fault_fixed_070)
        echo -e "${BLUE}TL fault FIXED severity 0.7 (confidence_scale=0.3) — 15s cap/cycle${NC}"
        run tl_fault_fixed_070 tl_fault_fixed_070 \
            --tl-fault tl_confidence \
            --tl-params '{"confidence_scale":0.3}' \
            --fault-duration 15
        ;;

    # ── IMU fault campaigns ────────────────────────────────────────────────────
    # Constant-bias tiers (old imu_fault_s1..s4) REMOVED 2026-07-24. Root cause
    # traced (docs/design_decisions.md item 7 / TODO.md): autoware_ekf_localizer
    # runs enable_yaw_bias_estimation: true — a constant gyro bias is exactly
    # the error shape that state is designed to learn and subtract via periodic
    # pose/NDT corrections. Below the Mahalanobis twist gate (twist_gate_dist
    # 46.1), a constant bias gets absorbed with little observable effect —
    # that's not our fault injector failing to reach the stack, it's Autoware's
    # own mitigation doing its job. Since this system exists to detect faults
    # that matter to safety, not ones Autoware already neutralizes, discrete
    # tiers sitting entirely in the "gets absorbed" regime aren't useful data —
    # ramp (below) supersedes them: it sweeps continuously through both the
    # absorbed regime AND the gate-rejection cliff in a single trial. See
    # imu_scale_factor / imu_stuck_at below for fault shapes that don't share
    # yaw_bias's blind spot (both are additive-bias-shaped mitigations;
    # scale-factor and stuck-at are structurally different error shapes,
    # not just different magnitudes).
    #
    # Reinstated 2026-07-25 as a deliberate CONTROL condition, not a
    # reversal of the 2026-07-24 removal above: ramp/scale/stuck all show
    # real, large fault effects (EKF-GT divergence up to 18m on goal_012,
    # see docs/research_notes/ — but with nothing in this repo's own data
    # demonstrating the OTHER side of the boundary, i.e. that Autoware's
    # yaw_bias estimation actually absorbs a bias below its capacity. S1
    # is the smallest of the old four tiers (0.03 rad/s x 20s = 0.6 rad
    # accumulated, comfortably inside the "gets absorbed" regime per the
    # removal comment above) — expected result is near-nominal EKF-GT
    # divergence and no stuck/collision outcome. If it ISN'T absorbed
    # cleanly, that itself is worth knowing (the boundary moved). Gated on
    # bias_leadin_zones like imu_fault_s3 (see that campaign's comment).
    # on_seconds stays a real FIXED duration here (unlike imu_fault_scale/
    # imu_fault_stuck below, whose on-window was switched 2026-07-28 to end
    # on turn-zone exit instead) — deliberately, not an inconsistency.
    # gyro_bias_rads is added unconditionally in _on_imu regardless of true
    # rate, so a stationary vehicle waiting at a red light still integrates
    # the full bias continuously; an open-ended on-window could accumulate
    # heading error well past the 1.2 rad ceiling these magnitudes were
    # calibrated against (see the removal comment above re: old S2 breaking
    # localization within ~1s). Confirmed with the user before making this
    # scale/stuck-only, not imu_bias-too.
    # off_seconds reduced 30 -> 10 (2026-07-28): with on_seconds=20, a 30s
    # recovery gap made the total on+off budget (50s) longer than real
    # observed inter-zone transit times on some routes (as low as ~28-31s,
    # goal_007 bias_leadin zone1->zone2) — the injector's zone-wait loop
    # blocks through the whole off_seconds sleep with no zone polling during
    # it (_imu_bias_loop), so a zone whose entire dwell window falls inside
    # that sleep is silently skipped, not just delayed (confirmed: goal_007's
    # middle injection point never armed in a real imu_fault_s3 trial,
    # traced to exactly this). 10s keeps a real recovery gap while fitting
    # under the tightest observed gap; doesn't guarantee every reachable
    # zone fires every trial (routes with even tighter spacing can still
    # skip one) — see docs/fault_scenario_table.md's note on this.
    imu_fault_s1)
        echo -e "${BLUE}IMU constant-bias fault (S1, control condition): gyro 0.03 rad/s, 20s on / 10s off${NC}"
        run imu_fault_s1 imu_fault_s1 \
            --imu-fault imu_bias \
            --imu-params '{"accel_bias_ms2":0.0,"gyro_bias_rads":0.03,"on_seconds":20,"off_seconds":10}'
        ;;

    # Added 2026-07-26 as S1's paired "above threshold" probe, replacing
    # imu_fault_ramp in the default production suite (run_fault_campaigns.sh):
    # a ramp confounds elapsed time with magnitude (both grow together within
    # one trial, so you can't tell which one drove a reaction) — a fixed step
    # like this has an unambiguous onset the instant it activates, a cleaner
    # lead-time measurement, and pairs with S1 as a proper two-point
    # dose-response design instead of one continuous sweep. Reuses the old
    # redesigned S3 magnitude (0.08 rad/s x 15s = 1.2 rad accumulated) —
    # comfortably under the 0.15 rad/s that broke localization near-instantly
    # (old S2, pre-2026-07-23 redesign; see the removal comment above), so
    # there should be actual seconds of lead time to measure, not ~0.
    # NOTE: not yet validated under Arm B/the precomputed turn-zone gating
    # (fault_injector.py's _load_turn_zones, 2026-07-26 — gates this fault's
    # on-transitions on bias_leadin_zones, points 10m of arc-length before a
    # real turn, since accumulation over on-time is what matters for a
    # constant bias, not turning itself) — S1 (0.03) is confirmed absorbed,
    # but whether 0.08 is cleanly ABOVE the boundary (vs. still marginal) is
    # this campaign's own open question, not a known fact going in.
    # off_seconds reduced 30 -> 10 (2026-07-28) — see imu_fault_s1's comment
    # above for the full reasoning (same shared _imu_bias_loop zone-skip bug).
    imu_fault_s3)
        echo -e "${BLUE}IMU constant-bias fault (S3, above-threshold probe): gyro 0.08 rad/s, 15s on / 10s off${NC}"
        run imu_fault_s3 imu_fault_s3 \
            --imu-fault imu_bias \
            --imu-params '{"accel_bias_ms2":0.0,"gyro_bias_rads":0.08,"on_seconds":15,"off_seconds":10}'
        ;;

    # One-shot linear ramp: gyro bias grows 0 -> max_gyro_bias_rads at
    # gyro_bias_rate_rads_per_s per second, starting immediately once the
    # runway gate clears (no --fault-delay — removed 2026-07-26, see the TL
    # campaign comments above) and not zone-gated at all (deliberately —
    # accumulates over elapsed time regardless of geometry, unlike
    # imu_fault_scale/stuck below). Gives a single trial that walks through
    # absorbed -> cliff -> gate-rejected, with a well-defined "when did this
    # become unsafe" timestamp for lead-time measurement against Autoware's
    # own MRM trigger (Arm B in the two-arm design —
    # docs/theoretical_framework.md §4).
    # Default rate/max are a first guess, not validated: reaches 0.4 rad/s
    # (>2.5x old S2's instant-catastrophic 0.15) at ~133s of ramp time if the
    # gate is never crossed first. Runway clearing happens BEFORE the ramp
    # clock starts, so total trial time before max is reached can approach
    # --stuck-timeout in run() (100s as of 2026-07-28, down from 200 — this
    # campaign's 133s-to-max figure now EXCEEDS it, meaning a run where the
    # vehicle is still moving normally when the ramp maxes out is fine (stuck
    # detection only counts consecutive non-movement), but any run that also
    # stalls for other reasons before then will likely hit stuck_timeout
    # before imu_ramp_reached_max ever fires — not currently a problem since
    # this campaign isn't part of the active scenario table, but re-check
    # this margin before ever routinely collecting it again) — if a run times
    # out without a clear crossing (check imu_ramp_level / imu_ramp_reached_max
    # in fault_log.jsonl against the MRM trigger time), widen
    # max_gyro_bias_rads or slow the rate rather than assuming the gate wasn't
    # found.
    imu_fault_ramp)
        echo -e "${BLUE}IMU ramp fault: 0 -> 0.4 rad/s at 0.003 rad/s per second${NC}"
        run imu_fault_ramp imu_fault_ramp \
            --imu-fault imu_bias_ramp \
            --imu-params '{"gyro_bias_rate_rads_per_s":0.003,"max_gyro_bias_rads":0.4,"ramp_log_interval_s":5.0}'
        ;;

    # Multiplicative gain error: angular_velocity.z *= gyro_scale_factor,
    # instead of an additive offset. Structurally unabsorbable by yaw_bias
    # (an additive-constant state can't represent a multiplicative error), and
    # naturally state-dependent — near-zero error on straight roads (true rate
    # ~0), scaling up specifically during turns/intersections. Periodic on/off
    # like the old bias tiers, for clean reaction/recovery segments. Gated on
    # precomputed turn zones (fault_injector.py's _load_turn_zones,
    # 2026-07-26) — real turn locations per goal, not a live threshold guess.
    # No on_seconds (removed 2026-07-28): the on-window now ends when the
    # vehicle exits the turn zone, not after a fixed duration — a fixed
    # duration could expire while stopped at a red light inside the zone,
    # switching the fault off before the vehicle ever actually turned once
    # the light went green. Safe to do here specifically because this
    # fault's error is proportional to true yaw rate (~0 while stopped);
    # NOT done the same way for imu_bias below (see its own comment).
    # off_seconds reduced 30 -> 10 (2026-07-28) — same shared _imu_bias_loop
    # zone-skip bug as imu_fault_s1/s3 (see that comment above); applies here
    # too since the on-window is now zone-exit-ended but the off-window is
    # still a blocking sleep with no zone polling during it.
    imu_fault_scale)
        echo -e "${BLUE}IMU scale-factor fault: gyro x1.8, on until turn-zone exit / 10s off${NC}"
        run imu_fault_scale imu_fault_scale \
            --imu-fault imu_scale_factor \
            --imu-params '{"gyro_scale_factor":1.8,"off_seconds":10}'
        ;;

    # Frozen sensor: angular_velocity.z held at whatever it read the instant
    # the fault activated, ignoring true motion for the rest of the "on"
    # window. Also structurally unabsorbable by yaw_bias (the discrepancy
    # tracks whatever the vehicle actually does, not a constant), and almost
    # certain to blow through the Mahalanobis gate the moment the vehicle
    # actually turns — same "definite, unambiguous ground-truth event" spirit
    # as tl_blackout. Same precomputed turn-zone gating AND zone-exit-ended
    # on-window as imu_fault_scale above (no on_seconds, removed 2026-07-28,
    # same reasoning) — frozen value can't accumulate error from elapsed
    # stopped time the way imu_bias's additive offset can, so this is safe.
    # off_seconds reduced 30 -> 10 (2026-07-28) — see imu_fault_s1's comment
    # above for the full reasoning (same shared _imu_bias_loop zone-skip bug).
    imu_fault_stuck)
        echo -e "${BLUE}IMU stuck-at fault: gyro frozen at activation value, on until turn-zone exit / 10s off${NC}"
        run imu_fault_stuck imu_fault_stuck \
            --imu-fault imu_stuck_at \
            --imu-params '{"off_seconds":10}'
        ;;

    *)
        echo -e "${RED}Unknown campaign: ${CAMPAIGN}${NC}"
        echo "Valid campaigns: nom_v5  nom_v7  nom_v11  obs_stuck  obs_recovery  obs_noescape  obs_singlelane  obs_tooclosetoreact"
        echo "                 tl_fault_s2..s4  tl_fault_ramp  tl_fault_fixed_030/050/070  imu_fault_s1  imu_fault_s3  imu_fault_ramp  imu_fault_scale  imu_fault_stuck"
        exit 1
        ;;
esac

echo ""
echo -e "${GREEN}Done. Data in: experiments/data/${CAMPAIGN}/${NC}"
