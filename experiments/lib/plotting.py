"""
Shared plotting utilities for this project's map/trajectory/fault
visualizations.

Consolidates what plot_routes.py, plot_fault_plan.py, and plot_fault_impact.py
had each partially re-derived or cross-imported from each other via
sys.path/module hacks: map loading, lanelet geometry helpers, bounding-box
computation, map-background rendering, outcome styling, and zone-drawing
conventions. Those three scripts now import from here instead of from one
another, so there is one place that defines e.g. what color a fault zone or
a "goal reached" trial gets, not three.

Also home to the NEW primitives the coming model-prediction-distribution
plots need (docs/theoretical_framework.md §5's divergence trace, TODO.md
Phase 1.5's "predicted mean ± variance vs. observed value" plots) — a shaded
mean±std band and fault-window shading — so those plots are built on the same
foundation as the map-based ones instead of starting from scratch.
"""

import math

import lanelet2
from autoware_lanelet2_extension_python.projection import MGRSProjector

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: F401 (re-exported for convenience — callers already need pyplot)
from matplotlib.patches import Polygon, Circle

import numpy as np


# ── Lanelet2 map loading ─────────────────────────────────────────────────────

def make_projector():
    """Autoware's own map loader (autoware_map_projection_loader) projects
    lanelet2 maps with lanelet::projection::MGRSProjector when no
    map_projector_info.yaml is present (the case for this project's map) —
    NOT a generic UtmProjector/LocalCartesianProjector with a guessed lat/lon
    origin. See CLAUDE.md's "Lanelet2 map projection" gotcha: a guessed
    origin is off by 1000s-10000s of metres from the real AWSIM/ROS2 frame."""
    return MGRSProjector(lanelet2.io.Origin(0.0, 0.0))


def load_map(map_file):
    projector = make_projector()
    map_data, _errs = lanelet2.io.loadRobust(map_file, projector)
    return map_data


# The vehicle's fixed spawn point in map coordinates — same value every
# script needs when drawing a "start" marker or seeding a route's bbox.
SPAWN = (81384.53, 49921.95)


# ── Lanelet geometry helpers ─────────────────────────────────────────────────

def ll_attr(ll, key, default=''):
    try:
        return ll.attributes[key]
    except Exception:
        return default


def lanelet_polygon_xy(ll):
    left = [(p.x, p.y) for p in ll.leftBound]
    right = [(p.x, p.y) for p in reversed(list(ll.rightBound))]
    return left + right


def bbox_hit(ll, xmin, xmax, ymin, ymax):
    for p in ll.centerline:
        if xmin <= p.x <= xmax and ymin <= p.y <= ymax:
            return True
    return False


def traffic_light_points(map_data, xmin, xmax, ymin, ymax):
    """Midpoint of each traffic-light-head linestring ('refers' role of
    traffic_light regulatory elements), deduped by regulatory element id,
    trimmed to the given bbox."""
    seen_reg = set()
    pts = []
    for ll in map_data.laneletLayer:
        for reg in ll.regulatoryElements:
            if ll_attr(reg, 'subtype') != 'traffic_light' or reg.id in seen_reg:
                continue
            seen_reg.add(reg.id)
            try:
                refers = reg.parameters['refers']
            except Exception:
                continue
            for ls in refers:
                pts_xy = [(p.x, p.y) for p in ls]
                mx = sum(p[0] for p in pts_xy) / len(pts_xy)
                my = sum(p[1] for p in pts_xy) / len(pts_xy)
                if xmin <= mx <= xmax and ymin <= my <= ymax:
                    pts.append((mx, my))
    return pts


# ── Bounding box ─────────────────────────────────────────────────────────────

def bbox_with_margin(xs, ys, margin):
    """(xmin, xmax, ymin, ymax) around the given points, padded by margin
    metres — the same min()/max()+-margin pattern each plotting script used
    to compute inline, separately."""
    return min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin


# ── Map background rendering ────────────────────────────────────────────────

ROAD_FACECOLOR = '#dddddd'
ROAD_EDGECOLOR = '#bbbbbb'


def draw_map_background(ax, map_data, xmin, xmax, ymin, ymax,
                         facecolor=ROAD_FACECOLOR, edgecolor=ROAD_EDGECOLOR, zorder=1):
    """Road-lanelet polygons trimmed to the given bbox — the
    for-ll-in-laneletLayer loop plot_routes.py/plot_fault_plan.py/
    plot_fault_impact.py each had their own near-identical copy of."""
    for ll in map_data.laneletLayer:
        if ll_attr(ll, 'subtype') != 'road':
            continue
        if not bbox_hit(ll, xmin, xmax, ymin, ymax):
            continue
        ax.add_patch(Polygon(lanelet_polygon_xy(ll), closed=True,
                              facecolor=facecolor, edgecolor=edgecolor,
                              linewidth=0.3, zorder=zorder))


def style_map_axes(ax, xmin, xmax, ymin, ymax, legend=True, legend_fontsize=7):
    """Common finishing touches for a map panel: equal aspect, axis limits,
    map-coordinate labels, legend — repeated verbatim at the end of every
    map-plotting function across all three scripts."""
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_aspect('equal')
    ax.set_xlabel('map x (m)')
    ax.set_ylabel('map y (m)')
    if legend:
        ax.legend(loc='best', fontsize=legend_fontsize)


# ── Outcome styling ──────────────────────────────────────────────────────────
# Reconciles plot_routes.py's STATUS_COLOR (color only) and
# plot_fault_impact.py's OUTCOME_STYLE (color+marker+label) into one
# convention — both already agreed on the colors (green=reached,
# red=stuck/fatal), just kept two separately-maintained copies of that fact.

OUTCOME_STYLE = {
    'goal_reached': dict(color='#2ca02c', marker='o', label='goal reached'),
    'stuck':        dict(color='#d62728', marker='X', label='stuck'),
    'fatal':        dict(color='#d62728', marker='X', label='fatal (did not reach goal)'),
}
DEFAULT_OUTCOME_STYLE = dict(color='#ff7f0e', marker='x', label='unknown')


def outcome_style(outcome: str) -> dict:
    return OUTCOME_STYLE.get(outcome, DEFAULT_OUTCOME_STYLE)


# ── Zone-drawing conventions (moved from plot_fault_plan.py, which
#    plot_fault_impact.py used to import these from directly) ──────────────

TL_COLOR = '#f1c40f'
TURN_COLOR = '#2ca02c'
LEADIN_COLOR = '#9467bd'
LANE_CHANGE_COLOR = '#999999'
RUNWAY_COLOR = '#555555'
INJECTION_COLOR = '#d62728'
FAULT_END_COLOR = '#17becf'
ROUTE_COLOR = '#1f77b4'
UNREACHABLE_COLOR = '#bbbbbb'


def draw_injection_points(ax, points, color=INJECTION_COLOR, label='injection point'):
    for i, (x, y) in enumerate(points):
        ax.plot(x, y, marker='*', color=color, markersize=14,
                markeredgecolor='black', markeredgewidth=0.6, zorder=6,
                label=label if i == 0 else None)


def draw_fault_end_points(ax, points, color=FAULT_END_COLOR, label='fault end'):
    """points may contain None (no stored end point for a zone, or a
    fixed-timer fault type with no geometric end at all) — skipped, not
    plotted as a fabricated position."""
    first = True
    for pt in points:
        if pt is None:
            continue
        x, y = pt
        ax.plot(x, y, marker='P', color=color, markersize=12,
                markeredgecolor='black', markeredgewidth=0.6, zorder=6,
                label=label if first else None)
        first = False


def draw_zones_with_reachability(ax, zones_xyr, injection_pts, radius, color, zone_label, kind_label):
    """zones_xyr: [(x, y, reachable), ...] for the zone CIRCLE (real
    geometry — drawn regardless of reachability); injection_pts: [(x, y),
    ...] same length/order, the actual arming-marker position for each (may
    differ from the circle center). Reachable zones get the real color + an
    injection star; unreachable ones (at/before the runway-clear point) are
    greyed out with no star, since a fault can never actually arm there on
    this route.

    zone_label is the CIRCLE's own legend entry (e.g. "IMU turn zone
    (r=15m)")."""
    reach_label_done = unreach_label_done = False
    for (x, y, reachable), (ix, iy) in zip(zones_xyr, injection_pts):
        if reachable:
            ax.add_patch(Circle((x, y), radius, facecolor=color, alpha=0.18,
                                 edgecolor=color, linewidth=1.2, zorder=4,
                                 label=zone_label if not reach_label_done else None))
            ax.plot(ix, iy, marker='*', color=INJECTION_COLOR, markersize=14,
                    markeredgecolor='black', markeredgewidth=0.6, zorder=6,
                    label=f'injection point ({kind_label})' if not reach_label_done else None)
            reach_label_done = True
        else:
            ax.add_patch(Circle((x, y), radius, facecolor=UNREACHABLE_COLOR, alpha=0.25,
                                 edgecolor=UNREACHABLE_COLOR, linewidth=1.0, zorder=4))
            ax.plot(x, y, marker='x', color=UNREACHABLE_COLOR, markersize=7, zorder=5,
                    label='unreachable zone (before runway-clear)' if not unreach_label_done else None)
            unreach_label_done = True


# ── Time-series conventions ──────────────────────────────────────────────────

def shade_fault_windows(ax, windows, color=INJECTION_COLOR, alpha=0.15, zorder=0):
    """windows: [{'start': t0, 'end': t1, ...}, ...] — generalizes
    plot_fault_impact.py's local annotate() closure so future residual-trace
    plots (st_gat/residuals.py's per-timestep traces) can shade fault-active
    regions the same way without re-deriving it."""
    for w in windows:
        ax.axvspan(w['start'], w['end'], color=color, alpha=alpha, zorder=zorder)


def mark_event_time(ax, t, color='black', linestyle=':', linewidth=1.5, zorder=1, label=None):
    """A single vertical marker at time t (e.g. permanent-stop time, MRM
    trigger, fault onset) — skipped cleanly if t is None/NaN rather than
    raising, since these ground-truth markers are frequently absent (a
    nominal trial has no fault onset; a trial that reached its goal has no
    permanent-stop time)."""
    if t is None or (isinstance(t, float) and math.isnan(t)):
        return
    ax.axvline(t, color=color, linestyle=linestyle, linewidth=linewidth, zorder=zorder, label=label)


# ── Distribution / divergence-trace panels ──────────────────────────────────
# New primitives for visualizing what the ST-GAT model predicted vs what
# actually happened — the next kind of plot this project needs (predicted
# mean ± variance vs observed value, per docs/theoretical_framework.md §5 /
# TODO.md Phase 1.5), built on the same module as the map-based plots rather
# than started from scratch.

def plot_mean_variance_band(ax, t, mean, var, actual=None,
                             label='predicted', actual_label='actual',
                             color='#1f77b4', actual_color='#d62728',
                             n_std=1.0, zorder=2):
    """Shaded predicted-mean +/- n_std*sqrt(var) band, with the actual
    observed value overlaid as a line on top. This is the core panel type
    for "does the predicted distribution widen / diverge from reality under
    a fault" — reusable across every feature (traffic_light_state, position,
    ...) rather than each future plot re-deriving its own fill_between call.
    """
    mean = np.asarray(mean, dtype=float)
    std  = np.sqrt(np.asarray(var, dtype=float))
    ax.plot(t, mean, color=color, linewidth=1.3, zorder=zorder, label=label)
    ax.fill_between(t, mean - n_std * std, mean + n_std * std,
                     color=color, alpha=0.2, zorder=max(zorder - 1, 0), linewidth=0)
    if actual is not None:
        ax.plot(t, actual, color=actual_color, linewidth=1.1, alpha=0.85,
                zorder=zorder + 1, label=actual_label)
