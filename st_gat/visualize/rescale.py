"""
Coordinate and feature rescaling utilities.

Sequences and model outputs live in a normalised [0, 1] space.
Everything here converts them back to physical units for plots.

Position:  [0,1] within graph_bounds  →  metres (lanelet2 LocalCartesian)
Velocity:  [0,1] clamped range        →  m/s
Steering:  [0,1] clamped range        →  rad
Accel:     [0,1] clamped range        →  m/s²
"""

import numpy as np

# Ranges copied from pipeline/config.py — kept here to avoid importing
# ROS-dependent modules when the visualizer runs offline.
_VX_MIN, _VX_MAX =  0.0,  12.0   # m/s longitudinal
_VY_MIN, _VY_MAX = -0.4,   0.4   # m/s lateral
_S_MIN,  _S_MAX  = -0.5,   0.5   # rad  steering
_A_MIN,  _A_MAX  = -1.0,   1.0   # m/s² acceleration
_U_CAP           =  0.5          # m²   uncertainty cap


# ── Position ───────────────────────────────────────────────────────────────

def pos_to_m(scaled: np.ndarray, bounds) -> np.ndarray:
    """
    scaled: (..., 2)  in [0,1]
    bounds: [x_min, x_max, y_min, y_max]  in metres
    returns: (..., 2)  in metres
    """
    x_min, x_max, y_min, y_max = bounds
    out = np.asarray(scaled, dtype=float).copy()
    out[..., 0] = out[..., 0] * (x_max - x_min) + x_min
    out[..., 1] = out[..., 1] * (y_max - y_min) + y_min
    return out


def pos_var_to_m2(var_scaled: np.ndarray, bounds) -> np.ndarray:
    """
    var_scaled: (..., 2)  variance in [0,1]^2 space
    returns:    (..., 2)  variance in metres^2
    """
    x_min, x_max, y_min, y_max = bounds
    out = np.asarray(var_scaled, dtype=float).copy()
    out[..., 0] = out[..., 0] * (x_max - x_min) ** 2
    out[..., 1] = out[..., 1] * (y_max - y_min) ** 2
    return out


# ── Scalar features ────────────────────────────────────────────────────────

def vel_to_ms(scaled: np.ndarray) -> np.ndarray:
    """scaled: (..., 2) → m/s  [longitudinal, lateral]"""
    out = np.asarray(scaled, dtype=float).copy()
    out[..., 0] = out[..., 0] * (_VX_MAX - _VX_MIN) + _VX_MIN
    out[..., 1] = out[..., 1] * (_VY_MAX - _VY_MIN) + _VY_MIN
    return out


def vel_var_to_ms2(var_scaled: np.ndarray) -> np.ndarray:
    out = np.asarray(var_scaled, dtype=float).copy()
    out[..., 0] = out[..., 0] * (_VX_MAX - _VX_MIN) ** 2
    out[..., 1] = out[..., 1] * (_VY_MAX - _VY_MIN) ** 2
    return out


def steer_to_rad(scaled: np.ndarray) -> np.ndarray:
    return np.asarray(scaled, dtype=float) * (_S_MAX - _S_MIN) + _S_MIN


def steer_var_to_rad2(var_scaled: np.ndarray) -> np.ndarray:
    return np.asarray(var_scaled, dtype=float) * (_S_MAX - _S_MIN) ** 2


def accel_to_ms2(scaled: np.ndarray) -> np.ndarray:
    return np.asarray(scaled, dtype=float) * (_A_MAX - _A_MIN) + _A_MIN


def accel_var_to_ms4(var_scaled: np.ndarray) -> np.ndarray:
    return np.asarray(var_scaled, dtype=float) * (_A_MAX - _A_MIN) ** 2


def uncertainty_to_m2(scaled: np.ndarray) -> np.ndarray:
    """EKF covariance: [0,1] → m²  (cap = 0.5 m²)"""
    return np.asarray(scaled, dtype=float) * _U_CAP
