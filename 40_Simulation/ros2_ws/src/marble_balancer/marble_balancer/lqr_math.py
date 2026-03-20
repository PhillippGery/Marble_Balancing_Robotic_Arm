"""
lqr_math.py

Linearized ball-on-plate model with PT1 robot delay.

System state: [x, x_dot, y, y_dot, alpha, omega_alpha, beta, omega_beta]
  x, y           - ball position on plate (m)
  alpha, beta    - plate tilt angles around X and Y axes (rad)
  omega_alpha    - actual plate angular velocity around X (rad/s)  — PT1 state
  omega_beta     - actual plate angular velocity around Y (rad/s)  — PT1 state

Control input: [omega_alpha_cmd, omega_beta_cmd]
  Velocity commands sent to MoveIt Servo (rad/s).
  The robot dynamics are modelled as a PT1 with time constant T:
    d(omega)/dt = (cmd - omega) / T

PT1 model rows (continuous):
  d(alpha)/dt      = omega_alpha
  d(omega_alpha)/dt = (omega_alpha_cmd - omega_alpha) / T  →  -omega_alpha/T + u/T
  d(beta)/dt       = omega_beta
  d(omega_beta)/dt  = (omega_beta_cmd - omega_beta) / T   →  -omega_beta/T + u/T
"""

import numpy as np
from scipy.linalg import expm, solve_discrete_are


# ── Physical parameters ───────────────────────────────────────────────────────
G  = 9.81
MB = 0.05
RB = 0.015
IB = (2.0 / 5.0) * MB * RB**2
C  = MB * G / (MB + IB / RB**2)   # ≈ (5/7)*g ≈ 7.0 m/s²

# Robot PT1 time constant (s) — tune if robot responds faster/slower
T_ROBOT = 0.35


# ── Continuous-time A, B (with PT1 robot model) ───────────────────────────────
#
#  State:   [x, xd, y, yd, alpha, omega_alpha, beta, omega_beta]
#  Control: [omega_alpha_cmd, omega_beta_cmd]

def _build_continuous(T: float):
    A = np.array([
        # x    xd    y    yd   al   om_al  be   om_be
        [0,    1,    0,    0,    0,    0,    0,    0   ],  # dx/dt = xd
        [0,    0,    0,    0,   -C,    0,    0,    0   ],  # dxd/dt = -C*alpha
        [0,    0,    0,    1,    0,    0,    0,    0   ],  # dy/dt = yd
        [0,    0,    0,    0,    0,    0,   -C,    0   ],  # dyd/dt = -C*beta
        [0,    0,    0,    0,    0,    1,    0,    0   ],  # dal/dt = omega_alpha
        [0,    0,    0,    0,    0, -1/T,    0,    0   ],  # dom_al/dt = -om_al/T + u1/T
        [0,    0,    0,    0,    0,    0,    0,    1   ],  # dbe/dt = omega_beta
        [0,    0,    0,    0,    0,    0,    0, -1/T   ],  # dom_be/dt = -om_be/T + u2/T
    ], dtype=float)

    B = np.array([
        [0,     0   ],
        [0,     0   ],
        [0,     0   ],
        [0,     0   ],
        [0,     0   ],
        [1/T,   0   ],   # omega_alpha_cmd → d(omega_alpha)/dt
        [0,     0   ],
        [0,     1/T ],   # omega_beta_cmd  → d(omega_beta)/dt
    ], dtype=float)

    return A, B


A_CONTINUOUS, B_CONTINUOUS = _build_continuous(T_ROBOT)


def discretize(A: np.ndarray, B: np.ndarray, dt: float):
    """ZOH discretization via matrix exponential."""
    n = A.shape[0]
    m = B.shape[1]
    M = np.zeros((n + m, n + m))
    M[:n, :n] = A
    M[:n, n:] = B
    eM = expm(M * dt)
    return eM[:n, :n], eM[:n, n:]


def compute_dlqr(Q: np.ndarray, R: np.ndarray, dt: float, T: float = T_ROBOT):
    """
    Compute discrete-time LQR gain K_d for the PT1 ball-on-plate model.

    Parameters
    ----------
    Q  : (8,8) state cost matrix
    R  : (2,2) control cost matrix
    dt : sample period (s)
    T  : PT1 robot time constant (s)

    Returns
    -------
    K_d : (2,8) gain matrix  — u[k] = -K_d @ x[k]
    Ad  : (8,8) discrete A
    Bd  : (8,2) discrete B
    """
    A, B = _build_continuous(T)
    Ad, Bd = discretize(A, B, dt)

    P = solve_discrete_are(Ad, Bd, Q, R)
    K_d = np.linalg.inv(R + Bd.T @ P @ Bd) @ (Bd.T @ P @ Ad)
    return K_d, Ad, Bd


# ── Default weights ───────────────────────────────────────────────────────────
#  State:   [x, vx, y, vy, alpha, omega_alpha, beta, omega_beta]
#
#  Tuning guide (see CLAUDE.md for full procedure):
#    Q[0]/Q[2]  (x, y position)     — larger = tighter centering, more overshoot risk
#    Q[1]/Q[3]  (vx, vy velocity)   — larger = more damping, reduces oscillation
#    Q[4]/Q[6]  (alpha, beta angle) — larger = plate tilts less, slower response
#    Q[5]/Q[7]  (omega_alpha/beta)  — larger = smoother rate changes
#    R diagonal                     — larger = less aggressive overall, both axes
#
#  Y axis (vy/omega_beta) uses higher weights than X because the TCP Lissajous
#  drives Y at 2× frequency (fb=2), producing 4× the pseudo-force vs X.
DEFAULT_Q = np.diag([100.0, 100.0, 200.0, 400.0, 5.0, 0.5, 5.0, 1.0])
#                     x      vx     y      vy↑    α    ωα   β    ωβ↑
DEFAULT_R = np.eye(2) * 5.0
