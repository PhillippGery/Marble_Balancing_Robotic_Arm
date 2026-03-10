"""
lqr_math.py

Ports the linearized ball-on-plate model from MarbelSysModel_mfile.m to Python
and computes a discrete-time LQR gain matrix K_d.

System state: [x, x_dot, y, y_dot, alpha, alpha_dot, beta, beta_dot]
  x, y       - ball position on plate (m)
  alpha, beta - plate tilt angles around X and Y axes (rad)
Control input: [alpha_ddot, beta_ddot] - plate angular accelerations (rad/s^2)

The continuous A, B matrices are identical to the MATLAB linearization.
They are then discretized via matrix exponential and K_d is solved with
scipy.linalg.solve_discrete_are (equivalent to MATLAB's dlqr).
"""

import numpy as np
from scipy.linalg import expm, solve_discrete_are


# ── Physical parameters (matches MATLAB) ─────────────────────────────────────
G  = 9.81          # gravitational acceleration (m/s^2)
MB = 0.05          # ball mass (kg)
RB = 0.015         # ball radius (m)
IB = (2.0 / 5.0) * MB * RB**2   # ball moment of inertia (kg·m^2)
C  = MB * G / (MB + IB / RB**2) # = (5/7)*g  — linearized coupling constant


# ── Continuous-time A, B matrices (linearized at equilibrium x=0, u=0) ──────
#
#  State ordering: [x, xd, y, yd, alpha, alphad, beta, betad]
#  Control:        [alpha_ddot, beta_ddot]
#
#  From the Jacobian of f_nonlin evaluated at equilibrium:
#    d(x_dot)/dt   = 0*x + 0*xd + 0*y + 0*yd - C*alpha + 0 + 0 + 0
#    d(alpha)/dt   = alpha_dot
#    d(alpha_dot)  = alpha_ddot  (input)
#  (same pattern for y / beta)

A_CONTINUOUS = np.array([
    # x    xd    y    yd   al   ald   be   bed
    [0,    1,    0,    0,    0,    0,    0,    0],   # dx/dt   = x_dot
    [0,    0,    0,    0,   -C,   0,    0,    0],   # dxd/dt  = -C*alpha
    [0,    0,    0,    1,    0,    0,    0,    0],   # dy/dt   = y_dot
    [0,    0,    0,    0,    0,    0,   -C,   0],   # dyd/dt  = -C*beta
    [0,    0,    0,    0,    0,    1,    0,    0],   # dal/dt  = alpha_dot
    [0,    0,    0,    0,    0,    0,    0,    0],   # dald/dt = u1
    [0,    0,    0,    0,    0,    0,    0,    1],   # dbe/dt  = beta_dot
    [0,    0,    0,    0,    0,    0,    0,    0],   # dbed/dt = u2
], dtype=float)

B_CONTINUOUS = np.array([
    [0, 0],
    [0, 0],
    [0, 0],
    [0, 0],
    [0, 0],
    [1, 0],   # alpha_ddot = u1
    [0, 0],
    [0, 1],   # beta_ddot  = u2
], dtype=float)


def discretize(A: np.ndarray, B: np.ndarray, dt: float):
    """
    Zero-order-hold (ZOH) discretization of continuous (A, B) pair.

    Ad = expm(A * dt)
    Bd = (Ad - I) @ inv(A) @ B   when A is invertible,
         approximated as dt*B for near-singular A rows.

    Returns: Ad (8x8), Bd (8x2)
    """
    n = A.shape[0]
    # Build augmented matrix for exact ZOH: M = [[A, B], [0, 0]]
    m = B.shape[1]
    M = np.zeros((n + m, n + m))
    M[:n, :n] = A
    M[:n, n:] = B
    eM = expm(M * dt)
    Ad = eM[:n, :n]
    Bd = eM[:n, n:]
    return Ad, Bd


def compute_dlqr(Q: np.ndarray, R: np.ndarray, dt: float):
    """
    Compute the discrete-time LQR gain K_d such that u[k] = -K_d @ x[k]
    minimises sum_k ( x'Qx + u'Ru ).

    Uses scipy.linalg.solve_discrete_are (DARE) — equivalent to MATLAB dlqr().

    Parameters
    ----------
    Q  : (8,8) state cost matrix
    R  : (2,2) control cost matrix
    dt : sample period (s)

    Returns
    -------
    K_d : (2,8) gain matrix
    Ad  : (8,8) discrete A
    Bd  : (8,2) discrete B
    """
    Ad, Bd = discretize(A_CONTINUOUS, B_CONTINUOUS, dt)

    # Solve Discrete Algebraic Riccati Equation: P = Ad'PA - (Ad'PBd)(R+Bd'PBd)^{-1}(Bd'PAd) + Q
    P = solve_discrete_are(Ad, Bd, Q, R)

    # Optimal gain
    K_d = np.linalg.inv(R + Bd.T @ P @ Bd) @ (Bd.T @ P @ Ad)
    return K_d, Ad, Bd


# ── Default LQR weights (identical to MATLAB) ─────────────────────────────────
DEFAULT_Q = np.diag([1000.0, 100.0, 1000.0, 100.0, 1.0, 0.1, 1.0, 0.1])
DEFAULT_R = np.eye(2) * 0.01
