"""
marble_ekf.py
-------------
Extended Kalman Filter for the 8-D ball-on-plate system.

State:   [x, vx, y, vy, alpha, omega_alpha, beta, omega_beta]
  x, y           — marble position on plate (m)
  vx, vy         — marble velocity (m/s)
  alpha          — plate pitch (rad), rotates about world Y, drives x marble
  omega_alpha    — plate pitch rate (rad/s)
  beta           — plate roll  (rad), rotates about world X, drives y marble
  omega_beta     — plate roll  rate (rad/s)

Input:   [omega_alpha_cmd, omega_beta_cmd]
  LQR velocity commands in model frame (rad/s, BEFORE hardware sign inversion).

Measurements:  [x_marble, y_marble, alpha, beta]
  from marble odometry and plate_tcp TF quaternion.

Process model (nonlinear, ball-on-plate Lagrangian):
  ẋ     = vx
  v̇x    = (mb·x·ωα² + mb·y·ωα·ωβ − mb·g·sin(α)) / (mb + Ib/rb²)
  ẏ     = vy
  v̇y    = (mb·y·ωβ² + mb·x·ωα·ωβ − mb·g·sin(β)) / (mb + Ib/rb²)
  α̇     = ωα
  ω̇α    = (u_α − ωα) / T_ROBOT          ← PT1 servo model
  β̇     = ωβ
  ω̇β    = (u_β − ωβ) / T_ROBOT          ← PT1 servo model

State propagation:  4th-order Runge-Kutta
Covariance:         Euler linearisation  P⁻ = Fd·P·Fdᵀ + Q·dt
Measurement update: linear H (x, y, α, β observed directly) → standard KF equations

Tuning
------
  Q[5,5] / Q[7,7]  — omega_alpha / omega_beta process noise
      Main knob.  Increase if omega estimates are too sluggish or lag commands.
      Decrease if estimates are noisy.
  Q[1,1] / Q[3,3]  — vx / vy process noise
      Increase if marble velocity estimates lag position changes.
  R[0,0] / R[1,1]  — marble position measurement noise (x, y)
      Increase if odom is noisy (marble bouncing, spinning).
  R[2,2] / R[3,3]  — plate angle measurement noise (alpha, beta)
      Usually very small — TF is accurate.
"""

import numpy as np
from marble_balancer.lqr_math import G, MB, RB, IB, T_ROBOT

_M_EFF = MB + IB / RB**2   # effective ball inertial mass  mb + Ib/rb²


# ── Nonlinear process function ────────────────────────────────────────────────

def _f(x: np.ndarray, u: np.ndarray) -> np.ndarray:
    """Continuous-time nonlinear dynamics  ẋ = f(x, u)."""
    x0, vx, x2, vy, alpha, wa, beta, wb = x
    ua, ub = u
    d = np.empty(8)
    d[0] = vx
    d[1] = (MB * x0 * wa**2 + MB * x2 * wa * wb - MB * G * np.sin(alpha)) / _M_EFF
    d[2] = vy
    d[3] = (MB * x2 * wb**2 + MB * x0 * wa * wb - MB * G * np.sin(beta))  / _M_EFF
    d[4] = wa
    d[5] = (ua - wa) / T_ROBOT
    d[6] = wb
    d[7] = (ub - wb) / T_ROBOT
    return d


# ── Analytical Jacobian F = ∂f/∂x ────────────────────────────────────────────

def _F_jac(x: np.ndarray) -> np.ndarray:
    """Analytical Jacobian  F = ∂f/∂x  evaluated at state x (8×8)."""
    x0, _, x2, _, alpha, wa, beta, wb = x
    F = np.zeros((8, 8))
    # ẋ = vx
    F[0, 1] = 1.0
    # v̇x
    F[1, 0] = MB * wa**2          / _M_EFF
    F[1, 2] = MB * wa * wb        / _M_EFF
    F[1, 4] = -MB * G * np.cos(alpha) / _M_EFF
    F[1, 5] = (2.0 * MB * x0 * wa + MB * x2 * wb) / _M_EFF
    F[1, 7] = MB * x2 * wa        / _M_EFF
    # ẏ = vy
    F[2, 3] = 1.0
    # v̇y
    F[3, 0] = MB * wa * wb        / _M_EFF
    F[3, 2] = MB * wb**2          / _M_EFF
    F[3, 5] = MB * x0 * wb        / _M_EFF
    F[3, 6] = -MB * G * np.cos(beta)  / _M_EFF
    F[3, 7] = (2.0 * MB * x2 * wb + MB * x0 * wa) / _M_EFF
    # α̇ = ωα
    F[4, 5] = 1.0
    # ω̇α = (u − ωα) / T  →  −1/T on ωα
    F[5, 5] = -1.0 / T_ROBOT
    # β̇ = ωβ
    F[6, 7] = 1.0
    # ω̇β = (u − ωβ) / T  →  −1/T on ωβ
    F[7, 7] = -1.0 / T_ROBOT
    return F


# ── Measurement matrix ────────────────────────────────────────────────────────
# We directly observe x, y, alpha, beta — no Jacobian needed for H.

_H = np.array([
    [1, 0, 0, 0, 0, 0, 0, 0],   # x marble
    [0, 0, 1, 0, 0, 0, 0, 0],   # y marble
    [0, 0, 0, 0, 1, 0, 0, 0],   # alpha (plate pitch)
    [0, 0, 0, 0, 0, 0, 1, 0],   # beta  (plate roll)
], dtype=float)


# ── EKF class ─────────────────────────────────────────────────────────────────

class MarbleEKF:
    """Extended Kalman Filter for the 8-D ball-on-plate system."""

    DEFAULT_Q = np.diag([
        1e-4,   # x
        5e-3,   # vx
        1e-4,   # y
        5e-3,   # vy
        1e-5,   # alpha        — TF-measured, model is accurate
        4e-2,   # omega_alpha  — PT1 approximation uncertainty (main tuning knob)
        1e-5,   # beta
        4e-2,   # omega_beta   — PT1 approximation uncertainty (main tuning knob)
    ])

    DEFAULT_R = np.diag([
        1e-8,   # x marble  — near-zero for Gazebo perfect odom → raise to ~1e-3 for real camera
        1e-8,   # y marble  — same
        1e-8,   # alpha     (TF accurate — keep small on real hardware too)
        1e-8,   # beta
    ])

    def __init__(
        self,
        Q:  np.ndarray = None,
        R:  np.ndarray = None,
        P0: np.ndarray = None,
    ):
        self.x = np.zeros(8)
        self.P = np.eye(8) * 0.1 if P0 is None else P0.copy()
        self.Q = self.DEFAULT_Q.copy() if Q is None else Q.copy()
        self.R = self.DEFAULT_R.copy() if R is None else R.copy()

    def reset(self, x0: np.ndarray = None) -> None:
        """Reset state and covariance. Warm-start with x0 if provided."""
        self.x = np.zeros(8) if x0 is None else x0.copy()
        self.P = np.eye(8) * 0.1

    def predict(self, u: np.ndarray, dt: float) -> None:
        """
        EKF predict step.

        State:      RK4 integration of f(x, u) over dt
        Covariance: Euler linearisation  P⁻ = Fd·P·Fdᵀ + Q·dt
                    Fd = I + Fc·dt  (first-order ZOH approximation)
        """
        if dt <= 0.0 or dt > 0.5:
            return

        # RK4 state propagation
        k1 = _f(self.x,                u)
        k2 = _f(self.x + 0.5 * dt * k1, u)
        k3 = _f(self.x + 0.5 * dt * k2, u)
        k4 = _f(self.x +       dt * k3, u)
        self.x = self.x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

        # Covariance propagation (Jacobian evaluated at updated x for stability)
        Fc = _F_jac(self.x)
        Fd = np.eye(8) + Fc * dt
        self.P = Fd @ self.P @ Fd.T + self.Q * dt

    def update(self, z: np.ndarray) -> None:
        """
        EKF update step with linear measurement model  z = H·x.

        z = [x_marble, y_marble, alpha, beta]  (4-D)
        """
        innov = z - _H @ self.x                  # innovation
        S     = _H @ self.P @ _H.T + self.R      # innovation covariance
        K     = self.P @ _H.T @ np.linalg.inv(S) # Kalman gain
        self.x = self.x + K @ innov
        self.P = (np.eye(8) - K @ _H) @ self.P
