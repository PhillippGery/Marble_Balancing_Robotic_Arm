"""
ball_plate_env.py
-----------------
Fast standalone Gymnasium environment for ball-on-plate RL training.
No ROS2 dependency — runs 100-1000× faster than Gazebo for training.

Physics model:
  - Linear ball-on-plate dynamics (same A/B as lqr_math.py)
  - Nonlinear Coulomb + viscous friction (tanh smoothed for gradients)
  - TCP Lissajous pseudo-forces (optional, enabled per curriculum stage)
  - PT1 servo delay (T_ROBOT)
  - Domain randomisation: C, T_ROBOT, friction coefficients randomised each episode

State vector (8-D, same as marble_servo_controller):
  [x, vx, y, vy, alpha, omega_alpha, beta, omega_beta]

Observation (24-D, normalised to ~[-1, 1]):
  [state(8), lqr_output(2), tcp_state(4), error_history(10)]

Action (2-D): residual [Δω_alpha, Δω_beta] — added to LQR output
  Clipped to ±clip_budget deg/s (set by curriculum stage)

Curriculum stages (call env.set_stage(n)):
  0 — clip ±2°/s,  no Lissajous, no random perturbations
  1 — clip ±5°/s,  no Lissajous, random impulse perturbations
  2 — clip ±10°/s, Lissajous active, perturbations
  3 — clip ±20°/s, Lissajous active, perturbations  (full authority)

Install deps (once):
  pip install gymnasium stable-baselines3[extra] numpy scipy
"""

import math
import collections
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from scipy.linalg import solve_discrete_are

# ── Physical constants (mirrors lqr_math.py) ──────────────────────────────────
G       = 9.81
MB      = 0.05
RB      = 0.015
IB      = (2.0 / 5.0) * MB * RB ** 2
C_NOM   = MB * G / (MB + IB / RB ** 2)   # ≈ 7.0 m/s²
T_NOM   = 0.35                             # PT1 servo time constant (s)
DT      = 1.0 / 30.0                       # control timestep (s)

PLATE_HALF = 0.20    # m — terminate if marble exits plate
MAX_RATE   = math.radians(45.0)  # hard ceiling on combined LQR+RL command (rad/s)

# ── Default LQR weights (keep in sync with lqr_math.py) ──────────────────────
DEFAULT_Q = np.diag([100.0, 100.0, 200.0, 400.0, 5.0, 0.5, 5.0, 1.0])
DEFAULT_R = np.eye(2) * 5.0

# ── Curriculum stage definitions ──────────────────────────────────────────────
# (clip_budget_rad, use_lissajous, use_perturbations)
STAGES = [
    (math.radians(2.0),  False, False),
    (math.radians(5.0),  False, True),
    (math.radians(10.0), True,  True),
    (math.radians(20.0), True,  True),
]

# ── Normalisation constants ────────────────────────────────────────────────────
_NORM = np.array([
    0.20,   # x  (m)
    0.50,   # vx (m/s)
    0.20,   # y  (m)
    0.50,   # vy (m/s)
    0.30,   # alpha (rad)
    MAX_RATE,  # omega_alpha (rad/s)
    0.30,   # beta (rad)
    MAX_RATE,  # omega_beta (rad/s)
])
_NORM_LQR    = np.array([MAX_RATE, MAX_RATE])
_NORM_TCP    = np.array([0.30, 0.30, 0.40, 0.40])   # pos (m), vel (m/s)
_NORM_ERR    = np.array([0.20, 0.20] * 5)            # 5 × (x_err, y_err)


def _compute_lqr_gain(C: float, T: float) -> np.ndarray:
    """Build discrete LQR gain for given physical params."""
    A = np.array([
        [0, 1,  0,  0,  0,    0,  0,    0],
        [0, 0,  0,  0, -C,    0,  0,    0],
        [0, 0,  0,  1,  0,    0,  0,    0],
        [0, 0,  0,  0,  0,    0, -C,    0],
        [0, 0,  0,  0,  0,    1,  0,    0],
        [0, 0,  0,  0,  0, -1/T,  0,    0],
        [0, 0,  0,  0,  0,    0,  0,    1],
        [0, 0,  0,  0,  0,    0,  0, -1/T],
    ], dtype=float)
    B = np.array([
        [0, 0], [0, 0], [0, 0], [0, 0],
        [0, 0], [1/T, 0], [0, 0], [0, 1/T],
    ], dtype=float)
    from scipy.linalg import expm
    n, m = 8, 2
    M = np.zeros((n + m, n + m))
    M[:n, :n] = A * DT
    M[:n, n:] = B * DT
    eM = expm(M)
    Ad, Bd = eM[:n, :n], eM[:n, n:]
    P = solve_discrete_are(Ad, Bd, DEFAULT_Q, DEFAULT_R)
    K = np.linalg.inv(DEFAULT_R + Bd.T @ P @ Bd) @ (Bd.T @ P @ Ad)
    return K


# Pre-compute nominal LQR gain once
_K_NOM = _compute_lqr_gain(C_NOM, T_NOM)


class BallPlateEnv(gym.Env):
    """
    Gymnasium environment for residual LQR training on the ball-on-plate system.

    The agent outputs a residual [Δω_alpha, Δω_beta] that is ADDED to the LQR
    command. Combined output is clipped to ±MAX_RATE.
    """

    metadata = {'render_modes': ['human']}

    def __init__(self, stage: int = 0, render_mode=None):
        super().__init__()
        self.render_mode = render_mode

        # Curriculum
        self._stage = stage
        self._clip, self._use_lissajous, self._use_perturb = STAGES[stage]

        # Episode state
        self._state    = np.zeros(8)
        self._desired  = np.zeros(2)   # [x_d, y_d]
        self._err_hist = collections.deque(
            [np.zeros(2)] * 5, maxlen=5)   # last 5 (x_err, y_err)
        self._t        = 0.0
        self._step_num = 0
        self._params   = {}   # episode physical params (domain rand)
        self._K        = _K_NOM.copy()

        # TCP Lissajous (same defaults as servo_balancer.launch.py)
        self._amp_x  = 0.10
        self._amp_y  = 0.10
        self._omega0 = 2.0 * math.pi / 12.0
        self._fa     = 1
        self._fb     = 2
        self._delta  = math.pi / 2.0

        # Spaces
        obs_dim = 8 + 2 + 4 + 10   # state + lqr_out + tcp + err_hist
        self.observation_space = spaces.Box(
            low=-3.0, high=3.0, shape=(obs_dim,), dtype=np.float32)
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

        self._last_residual = np.zeros(2)

    # ── Curriculum control ────────────────────────────────────────────────────

    def set_stage(self, stage: int):
        stage = min(stage, len(STAGES) - 1)
        self._stage = stage
        self._clip, self._use_lissajous, self._use_perturb = STAGES[stage]

    def get_stage(self) -> int:
        return self._stage

    # ── Gym interface ─────────────────────────────────────────────────────────

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        # Domain randomisation — sample new physical params each episode
        rng = self.np_random
        self._params = {
            'C':     rng.uniform(6.5, 7.5),
            'T':     rng.uniform(0.25, 0.45),
            'mu_c':  rng.uniform(0.0, 0.04),   # Coulomb friction
            'mu_v':  rng.uniform(0.0, 0.08),   # viscous friction
        }
        self._K = _compute_lqr_gain(self._params['C'], self._params['T'])

        # Random initial marble position (small perturbation from centre)
        x0  = rng.uniform(-0.05, 0.05)
        y0  = rng.uniform(-0.05, 0.05)
        vx0 = rng.uniform(-0.10, 0.10)
        vy0 = rng.uniform(-0.10, 0.10)
        a0  = rng.uniform(-0.05, 0.05)
        b0  = rng.uniform(-0.05, 0.05)
        self._state = np.array([x0, vx0, y0, vy0, a0, 0.0, b0, 0.0])

        # Desired position: zero (balance at centre)
        self._desired = np.zeros(2)

        self._err_hist = collections.deque([np.zeros(2)] * 5, maxlen=5)
        self._t        = 0.0
        self._step_num = 0
        self._last_residual = np.zeros(2)

        return self._get_obs(), {}

    def step(self, action: np.ndarray):
        # Scale action from [-1,1] to [-clip, +clip]
        residual = np.clip(action, -1.0, 1.0) * self._clip
        self._last_residual = residual

        # LQR output
        error = self._state.copy()
        error[0] -= self._desired[0]
        error[2] -= self._desired[1]
        lqr_u = -self._K @ error

        # Combined command, clipped to MAX_RATE
        u = np.clip(lqr_u + residual, -MAX_RATE, MAX_RATE)

        # TCP Lissajous disturbance
        tcp_acc = np.zeros(2)
        if self._use_lissajous:
            ox = self._fa * self._omega0
            oy = self._fb * self._omega0
            x_tcp = self._amp_x * math.sin(ox * self._t + self._delta)
            y_tcp = self._amp_y * math.sin(oy * self._t)
            tcp_acc[0] = -(ox ** 2) * x_tcp
            tcp_acc[1] = -(oy ** 2) * y_tcp

        # Integrate dynamics
        self._state = self._integrate(self._state, u, tcp_acc)
        self._t += DT
        self._step_num += 1

        # Random impulse perturbation
        if self._use_perturb and self.np_random.random() < 0.005:
            self._state[1] += self.np_random.uniform(-0.3, 0.3)   # vx impulse
            self._state[3] += self.np_random.uniform(-0.3, 0.3)   # vy impulse

        # Error history
        x_err = self._state[0] - self._desired[0]
        y_err = self._state[2] - self._desired[1]
        self._err_hist.append(np.array([x_err, y_err]))

        # Termination: marble off plate
        terminated = (abs(self._state[0]) > PLATE_HALF * 0.95 or
                      abs(self._state[2]) > PLATE_HALF * 0.95)
        truncated  = self._step_num >= 500

        reward = self._compute_reward(x_err, y_err, residual, terminated)

        return self._get_obs(), reward, terminated, truncated, {}

    # ── Physics ───────────────────────────────────────────────────────────────

    def _integrate(self, s: np.ndarray, u: np.ndarray, tcp_acc: np.ndarray) -> np.ndarray:
        """Euler integration of ball-on-plate dynamics with nonlinear friction."""
        p = self._params
        x, vx, y, vy, alpha, om_a, beta, om_b = s

        def friction(v):
            return -p['mu_c'] * np.tanh(v / 0.005) - p['mu_v'] * v

        ax = -p['C'] * alpha - (5.0 / 7.0) * tcp_acc[0] + friction(vx)
        ay = -p['C'] * beta  - (5.0 / 7.0) * tcp_acc[1] + friction(vy)

        d_om_a = (u[0] - om_a) / p['T']
        d_om_b = (u[1] - om_b) / p['T']

        return np.array([
            x  + vx   * DT,
            vx + ax   * DT,
            y  + vy   * DT,
            vy + ay   * DT,
            alpha + om_a  * DT,
            om_a  + d_om_a * DT,
            beta  + om_b  * DT,
            om_b  + d_om_b * DT,
        ])

    # ── Reward ────────────────────────────────────────────────────────────────

    def _compute_reward(self, x_err, y_err, residual, terminated) -> float:
        r  = -10.0 * (x_err ** 2 + y_err ** 2)             # centering
        r -=  0.5  * (self._state[1] ** 2 + self._state[3] ** 2)  # velocity
        r -=  0.05 * float(np.dot(residual, residual))      # effort
        r +=  0.1                                            # survival
        if terminated:
            r -= 100.0
        return float(r)

    # ── Observation ───────────────────────────────────────────────────────────

    def _get_obs(self) -> np.ndarray:
        # LQR output (what classical controller just computed)
        error = self._state.copy()
        error[0] -= self._desired[0]
        error[2] -= self._desired[1]
        lqr_u = np.clip(-self._K @ error, -MAX_RATE, MAX_RATE)

        # TCP Lissajous state
        if self._use_lissajous:
            ox = self._fa * self._omega0
            oy = self._fb * self._omega0
            tcp_state = np.array([
                self._amp_x * math.sin(ox * self._t + self._delta),
                self._amp_y * math.sin(oy * self._t),
                self._amp_x * ox * math.cos(ox * self._t + self._delta),
                self._amp_y * oy * math.cos(oy * self._t),
            ])
        else:
            tcp_state = np.zeros(4)

        # Error history (flattened, oldest first)
        err_flat = np.array(list(self._err_hist)).flatten()

        obs = np.concatenate([
            self._state / _NORM,
            lqr_u / _NORM_LQR,
            tcp_state / _NORM_TCP,
            err_flat / _NORM_ERR,
        ]).astype(np.float32)

        return np.clip(obs, -3.0, 3.0)

    # ── Render ────────────────────────────────────────────────────────────────

    def render(self):
        if self.render_mode == 'human':
            x, _, y = self._state[0], self._state[1], self._state[2]
            print(f't={self._t:.2f}s  marble=({x*100:+.1f},{y*100:+.1f})cm  '
                  f'stage={self._stage}  clip={math.degrees(self._clip):.0f}°/s')
