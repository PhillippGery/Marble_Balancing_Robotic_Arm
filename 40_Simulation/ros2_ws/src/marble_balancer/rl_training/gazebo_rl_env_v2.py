"""
gazebo_rl_env_v2.py
-------------------
Gymnasium environment wrapping live Gazebo for online TD3 training.
ENHANCED VERSION with 18-D observation and Random Jitter Walk disturbance.

This env IS the controller during training — do NOT run marble_servo_controller
simultaneously. It owns the servo command loop, LQR computation, and episode
management (delete → home → spawn → land).

KEY IMPROVEMENTS OVER V1:
  • 18-D observation (vs 32-D):
    - 8-D state (normalized)
    - 6-D TCP velocity window (allows NN to learn pseudo-force dynamics)
    - 2-D marble target (x_desired, y_desired)
    - 2-D action history
  • Random Jitter Walk: 50% episodes use random TCP velocity walk (insteadof always Lissajous)
  • Sharper reward: pos_reward = exp(-100.0 * (x² + y²))
  • Tilt penalty to reduce plate angles during high-acceleration TCP moves
  • Headless Gazebo: compatible with gzserver (no GUI required)

HEADLESS GAZEBO TUNING:
  To enable faster-than-realtime training, edit your Gazebo world file:
    <real_time_update_rate>0</real_time_update_rate>  <!-- disable real-time limit -->
    <max_step_size>0.01</max_step_size>                <!-- 10 ms steps for 30 Hz servo -->
  This lets Gazebo run at wall-clock speed uncapped, accelerating training ~2-3×.

Observation (21-D):
  obs[0:8]   = state / _NORM                    # [x, vx, y, vy, alpha, ωα, beta, ωβ]
  obs[8:17]  = tcp_vel_window / _TCP_NORM       # [vx_t, vy_t, vz_t, vx_{t-1}, vy_{t-1}, vz_{t-1}, vx_{t-2}, vy_{t-2}, vz_{t-2}]
  obs[17:19] = target / 0.20                    # [x_desired, y_desired]
  obs[19:21] = prev_action / 1.0                # [last_residual_x, last_residual_y]

Action (2-D, in [-1, 1]):
  residual = action * lambda              # Δω added to LQR output (rad/s)
  lambda anneals with curriculum stage: 5 → 10 → 15 deg/s

Servo command (matches marble_servo_controller.py exactly):
  frame_id  = 'base_link'
  angular.x = omega_beta_cmd   (world X → roll → Y marble dynamics)
  angular.y = omega_alpha_cmd  (world Y → pitch → X marble dynamics, negated for yaw≈180°)
"""

import os
import sys
import math
import time
import subprocess
import collections
import numpy as np

# Add workspace paths for imports (allows running from rl_training/ directory)
_ws_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
_src_path = os.path.join(_ws_root, 'src')
_install_path = os.path.join(_ws_root, 'install', 'lib', 'python3.10', 'site-packages')
if _src_path not in sys.path:
    sys.path.insert(0, _src_path)
if _install_path not in sys.path:
    sys.path.insert(0, _install_path)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float64MultiArray
from std_srvs.srv import Trigger
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
import tf2_ros
import gymnasium as gym
from gymnasium import spaces
from ament_index_python.packages import get_package_share_directory

from marble_balancer.lqr_math import compute_dlqr, DEFAULT_Q, DEFAULT_R

# ── Constants (match marble_servo_controller.py) ──────────────────────────────
CONTROL_HZ      = 30.0
MAX_RATE        = np.deg2rad(60.0)    # max angular rate to servo (rad/s)
OMEGA_LPF_TC    = 0.01                # joint velocity EMA time constant (s)
MARBLE_RADIUS   = 0.015               # m
PLATE_THICKNESS = 0.005               # m
PLATE_HALF      = 0.20                # half-side of plate for termination check (m)
DROP_HEIGHT     = 0.10                # m above plate surface for spawn
DELETE_SETTLE_S = 0.6                 # seconds to wait after delete (physics flush)

MAX_STEPS       = 600                 # 20 s at 30 Hz
LAND_Z_MARGIN   = 0.010              # ±1 cm z-window for instant landing detection (was 4 cm)
LAND_VZ_MAX     = 0.10               # max marble vz to count as landed (was 0.50 m/s, now stricter)
LAND_CONFIRM    = 1                   # INSTANT landing: single tick triggers LQR (was 3)
DELETE_VERIFY_TIMEOUT = 2.0          # seconds to wait for marble deletion verification
DELETE_ODOM_SILENCE = 0.2            # seconds of no odom = marble confirmed deleted
GHOST_DATA_THRESHOLD = 5             # consecutive steps at x,y=0.0 triggers ghost detection
ZERO_STATE_EPSILON = 1e-6            # tolerance for detecting x=0.0, y=0.0 (ghost marble)

# Observation normalisation — must match rl_residual_node.py
_NORM = np.array([0.20, 0.50, 0.20, 0.50, 0.30, MAX_RATE, 0.30, MAX_RATE])

# TCP velocity normalisation (3D window: vx_t, vy_t, vz_t, vx_{t-1}, vy_{t-1}, vz_{t-1}, vx_{t-2}, vy_{t-2}, vz_{t-2})
_TCP_NORM = 0.20  # m/s per axis (linear velocities)

# Marble target normalisation
_TARGET_NORM = 0.20  # m

# Curriculum: residual clip budgets per stage
_LAMBDAS = [math.radians(5.), math.radians(10.), math.radians(15.)]

# Latched QoS for marble landed / fell_off events
_LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)

# ── Jacobian helpers (copied verbatim from marble_servo_controller.py) ─────────
_JOINT_FRAMES = [
    'shoulder_link',    # shoulder_pan_joint
    'upper_arm_link',   # shoulder_lift_joint
    'forearm_link',     # elbow_joint
    'wrist_1_link',     # wrist_1_joint
    'wrist_2_link',     # wrist_2_joint
    'wrist_3_link',     # wrist_3_joint
]
_JOINT_NAMES = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]

CANDIDATE_FRAMES   = ('plate_tcp', 'marble_plate', 'tool0')
TOP_SURFACE_FRAMES = {'plate_tcp'}


def _quat_to_rot(qx, qy, qz, qw) -> np.ndarray:
    """Quaternion → 3×3 rotation matrix."""
    return np.array([
        [1 - 2*(qy*qy + qz*qz),   2*(qx*qy - qz*qw),   2*(qx*qz + qy*qw)],
        [    2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz),  2*(qy*qz - qx*qw)],
        [    2*(qx*qz - qy*qw),   2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
    ])


def _quat_to_rpy(qx, qy, qz, qw):
    """Extract roll (X), pitch (Y), yaw (Z) from quaternion."""
    roll  = math.atan2(2.0 * (qw * qx + qy * qz),
                       1.0 - 2.0 * (qx * qx + qy * qy))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx))))
    yaw   = math.atan2(2.0 * (qw * qz + qx * qy),
                       1.0 - 2.0 * (qy * qy + qz * qz))
    return roll, pitch, yaw


class GazeboRLEnvV2(gym.Env, Node):
    """
    Gymnasium environment v2: enhanced for high-acceleration TCP movements.

    Usage:
        rclpy.init()
        env = GazeboRLEnvV2(stage=0)
        obs, _ = env.reset()
        obs, r, term, trunc, info = env.step(action)

    IMPORTANT: rclpy.init() must be called BEFORE instantiating this class.
    Do NOT run marble_servo_controller simultaneously — this env owns the servo loop.
    """

    metadata = {'render_modes': []}

    def __init__(self, stage: int = 0, use_ekf: bool = False,
                 use_tcp_lissajous: bool = False, spawn_radius: float = 0.0):
        Node.__init__(self, 'gazebo_rl_env_v2')
        gym.Env.__init__(self)

        self._stage   = min(max(stage, 0), len(_LAMBDAS) - 1)
        self._lambda  = _LAMBDAS[self._stage]
        self._use_ekf = use_ekf

        # ── TCP movement strategy: 50% Lissajous, 50% Random Jitter Walk ────────
        # When use_tcp_lissajous=True: 50% episodes use Lissajous, 50% use jitter walk
        # When use_tcp_lissajous=False: all episodes stationary (tcp_vx, tcp_vy = 0)
        self._use_tcp_disturbance = use_tcp_lissajous
        self._tcp_episode_active = False   # set each episode in reset()
        self._tcp_use_lissajous = False    # set each episode in reset()

        # Lissajous parameters (matches tcp_lissajous_node.py defaults)
        self._tcp_amp_x  = 0.30                          # m
        self._tcp_amp_y  = 0.30                          # m
        self._tcp_omega0 = 2.0 * math.pi / 12.0         # rad/s  (period = 12 s)
        self._tcp_fa     = 1
        self._tcp_fb     = 2
        self._tcp_delta  = math.pi / 2.0
        self._tcp_t      = 0.0                           # Lissajous time accumulator

        # Random Jitter Walk parameters
        self._jitter_vx = 0.0                 # current random velocity (m/s)
        self._jitter_vy = 0.0
        self._jitter_vz = 0.0
        self._jitter_vx_target = 0.0          # target velocity (picked every 1s)
        self._jitter_vy_target = 0.0
        self._jitter_vz_target = 0.0
        self._jitter_time_since_update = 0.0  # time since last velocity update
        self._jitter_update_period = 1.0      # update every 1s
        self._jitter_interp_tau = 0.2         # smooth interp time constant (s)
        self._jitter_tcp_x_acc = 0.0          # accumulated TCP position (world frame)
        self._jitter_tcp_y_acc = 0.0
        self._jitter_tcp_z_acc = 0.0
        self._jitter_max_vel = 0.15           # m/s per axis (XY)
        self._jitter_max_vel_z = 0.05         # m/s (Z axis, smaller range)
        self._jitter_bounds = 0.15            # ±0.15 m x±0.15 m square (XY)
        self._jitter_bounds_z = 0.05          # ±0.05 m (Z, relative to home height)

        # Random spawn offset (plate-frame, uniform disc of radius spawn_radius)
        self._spawn_radius = float(min(spawn_radius, 0.18))  # cap at plate safe edge

        dt = 1.0 / CONTROL_HZ
        self._K, _, _, self._P = compute_dlqr(DEFAULT_Q, DEFAULT_R, dt)

        # ── Observation / action spaces (21-D obs, 2-D action) ────────────────────
        self.observation_space = spaces.Box(
            low=-3.0, high=3.0, shape=(21,), dtype=np.float32)
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

        # ── Internal state ────────────────────────────────────────────────────────
        self._state       = np.zeros(8)   # [x, vx, y, vy, alpha, ωα, beta, ωβ]
        self._prev_action = np.zeros(2)
        # TCP 3D velocity history window (3 timesteps back: t, t-1, t-2)
        self._tcp_vel_window = np.zeros(9)  # [vx_t, vy_t, vz_t, vx_{t-1}, vy_{t-1}, vz_{t-1}, vx_{t-2}, vy_{t-2}, vz_{t-2}]
        # Marble target (desired position)
        self._target = np.zeros(2)  # [x_desired, y_desired]

        # EMA velocity estimation
        self._prev_mx     = 0.0
        self._prev_my     = 0.0
        self._prev_odom_t = None

        # Jacobian state
        self._q_dot              = np.zeros(6)
        self._q_prev             = None
        self._q_prev_t           = None
        self._omega_alpha_actual = 0.0
        self._omega_beta_actual  = 0.0

        # Episode state
        self._marble_z   = None
        self._marble_vz  = 0.0
        self._plate_z    = None   # plate top z, cached on first TF read
        self._landed     = False
        self._land_ticks = 0
        self._step_count = 0
        self._new_odom   = False  # True when a fresh odom callback has fired
        
        # Ghost data detection: count consecutive steps where x,y ≈ 0.0
        self._zero_position_count = 0
        
        # Deletion verification: track last odom timestamp to confirm deletion
        self._last_odom_t = None

        # Potential shaping (Ng et al. 1999): Φ(s) = -s^T P s
        self._phi_old = 0.0

        # EKF bypass: state read directly from /marble/lqr_state
        self._lqr_state = np.zeros(10)

        # ── TF ────────────────────────────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Publisher ─────────────────────────────────────────────────────────────
        self._twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)

        # ── Subscribers ───────────────────────────────────────────────────────────
        self.create_subscription(Odometry,   '/marble/odom',    self._odom_cb, 10)
        self.create_subscription(JointState, '/joint_states',   self._js_cb,   10)
        self.create_subscription(
            Empty, '/marble/landed',   self._landed_cb,   _LATCHED)
        self.create_subscription(
            Empty, '/marble/fell_off', self._fell_off_cb, _LATCHED)
        if self._use_ekf:
            self.create_subscription(
                Float64MultiArray, '/marble/lqr_state', self._lqr_state_cb, 10)

        # ── Service clients ───────────────────────────────────────────────────────
        self._spawn_client       = self.create_client(SpawnEntity,  '/spawn_entity')
        self._delete_client      = self.create_client(DeleteEntity, '/delete_entity')
        self._start_servo_client = self.create_client(Trigger, '/servo_node/start_servo')

        self.get_logger().info(
            f'GazeboRLEnvV2 ready — stage={self._stage}, '
            f'lambda={math.degrees(self._lambda):.0f} deg/s, use_ekf={use_ekf}, '
            f'use_tcp_disturbance={self._use_tcp_disturbance}, '
            f'spawn_radius={self._spawn_radius:.2f} m')

    # ── ROS2 callbacks ────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self._marble_z  = msg.pose.pose.position.z
        self._marble_vz = msg.twist.twist.linear.z
        self._new_odom  = True
        self._last_odom_t = time.monotonic()  # Track for deletion verification

        tf, plate_top_z = self._get_plate_tf()
        if tf is None:
            return

        plate_x = tf.transform.translation.x
        plate_y = tf.transform.translation.y
        t_now   = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Marble position in plate frame — X inverted: plate_tcp yaw≈180° at home
        mx = (msg.pose.pose.position.x - plate_x) * -1.0
        my = (msg.pose.pose.position.y - plate_y)

        # Plate angles from TF quaternion
        q = tf.transform.rotation
        roll, pitch, _ = _quat_to_rpy(q.x, q.y, q.z, q.w)
        alpha = pitch   # pitch → controls X marble
        beta  = roll    # roll  → controls Y marble

        if self._use_ekf:
            self._state[:] = self._lqr_state[:8]
        else:
            # EMA velocity differentiation
            if self._prev_odom_t is not None:
                dt_ema = t_now - self._prev_odom_t
                if 0.0 < dt_ema < 0.5:
                    vx_raw = (mx - self._prev_mx) / dt_ema
                    vy_raw = (my - self._prev_my) / dt_ema
                    vf = dt_ema / (OMEGA_LPF_TC + dt_ema)
                    self._state[1] += vf * (vx_raw - self._state[1])
                    self._state[3] += vf * (vy_raw - self._state[3])
            self._prev_mx     = mx
            self._prev_my     = my
            self._prev_odom_t = t_now
            self._state[0] = mx
            self._state[2] = my
            self._state[4] = alpha
            self._state[6] = beta
            self._state[5] = self._omega_alpha_actual
            self._state[7] = self._omega_beta_actual

        # Landing detection
        if not self._landed:
            marble_in_z = (
                self._marble_z >= plate_top_z - LAND_Z_MARGIN
                and self._marble_z <= plate_top_z + MARBLE_RADIUS + LAND_Z_MARGIN
            )
            if marble_in_z and abs(self._marble_vz) < LAND_VZ_MAX:
                self._land_ticks += 1
                if self._land_ticks >= LAND_CONFIRM:
                    self._landed = True
                    self.get_logger().info('Marble landed — LQR active.')
            else:
                self._land_ticks = 0

    def _js_cb(self, msg: JointState):
        pos_map = dict(zip(msg.name, msg.position))
        q_new   = np.array([pos_map.get(j, 0.0) for j in _JOINT_NAMES])
        t_new   = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        dt_js = (t_new - self._q_prev_t) if self._q_prev_t is not None else 1.0 / CONTROL_HZ

        if msg.velocity and len(msg.velocity) == len(msg.name):
            vel_map   = dict(zip(msg.name, msg.velocity))
            q_dot_raw = np.array([vel_map.get(j, 0.0) for j in _JOINT_NAMES])
        elif self._q_prev is not None and 0.0 < dt_js < 0.5:
            q_dot_raw = (q_new - self._q_prev) / dt_js
        else:
            q_dot_raw = np.zeros(6)

        self._q_prev   = q_new
        self._q_prev_t = t_new

        alpha_f = dt_js / (OMEGA_LPF_TC + dt_js)
        self._q_dot = alpha_f * q_dot_raw + (1.0 - alpha_f) * self._q_dot

        omega = self._compute_plate_omega()
        if omega is not None:
            self._omega_alpha_actual = omega[1]   # world Y → α̇
            self._omega_beta_actual  = omega[0]   # world X → β̇

    def _landed_cb(self, _):
        self._landed = True

    def _fell_off_cb(self, _):
        self._landed = False

    def _lqr_state_cb(self, msg: Float64MultiArray):
        if len(msg.data) >= 8:
            self._lqr_state[:min(10, len(msg.data))] = msg.data[:min(10, len(msg.data))]

    # ── State extraction helpers ──────────────────────────────────────────────────

    def _get_plate_tf(self):
        """Return (tf, plate_top_z) or (None, None). Tries fallback frames."""
        for frame in CANDIDATE_FRAMES:
            try:
                tf = self._tf_buffer.lookup_transform(
                    'world', frame, rclpy.time.Time())
                plate_top_z = tf.transform.translation.z + PLATE_THICKNESS / 2.0
                if self._plate_z is None:
                    self._plate_z = plate_top_z
                    self.get_logger().info(
                        f'Plate top Z from "{frame}": {self._plate_z:.4f} m')
                return tf, plate_top_z
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                pass
        return None, None

    def _plate_offset_to_world(self, tf, offset_x: float, offset_y: float):
        """Convert plate-frame (x, y) offset to world-frame position offset."""
        q = tf.transform.rotation
        R = _quat_to_rot(q.x, q.y, q.z, q.w)
        world_offset = offset_x * R[:, 0] + offset_y * R[:, 1]
        return (tf.transform.translation.x + world_offset[0],
                tf.transform.translation.y + world_offset[1])

    def _compute_plate_omega(self):
        """Build 3×6 rotational Jacobian × q_dot → [ωx, ωy, ωz] in world frame."""
        J_rot = np.zeros((3, 6))
        for i, frame in enumerate(_JOINT_FRAMES):
            try:
                tf = self._tf_buffer.lookup_transform(
                    'world', frame, rclpy.time.Time())
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                return None
            q = tf.transform.rotation
            R = _quat_to_rot(q.x, q.y, q.z, q.w)
            J_rot[:, i] = R[:, 2]   # Z-column = joint rotation axis in world frame
        return J_rot @ self._q_dot

    # ── Random Jitter Walk logic ───────────────────────────────────────────────────

    def _update_jitter_velocity(self, dt: float):
        """Update random walk velocity with smooth interpolation (3D: X, Y, Z)."""
        self._jitter_time_since_update += dt

        # Pick new target velocity every 1s
        if self._jitter_time_since_update >= self._jitter_update_period:
            self._jitter_vx_target = np.random.uniform(-self._jitter_max_vel, self._jitter_max_vel)
            self._jitter_vy_target = np.random.uniform(-self._jitter_max_vel, self._jitter_max_vel)
            self._jitter_vz_target = np.random.uniform(-self._jitter_max_vel_z, self._jitter_max_vel_z)
            self._jitter_time_since_update = 0.0

        # Smooth exponential interpolation toward target
        alpha = dt / (self._jitter_interp_tau + dt)
        self._jitter_vx += alpha * (self._jitter_vx_target - self._jitter_vx)
        self._jitter_vy += alpha * (self._jitter_vy_target - self._jitter_vy)
        self._jitter_vz += alpha * (self._jitter_vz_target - self._jitter_vz)

    def _get_tcp_velocity(self, dt: float):
        """Compute 3D TCP velocity based on movement strategy (Lissajous or Jitter Walk)."""
        if self._tcp_use_lissajous:
            # Lissajous curve with Z-oscillation
            ox = self._tcp_fa * self._tcp_omega0
            oy = self._tcp_fb * self._tcp_omega0
            tcp_vx = self._tcp_amp_x * ox * math.cos(ox * self._tcp_t + self._tcp_delta)
            tcp_vy = self._tcp_amp_y * oy * math.cos(oy * self._tcp_t)
            tcp_vz = 0.02 * math.sin(self._tcp_omega0 * self._tcp_t)  # small Z-oscillation
            self._tcp_t += dt
        else:
            # Random Jitter Walk (3D)
            self._update_jitter_velocity(dt)
            tcp_vx = self._jitter_vx
            tcp_vy = self._jitter_vy
            tcp_vz = self._jitter_vz

        return tcp_vx, tcp_vy, tcp_vz

    # ── Gymnasium interface ───────────────────────────────────────────────────────

    def reset(self, *, seed=None, options=None):
        """ATOMIC RESET: Loops until marble is successfully landed. Never returns without marble."""
        return self._reset_until_landed(attempt=0)

    def _reset_until_landed(self, attempt: int = 0, max_attempts: int = 10):
        """Inner reset loop: keeps trying until marble lands successfully."""
        if attempt >= max_attempts:
            self.get_logger().error(
                f'Reset failed after {max_attempts} attempts! Giving up (this should not happen).')
            raise RuntimeError('gazebo_rl_env_v2: Marble spawn permanently failed.')

        super().reset(seed=None)

        # CRITICAL: Immediately zero all velocities to prevent TCP drift into new episode
        # This is the VERY FIRST action in reset() to ensure clean slate
        self._publish_zero_twist()

        # Stop servo and zero state
        self._landed      = False
        self._land_ticks  = 0
        self._step_count  = 0
        self._new_odom    = False
        self._state[:]    = 0.0
        self._q_dot[:]    = 0.0
        self._prev_odom_t = None
        self._last_odom_t = None
        self._omega_alpha_actual = 0.0
        self._omega_beta_actual  = 0.0
        self._prev_action = np.zeros(2)
        self._tcp_vel_window[:] = 0.0
        self._target[:] = 0.0
        self._zero_position_count = 0
        
        # Explicitly zero TCP velocities
        self._jitter_vx = 0.0
        self._jitter_vy = 0.0
        self._jitter_vz = 0.0

        # NOTE: If you see Gazebo GUI rendering on screen during training, it will slow
        # training by ~300%. Always launch with: ros2 launch ... gui:=false
        # For faster-than-realtime training: set real_time_update_rate=0 in world .sdf

        # 50% chance to use Lissajous if TCP disturbance enabled
        if self._use_tcp_disturbance:
            self._tcp_episode_active = True
            self._tcp_use_lissajous = (np.random.random() < 0.5)
            if self._tcp_use_lissajous:
                # Randomise Lissajous phase
                self._tcp_t = float(np.random.uniform(0.0, 2.0 * math.pi / self._tcp_omega0))
            else:
                # Initialize Jitter Walk
                self._jitter_vx = 0.0
                self._jitter_vy = 0.0
                self._jitter_vx_target = 0.0
                self._jitter_vy_target = 0.0
                self._jitter_time_since_update = 0.0
        else:
            self._tcp_episode_active = False
            self._tcp_use_lissajous = False

        mode_str = "Lissajous" if self._tcp_use_lissajous else "Jitter Walk"
        self.get_logger().info(
            f'Episode reset (attempt {attempt+1}/{max_attempts}) — tcp_active={self._tcp_episode_active}, '
            f'mode={mode_str if self._tcp_episode_active else "stationary"}')

        # Episode reset sequence: delete → home → spawn → wait for land
        self._delete_marble()
        self._home_arm()
        self._spawn_marble()

        self.get_logger().info('Waiting for marble to land…')
        deadline = time.monotonic() + 20.0
        while not self._landed and time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
        
        # ATOMIC CHECK: If marble didn't land, retry the entire reset
        if not self._landed:
            self.get_logger().warn(
                f'Land timeout (attempt {attempt+1}) — retrying reset sequence…')
            return self._reset_until_landed(attempt=attempt + 1, max_attempts=max_attempts)

        # Start (or restart) MoveIt Servo
        self._call_start_servo()

        # Initialise potential shaping baseline
        self._phi_old = -float(self._state @ self._P @ self._state)

        # SAFETY: Never return without a landed marble
        obs = self._get_obs()
        self.get_logger().info(
            f'✓ Reset complete — marble landed, LQR active. obs shape={obs.shape}')
        return obs, {}

    def step(self, action: np.ndarray):
        action = np.clip(action.astype(np.float32), -1., 1.)

        # Wait for fresh odom before computing state
        self._new_odom = False
        deadline = time.monotonic() + 0.2   # 200 ms timeout (>2 odom periods at 50 Hz)
        while not self._new_odom and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        dt = 1.0 / CONTROL_HZ

        # LQR base command + RL residual
        u_lqr    = np.clip(-self._K @ self._state, -MAX_RATE, MAX_RATE)
        residual = action * self._lambda
        u_total  = np.clip(u_lqr + residual, -MAX_RATE, MAX_RATE)

        # Servo command — sign convention exactly as marble_servo_controller.py
        omega_alpha_cmd = -float(u_total[0])
        omega_beta_cmd  = -float(u_total[1])

        # TCP velocity based on movement strategy
        if self._tcp_episode_active and self._landed:
            tcp_vx, tcp_vy, tcp_vz = self._get_tcp_velocity(dt)
        else:
            tcp_vx = 0.0
            tcp_vy = 0.0
            tcp_vz = 0.0

        # Publish servo command with 3D velocity
        twist = TwistStamped()
        twist.header.stamp    = self.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'
        twist.twist.angular.x = omega_beta_cmd
        twist.twist.angular.y = omega_alpha_cmd
        twist.twist.linear.x  = tcp_vx
        twist.twist.linear.y  = tcp_vy
        twist.twist.linear.z  = tcp_vz
        self._twist_pub.publish(twist)

        # Update TCP 3D velocity window (shift and append new: vx_t, vy_t, vz_t)
        self._tcp_vel_window[6:9] = self._tcp_vel_window[3:6]  # t-2 ← t-1
        self._tcp_vel_window[3:6] = self._tcp_vel_window[0:3]  # t-1 ← t
        self._tcp_vel_window[0:3] = [tcp_vx, tcp_vy, tcp_vz]   # t ← new

        # Compute reward
        reward = self._compute_reward(action)

        # Update action history (used in observation)
        self._prev_action = action.copy()

        # REWARD LOCKOUT: If marble not landed, penalize heavily and terminate
        if not self._landed:
            reward = -100.0
            terminated = True
            self.get_logger().warn('No marble landed! Emergency termination.')
            return self._get_obs(), float(reward), terminated, True, {}

        # Termination check
        x, y = self._state[0], self._state[2]
        plate_top_z  = self._plate_z if self._plate_z is not None else 0.8
        fell_off_xy  = abs(x) > PLATE_HALF or abs(y) > PLATE_HALF
        fell_off_z   = (self._marble_z is not None
                        and self._marble_z < plate_top_z - 0.05)
        terminated   = fell_off_xy or fell_off_z
        self._step_count += 1
        truncated = self._step_count >= MAX_STEPS

        if terminated:
            reward -= 50.0

        # GHOST DATA PROTECTION: Detect marble missing (x, y stuck at ~0.0)
        if abs(x) < ZERO_STATE_EPSILON and abs(y) < ZERO_STATE_EPSILON:
            self._zero_position_count += 1
            if self._zero_position_count >= GHOST_DATA_THRESHOLD:
                self.get_logger().error(
                    f'GHOST MARBLE DETECTED: x={x:.10f}, y={y:.10f} (zero for {self._zero_position_count} steps). '
                    'Marble missing from simulation!')
                reward -= 100.0
                terminated = True
        else:
            self._zero_position_count = 0  # reset counter

        # Print debug info every 10 steps
        if self._step_count % 10 == 0:
            x_error = abs(self._state[0] - self._target[0])
            y_error = abs(self._state[2] - self._target[1])
            ghost_warn = f" [GHOST WARNING: {self._zero_position_count} steps]" if self._zero_position_count > 0 else ""
            print(f'[Step {self._step_count:3d}] Reward={reward:+7.3f}  x_err={x_error:.3f}  y_err={y_error:.3f}{ghost_warn}')

        return self._get_obs(), float(reward), terminated, truncated, {}

    def _compute_reward(self, action: np.ndarray) -> float:
        """
        Enhanced reward function prioritizing marble centering:
          • pos_reward = exp(-100.0 * (x² + y²))  — sharp peak at origin
          • tilt_penalty = -0.2 * (α² + β²)       — penalize excessive plate tilt (INCREASED for Z-jitter)
          • survival_bonus = 0.1                   — encourage long episodes
          • shape = 0.99 * φ(s') - φ(s)          — potential shaping (Ng et al.)
        """
        x, y = self._state[0], self._state[2]
        alpha, beta = self._state[4], self._state[6]

        pos_reward = math.exp(-100.0 * (x ** 2 + y ** 2))
        tilt_penalty = -0.2 * (alpha ** 2 + beta ** 2)  # Increased from -0.1 to discourage cheating with extreme angles
        survival_bonus = 0.1

        # Potential shaping
        phi_new = -float(self._state @ self._P @ self._state)
        shape = 0.99 * phi_new - self._phi_old
        self._phi_old = phi_new

        return pos_reward + tilt_penalty + survival_bonus + shape

    def _get_obs(self) -> np.ndarray:
        """
        Return 21-D observation vector:
          [0:8]   state (normalized)
          [8:17]  TCP 3D velocity window (normalized)
          [17:19] marble target (normalized)
          [19:21] last action (residual)
        """
        state_norm = np.clip(self._state / _NORM, -3.0, 3.0).astype(np.float32)
        tcp_vel_norm = np.clip(self._tcp_vel_window / _TCP_NORM, -3.0, 3.0).astype(np.float32)
        target_norm = np.clip(self._target / _TARGET_NORM, -3.0, 3.0).astype(np.float32)
        prev_action_norm = self._prev_action.astype(np.float32)

        return np.concatenate([state_norm, tcp_vel_norm, target_norm, prev_action_norm])

    # ── Curriculum ────────────────────────────────────────────────────────────────

    def set_stage(self, stage: int):
        self._stage  = min(max(stage, 0), len(_LAMBDAS) - 1)
        self._lambda = _LAMBDAS[self._stage]
        self.get_logger().info(
            f'Curriculum stage → {self._stage} '
            f'(lambda={math.degrees(self._lambda):.0f} deg/s)')

    # ── Episode helpers ───────────────────────────────────────────────────────────

    def _publish_zero_twist(self):
        """Publish zero velocity command to ALL axes (linear + angular)."""
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        # Explicitly zero all fields
        msg.twist.linear.x  = 0.0
        msg.twist.linear.y  = 0.0
        msg.twist.linear.z  = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        self._twist_pub.publish(msg)

    def _delete_marble(self):
        """Delete marble and verify deletion via odom silence + settlement time."""
        # Robust service availability check with retries
        max_retries = 3
        for attempt in range(1, max_retries + 1):
            if self._delete_client.wait_for_service(timeout_sec=5.0):
                break
            self.get_logger().warn(
                f'/delete_entity not available (attempt {attempt}/{max_retries}) — '
                f'retrying in 0.5s...')
            time.sleep(0.5)
        else:
            self.get_logger().error('/delete_entity service unavailable after retries!')
            return
        
        req = DeleteEntity.Request()
        req.name = 'marble'
        future = self._delete_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Marble deletion request sent — verifying…')
        
        # ROBUST: Wait for odom to stop publishing (marble gone from simulation)
        self._last_odom_t = time.monotonic()  # Mark reference time
        deadline = time.monotonic() + DELETE_VERIFY_TIMEOUT
        odom_silence_start = None
        
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            
            # Check if enough time has passed without new odom
            if self._last_odom_t is not None:
                time_since_odom = time.monotonic() - self._last_odom_t
                if time_since_odom >= DELETE_ODOM_SILENCE:
                    if odom_silence_start is None:
                        odom_silence_start = time.monotonic()
                        self.get_logger().info(f'Odom silence detected ({time_since_odom:.2f}s) — marble deleted.')
                    break
        
        # Additional physics settlement time
        self.get_logger().info(
            f'Marble deleted — waiting {DELETE_SETTLE_S:.1f}s for physics flush…')
        deadline = time.monotonic() + DELETE_SETTLE_S
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _home_arm(self):
        """Launch go_to_pose and spin until it exits (blocking with callback processing)."""
        self.get_logger().info('Homing arm via go_to_pose…')
        proc = subprocess.Popen(['ros2', 'run', 'marble_balancer', 'go_to_pose'])
        deadline = time.monotonic() + 15.0
        while proc.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if proc.poll() is None:
            proc.terminate()
            self.get_logger().warn('go_to_pose timed out — terminated.')
        else:
            self.get_logger().info('Arm homed.')

    def _spawn_marble(self):
        """Spawn marble above plate; retry up to 3 times on fall-through."""
        # Robust service availability check with retries
        max_retries = 3
        for attempt in range(1, max_retries + 1):
            if self._spawn_client.wait_for_service(timeout_sec=5.0):
                break
            self.get_logger().warn(
                f'/spawn_entity not available (attempt {attempt}/{max_retries}) — '
                f'retrying in 0.5s...')
            time.sleep(0.5)
        else:
            self.get_logger().error('/spawn_entity service unavailable after retries!')
            return

        pkg = get_package_share_directory('marble_balancer')
        with open(os.path.join(pkg, 'urdf', 'marble.sdf'), 'r') as f:
            sdf_xml = f.read()

        for attempt in range(1, 4):
            tf, used_frame = None, None
            deadline = time.monotonic() + 5.0
            while tf is None and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
                for frame in CANDIDATE_FRAMES:
                    try:
                        tf = self._tf_buffer.lookup_transform(
                            'world', frame, rclpy.time.Time())
                        used_frame = frame
                        break
                    except (tf2_ros.LookupException,
                            tf2_ros.ConnectivityException,
                            tf2_ros.ExtrapolationException):
                        tf = None

            if tf is None:
                self.get_logger().error('TF unavailable for spawn!')
                return

            plate_z   = tf.transform.translation.z
            surface_z = (plate_z if used_frame in TOP_SURFACE_FRAMES
                         else plate_z + PLATE_THICKNESS / 2.0)
            spawn_z = surface_z + MARBLE_RADIUS + DROP_HEIGHT

            # Random plate-frame offset (uniform disc)
            if self._spawn_radius > 0.0:
                angle  = np.random.uniform(0.0, 2.0 * math.pi)
                radius = self._spawn_radius * math.sqrt(np.random.uniform(0.0, 1.0))
                off_x  = radius * math.cos(angle)
                off_y  = radius * math.sin(angle)
                spawn_x, spawn_y = self._plate_offset_to_world(tf, off_x, off_y)
            else:
                spawn_x = tf.transform.translation.x
                spawn_y = tf.transform.translation.y

            self.get_logger().info(
                f'[attempt {attempt}/3] Spawning at '
                f'({spawn_x:.3f}, {spawn_y:.3f}, {spawn_z:.3f})')

            req = SpawnEntity.Request()
            req.name                         = 'marble'
            req.xml                          = sdf_xml
            req.robot_namespace              = ''
            req.reference_frame              = 'world'
            req.initial_pose.position.x      = float(spawn_x)
            req.initial_pose.position.y      = float(spawn_y)
            req.initial_pose.position.z      = float(spawn_z)
            req.initial_pose.orientation.w   = 1.0

            future = self._spawn_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            if result is None or not result.success:
                err = result.status_message if result else 'no response'
                self.get_logger().warn(f'Spawn failed: {err} — retrying…')
                continue

            # Verify marble stays on plate
            self._marble_z = None
            fell_through   = False
            land_ticks     = 0
            rest_z         = surface_z + MARBLE_RADIUS
            deadline       = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if self._marble_z is not None:
                    if self._marble_z < surface_z - 0.05:
                        fell_through = True
                        break
                    if abs(self._marble_z - rest_z) < LAND_Z_MARGIN:
                        land_ticks += 1
                        if land_ticks >= LAND_CONFIRM:
                            break
                    else:
                        land_ticks = 0

            if not fell_through:
                self.get_logger().info('Marble confirmed on plate.')
                return

            self.get_logger().warn('Marble fell through — deleting and retrying…')
            self._delete_marble()

        self.get_logger().error('Marble failed to stay on plate after 3 attempts.')

    def _call_start_servo(self):
        if self._start_servo_client.wait_for_service(timeout_sec=3.0):
            future = self._start_servo_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future)


def main(args=None):
    """Quick smoke test: reset + 10 zero-action steps."""
    rclpy.init(args=args)
    env = GazeboRLEnvV2(stage=0)
    try:
        obs, _ = env.reset()
        print(f'reset OK — obs shape: {obs.shape}, '
              f'range: [{obs.min():.2f}, {obs.max():.2f}]')
        for i in range(10):
            obs, r, term, trunc, _ = env.step(np.zeros(2, dtype=np.float32))
            print(f'step {i:2d}: r={r:+.3f}  term={term}  trunc={trunc}')
            if term or trunc:
                break
    finally:
        env.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
