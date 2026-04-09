"""
gazebo_rl_env.py
----------------
Gymnasium environment wrapping live Gazebo for online TD3 training.

This env IS the controller during training — do NOT run marble_servo_controller
simultaneously.  It owns the servo command loop, LQR computation, and episode
management (delete → home → spawn → land).

Observation (28-D):
  obs[0:8]   = state / _NORM             # [x, vx, y, vy, alpha, ωα, beta, ωβ]
  obs[8:28]  = action_history.flatten()  # last 10 2-D actions, oldest-first

Action (2-D, in [-1, 1]):
  residual = action * lambda              # Δω added to LQR output (rad/s)
  lambda anneals with curriculum stage: 5 → 10 → 15 deg/s

Servo command (matches marble_servo_controller.py exactly):
  frame_id  = 'base_link'
  angular.x = omega_beta_cmd   (world X → roll → Y marble dynamics)
  angular.y = omega_alpha_cmd  (world Y → pitch → X marble dynamics, negated for yaw≈180°)
"""

import os
import math
import time
import subprocess
import collections
import numpy as np

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
LAND_Z_MARGIN   = 0.040              # ±4 cm z-window for landing detection
LAND_VZ_MAX     = 0.50               # max marble vz to count as landed
LAND_CONFIRM    = 3                   # consecutive odom ticks required

# Observation normalisation — must match rl_residual_node.py
_NORM = np.array([0.20, 0.50, 0.20, 0.50, 0.30, MAX_RATE, 0.30, MAX_RATE])

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


class GazeboRLEnv(gym.Env, Node):
    """
    Gymnasium environment wrapping live Gazebo for online TD3 training.

    Usage:
        rclpy.init()
        env = GazeboRLEnv(stage=0)
        obs, _ = env.reset()
        obs, r, term, trunc, info = env.step(action)

    IMPORTANT: rclpy.init() must be called BEFORE instantiating this class.
    Do NOT run marble_servo_controller simultaneously — this env owns the servo loop.
    """

    metadata = {'render_modes': []}

    def __init__(self, stage: int = 0, use_ekf: bool = False,
                 use_tcp_lissajous: bool = False, spawn_radius: float = 0.0):
        Node.__init__(self, 'gazebo_rl_env')
        gym.Env.__init__(self)

        self._stage   = min(max(stage, 0), len(_LAMBDAS) - 1)
        self._lambda  = _LAMBDAS[self._stage]
        self._use_ekf = use_ekf

        # ── TCP Lissajous disturbance (matches tcp_lissajous_node.py defaults) ─
        # tcp_lissajous_prob: probability that any given episode has TCP moving.
        # 0.0 = never (standard training), 0.5 = mixed (generalises both), 1.0 = always.
        self._tcp_lissajous_prob = 0.5 if use_tcp_lissajous else 0.0
        self._tcp_episode_active = False   # set each episode in reset()
        self._tcp_amp_x  = 0.04                          # m
        self._tcp_amp_y  = 0.04                          # m
        self._tcp_omega0 = 2.0 * math.pi / 20.0         # rad/s  (period = 20 s)
        self._tcp_fa     = 1
        self._tcp_fb     = 2
        self._tcp_delta  = math.pi / 2.0
        self._tcp_t      = 0.0                           # Lissajous time accumulator

        # ── Random spawn offset (plate-frame, uniform disc of radius spawn_radius) ─
        # spawn_radius=0 → always spawn at plate centre (default, backward-compat)
        # spawn_radius=0.15 → spawn anywhere within 15 cm of centre
        self._spawn_radius = float(min(spawn_radius, 0.18))  # cap at plate safe edge

        dt = 1.0 / CONTROL_HZ
        self._K, _, _, self._P = compute_dlqr(DEFAULT_Q, DEFAULT_R, dt)

        # ── Observation / action spaces ────────────────────────────────────────
        self.observation_space = spaces.Box(
            low=-3.0, high=3.0, shape=(28,), dtype=np.float32)
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

        # ── Internal state ────────────────────────────────────────────────────
        self._state       = np.zeros(8)   # [x, vx, y, vy, alpha, ωα, beta, ωβ]
        self._prev_action = np.zeros(2)
        # Action history: oldest at index 0, newest at index -1 (deque maxlen=10)
        self._action_hist = collections.deque([np.zeros(2)] * 10, maxlen=10)

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

        # Potential shaping (Ng et al. 1999): Φ(s) = -s^T P s
        self._phi_old = 0.0

        # EKF bypass: state read directly from /marble/lqr_state
        self._lqr_state = np.zeros(10)

        # ── TF ────────────────────────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Publisher ─────────────────────────────────────────────────────────
        self._twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(Odometry,   '/marble/odom',    self._odom_cb, 10)
        self.create_subscription(JointState, '/joint_states',   self._js_cb,   10)
        self.create_subscription(
            Empty, '/marble/landed',   self._landed_cb,   _LATCHED)
        self.create_subscription(
            Empty, '/marble/fell_off', self._fell_off_cb, _LATCHED)
        if self._use_ekf:
            self.create_subscription(
                Float64MultiArray, '/marble/lqr_state', self._lqr_state_cb, 10)

        # ── Service clients ───────────────────────────────────────────────────
        self._spawn_client       = self.create_client(SpawnEntity,  '/spawn_entity')
        self._delete_client      = self.create_client(DeleteEntity, '/delete_entity')
        self._start_servo_client = self.create_client(Trigger, '/servo_node/start_servo')

        self.get_logger().info(
            f'GazeboRLEnv ready — stage={self._stage}, '
            f'lambda={math.degrees(self._lambda):.0f} deg/s, use_ekf={use_ekf}, '
            f'tcp_lissajous_prob={self._tcp_lissajous_prob:.0%}, '
            f'spawn_radius={self._spawn_radius:.2f} m')

    # ── ROS2 callbacks ────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self._marble_z  = msg.pose.pose.position.z
        self._marble_vz = msg.twist.twist.linear.z
        self._new_odom  = True

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
            # Use EKF-estimated state from marble_servo_controller
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

        # Landing detection (matches marble_servo_controller.py logic)
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

        alpha_f     = dt_js / (OMEGA_LPF_TC + dt_js)
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

    # ── State extraction helpers ──────────────────────────────────────────────

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
        """Convert plate-frame (x, y) offset to world-frame position offset.
        Accounts for plate rotation (yaw≈180° at home) via rotation matrix."""
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

    # ── Gymnasium interface ───────────────────────────────────────────────────

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)

        # Stop servo and zero state
        self._publish_zero_twist()
        self._landed      = False
        self._land_ticks  = 0
        self._step_count  = 0
        self._new_odom    = False
        self._state[:]    = 0.0
        self._q_dot[:]    = 0.0
        self._prev_odom_t = None
        self._omega_alpha_actual = 0.0
        self._omega_beta_actual  = 0.0
        self._action_hist = collections.deque([np.zeros(2)] * 10, maxlen=10)
        self._prev_action = np.zeros(2)
        # Randomly activate TCP Lissajous for this episode (50% when enabled)
        # — model sees both stationary and moving TCP → generalises to both
        self._tcp_episode_active = (
            np.random.random() < self._tcp_lissajous_prob)
        # Randomise phase so model sees all phases, not always phase=0
        self._tcp_t = float(np.random.uniform(0.0, 2.0 * math.pi / self._tcp_omega0)) \
                      if self._tcp_episode_active else 0.0
        self.get_logger().info(
            f'Episode reset — tcp_lissajous_active={self._tcp_episode_active}')

        # Episode reset sequence: delete → home → spawn → wait for land
        self._delete_marble()
        self._home_arm()
        self._spawn_marble()

        self.get_logger().info('Waiting for marble to land…')
        deadline = time.monotonic() + 20.0
        while not self._landed and time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not self._landed:
            self.get_logger().warn('Land timeout — proceeding anyway.')

        # Start (or restart) MoveIt Servo
        self._call_start_servo()

        # Initialise potential shaping baseline from post-spawn state
        self._phi_old = -float(self._state @ self._P @ self._state)

        return self._get_obs(), {}

    def step(self, action: np.ndarray):
        action = np.clip(action.astype(np.float32), -1., 1.)

        # Wait for fresh odom before computing state
        self._new_odom = False
        deadline = time.monotonic() + 0.2   # 200 ms timeout (>2 odom periods at 50 Hz)
        while not self._new_odom and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        # LQR base command + RL residual
        u_lqr    = np.clip(-self._K @ self._state, -MAX_RATE, MAX_RATE)
        residual = action * self._lambda
        u_total  = np.clip(u_lqr + residual, -MAX_RATE, MAX_RATE)

        # Servo command — sign convention exactly as marble_servo_controller.py:456-465
        # plate_tcp yaw≈180° at home requires negation of both alpha and beta commands
        omega_alpha_cmd = -float(u_total[0])
        omega_beta_cmd  = -float(u_total[1])

        # TCP Lissajous linear velocity (matches tcp_lissajous_node.py formula)
        if self._tcp_episode_active and self._landed:
            ox = self._tcp_fa * self._tcp_omega0
            oy = self._tcp_fb * self._tcp_omega0
            tcp_vx = self._tcp_amp_x * ox * math.cos(ox * self._tcp_t + self._tcp_delta)
            tcp_vy = self._tcp_amp_y * oy * math.cos(oy * self._tcp_t)
            self._tcp_t += 1.0 / CONTROL_HZ
        else:
            tcp_vx = 0.0
            tcp_vy = 0.0

        twist = TwistStamped()
        twist.header.stamp    = self.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'
        twist.twist.angular.x = omega_beta_cmd    # world X → roll → Y dynamics
        twist.twist.angular.y = omega_alpha_cmd   # world Y → pitch → X dynamics
        twist.twist.linear.x  = tcp_vx            # TCP X linear velocity (m/s)
        twist.twist.linear.y  = tcp_vy            # TCP Y linear velocity (m/s)
        self._twist_pub.publish(twist)

        # Reward (computed before updating history so prev_action is still last step's)
        reward = self._compute_reward(action)

        # Update action history (oldest first → append at right)
        self._action_hist.append(action.copy())
        self._prev_action = action.copy()

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

        return self._get_obs(), float(reward), terminated, truncated, {}

    def _compute_reward(self, action: np.ndarray) -> float:
        x, vx, y, vy = (self._state[0], self._state[1],
                         self._state[2], self._state[3])
        pos    = math.exp(-50.0 * (x ** 2 + y ** 2))
        vel    = -0.1 * (vx ** 2 + vy ** 2)
        smooth = -0.02 * float(np.dot(action - self._prev_action,
                                       action - self._prev_action))
        surv   = 0.05
        # LQR potential shaping: Φ(s) = -s^T P s  (zero at origin, negative elsewhere)
        # gamma * Φ(s') - Φ(s) rewards moving toward origin without changing optimal policy
        phi_new       = -float(self._state @ self._P @ self._state)
        shape         = 0.99 * phi_new - self._phi_old
        self._phi_old = phi_new
        return pos + vel + smooth + surv + shape

    def _get_obs(self) -> np.ndarray:
        state_norm = np.clip(self._state / _NORM, -3.0, 3.0).astype(np.float32)
        # Action history: oldest-first (index 0 = oldest), newest at index -1
        hist_flat  = np.array(list(self._action_hist), dtype=np.float32).flatten()
        return np.concatenate([state_norm, hist_flat])

    # ── Curriculum ────────────────────────────────────────────────────────────

    def set_stage(self, stage: int):
        self._stage  = min(max(stage, 0), len(_LAMBDAS) - 1)
        self._lambda = _LAMBDAS[self._stage]
        self.get_logger().info(
            f'Curriculum stage → {self._stage} '
            f'(lambda={math.degrees(self._lambda):.0f} deg/s)')

    # ── Episode helpers ───────────────────────────────────────────────────────

    def _publish_zero_twist(self):
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        self._twist_pub.publish(msg)

    def _delete_marble(self):
        if not self._delete_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('/delete_entity not available — skipping delete.')
            return
        req = DeleteEntity.Request()
        req.name = 'marble'
        future = self._delete_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
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
        if not self._spawn_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/spawn_entity not available!')
            return

        pkg = get_package_share_directory('marble_balancer')
        with open(os.path.join(pkg, 'urdf', 'marble.sdf'), 'r') as f:
            sdf_xml = f.read()

        for attempt in range(1, 4):
            # Fresh TF read for current plate position after homing
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

            # Random plate-frame offset (uniform disc) when spawn_radius > 0
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

            # Verify marble stays on plate (3 s monitor)
            self._marble_z = None
            fell_through   = False
            deadline       = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if self._marble_z is not None and self._marble_z < surface_z - 0.05:
                    fell_through = True
                    break

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
    env = GazeboRLEnv(stage=0)
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
