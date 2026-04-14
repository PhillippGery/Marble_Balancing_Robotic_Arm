"""
rl_residual_node.py
-------------------
ROS2 deployment node for the trained TD3 residual controller.

Subscribes to:
  /marble/lqr_state       (std_msgs/Float64MultiArray) — 8-D state + 2-D LQR output
  /marble/lqr_twist       (geometry_msgs/TwistStamped)  — raw LQR twist (for passthrough)
  /marble/landed          (std_msgs/Empty, LATCHED)
  /marble/fell_off        (std_msgs/Empty, LATCHED)

Publishes to:
  /marble_servo_rl/delta_twist_cmds  (geometry_msgs/TwistStamped) — LQR + RL residual

The mux_controller is configured via `auto_topic` parameter to subscribe to
/marble_servo_rl/delta_twist_cmds instead of /marble_servo/delta_twist_cmds
when rl:=true.

Launch parameters:
  rl_model  — path to trained TD3 model (.zip)
  rl_norm   — path to running_stats.pkl (mean/var from train_td3_gazebo.py)
  rl_stage  — curriculum stage (0-3) controls residual clip budget
"""

import os
import sys
import math
import pickle
import collections
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Empty, Float64MultiArray

_LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)

# Observation normalisation constants — must match gazebo_rl_env.py
MAX_RATE    = math.radians(60.0)   # was 45° — fixed to match training env
_NORM       = np.array([0.20, 0.50, 0.20, 0.50, 0.30, MAX_RATE, 0.30, MAX_RATE])
_TWIST_NORM = np.array([MAX_RATE, MAX_RATE, 0.20, 0.35])   # sized for 0.30m/12s TCP

# TCP Lissajous phase-encoding constants — must match launch file + gazebo_rl_env.py
_TCP_OMEGA0   = 2.0 * math.pi / 12.0   # 12 s period (matches servo_balancer.launch.py)
_TCP_FA       = 1
_TCP_FB       = 2
_TCP_DELTA    = math.pi / 2.0
_CONTROL_HZ   = 30.0

# Curriculum residual clip budgets for TD3 (3 training stages + 1 alias for compat)
_CLIP_BUDGETS = [
    math.radians(5.0),    # stage 0
    math.radians(10.0),   # stage 1
    math.radians(15.0),   # stage 2
    math.radians(15.0),   # stage 3 (alias — same as 2, for backward compat with launch arg)
]


class RLResidualNode(Node):

    def __init__(self):
        super().__init__('rl_residual_node')

        self.declare_parameter('rl_model', '')
        self.declare_parameter('rl_norm',  '')
        self.declare_parameter('rl_stage', 3)
        self.declare_parameter('use_rl',   True)

        model_path    = self.get_parameter('rl_model').value
        norm_path     = self.get_parameter('rl_norm').value
        stage         = int(self.get_parameter('rl_stage').value)
        self._clip    = _CLIP_BUDGETS[min(stage, len(_CLIP_BUDGETS) - 1)]
        self._enabled = self.get_parameter('use_rl').value

        # ── Load TD3 policy ───────────────────────────────────────────────────
        self._model          = None
        self._running_stats  = None   # {'mean': ndarray, 'var': ndarray}
        self._use_rl         = False

        if model_path and os.path.exists(model_path):
            try:
                from stable_baselines3 import TD3
                self._model  = TD3.load(model_path)
                self._use_rl = True
                self.get_logger().info(f'Loaded TD3 policy: {model_path}')

                if norm_path and os.path.exists(norm_path):
                    with open(norm_path, 'rb') as f:
                        self._running_stats = pickle.load(f)
                    self.get_logger().info(
                        f'Loaded running stats: {norm_path}  '
                        f'(count={self._running_stats.get("count", "?")})')
            except Exception as e:
                self.get_logger().error(
                    f'Failed to load TD3 model: {e} — running as LQR passthrough')
        else:
            self.get_logger().warn(
                f'rl_model not found: "{model_path}" — running as LQR passthrough')

        # ── State ─────────────────────────────────────────────────────────────
        self._lqr_state = np.zeros(10)   # 8-D state + 2-D LQR output
        self._landed    = False
        self._last_twist = None           # passthrough from LQR

        # Action history: oldest at index 0, newest at index -1 (10 × 2-D actions)
        # Must match gazebo_rl_env.py ordering (oldest-first).
        self._action_hist = collections.deque([np.zeros(2)] * 10, maxlen=10)
        # Last full twist sent: [omega_beta_cmd, omega_alpha_cmd, tcp_vx, tcp_vy]
        self._last_twist_cmd = np.zeros(4)

        # TCP Lissajous phase counter (for phase encoding in obs[32:36])
        self._tcp_t = 0.0

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            Float64MultiArray, '/marble/lqr_state', self._state_cb, 10)
        self.create_subscription(
            TwistStamped, '/marble_servo/delta_twist_cmds', self._lqr_twist_cb, 10)
        self.create_subscription(
            Empty, '/marble/landed',   self._landed_cb, _LATCHED)
        self.create_subscription(
            Empty, '/marble/fell_off', self._fell_cb,   _LATCHED)

        # ── Publisher ─────────────────────────────────────────────────────────
        self._pub = self.create_publisher(
            TwistStamped, '/marble_servo_rl/delta_twist_cmds', 10)

        self.add_on_set_parameters_callback(self._on_param_change)

        mode = 'TD3 residual' if self._use_rl else 'LQR passthrough'
        self.get_logger().info(
            f'RL residual node ready — mode={mode}  '
            f'stage={stage}  clip=±{math.degrees(self._clip):.0f}°/s  '
            f'use_rl={self._enabled}')

    # ── Parameter callback ────────────────────────────────────────────────────

    def _on_param_change(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'use_rl':
                self._enabled = bool(p.value)
                self.get_logger().info(
                    f'use_rl set to {self._enabled} — '
                    f'{"TD3 residual active" if self._enabled and self._use_rl else "LQR passthrough"}')
        return SetParametersResult(successful=True)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _state_cb(self, msg: Float64MultiArray):
        if len(msg.data) >= 10:
            self._lqr_state = np.array(msg.data[:10])
            state = self._lqr_state[:8]
            lqr_u = self._lqr_state[8:10]
            if self._use_rl and self._enabled and self._landed:
                self._compute_and_publish(state, lqr_u)

    def _lqr_twist_cb(self, msg: TwistStamped):
        self._last_twist = msg
        # Store full twist for observation: [omega_beta_cmd, omega_alpha_cmd, tcp_vx, tcp_vy]
        self._last_twist_cmd = np.array([
            msg.twist.angular.x,
            msg.twist.angular.y,
            msg.twist.linear.x,
            msg.twist.linear.y,
        ])
        if not (self._use_rl and self._enabled) or not self._landed:
            # Passthrough: RL disabled or marble not yet landed
            self._pub.publish(msg)

    def _landed_cb(self, _):
        self._landed = True
        self._tcp_t  = 0.0   # reset phase counter at each marble landing

    def _fell_cb(self, _):
        self._landed = False
        self._tcp_t  = 0.0
        self._action_hist = collections.deque([np.zeros(2)] * 10, maxlen=10)
        self._last_twist_cmd = np.zeros(4)

    # ── Inference ─────────────────────────────────────────────────────────────

    def _compute_and_publish(self, state: np.ndarray, lqr_u: np.ndarray):
        obs    = self._build_obs(state)
        # Advance phase counter AFTER building obs (matches gazebo_rl_env.py step() ordering)
        self._tcp_t += 1.0 / _CONTROL_HZ
        action, _ = self._model.predict(obs, deterministic=True)
        action = np.clip(action, -1.0, 1.0)

        # Scale to residual rad/s and add to LQR base twist
        residual = action * self._clip

        if self._last_twist is None:
            return

        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        base_x = self._last_twist.twist.angular.x   # omega_beta_cmd
        base_y = self._last_twist.twist.angular.y   # omega_alpha_cmd
        msg.twist.angular.x = float(np.clip(base_x + residual[1], -MAX_RATE, MAX_RATE))
        msg.twist.angular.y = float(np.clip(base_y + residual[0], -MAX_RATE, MAX_RATE))
        msg.twist.linear.x  = self._last_twist.twist.linear.x
        msg.twist.linear.y  = self._last_twist.twist.linear.y

        self._pub.publish(msg)

        # Update action history (oldest-first: append at right)
        self._action_hist.append(action.copy())

    def _build_obs(self, state: np.ndarray) -> np.ndarray:
        """Build 36-D observation: [state_norm(8), action_history_flat(20), twist_norm(4), phase(4)]."""
        state_norm = np.clip(state / _NORM, -3.0, 3.0)
        # Action history oldest-first (index 0 = oldest) — matches gazebo_rl_env.py
        hist_flat  = np.array(list(self._action_hist), dtype=np.float32).flatten()
        twist_norm = np.clip(self._last_twist_cmd / _TWIST_NORM, -3.0, 3.0).astype(np.float32)
        # Lissajous phase encoding — always computed when landed (TCP Lissajous is active at deployment)
        ox    = _TCP_FA * _TCP_OMEGA0
        oy    = _TCP_FB * _TCP_OMEGA0
        phi_x = ox * self._tcp_t + _TCP_DELTA
        phi_y = oy * self._tcp_t
        phase = np.array([math.sin(phi_x), math.cos(phi_x),
                           math.sin(phi_y), math.cos(phi_y)], dtype=np.float32)
        obs = np.concatenate([state_norm, hist_flat, twist_norm, phase]).astype(np.float32)

        if self._running_stats is not None:
            mean = self._running_stats['mean'].astype(np.float32)
            std  = np.sqrt(self._running_stats['var'].astype(np.float32) + 1e-8)
            obs  = np.clip((obs - mean) / std, -3.0, 3.0)

        return obs


def main(args=None):
    rclpy.init(args=args)
    node = RLResidualNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
