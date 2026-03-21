"""
rl_residual_node.py
-------------------
ROS2 deployment node for the trained SAC residual controller.

Subscribes to:
  /marble/lqr_state       (std_msgs/Float64MultiArray) — 8-D state + 2-D LQR output
  /marble/desired_pos     (geometry_msgs/Point)         — Lissajous setpoint
  /tcp/lissajous_vel      (geometry_msgs/TwistStamped)  — TCP velocity
  /marble/lqr_twist       (geometry_msgs/TwistStamped)  — raw LQR twist (for passthrough)
  /marble/landed          (std_msgs/Empty, LATCHED)
  /marble/fell_off        (std_msgs/Empty, LATCHED)

Publishes to:
  /marble_servo_rl/delta_twist_cmds  (geometry_msgs/TwistStamped) — LQR + RL residual

The mux_controller is configured via `auto_topic` parameter to subscribe to
/marble_servo_rl/delta_twist_cmds instead of /marble_servo/delta_twist_cmds
when rl:=true.

Launch parameters:
  rl_model  — path to trained SAC model (.zip)
  rl_norm   — path to VecNormalize stats (.pkl) — optional
  rl_stage  — curriculum stage (0-3) controls residual clip budget
"""

import os
import sys
import math
import collections
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TwistStamped, Point
from std_msgs.msg import Empty, Float64MultiArray

_LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)

# Normalisation constants (must match ball_plate_env.py)
MAX_RATE = math.radians(45.0)
_NORM    = np.array([0.20, 0.50, 0.20, 0.50, 0.30, MAX_RATE, 0.30, MAX_RATE])
_NORM_LQR  = np.array([MAX_RATE, MAX_RATE])
_NORM_TCP  = np.array([0.30, 0.30, 0.40, 0.40])
_NORM_ERR  = np.array([0.20, 0.20] * 5)

# Curriculum clip budgets (matches ball_plate_env.STAGES)
_CLIP_BUDGETS = [
    math.radians(2.0),
    math.radians(5.0),
    math.radians(10.0),
    math.radians(20.0),
]


class RLResidualNode(Node):

    def __init__(self):
        super().__init__('rl_residual_node')

        self.declare_parameter('rl_model', '')
        self.declare_parameter('rl_norm',  '')
        self.declare_parameter('rl_stage', 3)

        model_path = self.get_parameter('rl_model').value
        norm_path  = self.get_parameter('rl_norm').value
        stage      = int(self.get_parameter('rl_stage').value)
        self._clip = _CLIP_BUDGETS[min(stage, len(_CLIP_BUDGETS) - 1)]

        # ── Load policy ───────────────────────────────────────────────────────
        self._model     = None
        self._vec_norm  = None
        self._use_rl    = False

        if model_path and os.path.exists(model_path):
            try:
                # Add rl_training to path for BallPlateEnv import
                rl_dir = os.path.join(
                    os.path.dirname(__file__), '..', 'rl_training')
                sys.path.insert(0, os.path.abspath(rl_dir))

                from stable_baselines3 import SAC
                self._model = SAC.load(model_path)
                self._use_rl = True
                self.get_logger().info(f'Loaded RL policy: {model_path}')

                if norm_path and os.path.exists(norm_path):
                    from stable_baselines3.common.vec_env import (
                        VecNormalize, DummyVecEnv)
                    from ball_plate_env import BallPlateEnv
                    dummy = DummyVecEnv([lambda: BallPlateEnv(stage=stage)])
                    self._vec_norm = VecNormalize.load(norm_path, dummy)
                    self._vec_norm.training = False
                    self._vec_norm.norm_reward = False
                    self.get_logger().info(f'Loaded normalisation stats: {norm_path}')
            except Exception as e:
                self.get_logger().error(
                    f'Failed to load RL model: {e} — running as LQR passthrough')
        else:
            self.get_logger().warn(
                f'rl_model not found: "{model_path}" — running as LQR passthrough')

        # ── State ─────────────────────────────────────────────────────────────
        self._lqr_state   = np.zeros(10)   # 8-D state + 2-D LQR output
        self._desired     = np.zeros(2)
        self._tcp_vel_x   = 0.0
        self._tcp_vel_y   = 0.0
        self._err_hist    = collections.deque([np.zeros(2)] * 5, maxlen=5)
        self._landed      = False
        self._last_twist  = None   # passthrough from LQR

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            Float64MultiArray, '/marble/lqr_state', self._state_cb, 10)
        self.create_subscription(
            TwistStamped, '/marble_servo/delta_twist_cmds', self._lqr_twist_cb, 10)
        self.create_subscription(
            Point, '/marble/desired_pos', self._desired_cb, 10)
        self.create_subscription(
            TwistStamped, '/tcp/lissajous_vel', self._tcp_vel_cb, 10)
        self.create_subscription(
            Empty, '/marble/landed',   self._landed_cb,  _LATCHED)
        self.create_subscription(
            Empty, '/marble/fell_off', self._fell_cb,    _LATCHED)

        # ── Publisher ─────────────────────────────────────────────────────────
        self._pub = self.create_publisher(
            TwistStamped, '/marble_servo_rl/delta_twist_cmds', 10)

        mode = 'SAC residual' if self._use_rl else 'LQR passthrough'
        self.get_logger().info(
            f'RL residual node ready — mode={mode}  '
            f'stage={stage}  clip=±{math.degrees(self._clip):.0f}°/s')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _state_cb(self, msg: Float64MultiArray):
        if len(msg.data) >= 10:
            self._lqr_state = np.array(msg.data[:10])
            state  = self._lqr_state[:8]
            lqr_u  = self._lqr_state[8:10]
            x_err  = state[0] - self._desired[0]
            y_err  = state[2] - self._desired[1]
            self._err_hist.append(np.array([x_err, y_err]))

            if self._use_rl and self._landed:
                self._compute_and_publish(state, lqr_u)

    def _lqr_twist_cb(self, msg: TwistStamped):
        self._last_twist = msg
        if not self._use_rl or not self._landed:
            # Passthrough: RL disabled or marble not landed
            self._pub.publish(msg)

    def _desired_cb(self, msg: Point):
        self._desired = np.array([msg.x, msg.y])

    def _tcp_vel_cb(self, msg: TwistStamped):
        self._tcp_vel_x = msg.twist.linear.x
        self._tcp_vel_y = msg.twist.linear.y

    def _landed_cb(self, _):
        self._landed = True

    def _fell_cb(self, _):
        self._landed = False
        self._err_hist = collections.deque([np.zeros(2)] * 5, maxlen=5)

    # ── Inference ─────────────────────────────────────────────────────────────

    def _compute_and_publish(self, state: np.ndarray, lqr_u: np.ndarray):
        obs = self._build_obs(state, lqr_u)
        action, _ = self._model.predict(obs, deterministic=True)

        # Scale action [-1,1] → residual in rad/s
        residual = np.clip(action, -1.0, 1.0) * self._clip

        # Build output twist: LQR base + RL residual
        if self._last_twist is None:
            return

        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        # Angular: add residual (clamped to MAX_RATE combined)
        base_x = self._last_twist.twist.angular.x
        base_y = self._last_twist.twist.angular.y
        msg.twist.angular.x = float(
            np.clip(base_x + residual[1], -MAX_RATE, MAX_RATE))  # beta
        msg.twist.angular.y = float(
            np.clip(base_y + residual[0], -MAX_RATE, MAX_RATE))  # alpha
        # Linear: pass through unchanged (TCP Lissajous motion)
        msg.twist.linear.x = self._last_twist.twist.linear.x
        msg.twist.linear.y = self._last_twist.twist.linear.y

        self._pub.publish(msg)

    def _build_obs(self, state: np.ndarray, lqr_u: np.ndarray) -> np.ndarray:
        tcp_state = np.array([0.0, 0.0, self._tcp_vel_x, self._tcp_vel_y])
        err_flat  = np.array(list(self._err_hist)).flatten()

        obs = np.concatenate([
            state / _NORM,
            lqr_u / _NORM_LQR,
            tcp_state / _NORM_TCP,
            err_flat / _NORM_ERR,
        ]).astype(np.float32)

        if self._vec_norm is not None:
            obs = self._vec_norm.normalize_obs(obs.reshape(1, -1)).flatten()

        return np.clip(obs, -3.0, 3.0)


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
