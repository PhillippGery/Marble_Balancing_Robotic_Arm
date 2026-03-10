"""
marble_servo_controller.py
--------------------------
Real-time LQR marble balancing controller.

Subscribes to /marble/odom for marble ground-truth position,
runs discrete-time LQR, and publishes angular velocity twist
commands to MoveIt Servo (/servo_node/delta_twist_cmds).

Control pipeline:
  /marble/odom  ──►  state [x, ẋ, y, ẏ, α, α̇, β, β̇]
                          ──►  u = -K @ state  (LQR)
                                   ──►  integrate α̈,β̈ → α̇,β̇
                                            ──►  TwistStamped  ──►  MoveIt Servo
                                                                ──►  joint trajectory
                                                                ──►  UR5e tilts plate

Sign convention (matches lqr_math.py A matrix):
  alpha = plate roll  (rotation around world X): positive α → marble accelerates in -X
  beta  = plate pitch (rotation around world Y): positive β → marble accelerates in -Y

  MoveIt Servo frame: base_link
    angular.x = roll rate  → alpha_dot
    angular.y = pitch rate → beta_dot
  (flip signs below if observed marble motion is opposite to expected)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
import numpy as np

from marble_balancer.lqr_math import compute_dlqr, DEFAULT_Q, DEFAULT_R


# ── Tunable parameters ──────────────────────────────────────────────────────────
CONTROL_HZ = 30.0                    # must match servo publish_period (~30 Hz)
MAX_ANGLE  = np.deg2rad(15.0)        # plate tilt saturation  (rad)
MAX_RATE   = np.deg2rad(20.0)        # max angular rate command to servo (rad/s)

# TCP home position — must match go_to_pose TARGET_X / Y / Z
TCP_X = 0.133
TCP_Y = 0.250
# ───────────────────────────────────────────────────────────────────────────────


class MarbleServoController(Node):

    def __init__(self):
        super().__init__('marble_servo_controller')

        dt = 1.0 / CONTROL_HZ

        # ── Compute LQR gain once at startup ──────────────────────────────────
        self._K, _, _ = compute_dlqr(DEFAULT_Q, DEFAULT_R, dt)
        self.get_logger().info(
            f'LQR gain K (2x8) computed:\n{np.array2string(self._K, precision=4)}')

        # ── State vector: [x, ẋ, y, ẏ, α, α̇, β, β̇] ─────────────────────────
        self._state = np.zeros(8)

        # Flag: hold zero twist until first marble reading arrives
        self._marble_received = False

        self._dt = dt

        # ── Publishers / subscribers ──────────────────────────────────────────
        self._twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)

        self._odom_sub = self.create_subscription(
            Odometry, '/marble/odom', self._odom_cb, 10)

        self._timer = self.create_timer(dt, self._control_cb)

        self.get_logger().info(
            f'Marble servo controller started at {CONTROL_HZ:.0f} Hz. '
            'Waiting for /marble/odom...')

    # ── Marble odometry callback ───────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        # Position in world frame → relative to plate centre (small-angle approx)
        self._state[0] = msg.pose.pose.position.x - TCP_X   # x
        self._state[2] = msg.pose.pose.position.y - TCP_Y   # y

        # Velocity: use odometry twist (p3d publishes linear velocity in world frame)
        self._state[1] = msg.twist.twist.linear.x            # ẋ
        self._state[3] = msg.twist.twist.linear.y            # ẏ

        if not self._marble_received:
            self.get_logger().info(
                f'Marble detected — plate-relative pos: '
                f'({self._state[0]:.3f}, {self._state[2]:.3f}) m')
            self._marble_received = True

    # ── Control loop ──────────────────────────────────────────────────────────
    def _control_cb(self):
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        if not self._marble_received:
            # Marble not yet in scene — keep robot still with zero twist
            self._twist_pub.publish(msg)
            return

        # ── LQR control law ────────────────────────────────────────────────
        # u = [alpha_ddot, beta_ddot]
        u = -self._K @ self._state

        # ── Integrate accelerations → rates ───────────────────────────────
        alpha_dot_new = self._state[5] + u[0] * self._dt
        beta_dot_new  = self._state[7] + u[1] * self._dt

        # ── Integrate rates → angles (for saturation) ──────────────────────
        alpha_new = self._state[4] + alpha_dot_new * self._dt
        beta_new  = self._state[6] + beta_dot_new  * self._dt

        # ── Angle saturation — clamp and zero the rate if at limit ─────────
        if abs(alpha_new) >= MAX_ANGLE:
            alpha_new     = np.clip(alpha_new, -MAX_ANGLE, MAX_ANGLE)
            alpha_dot_new = 0.0

        if abs(beta_new) >= MAX_ANGLE:
            beta_new     = np.clip(beta_new, -MAX_ANGLE, MAX_ANGLE)
            beta_dot_new = 0.0

        # ── Rate saturation ────────────────────────────────────────────────
        alpha_dot_new = np.clip(alpha_dot_new, -MAX_RATE, MAX_RATE)
        beta_dot_new  = np.clip(beta_dot_new,  -MAX_RATE, MAX_RATE)

        # ── Write back angle states ────────────────────────────────────────
        self._state[4] = alpha_new
        self._state[5] = alpha_dot_new
        self._state[6] = beta_new
        self._state[7] = beta_dot_new

        # ── Publish to MoveIt Servo ────────────────────────────────────────
        # NOTE: if the marble moves opposite to expected, negate alpha_dot / beta_dot
        msg.twist.angular.x = alpha_dot_new   # roll  rate (affects marble X)
        msg.twist.angular.y = beta_dot_new    # pitch rate (affects marble Y)

        self._twist_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MarbleServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
