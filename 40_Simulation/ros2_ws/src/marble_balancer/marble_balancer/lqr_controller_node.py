"""
lqr_controller_node.py

ROS2 node: marble_lqr_controller

Subscribes to the Gazebo ground-truth marble pose, runs a discrete-time LQR
controller, and publishes a target TCP pose for MoveIt to execute.

Topic flow:
  /model/marble/pose  (gz_msgs/Pose via ros_gz_bridge)
      → discrete LQR (lqr_math.py)
      → /target_tcp_pose  (geometry_msgs/PoseStamped)
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation

from marble_balancer.lqr_math import compute_dlqr, DEFAULT_Q, DEFAULT_R


class MarbleLQRController(Node):

    def __init__(self):
        super().__init__('marble_lqr_controller')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('tcp_x', 0.0)
        self.declare_parameter('tcp_y', 0.4)
        self.declare_parameter('tcp_z', 0.7)
        self.declare_parameter('Q_diag', [1000.0, 100.0, 1000.0, 100.0, 1.0, 0.1, 1.0, 0.1])
        self.declare_parameter('R_scale', 0.01)
        self.declare_parameter('max_angle_deg', 15.0)
        self.declare_parameter('control_rate_hz', 10.0)

        tcp_x = self.get_parameter('tcp_x').value
        tcp_y = self.get_parameter('tcp_y').value
        tcp_z = self.get_parameter('tcp_z').value
        q_diag = self.get_parameter('Q_diag').value
        r_scale = self.get_parameter('R_scale').value
        self.max_angle = math.radians(self.get_parameter('max_angle_deg').value)
        rate_hz = self.get_parameter('control_rate_hz').value

        self.tcp_pos = np.array([tcp_x, tcp_y, tcp_z])
        self.dt = 1.0 / rate_hz

        # ── Discrete LQR ──────────────────────────────────────────────────────
        Q = np.diag(q_diag)
        R = np.eye(2) * r_scale
        self.K_d, self.Ad, self.Bd = compute_dlqr(Q, R, self.dt)
        self.get_logger().info(
            f'Discrete LQR ready  (dt={self.dt:.3f}s)\n'
            f'K_d =\n{self.K_d}'
        )

        # ── Internal state ─────────────────────────────────────────────────────
        # Full state: [x, x_dot, y, y_dot, alpha, alpha_dot, beta, beta_dot]
        self._state = np.zeros(8)
        self._marble_x_prev = None
        self._marble_y_prev = None
        self._got_marble = False

        # ── Pub / Sub ─────────────────────────────────────────────────────────
        #self.tcp_pub = self.create_publisher(PoseStamped, '/target_tcp_pose', 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel_ee', 10)

        # Marble ground truth: classic Gazebo P3D plugin → nav_msgs/Odometry on /marble/odom
        self.create_subscription(
            Odometry,
            '/marble/odom',
            self._marble_cb,
            10
        )

        # Control loop timer
        self.create_timer(self.dt, self._control_loop)

        self.get_logger().info(
            f'Marble LQR controller started  '
            f'(rate={rate_hz} Hz, tcp=[{tcp_x},{tcp_y},{tcp_z}])'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _marble_cb(self, msg: Odometry):
        """Store latest marble XY position from Gazebo P3D ground truth (nav_msgs/Odometry)."""
        mx = msg.pose.pose.position.x
        my = msg.pose.pose.position.y

        if self._marble_x_prev is None:
            self._marble_x_prev = mx
            self._marble_y_prev = my

        # Finite-difference velocity estimate
        xd = (mx - self._marble_x_prev) / self.dt
        yd = (my - self._marble_y_prev) / self.dt

        # Update ball states in full state vector
        self._state[0] = mx
        self._state[1] = xd
        self._state[2] = my
        self._state[3] = yd

        self._marble_x_prev = mx
        self._marble_y_prev = my
        self._got_marble = True

    def _control_loop(self):
        """Discrete-time LQR step, publishes target TCP pose."""

        
        cmd = Twist()

        # Reference: all zeros (marble centered, board flat)
        s_ref = np.zeros(8)
        error = self._state - s_ref

        # Discrete control law: u[k] = -K_d @ error[k]
        u = -self.K_d @ error   # [alpha_ddot, beta_ddot]

        # Discrete plate state update (states 4-7)
        # alpha_dot[k+1] = alpha_dot[k] + alpha_ddot * dt
        # alpha[k+1]     = alpha[k]     + alpha_dot[k] * dt
        alpha     = self._state[4]
        alpha_dot = self._state[5]
        beta      = self._state[6]
        beta_dot  = self._state[7]

        alpha_ddot = u[0]
        beta_ddot  = u[1]

        alpha_dot_new = alpha_dot + alpha_ddot * self.dt
        alpha_new     = alpha     + alpha_dot  * self.dt
        beta_dot_new  = beta_dot  + beta_ddot  * self.dt
        beta_new      = beta      + beta_dot   * self.dt

        # Angle saturation (±max_angle)
        alpha_new = float(np.clip(alpha_new, -self.max_angle, self.max_angle))
        beta_new  = float(np.clip(beta_new,  -self.max_angle, self.max_angle))

        # Update plate states
        self._state[4] = alpha_new
        self._state[5] = alpha_dot_new
        self._state[6] = beta_new
        self._state[7] = beta_dot_new

        # ── Build TCP pose ─────────────────────────────────────────────────
        # Rotate around X by alpha (plate tilt in X) then around Y by beta
        rot = Rotation.from_euler('xy', [alpha_new, beta_new])
        q = rot.as_quat()   # [x, y, z, w]

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x = self.tcp_pos[0]
        msg.pose.position.y = self.tcp_pos[1]
        msg.pose.position.z = self.tcp_pos[2]
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self.tcp_pub.publish(msg)

        self.get_logger().debug(
            f'alpha={math.degrees(alpha_new):.2f}° '
            f'beta={math.degrees(beta_new):.2f}°  '
            f'marble=({self._state[0]:.3f},{self._state[2]:.3f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MarbleLQRController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
