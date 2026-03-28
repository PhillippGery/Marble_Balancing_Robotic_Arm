"""
marble_servo_controller.py
--------------------------
Real-time LQR marble balancing controller with PT1 robot delay model.

State vector: [x, vx, y, vy, alpha, omega_alpha, beta, omega_beta]
  alpha/beta         — plate tilt angles from plate_tcp TF quaternion (ZYX Euler)
  omega_alpha/beta   — plate angular velocities from rotational Jacobian × q_dot

Control inputs: [omega_alpha_cmd, omega_beta_cmd]  (rad/s velocity commands)

Servo command frame: base_link — MoveIt Servo resolves the transform to plate_tcp via TF.
  angular.x = omega_beta_cmd   (world X rotation → controls Y marble dynamics)
  angular.y = omega_alpha_cmd  (world Y rotation → controls X marble dynamics)
"""

import math
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, Point, Vector3
from std_msgs.msg import Empty, Float64MultiArray
from std_srvs.srv import Trigger
import numpy as np
import tf2_ros

# Latched QoS: late subscribers always receive the last published value
_LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)

from marble_balancer.lqr_math import compute_dlqr, DEFAULT_Q, DEFAULT_R, T_ROBOT
from marble_balancer.marble_ekf import MarbleEKF
from sensor_msgs.msg import JointState


# ── Jacobian: joint child-link frames (all joints rotate about local Z) ───────
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


def _quat_to_rot(qx, qy, qz, qw) -> np.ndarray:
    """Quaternion → 3×3 rotation matrix."""
    return np.array([
        [1-2*(qy*qy+qz*qz),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [  2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz),   2*(qy*qz-qx*qw)],
        [  2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)],
    ])


# ── Tunable parameters ────────────────────────────────────────────────────────
CONTROL_HZ = 30.0
MAX_RATE   = np.deg2rad(45.0)     # max angular rate command to servo (rad/s)
OMEGA_LPF_TC = 0.01               # low-pass filter time constant for Jacobian omega (s)
USE_EKF    = True                 # True = EKF omega,  False = Jacobian omega
                                  # smaller = faster response but noisier
                                  # larger  = smoother but more lag (PT1 limit)

# Landing detection — relaxed so marble is detected even when sliding on arrival
LAND_Z_MARGIN  = 0.030    # ±3 cm z-window around plate top
LAND_VZ_MAX    = 0.50     # allow sliding marble (only blocks free-falling marble)
LAND_CONFIRM   = 5        # 5 consecutive odom ticks ≈ 0.17 s at 30 Hz
# ─────────────────────────────────────────────────────────────────────────────

MARBLE_RADIUS   = 0.015
PLATE_THICKNESS = 0.005

HOME_TIME_S  = 4   # seconds to wait before clearing _homing (go_to_pose takes ~6s)


def _quat_to_rpy(qx, qy, qz, qw):
    """Extract roll (X), pitch (Y), yaw (Z) from quaternion."""
    roll  = math.atan2(2.0 * (qw * qx + qy * qz),
                       1.0 - 2.0 * (qx * qx + qy * qy))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx))))
    yaw   = math.atan2(2.0 * (qw * qz + qx * qy),
                       1.0 - 2.0 * (qy * qy + qz * qz))
    return roll, pitch, yaw


class MarbleServoController(Node):

    def __init__(self):
        super().__init__('marble_servo_controller')

        dt = 1.0 / CONTROL_HZ

        # ── LQR gain ──────────────────────────────────────────────────────────
        self._K, _, _ = compute_dlqr(DEFAULT_Q, DEFAULT_R, dt)
        self.get_logger().info(
            f'LQR K (PT1 model, T={T_ROBOT}s):\n{np.array2string(self._K, precision=3)}')

        self._dt = dt

        self.get_logger().info(f'Omega source: {"EKF" if USE_EKF else "Jacobian"}')

        # State: [x, vx, y, vy, alpha, omega_alpha, beta, omega_beta]
        self._state = np.zeros(8)

        # EKF — omega estimation only (vx/vy use proven EMA below)
        self._ekf        = MarbleEKF()
        self._u_ekf      = np.zeros(2)   # last hardware-sense command fed to EKF predict
        self._last_meas_t = None          # timestamp of last EKF update (s)

        # Jacobian omega fallback (used when use_ekf:=false)
        self._omega_alpha_actual = 0.0
        self._omega_beta_actual  = 0.0

        # Marble position history for EMA velocity differentiation (unchanged from pre-EKF)
        self._prev_odom_t = None
        self._prev_mx     = 0.0
        self._prev_my     = 0.0

        # Joint state for Jacobian: q_dot from velocity field or position differentiation
        # (Jacobian omega published to /marble/plate_omega for diagnostics only)
        self._q_dot      = np.zeros(6)
        self._q_prev     = None
        self._q_prev_t   = None

        # ── TF ────────────────────────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._plate_z     = None

        # ── Landing / homing state ────────────────────────────────────────────
        self._marble_z    = None
        self._marble_vz   = 0.0
        self._landed      = False
        self._land_ticks  = 0
        self._homing      = False   # True while robot returns home; Servo is stopped

        # ── Publishers / subscribers ──────────────────────────────────────────
        self._twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)
        # LQR state published for rl_residual_node (8-D + 2-D LQR output = 10 floats)
        self._state_pub = self.create_publisher(
            Float64MultiArray, '/marble/lqr_state', 10)
        self._plate_omega_pub = self.create_publisher(
            TwistStamped, '/marble/plate_omega', 10)
        self._fell_off_pub = self.create_publisher(
            Empty, '/marble/fell_off', _LATCHED)
        self._landed_pub = self.create_publisher(
            Empty, '/marble/landed', _LATCHED)

        # Desired marble position — published by marble_lissajous_node (or stays 0/0)
        self._desired = np.zeros(2)   # [x_d, y_d] in plate frame (m)
        self._desired_sub = self.create_subscription(
            Point, '/marble/desired_pos', self._desired_cb, 10)

        # TCP Lissajous — linear velocity to blend into servo command + feedforward tilt
        self._tcp_lin_x  = 0.0
        self._tcp_lin_y  = 0.0
        self._ff_alpha   = 0.0   # feedforward desired alpha (rad)
        self._ff_beta    = 0.0   # feedforward desired beta  (rad)
        self.create_subscription(
            TwistStamped, '/tcp/lissajous_vel', self._tcp_vel_cb, 10)
        self.create_subscription(
            Vector3, '/tcp/lissajous_ff_tilt', self._tcp_ff_cb, 10)

        self._odom_sub = self.create_subscription(
            Odometry, '/marble/odom', self._odom_cb, 10)
        self._js_sub = self.create_subscription(
            JointState, '/joint_states', self._js_cb, 10)

        self._timer = self.create_timer(dt, self._control_cb)

        # ── Start MoveIt Servo ────────────────────────────────────────────────
        self._start_servo()

        self.get_logger().info(
            f'Marble servo controller ready at {CONTROL_HZ:.0f} Hz — '
            'waiting for marble to land…')

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _start_servo(self):
        client = self.create_client(Trigger, '/servo_node/start_servo')
        self.get_logger().info('Waiting for /servo_node/start_servo …')
        if client.wait_for_service(timeout_sec=10.0):
            future = client.call_async(Trigger.Request())
            future.add_done_callback(self._on_servo_started)
        else:
            self.get_logger().error(
                '/servo_node/start_servo not available — Servo will not move!')

    def _on_servo_started(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('MoveIt Servo started.')
            else:
                self.get_logger().warn(f'start_servo: {result.message}')
        except Exception as e:
            self.get_logger().error(f'start_servo call failed: {e}')

    def _stop_servo(self):
        client = self.create_client(Trigger, '/servo_node/stop_servo')
        if client.wait_for_service(timeout_sec=2.0):
            future = client.call_async(Trigger.Request())
            future.add_done_callback(
                lambda f: self.get_logger().info('MoveIt Servo stopped.'))
        else:
            self.get_logger().warn('stop_servo service not available.')

    def _js_cb(self, msg: JointState):
        """Compute q_dot, then update plate angular velocity via rotational Jacobian.

        Velocity source priority:
          1. msg.velocity  — encoder-based, available on real UR5e driver and Gazebo
          2. Position differentiation — fallback if velocity field is empty
        """
        pos_map = dict(zip(msg.name, msg.position))
        q_new = np.array([pos_map.get(j, 0.0) for j in _JOINT_NAMES])
        t_new = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # dt for LPF (computed before updating history)
        if self._q_prev_t is not None:
            dt_js = (t_new - self._q_prev_t)
        else:
            dt_js = 1.0 / CONTROL_HZ

        

        # Prefer velocity field (no differentiation noise); fall back to position diff
        if msg.velocity and len(msg.velocity) == len(msg.name):
            vel_map = dict(zip(msg.name, msg.velocity))
            q_dot_raw = np.array([vel_map.get(j, 0.0) for j in _JOINT_NAMES])
        elif self._q_prev is not None and 0.0 < dt_js < 0.5:
            q_dot_raw = (q_new - self._q_prev) / dt_js
        else:
            q_dot_raw = np.zeros(6)

        self._q_prev   = q_new
        self._q_prev_t = t_new

        # EMA low-pass to smooth Jacobian output
        alpha_f = dt_js / (OMEGA_LPF_TC + dt_js)
        self._q_dot = alpha_f * q_dot_raw + (1.0 - alpha_f) * self._q_dot
        omega = self._compute_plate_omega()
        if omega is not None:
            self._omega_alpha_actual = omega[1]   # world Y → α̇
            self._omega_beta_actual  = omega[0]   # world X → β̇
            om_msg = TwistStamped()
            om_msg.header.stamp    = msg.header.stamp
            om_msg.header.frame_id = 'world'
            om_msg.twist.angular.x = float(omega[0])   # β̇  (roll rate)
            om_msg.twist.angular.y = float(omega[1])   # α̇  (pitch rate)
            self._plate_omega_pub.publish(om_msg)

    def _compute_plate_omega(self):
        """
        Build the 3×6 rotational Jacobian from TF and multiply by q_dot.
        Each column = Z-axis of the joint's child-link frame in world frame,
        because every UR5e joint rotates about its local Z axis.
        Returns [ω_x, ω_y, ω_z] in world frame, or None if TF not ready.
        """
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

    def _get_plate_tf(self):
        """Return (plate_tf, plate_top_z) or (None, None)."""
        for frame in ('plate_tcp', 'marble_plate', 'tool0'):
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

    # ── Odometry callback ─────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self._marble_z  = msg.pose.pose.position.z
        self._marble_vz = msg.twist.twist.linear.z

        tf, plate_top_z = self._get_plate_tf()
        if tf is None:
            return

        plate_x = tf.transform.translation.x
        plate_y = tf.transform.translation.y

        # Timestamp from odom message
        t_now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # ── Marble position relative to plate centre (world frame) ────────────
        mx = (msg.pose.pose.position.x - plate_x) * -1.0   # invert X: plate_tcp yaw≈180° at home
        my = (msg.pose.pose.position.y - plate_y)

        # ── Plate roll/pitch from TF quaternion ───────────────────────────────
        # alpha = pitch (Y-axis rotation) → controls X ball motion  (dxd/dt = -C*alpha)
        # beta  = roll  (X-axis rotation) → controls Y ball motion  (dyd/dt = -C*beta)
        q = tf.transform.rotation
        roll, pitch, _ = _quat_to_rpy(q.x, q.y, q.z, q.w)
        alpha = pitch
        beta  = roll

        # ── EMA marble velocity (same as pre-EKF — proven performance) ──────────
        if self._prev_odom_t is not None:
            dt_odom_ema = t_now - self._prev_odom_t
            if 0.0 < dt_odom_ema < 0.5:
                vx_raw = (mx - self._prev_mx) / dt_odom_ema
                vy_raw = (my - self._prev_my) / dt_odom_ema
                vf = dt_odom_ema / (OMEGA_LPF_TC + dt_odom_ema)
                self._state[1] += vf * (vx_raw - self._state[1])
                self._state[3] += vf * (vy_raw - self._state[3])
        self._prev_mx     = mx
        self._prev_my     = my
        self._prev_odom_t = t_now

        # Direct measurements
        self._state[0] = mx
        self._state[2] = my
        self._state[4] = alpha
        self._state[6] = beta

        # ── Omega estimation ──────────────────────────────────────────────────
        if USE_EKF:
            dt_odom = (t_now - self._last_meas_t) if self._last_meas_t is not None else 0.02
            self._last_meas_t = t_now
            self._ekf.predict(self._u_ekf, dt_odom)
            self._ekf.update(np.array([mx, my, alpha, beta]))
            self._state[5] = self._ekf.x[5]
            self._state[7] = self._ekf.x[7]
        else:
            self._state[5] = self._omega_alpha_actual   # Jacobian
            self._state[7] = self._omega_beta_actual

        if self._landed:
            self.get_logger().info(
                f'err x:{self._state[0]:+.4f} y:{self._state[2]:+.4f}  '
                f'ωα:{np.rad2deg(self._state[5]):+.1f}°/s '
                f'ωβ:{np.rad2deg(self._state[7]):+.1f}°/s',
                throttle_duration_sec=1.0)



        # Landing detection
        marble_in_z = (
            self._marble_z >= plate_top_z - LAND_Z_MARGIN
            and self._marble_z <= plate_top_z + MARBLE_RADIUS + LAND_Z_MARGIN
        )

        if not self._landed:
            if not self._homing and marble_in_z and abs(self._marble_vz) < LAND_VZ_MAX:
                self._land_ticks += 1
                if self._land_ticks >= LAND_CONFIRM:
                    self._landed = True
                    # Warm-start EKF with current measurements
                    x0_ekf = np.array([mx, 0.0, my, 0.0, alpha, 0.0, beta, 0.0])
                    self._ekf.reset(x0_ekf)
                    self._u_ekf[:] = 0.0
                    self._landed_pub.publish(Empty())
                    self.get_logger().info(
                        f'Marble landed at z={self._marble_z:.4f} m — '
                        'restarting Servo, activating LQR.')
                    self._start_servo()
            else:
                self._land_ticks = 0
        else:
            if self._marble_z < plate_top_z - 0.05:
                self.get_logger().warn(
                    f'Marble fell off (z={self._marble_z:.3f} m) — homing robot.')
                self._landed     = False
                self._homing     = True
                self._land_ticks = 0
                self._ekf.reset()
                self._u_ekf[:] = 0.0
                self._omega_alpha_actual = 0.0
                self._omega_beta_actual  = 0.0
                self._prev_odom_t = None   # reset EMA velocity history

                # Notify plotter to save & plot
                self._fell_off_pub.publish(Empty())

                # Stop Servo so it doesn't override the home trajectory, then home
                self._stop_servo()
                self._send_home()

    def _send_home(self):
        """Launch go_to_pose in a subprocess — uses the same IK target as the initial homing."""
        subprocess.Popen(['ros2', 'run', 'marble_balancer', 'go_to_pose'])
        self.get_logger().info('go_to_pose launched — robot returning to home.')
        # Clear homing flag after go_to_pose is expected to finish
        self.create_timer(
            float(HOME_TIME_S),
            self._on_home_complete,
            clock=self.get_clock())

    def _on_home_complete(self):
        self._homing = False
        self.get_logger().info('Robot at home — ready for new marble.')

    def _tcp_vel_cb(self, msg: TwistStamped):
        """Receive TCP linear velocity from tcp_lissajous_node."""
        self._tcp_lin_x = msg.twist.linear.x
        self._tcp_lin_y = msg.twist.linear.y

    def _tcp_ff_cb(self, msg: Vector3):
        """Receive feedforward tilt from tcp_lissajous_node."""
        self._ff_alpha = msg.x
        self._ff_beta  = msg.y

    def _desired_cb(self, msg: Point):
        """Update desired marble position from Lissajous node (or any setpoint publisher)."""
        self._desired[0] = msg.x
        self._desired[1] = msg.y

    # ── Control loop ──────────────────────────────────────────────────────────

    def _control_cb(self):
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        if not self._landed:
            if not self._homing:
                self._twist_pub.publish(msg)   # zero twist keeps Servo happy
            return

        # LQR: u = -K @ (x - x_d)  — subtract desired setpoint (default 0/0)
        error = self._state.copy()
        error[0] -= self._desired[0]   # x error
        error[2] -= self._desired[1]   # y error
        # Feedforward tilt from TCP Lissajous: shift desired plate angle so LQR
        # pre-compensates the pseudo-forces induced by TCP acceleration.
        error[4] -= self._ff_alpha     # desired alpha offset
        error[6] -= self._ff_beta      # desired beta offset
        u_clip = np.clip(-self._K @ error, -MAX_RATE, MAX_RATE)
        omega_alpha_cmd = -float(u_clip[0])   # negate for plate_tcp yaw≈180°
        omega_beta_cmd  = -float(u_clip[1])
        # EKF input: model-frame command (pre-negation) — consistent with LQR design
        self._u_ekf[:] = u_clip

        # ── World-frame twist (base_link) — servo resolves transform to plate_tcp via TF ─
        # angular.x (world X) → roll  → beta  → Y marble dynamics
        # angular.y (world Y) → pitch → alpha → X marble dynamics
        msg.twist.angular.x  = omega_beta_cmd
        msg.twist.angular.y  = omega_alpha_cmd   # world Y rotation → alpha → X dynamics
        # TCP Lissajous linear motion (zeros when tcp_lissajous_node is not running)
        msg.twist.linear.x   = self._tcp_lin_x
        msg.twist.linear.y   = self._tcp_lin_y

        # self.get_logger().info(
        #     f'u: ωα={np.rad2deg(omega_alpha_cmd):+.1f}°/s '
        #     f'ωβ={np.rad2deg(omega_beta_cmd):+.1f}°/s  '
        #     f'angular x={np.rad2deg(msg.twist.angular.x):+.1f} y={np.rad2deg(msg.twist.angular.y):+.1f}°/s',
        #     throttle_duration_sec=1.0)
        

        self._twist_pub.publish(msg)

        # Publish state + LQR output for rl_residual_node (10 floats)
        state_msg = Float64MultiArray()
        state_msg.data = list(self._state) + [omega_alpha_cmd, omega_beta_cmd]
        self._state_pub.publish(state_msg)


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
