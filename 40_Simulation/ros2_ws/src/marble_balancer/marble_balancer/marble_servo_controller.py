"""
marble_servo_controller.py
--------------------------
Real-time LQR marble balancing controller with PT1 robot delay model.

State vector: [x, vx, y, vy, alpha, omega_alpha, beta, omega_beta]
  alpha/beta     — plate tilt angles read from TF (world → plate_tcp)
  omega_alpha/beta — actual plate angular velocities, estimated via PT1

Control inputs: [omega_alpha_cmd, omega_beta_cmd]  (rad/s velocity commands)

The LQR output is in world frame (alpha around world-X, beta around world-Y).
Before sending to MoveIt Servo (which operates in plate_tcp frame), the
commands are rotated by the plate yaw angle extracted from TF.

World-frame convention:
  omega_x_world =  omega_beta_cmd   (tilt around world X → marble moves in -Y)
  omega_y_world = -omega_alpha_cmd  (tilt around world Y → marble moves in +X)

Then rotate into plate_tcp frame:
  angular.x_plate =  omega_x_world * cos(yaw) + omega_y_world * sin(yaw)
  angular.y_plate = -omega_x_world * sin(yaw) + omega_y_world * cos(yaw)
"""

import math
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Empty
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


# ── Tunable parameters ────────────────────────────────────────────────────────
CONTROL_HZ = 30.0
MAX_RATE   = np.deg2rad(45.0)     # max angular rate command to servo (rad/s)

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

        # State: [x, vx, y, vy, alpha, omega_alpha, beta, omega_beta]
        self._state = np.zeros(8)

        # Internal PT1 estimate of actual omega (updated every control tick)
        self._omega_alpha_est = 0.0
        self._omega_beta_est  = 0.0

        # Last command sent (for PT1 propagation in control loop)
        self._u_prev = np.zeros(2)

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
        self._fell_off_pub = self.create_publisher(
            Empty, '/marble/fell_off', _LATCHED)
        self._landed_pub = self.create_publisher(
            Empty, '/marble/landed', _LATCHED)

        self._odom_sub = self.create_subscription(
            Odometry, '/marble/odom', self._odom_cb, 10)

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

        # Marble position/velocity relative to plate centre (world frame)
        self._state[0] = msg.pose.pose.position.x - plate_x
        self._state[1] = msg.twist.twist.linear.x
        self._state[2] = msg.pose.pose.position.y - plate_y
        self._state[3] = msg.twist.twist.linear.y

        # Plate roll/pitch from TF quaternion
        # alpha = pitch (Y-axis rotation) → controls X ball motion  (dxd/dt = -C*alpha)
        # beta  = roll  (X-axis rotation) → controls Y ball motion  (dyd/dt = -C*beta)
        q = tf.transform.rotation
        roll, pitch, yaw = _quat_to_rpy(q.x, q.y, q.z, q.w)
        alpha = pitch
        beta  = roll

        # Fill state — omega comes from internal PT1 estimate (updated in control_cb)
        self._state[4] = alpha
        self._state[5] = self._omega_alpha_est
        self._state[6] = beta
        self._state[7] = self._omega_beta_est

        if self._landed:
            self.get_logger().info(
                f'err x:{self._state[0]:+.4f} y:{self._state[2]:+.4f}  '
                f'vx:{self._state[1]:+.4f} vy:{self._state[3]:+.4f}  '
                f'α:{np.rad2deg(alpha):+.2f}° β:{np.rad2deg(beta):+.2f}°  '
                f'ωα:{np.rad2deg(self._omega_alpha_est):+.2f}°/s '
                f'ωβ:{np.rad2deg(self._omega_beta_est):+.2f}°/s',
                throttle_duration_sec=0.5)

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
                self._omega_alpha_est = 0.0
                self._omega_beta_est  = 0.0
                self._u_prev[:]       = 0.0

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

    # ── Control loop ──────────────────────────────────────────────────────────

    def _control_cb(self):
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'plate_tcp'

        # Always propagate PT1 estimate (even when not active)
        self._omega_alpha_est += (self._u_prev[0] - self._omega_alpha_est) * self._dt / T_ROBOT
        self._omega_beta_est  += (self._u_prev[1] - self._omega_beta_est)  * self._dt / T_ROBOT

        if not self._landed:
            self._u_prev[:] = 0.0
            if not self._homing:
                self._twist_pub.publish(msg)   # zero twist keeps Servo happy
            return

        # LQR: u = -K @ x  →  [omega_alpha_cmd, omega_beta_cmd] in world frame
        u = -self._K @ self._state
        omega_alpha_cmd = float(np.clip(u[0], -MAX_RATE, MAX_RATE))
        omega_beta_cmd  = -float(np.clip(u[1], -MAX_RATE, MAX_RATE))

        # Update PT1 state with the new command
        self._u_prev[0] = omega_alpha_cmd
        self._u_prev[1] = omega_beta_cmd

        # ── World-frame twist (base_link) — servo transforms to plate_tcp frame ─
        # Rotation around world X (angular.x) → tilts plate so +Y side goes up → marble in -Y → controls beta
        # Rotation around world Y (angular.y) → tilts plate so +X side goes up → marble in -X → controls alpha
        # No manual yaw rotation: servo handles base_link → plate_tcp transform via TF
        msg.header.frame_id = 'base_link'
        msg.twist.angular.x = omega_beta_cmd    # world X rotation → beta → Y dynamics
        msg.twist.angular.y = omega_alpha_cmd   # world Y rotation → alpha → X dynamics

        self.get_logger().info(
            f'u: ωα={np.rad2deg(omega_alpha_cmd):+.1f}°/s '
            f'ωβ={np.rad2deg(omega_beta_cmd):+.1f}°/s  '
            f'angular x={np.rad2deg(msg.twist.angular.x):+.1f} y={np.rad2deg(msg.twist.angular.y):+.1f}°/s',
            throttle_duration_sec=0.5)

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
