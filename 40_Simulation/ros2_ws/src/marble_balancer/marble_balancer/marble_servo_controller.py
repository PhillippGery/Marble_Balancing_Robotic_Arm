"""
marble_servo_controller.py
--------------------------
Real-time LQR marble balancing controller.

Subscribes to /marble/odom and uses TF (world → marble_plate) to detect
when the marble has landed on the plate, then activates the LQR loop.

Control pipeline:
  /marble/odom  ──►  landing detection  ──►  LQR  ──►  TwistStamped  ──►  MoveIt Servo

Sign convention (matches lqr_math.py A matrix):
  alpha = plate roll  (rotation around world X): positive α → marble accelerates in -X
  beta  = plate pitch (rotation around world Y): positive β → marble accelerates in -Y
  (flip signs in the publish block if observed motion is opposite to expected)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger
import numpy as np
import tf2_ros

from marble_balancer.lqr_math import compute_dlqr, DEFAULT_Q, DEFAULT_R


# ── Tunable parameters ─────────────────────────────────────────────────────────
CONTROL_HZ = 30.0                 # must match servo publish_period (~30 Hz)
MAX_ANGLE  = np.deg2rad(15.0)     # plate tilt saturation (rad)
MAX_RATE   = np.deg2rad(20.0)     # max angular rate command to servo (rad/s)

# Landing detection thresholds
LAND_Z_MARGIN  = 0.025            # marble centre must be within this of plate top (m)
LAND_VZ_MAX    = 0.10             # max vertical speed to be considered "landed" (m/s)
LAND_CONFIRM   = 10               # consecutive odom ticks that must pass the check
# ──────────────────────────────────────────────────────────────────────────────

MARBLE_RADIUS   = 0.015
PLATE_THICKNESS = 0.005


class MarbleServoController(Node):

    def __init__(self):
        super().__init__('marble_servo_controller')

        dt = 1.0 / CONTROL_HZ

        # ── LQR gain ──────────────────────────────────────────────────────────
        self._K, _, _ = compute_dlqr(DEFAULT_Q, DEFAULT_R, dt)
        self.get_logger().info(
            f'LQR K computed:\n{np.array2string(self._K, precision=3)}')

        self._state = np.zeros(8)
        self._dt    = dt

        # ── TF for plate position ─────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._plate_z     = None   # resolved once from TF

        # ── Landing detection ─────────────────────────────────────────────────
        self._marble_z    = None
        self._marble_vz   = 0.0
        self._landed      = False
        self._land_ticks  = 0      # consecutive ticks meeting landing criteria

        # ── Publishers / subscribers ──────────────────────────────────────────
        self._twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)

        self._odom_sub = self.create_subscription(
            Odometry, '/marble/odom', self._odom_cb, 10)

        self._timer = self.create_timer(dt, self._control_cb)

        # ── Start MoveIt Servo (it begins in stopped state) ───────────────────
        self._start_servo()

        self.get_logger().info(
            f'Marble servo controller ready at {CONTROL_HZ:.0f} Hz — '
            'waiting for marble to land…')

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _start_servo(self):
        """Call /servo_node/start_servo so Servo processes our twist commands."""
        client = self.create_client(Trigger, '/servo_node/start_servo')
        self.get_logger().info('Waiting for /servo_node/start_servo …')
        if client.wait_for_service(timeout_sec=10.0):
            future = client.call_async(Trigger.Request())
            # Non-blocking — result logged when it arrives
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
                self.get_logger().warn(
                    f'start_servo returned: {result.message}')
        except Exception as e:
            self.get_logger().error(f'start_servo call failed: {e}')

    def _get_plate_z(self):
        """Look up plate top surface Z from TF (cached after first success)."""
        if self._plate_z is not None:
            return self._plate_z
        for frame in ('marble_plate', 'tool0'):
            try:
                tf = self._tf_buffer.lookup_transform(
                    'world', frame, rclpy.time.Time())
                # plate_z = centre of plate link; top = centre + half thickness
                self._plate_z = tf.transform.translation.z + PLATE_THICKNESS / 2.0
                self.get_logger().info(
                    f'Plate top Z from "{frame}": {self._plate_z:.4f} m')
                return self._plate_z
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                pass
        return None

    # ── Odometry callback ─────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        # Update marble Z and vertical velocity for landing detection
        self._marble_z  = msg.pose.pose.position.z
        self._marble_vz = msg.twist.twist.linear.z

        # XY state for LQR (relative to plate centre)
        plate_z = self._get_plate_z()
        if plate_z is None:
            return

        # Use plate TF X/Y as the reference (cached plate_z already available)
        try:
            tf = self._tf_buffer.lookup_transform(
                'world', 'marble_plate', rclpy.time.Time())
            plate_x = tf.transform.translation.x
            plate_y = tf.transform.translation.y
        except Exception:
            return

        self._state[0] = msg.pose.pose.position.x - plate_x
        self._state[1] = msg.twist.twist.linear.x
        self._state[2] = msg.pose.pose.position.y - plate_y
        self._state[3] = msg.twist.twist.linear.y

        # ── Debug: marble error relative to plate centre ───────────────────
        self.get_logger().info(
            f'Marble error — x: {self._state[0]:+.4f} m  '
            f'y: {self._state[2]:+.4f} m  '
            f'vx: {self._state[1]:+.4f} m/s  '
            f'vy: {self._state[3]:+.4f} m/s',
            throttle_duration_sec=0.5)

        # ── Landing detection ──────────────────────────────────────────────
        if not self._landed:
            marble_on_plate = (
                self._marble_z is not None
                and self._marble_z <= plate_z + MARBLE_RADIUS + LAND_Z_MARGIN
                and abs(self._marble_vz) < LAND_VZ_MAX
            )
            if marble_on_plate:
                self._land_ticks += 1
                if self._land_ticks >= LAND_CONFIRM:
                    self._landed = True
                    self.get_logger().info(
                        f'Marble landed at z={self._marble_z:.4f} m — '
                        'LQR control activated!')
            else:
                self._land_ticks = 0   # reset if marble bounces back up

    # ── Control loop ──────────────────────────────────────────────────────────

    def _control_cb(self):
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        if not self._landed:
            # Hold position with zero twist while marble is still falling
            self._twist_pub.publish(msg)
            return

        # ── LQR ───────────────────────────────────────────────────────────
        u = -self._K @ self._state   # [alpha_ddot, beta_ddot]

        alpha_dot_new = self._state[5] + u[0] * self._dt
        beta_dot_new  = self._state[7] + u[1] * self._dt

        alpha_new = self._state[4] + alpha_dot_new * self._dt
        beta_new  = self._state[6] + beta_dot_new  * self._dt

        # Angle saturation
        if abs(alpha_new) >= MAX_ANGLE:
            alpha_new, alpha_dot_new = np.clip(alpha_new, -MAX_ANGLE, MAX_ANGLE), 0.0
        if abs(beta_new) >= MAX_ANGLE:
            beta_new,  beta_dot_new  = np.clip(beta_new,  -MAX_ANGLE, MAX_ANGLE), 0.0

        # Rate saturation
        alpha_dot_new = np.clip(alpha_dot_new, -MAX_RATE, MAX_RATE)
        beta_dot_new  = np.clip(beta_dot_new,  -MAX_RATE, MAX_RATE)

        self._state[4] = alpha_new
        self._state[5] = alpha_dot_new
        self._state[6] = beta_new
        self._state[7] = beta_dot_new

        msg.twist.angular.x = alpha_dot_new
        msg.twist.angular.y = beta_dot_new

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
