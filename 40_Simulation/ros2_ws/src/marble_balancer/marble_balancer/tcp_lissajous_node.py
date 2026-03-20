"""
tcp_lissajous_node.py
---------------------
Generates TCP linear velocity commands for a Lissajous trajectory in the XY plane
at constant Z, plus feedforward tilt angles to pre-compensate for the pseudo-forces
the TCP acceleration induces on the marble.

Trajectory equations (same form as marble_lissajous_node):
  x(t) = amp_x * sin(fa * omega0 * t + delta)
  y(t) = amp_y * sin(fb * omega0 * t)

Published velocity = dx/dt, dy/dt.
Feedforward tilt:
  alpha_ff = -ax_tcp / g   (compensates X pseudo-force: dvx/dt = -C*alpha - (5/7)*ax_tcp)
  beta_ff  = -ay_tcp / g

Publishes:
  /tcp/lissajous_vel      (geometry_msgs/TwistStamped) — linear.x/y in base_link frame
  /tcp/lissajous_ff_tilt  (geometry_msgs/Vector3)      — x=alpha_ff, y=beta_ff (rad)

Activates when /marble/landed received; resets and deactivates on /marble/fell_off.
Enabled via launch argument tcp_lissajous:=true.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TwistStamped, Vector3
from std_msgs.msg import Empty

_LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)

G = 9.81  # m/s²


class TcpLissajousNode(Node):

    def __init__(self):
        super().__init__('tcp_lissajous_node')

        self.declare_parameter('amplitude_x',  0.04)   # m  (~4 cm default)
        self.declare_parameter('amplitude_y',  0.04)   # m
        self.declare_parameter('period',       20.0)   # s  (full base cycle)
        self.declare_parameter('fa',           1)      # frequency ratio for X
        self.declare_parameter('fb',           2)      # frequency ratio for Y
        self.declare_parameter('delta',        math.pi / 2.0)  # phase offset (rad)
        self.declare_parameter('ff_gain',      1.0)    # feedforward scale (0 = off, -1 = flip sign)
        self.declare_parameter('publish_rate', 30.0)   # Hz

        self._amp_x  = self.get_parameter('amplitude_x').value
        self._amp_y  = self.get_parameter('amplitude_y').value
        period       = self.get_parameter('period').value
        self._fa     = self.get_parameter('fa').value
        self._fb     = self.get_parameter('fb').value
        self._delta  = self.get_parameter('delta').value
        self._ff_gain = self.get_parameter('ff_gain').value
        rate         = self.get_parameter('publish_rate').value

        self._omega0 = 2.0 * math.pi / period
        self._dt     = 1.0 / rate
        self._t      = 0.0
        self._active = False

        self._pub_vel = self.create_publisher(TwistStamped, '/tcp/lissajous_vel',     10)
        self._pub_ff  = self.create_publisher(Vector3,      '/tcp/lissajous_ff_tilt', 10)

        self.create_subscription(Empty, '/marble/landed',   self._on_landed,   _LATCHED)
        self.create_subscription(Empty, '/marble/fell_off', self._on_fell_off, _LATCHED)

        self.create_timer(self._dt, self._timer_cb)

        omega_x = self._fa * self._omega0
        omega_y = self._fb * self._omega0
        self.get_logger().info(
            f'TCP Lissajous ready — period={period:.1f}s, '
            f'amp=({self._amp_x * 100:.1f}, {self._amp_y * 100:.1f}) cm, '
            f'fa={self._fa} fb={self._fb} ff_gain={self._ff_gain:.2f} '
            f'max_vel=({self._amp_x * omega_x * 100:.1f}, {self._amp_y * omega_y * 100:.1f}) cm/s '
            '— waiting for marble to land...'
        )

    def _on_landed(self, _):
        self._t = 0.0
        self._active = True
        self.get_logger().info('TCP Lissajous trajectory activated.')

    def _on_fell_off(self, _):
        self._active = False
        self._t = 0.0
        self.get_logger().info('TCP Lissajous deactivated — marble fell off.')

    def _timer_cb(self):
        now = self.get_clock().now().to_msg()

        if not self._active:
            # Publish zeros so marble_servo_controller doesn't accumulate stale data
            zero_vel = TwistStamped()
            zero_vel.header.stamp    = now
            zero_vel.header.frame_id = 'base_link'
            self._pub_vel.publish(zero_vel)
            self._pub_ff.publish(Vector3())
            return

        omega_x = self._fa  * self._omega0
        omega_y = self._fb  * self._omega0

        # Position
        x = self._amp_x * math.sin(omega_x * self._t + self._delta)
        y = self._amp_y * math.sin(omega_y * self._t)

        # Velocity (d/dt of position)
        vx = self._amp_x * omega_x * math.cos(omega_x * self._t + self._delta)
        vy = self._amp_y * omega_y * math.cos(omega_y * self._t)

        # Acceleration: a = -omega^2 * x  (exact for sinusoidal trajectory)
        ax = -(omega_x ** 2) * x
        ay = -(omega_y ** 2) * y

        # Feedforward tilt: alpha_ff = -ax/g, beta_ff = -ay/g
        # When TCP accelerates in +X, marble pseudo-force is in -X.
        # Tilting plate to negative alpha (dvx/dt = -C*alpha) corrects this.
        alpha_ff = -ax / G * self._ff_gain
        beta_ff  = -ay / G * self._ff_gain

        vel_msg = TwistStamped()
        vel_msg.header.stamp    = now
        vel_msg.header.frame_id = 'base_link'
        vel_msg.twist.linear.x  = vx
        vel_msg.twist.linear.y  = vy

        ff_msg   = Vector3()
        ff_msg.x = alpha_ff
        ff_msg.y = beta_ff

        self._pub_vel.publish(vel_msg)
        self._pub_ff.publish(ff_msg)

        self._t += self._dt


def main(args=None):
    rclpy.init(args=args)
    node = TcpLissajousNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
