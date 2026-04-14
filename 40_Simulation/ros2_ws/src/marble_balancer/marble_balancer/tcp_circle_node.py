"""
tcp_circle_node.py
------------------
Generates TCP linear velocity commands for a circular trajectory in the XY plane
at constant Z, plus optional feedforward tilt to pre-compensate the centripetal
pseudo-force the TCP acceleration induces on the marble.

Trajectory:
  x(t) = radius * cos(omega * t + phase)
  y(t) = radius * sin(omega * t + phase)

Velocity (d/dt of position):
  vx(t) = -radius * omega * sin(omega * t + phase)
  vy(t) =  radius * omega * cos(omega * t + phase)

Centripetal acceleration (points toward circle centre):
  ax(t) = -omega^2 * x(t)
  ay(t) = -omega^2 * y(t)

Feedforward tilt (opposes marble pseudo-force from TCP acceleration):
  alpha_ff = -ax / g * ff_gain
  beta_ff  = -ay / g * ff_gain

Publishes:
  /tcp/lissajous_vel      (geometry_msgs/TwistStamped) — linear.x/y in base_link frame
  /tcp/lissajous_ff_tilt  (geometry_msgs/Vector3)      — x=alpha_ff, y=beta_ff (rad)

Activates when /marble/landed received; resets and deactivates on /marble/fell_off.
Enabled via launch argument tcp_circle:=true.
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


class TcpCircleNode(Node):

    def __init__(self):
        super().__init__('tcp_circle_node')

        self.declare_parameter('radius',       0.04)   # m
        self.declare_parameter('period',       20.0)   # s (one full revolution)
        self.declare_parameter('phase',        0.0)    # rad (initial phase offset)
        self.declare_parameter('ff_gain',      0.0)    # feedforward scale (0 = off)
        self.declare_parameter('publish_rate', 30.0)   # Hz

        self._radius   = self.get_parameter('radius').value
        self._period   = self.get_parameter('period').value
        self._phase    = self.get_parameter('phase').value
        self._ff_gain  = self.get_parameter('ff_gain').value
        rate           = self.get_parameter('publish_rate').value

        self._omega  = 2.0 * math.pi / self._period
        self._dt     = 1.0 / rate
        self._t      = 0.0
        self._active = False

        self._pub_vel = self.create_publisher(TwistStamped, '/tcp/lissajous_vel',     10)
        self._pub_ff  = self.create_publisher(Vector3,      '/tcp/lissajous_ff_tilt', 10)

        self.create_subscription(Empty, '/marble/landed',   self._on_landed,   _LATCHED)
        self.create_subscription(Empty, '/marble/fell_off', self._on_fell_off, _LATCHED)

        self.create_timer(self._dt, self._timer_cb)

        max_vel = self._radius * self._omega
        self.get_logger().info(
            f'TCP Circle ready — radius={self._radius * 100:.1f} cm, '
            f'period={self._period:.1f} s, '
            f'max_vel={max_vel * 100:.1f} cm/s, '
            f'ff_gain={self._ff_gain:.2f} '
            '— waiting for marble to land...'
        )

    def _on_landed(self, _):
        self._t = 0.0
        self._active = True
        self.get_logger().info('TCP Circle trajectory activated.')

    def _on_fell_off(self, _):
        self._active = False
        self._t = 0.0
        self.get_logger().info('TCP Circle deactivated — marble fell off.')

    def _timer_cb(self):
        now = self.get_clock().now().to_msg()

        if not self._active:
            zero_vel = TwistStamped()
            zero_vel.header.stamp    = now
            zero_vel.header.frame_id = 'base_link'
            self._pub_vel.publish(zero_vel)
            self._pub_ff.publish(Vector3())
            return

        angle = self._omega * self._t + self._phase

        # Position
        x = self._radius * math.cos(angle)
        y = self._radius * math.sin(angle)

        # Velocity
        vx = -self._radius * self._omega * math.sin(angle)
        vy =  self._radius * self._omega * math.cos(angle)

        # Centripetal acceleration: a = -omega^2 * position
        ax = -(self._omega ** 2) * x
        ay = -(self._omega ** 2) * y

        # Feedforward tilt (0 when ff_gain=0.0)
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
    node = TcpCircleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
