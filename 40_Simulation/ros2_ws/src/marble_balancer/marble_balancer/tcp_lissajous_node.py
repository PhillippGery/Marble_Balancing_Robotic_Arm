"""
tcp_lissajous_node.py
---------------------
Generates TCP linear velocity commands for a Lissajous trajectory in the XY plane
at constant Z.

Trajectory equations:
  x(t) = amp_x * sin(fa * omega0 * t + delta)
  y(t) = amp_y * sin(fb * omega0 * t)

Published velocity = dx/dt, dy/dt  on /tcp/lissajous_vel.

Pseudo-force compensation (TCP acceleration disturbance on marble) is handled
by the RL residual controller — not this node.

Activates when /marble/landed received; resets on /marble/fell_off.
Enabled via launch argument tcp_lissajous:=true.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TwistStamped, Point
from std_msgs.msg import Empty

_LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)


class TcpLissajousNode(Node):

    def __init__(self):
        super().__init__('tcp_lissajous_node')

        self.declare_parameter('amplitude_x',  0.04)
        self.declare_parameter('amplitude_y',  0.04)
        self.declare_parameter('period',       20.0)
        self.declare_parameter('fa',           1)
        self.declare_parameter('fb',           2)
        self.declare_parameter('delta',        math.pi / 2.0)
        self.declare_parameter('publish_rate', 30.0)

        self._amp_x  = self.get_parameter('amplitude_x').value
        self._amp_y  = self.get_parameter('amplitude_y').value
        period       = self.get_parameter('period').value
        self._fa     = self.get_parameter('fa').value
        self._fb     = self.get_parameter('fb').value
        self._delta  = self.get_parameter('delta').value
        rate         = self.get_parameter('publish_rate').value

        self._omega0 = 2.0 * math.pi / period
        self._dt     = 1.0 / rate
        self._t      = 0.0
        self._active = False

        self._pub_vel = self.create_publisher(TwistStamped, '/tcp/lissajous_vel', 10)
        self._pub_pos = self.create_publisher(Point, '/tcp/lissajous_pos', 10)

        self.create_subscription(Empty, '/marble/landed',   self._on_landed,   _LATCHED)
        self.create_subscription(Empty, '/marble/fell_off', self._on_fell_off, _LATCHED)

        self.create_timer(self._dt, self._timer_cb)

        omega_x = self._fa * self._omega0
        omega_y = self._fb * self._omega0
        self.get_logger().info(
            f'TCP Lissajous ready — period={period:.1f}s  '
            f'amp=({self._amp_x*100:.1f}, {self._amp_y*100:.1f}) cm  '
            f'fa={self._fa} fb={self._fb}  '
            f'max_vel=({self._amp_x*omega_x*100:.1f}, {self._amp_y*omega_y*100:.1f}) cm/s '
            '— waiting for marble to land...'
        )

    def _on_landed(self, _):
        self._t = 0.0
        self._active = True
        self.get_logger().info('TCP Lissajous activated.')

    def _on_fell_off(self, _):
        self._active = False
        self._t = 0.0
        self.get_logger().info('TCP Lissajous deactivated.')

    def _timer_cb(self):
        now = self.get_clock().now().to_msg()
        msg = TwistStamped()
        msg.header.stamp    = now
        msg.header.frame_id = 'base_link'

        pos = Point()
        if self._active:
            omega_x = self._fa * self._omega0
            omega_y = self._fb * self._omega0
            msg.twist.linear.x = self._amp_x * omega_x * math.cos(omega_x * self._t + self._delta)
            msg.twist.linear.y = self._amp_y * omega_y * math.cos(omega_y * self._t)
            pos.x = self._amp_x * math.sin(omega_x * self._t + self._delta)
            pos.y = self._amp_y * math.sin(omega_y * self._t)
            self._t += self._dt

        self._pub_vel.publish(msg)
        self._pub_pos.publish(pos)


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
