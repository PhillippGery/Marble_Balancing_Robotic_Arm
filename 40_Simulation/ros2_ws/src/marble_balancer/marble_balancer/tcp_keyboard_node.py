"""
tcp_keyboard_node.py
--------------------
Keyboard control of TCP linear velocity while the marble is balancing.

Holding a key accelerates the TCP; releasing it decelerates back to zero.
Publishes on /tcp/lissajous_vel — the same topic as tcp_lissajous_node.
Run this instead of tcp_lissajous_node, not both simultaneously.

Keys
----
  W / ↑     +X  (TCP moves forward)
  S / ↓     −X  (TCP moves backward)
  A / ←     +Y  (TCP moves left)
  D / →     −Y  (TCP moves right)
  R         +Z  (TCP moves up)
  F         −Z  (TCP moves down)
  SPACE     hard brake (instant zero on all axes)
  Q / ESC   quit node

Requires: pip install pynput
"""

import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Empty

try:
    from pynput import keyboard as _kb
    _HAS_PYNPUT = True
except ImportError:
    _HAS_PYNPUT = False

_LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)


class TcpKeyboardNode(Node):

    def __init__(self):
        super().__init__('tcp_keyboard_node')

        self.declare_parameter('max_vel',      0.30)   # m/s  — maximum TCP speed
        self.declare_parameter('accel',        0.25)   # m/s² — acceleration while key held
        self.declare_parameter('decel',        0.50)   # m/s² — deceleration after key release
        self.declare_parameter('publish_rate', 30.0)

        self._max_vel = self.get_parameter('max_vel').value
        self._accel   = self.get_parameter('accel').value
        self._decel   = self.get_parameter('decel').value
        rate          = self.get_parameter('publish_rate').value

        self._dt      = 1.0 / rate
        self._vel_x   = 0.0
        self._vel_y   = 0.0
        self._vel_z   = 0.0
        self._active  = False

        self._keys    = set()   # currently held keys (string identifiers)
        self._lock    = threading.Lock()

        self._pub = self.create_publisher(TwistStamped, '/tcp/lissajous_vel', 10)

        self.create_subscription(Empty, '/marble/landed',   self._on_landed,   _LATCHED)
        self.create_subscription(Empty, '/marble/fell_off', self._on_fell_off, _LATCHED)

        self.create_timer(self._dt, self._timer_cb)

        if not _HAS_PYNPUT:
            self.get_logger().error(
                'pynput is not installed — run:  pip install pynput\n'
                'TCP keyboard control will NOT work until pynput is available.')
            return

        self._listener = _kb.Listener(
            on_press=self._on_press,
            on_release=self._on_release,
            suppress=False,
        )
        self._listener.daemon = True
        self._listener.start()

        self.get_logger().info(
            f'TCP Keyboard ready — '
            f'max_vel={self._max_vel:.2f} m/s  '
            f'accel={self._accel:.2f}  decel={self._decel:.2f} m/s²\n'
            '  W/↑  +X    S/↓  −X    A/←  +Y    D/→  −Y    R  +Z    F  −Z\n'
            '  SPACE = hard brake    Q / ESC = quit\n'
            '— waiting for marble to land…'
        )

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def _on_landed(self, _):
        self._active = True
        self.get_logger().info('TCP Keyboard activated.')

    def _on_fell_off(self, _):
        self._active = False
        self._vel_x  = 0.0
        self._vel_y  = 0.0
        self._vel_z  = 0.0
        with self._lock:
            self._keys.clear()
        self.get_logger().info('TCP Keyboard deactivated.')

    # ── pynput callbacks (run in listener thread) ─────────────────────────────

    def _on_press(self, key):
        k = self._key_id(key)
        if k in ('q', 'esc'):
            self.get_logger().info('TCP Keyboard: quit requested.')
            rclpy.shutdown()
            return
        with self._lock:
            self._keys.add(k)

    def _on_release(self, key):
        with self._lock:
            self._keys.discard(self._key_id(key))

    @staticmethod
    def _key_id(key) -> str:
        """Normalise a pynput key to a short string identifier."""
        try:
            return key.char.lower()          # regular character key
        except AttributeError:
            return str(key).replace('Key.', '')   # 'up', 'down', 'left', 'right', 'space', 'esc'

    # ── 30 Hz timer ───────────────────────────────────────────────────────────

    def _timer_cb(self):
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        if self._active:
            with self._lock:
                keys = set(self._keys)

            if 'space' in keys:
                # Hard brake on all axes
                self._vel_x = 0.0
                self._vel_y = 0.0
                self._vel_z = 0.0
            else:
                self._vel_x = self._step(
                    self._vel_x,
                    pos='w' in keys or 'up' in keys,
                    neg='s' in keys or 'down' in keys,
                )
                self._vel_y = self._step(
                    self._vel_y,
                    pos='a' in keys or 'left' in keys,
                    neg='d' in keys or 'right' in keys,
                )
                self._vel_z = self._step(
                    self._vel_z,
                    pos='r' in keys,
                    neg='f' in keys,
                )

            msg.twist.linear.x = self._vel_x
            msg.twist.linear.y = self._vel_y
            msg.twist.linear.z = self._vel_z

            self.get_logger().info(
                f'TCP  vx={self._vel_x:+.3f}  vy={self._vel_y:+.3f}  vz={self._vel_z:+.3f} m/s',
                throttle_duration_sec=0.5,
            )

        self._pub.publish(msg)

    def _step(self, vel: float, pos: bool, neg: bool) -> float:
        """Advance one velocity component by one time step."""
        if pos and not neg:
            return min(vel + self._accel * self._dt,  self._max_vel)
        if neg and not pos:
            return max(vel - self._accel * self._dt, -self._max_vel)
        # No key held — decelerate toward zero
        step = self._decel * self._dt
        if abs(vel) <= step:
            return 0.0
        return vel - math.copysign(step, vel)


def main(args=None):
    rclpy.init(args=args)
    node = TcpKeyboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
