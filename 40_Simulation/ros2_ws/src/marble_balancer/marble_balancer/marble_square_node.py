"""
marble_square_node.py
---------------------
Publishes a cyclic square step-response pattern as the desired marble position
on /marble/desired_pos.

The marble is commanded to visit the 4 corners of a square in order:
  (+half, +half) → (-half, +half) → (-half, -half) → (+half, -half) → repeat

The node is inactive until /marble/landed is received.
On /marble/fell_off it resets to corner 0 and publishes (0, 0) until re-landing.

ROS2 parameters:
  half_side   (float, m)  default 0.10   — half the square side length
                                           corners at ±half_side in x and y
  dwell_time  (float, s)  default 3.0    — seconds to hold each corner before
                                           stepping to the next

Launch:
  ros2 launch marble_balancer servo_balancer.launch.py square:=true
  ros2 launch marble_balancer servo_balancer.launch.py square:=true \\
    --ros-args -p marble_square_node:half_side:=0.08 -p marble_square_node:dwell_time:=2.0

Run standalone:
  ros2 run marble_balancer marble_square \\
    --ros-args -p half_side:=0.10 -p dwell_time:=3.0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Point
from std_msgs.msg import Empty

_LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)

PUBLISH_HZ = 30.0

# Corner order: top-right → top-left → bottom-left → bottom-right
_CORNER_OFFSETS = [(1, 1), (-1, 1), (-1, -1), (1, -1)]


class MarbleSquareNode(Node):

    def __init__(self):
        super().__init__('marble_square')

        self.declare_parameter('half_side',  0.10)
        self.declare_parameter('dwell_time', 3.0)

        self._half  = float(self.get_parameter('half_side').value)
        self._dwell = float(self.get_parameter('dwell_time').value)

        self._active        = False
        self._corner_idx    = 0
        self._corner_start  = None   # rclpy.time.Time when we arrived at current corner

        self._landed_sub = self.create_subscription(
            Empty, '/marble/landed', self._on_landed, _LATCHED)
        self._fell_sub = self.create_subscription(
            Empty, '/marble/fell_off', self._on_fell, _LATCHED)

        self._pub   = self.create_publisher(Point, '/marble/desired_pos', 10)
        self._timer = self.create_timer(1.0 / PUBLISH_HZ, self._publish_cb)

        corners_str = '  '.join(
            f'({self._half * sx:+.3f}, {self._half * sy:+.3f})'
            for sx, sy in _CORNER_OFFSETS)
        self.get_logger().info(
            f'Square step node ready — '
            f'half_side={self._half:.3f} m  dwell={self._dwell:.1f} s\n'
            f'  Corners: {corners_str}')

    def _on_landed(self, _msg):
        self._active       = True
        self._corner_idx   = 0
        self._corner_start = self.get_clock().now()
        sx, sy = _CORNER_OFFSETS[0]
        self.get_logger().info(
            f'Square pattern activated — starting at corner 0 '
            f'({self._half * sx:+.3f}, {self._half * sy:+.3f}) m')

    def _on_fell(self, _msg):
        self._active       = False
        self._corner_idx   = 0
        self._corner_start = None
        self.get_logger().info('Square pattern deactivated — publishing (0, 0).')

    def _publish_cb(self):
        msg = Point()   # z stays 0.0

        if not self._active or self._corner_start is None:
            self._pub.publish(msg)
            return

        # Check if dwell time at current corner has elapsed
        elapsed = (self.get_clock().now() - self._corner_start).nanoseconds * 1e-9
        if elapsed >= self._dwell:
            self._corner_idx   = (self._corner_idx + 1) % len(_CORNER_OFFSETS)
            self._corner_start = self.get_clock().now()
            sx, sy = _CORNER_OFFSETS[self._corner_idx]
            self.get_logger().info(
                f'Step to corner {self._corner_idx} '
                f'({self._half * sx:+.3f}, {self._half * sy:+.3f}) m',
                throttle_duration_sec=0.0)

        sx, sy = _CORNER_OFFSETS[self._corner_idx]
        msg.x = self._half * sx
        msg.y = self._half * sy
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MarbleSquareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
