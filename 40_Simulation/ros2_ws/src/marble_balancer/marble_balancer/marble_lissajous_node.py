"""
marble_lissajous_node.py

Publishes a Lissajous curve as the desired marble position on /marble/desired_pos.

When the marble is not on the plate, publishes (0, 0) so the controller
stabilises at the centre.  When the marble lands, starts the Lissajous
curve from t=0 and publishes at PUBLISH_HZ.

Lissajous equations:
  x_d(t) = amplitude_x * sin(freq_a * w0 * t + phase_delta)
  y_d(t) = amplitude_y * sin(freq_b * w0 * t)
  w0     = 2*pi * base_freq_hz

With freq_a=1, freq_b=2, phase_delta=pi/2 and base_freq_hz=0.1 Hz
one period of the figure-eight is 10 seconds — a safe default for a
plate of 0.70×0.70 m.

ROS2 parameters (all settable at launch or from command line):
  amplitude_x  (float, m)    default 0.12
  amplitude_y  (float, m)    default 0.12
  freq_a       (int)         default 1
  freq_b       (int)         default 2
  base_freq_hz (float, Hz)   default 0.10
  phase_delta  (float, rad)  default pi/2

Launch with lissajous:=true:
  ros2 launch marble_balancer servo_balancer.launch.py lissajous:=true

Run standalone with custom parameters:
  ros2 run marble_balancer marble_lissajous \\
    --ros-args -p amplitude_x:=0.10 -p freq_a:=3 -p freq_b:=2
"""

import math
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


class MarbleLissajousNode(Node):

    def __init__(self):
        super().__init__('marble_lissajous')

        self.declare_parameter('amplitude_x',  0.12)
        self.declare_parameter('amplitude_y',  0.12)
        self.declare_parameter('freq_a',       1)
        self.declare_parameter('freq_b',       2)
        self.declare_parameter('base_freq_hz', 0.20)
        self.declare_parameter('phase_delta',  math.pi / 2.0)

        self._amp_x  = float(self.get_parameter('amplitude_x').value)
        self._amp_y  = float(self.get_parameter('amplitude_y').value)
        self._fa     = int(self.get_parameter('freq_a').value)
        self._fb     = int(self.get_parameter('freq_b').value)
        self._f0     = float(self.get_parameter('base_freq_hz').value)
        self._delta  = float(self.get_parameter('phase_delta').value)
        self._w0     = 2.0 * math.pi * self._f0

        self._active  = False
        self._t_start = None   # rclpy.time.Time of last landing

        # TRANSIENT_LOCAL: catch the latched /marble/landed even if we start late
        self._landed_sub = self.create_subscription(
            Empty, '/marble/landed', self._on_landed, _LATCHED)
        self._fell_sub = self.create_subscription(
            Empty, '/marble/fell_off', self._on_fell, _LATCHED)

        self._pub = self.create_publisher(Point, '/marble/desired_pos', 10)
        self._timer = self.create_timer(1.0 / PUBLISH_HZ, self._publish_cb)

        self.get_logger().info(
            f'Lissajous node ready — '
            f'A=[{self._amp_x:.3f}, {self._amp_y:.3f}] m  '
            f'ratio [{self._fa}:{self._fb}]  '
            f'f0={self._f0:.3f} Hz  '
            f'δ={math.degrees(self._delta):.1f}°  '
            f'Period={1.0/self._f0:.1f} s')

    def _on_landed(self, _msg):
        self._t_start = self.get_clock().now()
        self._active  = True
        self.get_logger().info('Lissajous activated — curve starts from t=0.')

    def _on_fell(self, _msg):
        self._active  = False
        self._t_start = None
        self.get_logger().info('Lissajous deactivated — publishing (0, 0).')

    def _publish_cb(self):
        msg = Point()  # z stays 0.0; only x and y are used by the controller
        if self._active and self._t_start is not None:
            t = (self.get_clock().now() - self._t_start).nanoseconds * 1e-9
            msg.x = self._amp_x * math.sin(self._fa * self._w0 * t + self._delta)
            msg.y = self._amp_y * math.sin(self._fb * self._w0 * t)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MarbleLissajousNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
