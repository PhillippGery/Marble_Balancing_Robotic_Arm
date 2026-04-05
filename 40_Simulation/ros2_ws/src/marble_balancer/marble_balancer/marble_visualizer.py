"""
marble_visualizer.py
--------------------
Real-time 2D matplotlib window showing the marble's position on the plate.

Visual elements:
  - Octagon plate boundary (diameter controlled by plate_diameter parameter)
  - Thin crosshair at plate centre (0, 0)
  - Fading trail of the last N marble positions
  - Red filled circle for current marble position (true radius: 0.015 m)
  - Green X marker for desired setpoint (when Lissajous node is active)
  - Title showing ACTIVE / WAITING state

Plate diameter: 0.26 m  (circumscribed circle, matches ur5e_marble_balancer.urdf.xacro)
Ball radius:    0.015 m  (urdf/marble.sdf)

Position convention (matches marble_servo_controller.py):
  mx = -(marble_world_x - plate_x)
  my =   marble_world_y - plate_y
"""

import collections
import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # use TkAgg backend for a separate window; fall back handled below
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.collections import LineCollection

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import Empty
import tf2_ros

_LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)

BALL_RADIUS = 0.015   # m


class MarbleVisualizer(Node):

    def __init__(self):
        super().__init__('marble_visualizer')

        self.declare_parameter('trail_length',   200)
        self.declare_parameter('update_rate',    20.0)
        self.declare_parameter('plate_diameter', 0.26)  # m — must match URDF xacro

        trail_length        = self.get_parameter('trail_length').value
        self._update_period = 1.0 / self.get_parameter('update_rate').value
        plate_diameter      = self.get_parameter('plate_diameter').value

        self._plate_radius  = plate_diameter / 2.0
        self._axis_limit    = self._plate_radius * 1.12   # slight margin

        # ── TF ────────────────────────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Data ──────────────────────────────────────────────────────────────
        self._trail   = collections.deque(maxlen=trail_length)  # [(mx, my), ...]
        self._current = None      # (mx, my) or None
        self._desired = None      # (dx, dy) or None — from Lissajous node
        self._active  = False     # True after marble lands
        self._dirty   = False     # True when new data arrived since last draw
        self._last_draw_time = 0.0

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(Odometry, '/marble/odom',        self._odom_cb,    10)
        self.create_subscription(Point,    '/marble/desired_pos', self._desired_cb, 10)
        self.create_subscription(Empty,    '/marble/landed',      self._landed_cb,  _LATCHED)
        self.create_subscription(Empty,    '/marble/fell_off',    self._fell_cb,    _LATCHED)

        # ── Matplotlib setup ──────────────────────────────────────────────────
        plt.ion()
        self._fig, self._ax = plt.subplots(figsize=(6, 6))
        self._setup_static_elements()
        self._fig.canvas.draw()
        plt.pause(0.001)

        self.get_logger().info(
            f'Marble visualizer ready — trail={trail_length} pts, '
            f'update={1.0/self._update_period:.0f} Hz')

    # ── Static background elements (drawn once) ────────────────────────────

    def _setup_static_elements(self):
        ax   = self._ax
        R    = self._plate_radius
        lim  = self._axis_limit

        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.set_aspect('equal')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_facecolor('#1a1a2e')
        self._fig.patch.set_facecolor('#1a1a2e')
        ax.tick_params(colors='white')
        ax.xaxis.label.set_color('white')
        ax.yaxis.label.set_color('white')
        for spine in ax.spines.values():
            spine.set_edgecolor('#444466')

        # Octagon plate boundary — vertices at (k+0.5)*45° matching the STL mesh
        oct_angles = np.array([(k + 0.5) * np.pi / 4 for k in range(9)])  # 9th closes the polygon
        oct_x = R * np.cos(oct_angles)
        oct_y = R * np.sin(oct_angles)
        ax.plot(oct_x, oct_y, color='#8888cc', linewidth=2, zorder=1)

        # Shaded octagon fill (subtle, so the trail is readable)
        oct_patch = patches.Polygon(
            np.column_stack([oct_x[:-1], oct_y[:-1]]),
            closed=True, facecolor='#22224a', edgecolor='none', zorder=0)
        ax.add_patch(oct_patch)

        # Centre crosshair
        ax.axhline(0, color='#555577', linewidth=0.8, linestyle='--', zorder=1)
        ax.axvline(0, color='#555577', linewidth=0.8, linestyle='--', zorder=1)

        # Grid ticks rounded to nearest 5 cm
        tick_step = 0.05
        n_ticks   = int(lim / tick_step)
        ticks     = np.arange(-n_ticks * tick_step, n_ticks * tick_step + 1e-9, tick_step)
        ticks     = np.round(ticks, 3)
        ax.set_xticks(ticks)
        ax.set_yticks(ticks)
        ax.set_xticklabels([f'{t:.2f}' for t in ticks], fontsize=7, color='#aaaacc')
        ax.set_yticklabels([f'{t:.2f}' for t in ticks], fontsize=7, color='#aaaacc')
        ax.grid(True, color='#333355', linewidth=0.4, zorder=0)

        # Dynamic elements (initialised empty, updated in update_plot)
        self._trail_line, = ax.plot([], [], color='#44aaff', linewidth=1.2,
                                    alpha=0.6, zorder=2)
        self._ball_circle = plt.Circle((0, 0), BALL_RADIUS,
                                       color='#ff4444', zorder=4)
        ax.add_patch(self._ball_circle)
        self._ball_circle.set_visible(False)

        self._setpoint_marker, = ax.plot([], [], 'x', color='#44ff88',
                                          markersize=10, markeredgewidth=2, zorder=3)

        self._title = ax.set_title('Marble Visualizer — WAITING FOR MARBLE',
                                   color='white', fontsize=11, pad=10)
        self._fig.tight_layout()

    # ── ROS callbacks ──────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        for frame in ('plate_tcp', 'marble_plate', 'tool0'):
            try:
                tf = self._tf_buffer.lookup_transform('world', frame, rclpy.time.Time())
                plate_x = tf.transform.translation.x
                plate_y = tf.transform.translation.y
                mx = -(msg.pose.pose.position.x - plate_x)
                my =   msg.pose.pose.position.y - plate_y
                self._current = (mx, my)
                self._trail.append((mx, my))
                self._dirty = True
                return
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                pass

    def _desired_cb(self, msg: Point):
        self._desired = (msg.x, msg.y)
        self._dirty = True

    def _landed_cb(self, _):
        self._active = True
        self._dirty = True

    def _fell_cb(self, _):
        self._active = False
        self._trail.clear()
        self._current = None
        self._dirty = True

    # ── Plot update ────────────────────────────────────────────────────────

    def update_plot(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if not self._dirty or (now - self._last_draw_time) < self._update_period:
            return
        self._last_draw_time = now
        self._dirty = False

        # Trail
        if len(self._trail) >= 2:
            pts = list(self._trail)
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            self._trail_line.set_data(xs, ys)
        else:
            self._trail_line.set_data([], [])

        # Ball
        if self._current is not None:
            self._ball_circle.set_center(self._current)
            self._ball_circle.set_visible(True)
        else:
            self._ball_circle.set_visible(False)

        # Desired setpoint
        if self._desired is not None:
            self._setpoint_marker.set_data([self._desired[0]], [self._desired[1]])
        else:
            self._setpoint_marker.set_data([], [])

        # Title
        state_str = 'ACTIVE' if self._active else 'WAITING FOR MARBLE'
        pos_str = ''
        if self._current is not None:
            pos_str = f'  |  pos ({self._current[0]*100:+.1f}, {self._current[1]*100:+.1f}) cm'
        self._title.set_text(f'Marble Visualizer — {state_str}{pos_str}')

        self._fig.canvas.draw_idle()


def main(args=None):
    # Try TkAgg; if unavailable fall back to Qt5Agg then default
    for backend in ('TkAgg', 'Qt5Agg', 'Agg'):
        try:
            matplotlib.use(backend)
            break
        except Exception:
            continue

    rclpy.init(args=args)
    node = MarbleVisualizer()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            node.update_plot()
            plt.pause(0.05)   # 20 Hz render ceiling; yields to GUI event loop
    except KeyboardInterrupt:
        pass
    finally:
        plt.close('all')
        node.destroy_node()
        rclpy.shutdown()
