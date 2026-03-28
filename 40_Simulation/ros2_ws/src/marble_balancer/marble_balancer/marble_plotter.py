#!/usr/bin/env python3
"""
marble_plotter.py — Data recorder + visualizer for the marble balancer.

RECORD mode  (live ROS 2 node — records to CSV, plots on Ctrl-C):
    ros2 run marble_balancer marble_plotter
    ros2 run marble_balancer marble_plotter --ros-args -p output:=/tmp/run.csv

PLOT-ONLY mode  (no ROS needed, works offline):
    python3 marble_plotter.py --plot /tmp/run.csv
    ros2 run marble_balancer marble_plotter --ros-args -p plot_file:=/tmp/run.csv

Three figures are produced:
  1. Bird's-eye XY trajectory of the marble on the plate  (colour = time)
  2. Commanded vs actual plate angular velocity (ω_alpha / ω_beta)
  3. TCP (plate_tcp) XYZ position over time
"""

import sys
import csv
import math
import os
import shutil
import subprocess
from pathlib import Path
from datetime import datetime

import numpy as np
import matplotlib
matplotlib.use('Agg')   # non-interactive: always works, no display/Tk needed
                        # figures are saved as PNG and opened with xdg-open
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# ── Plate dimensions (half-width, metres) — adjust if your plate differs ──────
PLATE_HALF_SIZE = 0.35   # 30 × 30 cm plate assumed

# Default log directory
LOG_DIR = Path.home() / 'marble_logs'

# CSV columns
FIELDNAMES = [
    'time',
    'marble_x_rel', 'marble_y_rel', 'marble_z_abs',
    'desired_x', 'desired_y',        # setpoint from lissajous / square node
    'tcp_x', 'tcp_y', 'tcp_z',
    'plate_alpha_deg',                # pitch (Y-axis rotation → controls X)
    'plate_beta_deg',                 # roll  (X-axis rotation → controls Y)
    'cmd_ang_x_deg',                  # angular.x sent to Servo (→ ω_beta cmd)
    'cmd_ang_y_deg',                  # angular.y sent to Servo (→ ω_alpha cmd)
    'actual_omega_alpha_deg',         # Jacobian ω_alpha from /marble/plate_omega
    'actual_omega_beta_deg',          # Jacobian ω_beta  from /marble/plate_omega
]


# ── Helpers ───────────────────────────────────────────────────────────────────

def _quat_to_rpy(qx, qy, qz, qw):
    roll  = math.atan2(2.0 * (qw * qx + qy * qz),
                       1.0 - 2.0 * (qx * qx + qy * qy))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx))))
    yaw   = math.atan2(2.0 * (qw * qz + qx * qy),
                       1.0 - 2.0 * (qy * qy + qz * qz))
    return roll, pitch, yaw


# ── PLOT ──────────────────────────────────────────────────────────────────────

def plot_from_csv(csv_path: str):
    rows = []
    with open(csv_path, newline='') as f:
        for row in csv.DictReader(f):
            rows.append({k: float(v) for k, v in row.items()})

    if not rows:
        print('CSV is empty — nothing to plot.')
        return

    t  = np.array([r['time'] for r in rows]);  t -= t[0]
    mx = np.array([r['marble_x_rel'] for r in rows])
    my = np.array([r['marble_y_rel'] for r in rows])

    tcp_x = np.array([r['tcp_x'] for r in rows])
    tcp_y = np.array([r['tcp_y'] for r in rows])
    tcp_z = np.array([r['tcp_z'] for r in rows])

    alpha_deg = np.array([r['plate_alpha_deg'] for r in rows])   # pitch
    beta_deg  = np.array([r['plate_beta_deg']  for r in rows])   # roll
    cmd_x_deg = np.array([r['cmd_ang_x_deg']   for r in rows])   # → ω_beta
    cmd_y_deg = np.array([r['cmd_ang_y_deg']   for r in rows])   # → ω_alpha

    # Actual plate angular velocities from Jacobian (recorded from /marble/plate_omega)
    omega_alpha_actual = np.array([r['actual_omega_alpha_deg'] for r in rows])
    omega_beta_actual  = np.array([r['actual_omega_beta_deg']  for r in rows])

    # # ── Figure 1: Bird's-eye view ─────────────────────────────────────────────
    # fig1, ax1 = plt.subplots(figsize=(7, 7))
    # ax1.set_aspect('equal')

    # # Plate boundary
    # s = PLATE_HALF_SIZE
    # ax1.add_patch(plt.Rectangle((-s, -s), 2 * s, 2 * s,
    #                             fill=False, edgecolor='steelblue',
    #                             linewidth=2, linestyle='--', label='Plate boundary'))
    # ax1.axhline(0, color='gray', lw=0.5, ls=':')
    # ax1.axvline(0, color='gray', lw=0.5, ls=':')
    # ax1.plot(0, 0, '+', color='black', ms=12, mew=2, label='Center target')

    # sc = ax1.scatter(mx, my, c=t, cmap='plasma', s=8, zorder=3)
    # ax1.plot(mx[0],  my[0],  'go', ms=10, label='Start', zorder=4)
    # ax1.plot(mx[-1], my[-1], 'rs', ms=10, label='End',   zorder=4)

    # plt.colorbar(sc, ax=ax1, label='Time (s)')
    # ax1.set_xlabel('X on plate (m)')
    # ax1.set_ylabel('Y on plate (m)')
    # ax1.set_title("Marble trajectory — bird's-eye view")
    # ax1.legend(loc='upper right')
    # ax1.set_xlim(-s * 1.5, s * 1.5)
    # ax1.set_ylim(-s * 1.5, s * 1.5)
    # fig1.tight_layout()

    # ── Figure 2: Commanded vs actual ω ──────────────────────────────────────
    MAX_RATE_DEG = math.degrees(np.deg2rad(45))   # show clamp line (update if changed)
    fig2, (ax2a, ax2b) = plt.subplots(2, 1, figsize=(11, 6), sharex=True)

    ax2a.plot(t, cmd_y_deg,          label='ω_alpha commanded (angular.y)',
              color='tab:blue',   lw=1.5)
    ax2a.plot(t, omega_alpha_actual, label='ω_alpha actual (Jacobian)',
              color='tab:orange', lw=1.0, alpha=0.85)
    ax2a.axhline( MAX_RATE_DEG, color='red', lw=0.8, ls='--', label='±clamp')
    ax2a.axhline(-MAX_RATE_DEG, color='red', lw=0.8, ls='--')
    ax2a.axhline(0, color='gray', lw=0.4)
    ax2a.set_ylabel('ω_alpha (°/s)')
    ax2a.set_title('Commanded vs actual plate angular velocity')
    ax2a.legend(fontsize=8)
    ax2a.grid(True, alpha=0.3)

    ax2b.plot(t, cmd_x_deg,         label='ω_beta commanded (angular.x)',
              color='tab:green', lw=1.5)
    ax2b.plot(t, omega_beta_actual, label='ω_beta actual (Jacobian)',
              color='tab:red',   lw=1.0, alpha=0.85)
    ax2b.axhline( MAX_RATE_DEG, color='red', lw=0.8, ls='--', label='±clamp')
    ax2b.axhline(-MAX_RATE_DEG, color='red', lw=0.8, ls='--')
    ax2b.axhline(0, color='gray', lw=0.4)
    ax2b.set_xlabel('Time (s)')
    ax2b.set_ylabel('ω_beta (°/s)')
    ax2b.legend(fontsize=8)
    ax2b.grid(True, alpha=0.3)

    fig2.tight_layout()

    # ── Figure 3: Step response — marble position vs setpoint ────────────────
    desired_x = np.array([r['desired_x'] for r in rows])
    desired_y = np.array([r['desired_y'] for r in rows])

    fig3, (ax3a, ax3b, ax3c, ax3d) = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    fig3.suptitle('LQR Step Response', fontsize=13)

    ax3a.plot(t, mx,        label='x actual',  color='tab:blue',   lw=1.5)
    ax3a.plot(t, desired_x, label='x desired', color='tab:blue',   lw=1.0, ls='--', alpha=0.7)
    ax3a.axhline(0, color='gray', lw=0.4)
    ax3a.set_ylabel('X (m)')
    ax3a.legend(fontsize=8, loc='upper right')
    ax3a.grid(True, alpha=0.3)

    ax3b.plot(t, my,        label='y actual',  color='tab:orange', lw=1.5)
    ax3b.plot(t, desired_y, label='y desired', color='tab:orange', lw=1.0, ls='--', alpha=0.7)
    ax3b.axhline(0, color='gray', lw=0.4)
    ax3b.set_ylabel('Y (m)')
    ax3b.legend(fontsize=8, loc='upper right')
    ax3b.grid(True, alpha=0.3)

    ax3c.plot(t, alpha_deg, label='α (pitch, controls X)', color='tab:green',  lw=1.2)
    ax3c.plot(t, beta_deg,  label='β (roll,  controls Y)', color='tab:purple', lw=1.2)
    ax3c.axhline(0, color='gray', lw=0.4)
    ax3c.set_ylabel('Plate angle (°)')
    ax3c.legend(fontsize=8, loc='upper right')
    ax3c.grid(True, alpha=0.3)

    err_x = mx - desired_x
    err_y = my - desired_y
    ax3d.plot(t, err_x, label='error X', color='tab:blue',   lw=1.2)
    ax3d.plot(t, err_y, label='error Y', color='tab:orange', lw=1.2)
    ax3d.axhline(0, color='gray', lw=0.4)
    ax3d.set_ylabel('Error (m)')
    ax3d.set_xlabel('Time (s)')
    ax3d.legend(fontsize=8, loc='upper right')
    ax3d.grid(True, alpha=0.3)

    fig3.tight_layout()

    # ── Console summary ───────────────────────────────────────────────────────
    sat_alpha = np.sum(np.abs(cmd_y_deg) >= MAX_RATE_DEG * 0.99)
    sat_beta  = np.sum(np.abs(cmd_x_deg) >= MAX_RATE_DEG * 0.99)
    print(f'\n=== Run summary  ({len(rows)} samples, {t[-1]:.1f} s) ===')
    print(f'Marble range — X: [{mx.min():.3f}, {mx.max():.3f}] m  '
          f'Y: [{my.min():.3f}, {my.max():.3f}] m')
    print(f'Max |α|: {np.abs(alpha_deg).max():.1f}°   '
          f'Max |β|: {np.abs(beta_deg).max():.1f}°')
    print(f'Rate saturation events — α: {sat_alpha}  β: {sat_beta}')
    rms_x = np.sqrt(np.mean(err_x**2))
    rms_y = np.sqrt(np.mean(err_y**2))
    print(f'RMS error — X: {rms_x*100:.1f} cm  Y: {rms_y*100:.1f} cm')

    # Save PNGs and open with system image viewer (xdg-open)
    base = Path(csv_path).with_suffix('')
    paths = [base.parent / (base.name + '_omega.png'),
             base.parent / (base.name + '_step.png')]
    fig2.savefig(paths[0], dpi=120)
    fig3.savefig(paths[1], dpi=120)
    plt.close('all')
    for p in paths:
        print(f'Saved: {p}')
        subprocess.Popen(['xdg-open', str(p)],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                         start_new_session=True)


# ── RECORD (ROS 2 node) ───────────────────────────────────────────────────────

def record_node(default_output: Path):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TwistStamped, Point
    from std_msgs.msg import Empty
    from std_srvs.srv import Trigger
    import tf2_ros

    _LATCHED = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
    )

    # Prefer the installed binary so Python path is set up correctly.
    # Fall back to calling the source file directly with the current interpreter.
    _plot_bin = shutil.which('marble_plotter') or str(Path(__file__).resolve())
    _use_bin  = shutil.which('marble_plotter') is not None

    class PlotterNode(Node):
        def __init__(self):
            super().__init__('marble_plotter')

            self.declare_parameter('output',    str(default_output))
            self.declare_parameter('plot_file', '')

            # Plot-only mode via ROS arg
            plot_file = self.get_parameter('plot_file').get_parameter_value().string_value
            if plot_file:
                self._plot_only = plot_file
                return

            self._plot_only = None
            self._open_new_csv(
                Path(self.get_parameter('output').get_parameter_value().string_value))

            self._recording_active = False   # only True while marble is on the plate
            self._cmd_ang_x_deg        = 0.0
            self._cmd_ang_y_deg        = 0.0
            self._actual_omega_alpha   = 0.0
            self._actual_omega_beta    = 0.0
            self._desired_x            = 0.0
            self._desired_y            = 0.0

            self._tf_buf = tf2_ros.Buffer()
            self._tf_lis = tf2_ros.TransformListener(self._tf_buf, self)

            self.create_subscription(
                Odometry, '/marble/odom', self._odom_cb, 10)
            self.create_subscription(
                TwistStamped, '/servo_node/delta_twist_cmds', self._cmd_cb, 10)
            self.create_subscription(
                TwistStamped, '/marble/plate_omega', self._omega_cb, 10)
            self.create_subscription(
                Point, '/marble/desired_pos', self._desired_cb, 10)
            self.create_subscription(
                Empty, '/marble/landed',   self._landed_cb,   _LATCHED)
            self.create_subscription(
                Empty, '/marble/fell_off', self._fell_off_cb, _LATCHED)

            self.create_service(Trigger, '/marble/plot_now', self._plot_now_cb)

        def _open_new_csv(self, path: Path):
            path.parent.mkdir(parents=True, exist_ok=True)
            self._csv_path  = path
            self._file      = path.open('w', newline='')
            self._writer    = csv.DictWriter(self._file, fieldnames=FIELDNAMES)
            self._writer.writeheader()
            self._row_count = 0
            self.get_logger().info(f'Recording to {path}')

        def _landed_cb(self, _msg):
            """Marble confirmed on plate — start recording."""
            self._recording_active = True
            self.get_logger().info('Marble landed — recording started.')

        def _fell_off_cb(self, _msg):
            """Marble fell off: stop recording, save CSV, spawn plot window, reset for next marble."""
            self._recording_active = False
            if self._row_count == 0:
                self.get_logger().info('Marble fell off — no data recorded yet.')
                return
            saved = self._csv_path
            self._file.flush()
            self._file.close()
            self.get_logger().info(
                f'Marble fell off — saved {self._row_count} rows → {saved}')

            # Spawn plot in a detached session so it survives ros2 launch shutdown.
            cmd = ([_plot_bin, '--plot', str(saved)] if _use_bin
                   else [sys.executable, _plot_bin, '--plot', str(saved)])
            log_path = saved.with_suffix('.plot.log')
            with log_path.open('w') as log_f:
                proc = subprocess.Popen(
                    cmd,
                    env=os.environ.copy(),
                    stdout=log_f,
                    stderr=log_f,
                    start_new_session=True,   # detach from launch process group
                )
            self.get_logger().info(
                f'Plot process started (pid {proc.pid}), log → {log_path}')

            # Fresh CSV ready for the next marble
            new_path = LOG_DIR / f'marble_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'
            self._open_new_csv(new_path)

        def _plot_now_cb(self, _req, response):
            """Service handler: save current CSV and open plots without stopping recording."""
            if self._row_count == 0:
                response.success = False
                response.message = 'No data recorded yet.'
                return response
            self._file.flush()
            cmd = ([_plot_bin, '--plot', str(self._csv_path)] if _use_bin
                   else [sys.executable, _plot_bin, '--plot', str(self._csv_path)])
            log_path = self._csv_path.with_suffix('.plot.log')
            with log_path.open('w') as log_f:
                subprocess.Popen(cmd, env=os.environ.copy(),
                                 stdout=log_f, stderr=log_f,
                                 start_new_session=True)
            response.success = True
            response.message = f'Plotting {self._row_count} rows from {self._csv_path}'
            self.get_logger().info(response.message)
            return response

        def _desired_cb(self, msg: Point):
            self._desired_x = msg.x
            self._desired_y = msg.y

        def _cmd_cb(self, msg: TwistStamped):
            self._cmd_ang_x_deg = math.degrees(msg.twist.angular.x)
            self._cmd_ang_y_deg = math.degrees(msg.twist.angular.y)

        def _omega_cb(self, msg: TwistStamped):
            self._actual_omega_alpha = math.degrees(msg.twist.angular.y)  # α̇
            self._actual_omega_beta  = math.degrees(msg.twist.angular.x)  # β̇

        def _odom_cb(self, msg: Odometry):
            if not self._recording_active:
                return
            try:
                tf = self._tf_buf.lookup_transform(
                    'world', 'plate_tcp', rclpy.time.Time())
            except Exception:
                return

            q = tf.transform.rotation
            roll, pitch, _ = _quat_to_rpy(q.x, q.y, q.z, q.w)

            plate_x = tf.transform.translation.x
            plate_y = tf.transform.translation.y
            plate_z = tf.transform.translation.z
            t_sec   = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            self._writer.writerow({
                'time':            t_sec,
                'marble_x_rel':    msg.pose.pose.position.x - plate_x,
                'marble_y_rel':    msg.pose.pose.position.y - plate_y,
                'marble_z_abs':    msg.pose.pose.position.z,
                'desired_x':       self._desired_x,
                'desired_y':       self._desired_y,
                'tcp_x':           plate_x,
                'tcp_y':           plate_y,
                'tcp_z':           plate_z,
                'plate_alpha_deg': math.degrees(pitch),
                'plate_beta_deg':  math.degrees(roll),
                'cmd_ang_x_deg':         self._cmd_ang_x_deg,
                'cmd_ang_y_deg':         self._cmd_ang_y_deg,
                'actual_omega_alpha_deg': self._actual_omega_alpha,
                'actual_omega_beta_deg':  self._actual_omega_beta,
            })
            self._row_count += 1

        def destroy_node(self):
            if hasattr(self, '_file') and not self._file.closed:
                self._file.flush()
                self._file.close()
                self.get_logger().info(
                    f'Saved {self._row_count} rows → {self._csv_path}')
            super().destroy_node()

    rclpy.init()
    node = PlotterNode()

    if node._plot_only:
        node.destroy_node()
        rclpy.shutdown()
        plot_from_csv(node._plot_only)
        return

    last_csv = node._csv_path
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        last_csv = node._csv_path
        node.destroy_node()
        rclpy.shutdown()

    # On Ctrl-C: spawn a detached plot process so it outlives this node
    if last_csv.exists() and last_csv.stat().st_size > 0:
        cmd = ([_plot_bin, '--plot', str(last_csv)] if _use_bin
               else [sys.executable, _plot_bin, '--plot', str(last_csv)])
        subprocess.Popen(cmd, env=os.environ.copy(), start_new_session=True)


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    # Standalone --plot mode (no ROS needed)
    if '--plot' in sys.argv:
        idx = sys.argv.index('--plot')
        plot_from_csv(sys.argv[idx + 1])
        return

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    default_out = LOG_DIR / f'marble_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'
    record_node(default_out)


if __name__ == '__main__':
    main()
