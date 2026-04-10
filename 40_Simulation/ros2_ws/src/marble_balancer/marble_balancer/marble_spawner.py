"""
marble_spawner.py
-----------------
Spawns the marble in Gazebo directly above the marble plate centre.

Reads world → plate_tcp (or fallback frames) from TF, adds a drop offset,
calls /spawn_entity.  No hardcoded coordinates.

Robustness fixes:
  1. Waits 0.5 s after delete so Gazebo flushes the old physics body before
     the new entity is inserted — prevents spawn-inside-old-body ejection.
  2. Re-reads TF *after* the delete wait so the spawn Z uses the current
     plate position, not a stale value.
  3. Monitors /marble/odom for up to MAX_VERIFY_S seconds after spawn.
     If the marble's Z drops below the plate surface it fell through —
     the node retries the spawn up to MAX_RETRIES times.
"""

import os
import time
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from nav_msgs.msg import Odometry
import tf2_ros
from ament_index_python.packages import get_package_share_directory


MARBLE_RADIUS    = 0.015   # m
PLATE_THICKNESS  = 0.005   # m
DROP_HEIGHT      = 0.10    # m above plate surface  (was 0.08; slightly higher = safer)
DELETE_SETTLE_S  = 0.6     # seconds to wait after delete before spawning
MAX_VERIFY_S     = 3.0     # seconds to watch odom after spawn
FALL_THROUGH_Z   = -0.05   # if marble drops this far below surface_z → fell through
LAND_Z_MARGIN    = 0.040   # ±4 cm z-window around expected resting z (surface + radius)
LAND_CONFIRM     = 5       # consecutive odom ticks within window → confirmed landed
MAX_RETRIES      = 3

CANDIDATE_FRAMES   = ['plate_tcp', 'marble_plate', 'tool0']
TOP_SURFACE_FRAMES = {'plate_tcp'}


class MarbleSpawner(Node):

    def __init__(self):
        super().__init__('marble_spawner')
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._spawn_client  = self.create_client(SpawnEntity,  '/spawn_entity')
        self._delete_client = self.create_client(DeleteEntity, '/delete_entity')

        # odom monitoring
        self._marble_z    = None
        self._odom_sub    = self.create_subscription(
            Odometry, '/marble/odom', self._odom_cb, 10)

    def _odom_cb(self, msg: Odometry):
        self._marble_z = msg.pose.pose.position.z

    # ── helpers ───────────────────────────────────────────────────────────────

    def _get_tf(self):
        """Return (tf, used_frame) or (None, None). Pumps callbacks until TF arrives."""
        self.get_logger().info(f'Waiting for TF (trying: {CANDIDATE_FRAMES})…')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            for frame in CANDIDATE_FRAMES:
                try:
                    tf = self._tf_buffer.lookup_transform(
                        'world', frame, rclpy.time.Time())
                    return tf, frame
                except (tf2_ros.LookupException,
                        tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException):
                    pass
        return None, None

    def _spawn_once(self, spawn_x, spawn_y, spawn_z):
        """Call /spawn_entity and return True on service success."""
        pkg = get_package_share_directory('marble_balancer')
        with open(os.path.join(pkg, 'urdf', 'marble.sdf'), 'r') as f:
            sdf_xml = f.read()

        req = SpawnEntity.Request()
        req.name            = 'marble'
        req.xml             = sdf_xml
        req.robot_namespace = ''
        req.reference_frame = 'world'
        req.initial_pose.position.x    = float(spawn_x)
        req.initial_pose.position.y    = float(spawn_y)
        req.initial_pose.position.z    = float(spawn_z)
        req.initial_pose.orientation.w = 1.0

        future = self._spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is not None and result.success:
            return True
        msg = result.status_message if result else 'no response'
        self.get_logger().error(f'Spawn service error: {msg}')
        return False

    def _delete_marble(self):
        """Delete the existing marble and wait for Gazebo to flush the physics body."""
        if not self._delete_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('/delete_entity not available — skipping delete.')
            return

        del_req = DeleteEntity.Request()
        del_req.name = 'marble'
        future = self._delete_client.call_async(del_req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info('Deleted existing marble.')
        # Always wait for Gazebo to finish removing the physics body.
        # Without this delay the new entity is inserted while the old one
        # still occupies the same space → collision impulse ejects it downward.
        self.get_logger().info(
            f'Waiting {DELETE_SETTLE_S:.1f} s for physics flush…')
        deadline = time.monotonic() + DELETE_SETTLE_S
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    # ── main ──────────────────────────────────────────────────────────────────

    def run(self):
        # ── 1. Delete old marble, wait for physics flush ──────────────────────
        self._delete_marble()

        # ── 2. Wait for /spawn_entity ─────────────────────────────────────────
        self.get_logger().info('Waiting for /spawn_entity service…')
        while rclpy.ok() and not self._spawn_client.service_is_ready():
            rclpy.spin_once(self, timeout_sec=0.5)

        for attempt in range(1, MAX_RETRIES + 1):
            # ── 3. Fresh TF read — plate position after settle ────────────────
            tf, used_frame = self._get_tf()
            if tf is None:
                return

            plate_x   = tf.transform.translation.x
            plate_y   = tf.transform.translation.y
            plate_z   = tf.transform.translation.z
            surface_z = (plate_z if used_frame in TOP_SURFACE_FRAMES
                         else plate_z + PLATE_THICKNESS / 2.0)
            spawn_z   = surface_z + MARBLE_RADIUS + DROP_HEIGHT

            self.get_logger().info(
                f'[attempt {attempt}/{MAX_RETRIES}]  '
                f'plate "{used_frame}" z={surface_z:.4f}  '
                f'→ spawn at ({plate_x:.4f}, {plate_y:.4f}, {spawn_z:.4f})')

            # ── 4. Spawn ──────────────────────────────────────────────────────
            if not self._spawn_once(plate_x, plate_y, spawn_z):
                self.get_logger().warn('Spawn failed — retrying after short wait…')
                deadline = time.monotonic() + 0.5
                while time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=0.05)
                continue

            self.get_logger().info('Spawn accepted — verifying marble stays on plate…')

            # ── 5. Monitor odom: confirm marble does NOT fall through ──────────
            # Exits early once marble rests stably near surface + MARBLE_RADIUS.
            # Falls back to full MAX_VERIFY_S timeout only if marble bounces long.
            self._marble_z = None
            fell_through   = False
            land_ticks     = 0
            rest_z         = surface_z + MARBLE_RADIUS
            deadline       = time.monotonic() + MAX_VERIFY_S

            while time.monotonic() < deadline and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                if self._marble_z is not None:
                    if self._marble_z < surface_z + FALL_THROUGH_Z:
                        fell_through = True
                        break
                    if abs(self._marble_z - rest_z) < LAND_Z_MARGIN:
                        land_ticks += 1
                        if land_ticks >= LAND_CONFIRM:
                            break   # marble settled — exit early
                    else:
                        land_ticks = 0   # still bouncing

            if not fell_through:
                self.get_logger().info(
                    f'Marble confirmed on plate (z={self._marble_z:.4f} m).')
                return   # success

            # Fell through — delete and retry
            self.get_logger().warn(
                f'Marble fell through plate (z={self._marble_z:.4f} m, '
                f'surface={surface_z:.4f} m) — deleting and retrying…')
            self._delete_marble()

        self.get_logger().error(
            f'Marble failed to stay on plate after {MAX_RETRIES} attempts.')


def main(args=None):
    rclpy.init(args=args)
    node = MarbleSpawner()
    try:
        node.run()
    except Exception as e:
        node.get_logger().error(f'MarbleSpawner exception: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()
