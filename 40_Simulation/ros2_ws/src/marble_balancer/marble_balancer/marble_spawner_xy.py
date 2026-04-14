"""
marble_spawner_xy.py
--------------------
Spawns the marble at a specific (x, y) offset from the plate centre.

The x/y coordinates are in the PLATE frame — (0, 0) is the plate centre,
positive X is forward, positive Y is left (same convention as the LQR state).

Usage (via launch file):
  ros2 launch marble_balancer spawn_xy.launch.py x:=0.10 y:=-0.05

Usage (direct):
  ros2 run marble_balancer marble_spawner_xy --ros-args -p x:=0.10 -p y:=0.05

Parameters:
  x          (float, default 0.0) — plate-frame X offset from centre (m)
  y          (float, default 0.0) — plate-frame Y offset from centre (m)
  drop_height (float, default 0.05) — height above plate surface to drop from (m)

Coordinates are clamped to ±0.18 m so the marble always starts on the plate.
"""

import os
import time
import math
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from nav_msgs.msg import Odometry
import tf2_ros
import numpy as np
from ament_index_python.packages import get_package_share_directory


MARBLE_RADIUS   = 0.015   # m
PLATE_THICKNESS = 0.005   # m
DELETE_SETTLE_S = 0.6     # s to wait after delete for physics flush
MAX_VERIFY_S    = 3.0     # s to monitor odom after spawn
FALL_THROUGH_Z  = -0.05   # marble fell through if z drops this far below surface
MAX_RETRIES     = 3
PLATE_HALF      = 0.18    # clamp XY so marble stays on plate (< 0.20 hard edge)

CANDIDATE_FRAMES   = ['plate_tcp', 'marble_plate', 'tool0']
TOP_SURFACE_FRAMES = {'plate_tcp'}


def _quat_to_rot(qx, qy, qz, qw) -> np.ndarray:
    """Quaternion → 3×3 rotation matrix."""
    return np.array([
        [1 - 2*(qy*qy + qz*qz),   2*(qx*qy - qz*qw),   2*(qx*qz + qy*qw)],
        [    2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz),  2*(qy*qz - qx*qw)],
        [    2*(qx*qz - qy*qw),   2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
    ])


class MarbleSpawnerXY(Node):

    def __init__(self):
        super().__init__('marble_spawner_xy')

        self.declare_parameter('x',           0.0)
        self.declare_parameter('y',           0.0)
        self.declare_parameter('drop_height', 0.05)

        raw_x = float(self.get_parameter('x').value)
        raw_y = float(self.get_parameter('y').value)
        self._offset_x   = max(-PLATE_HALF, min(PLATE_HALF, raw_x))
        self._offset_y   = max(-PLATE_HALF, min(PLATE_HALF, raw_y))
        self._drop_height = float(self.get_parameter('drop_height').value)

        if abs(raw_x) > PLATE_HALF or abs(raw_y) > PLATE_HALF:
            self.get_logger().warn(
                f'Requested ({raw_x:.3f}, {raw_y:.3f}) exceeds plate half-size '
                f'±{PLATE_HALF} m — clamped to ({self._offset_x:.3f}, {self._offset_y:.3f})')

        self.get_logger().info(
            f'Spawn offset: x={self._offset_x:+.3f} m, y={self._offset_y:+.3f} m '
            f'(plate frame)  drop_height={self._drop_height:.3f} m')

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._spawn_client  = self.create_client(SpawnEntity,  '/spawn_entity')
        self._delete_client = self.create_client(DeleteEntity, '/delete_entity')

        self._marble_z = None
        self.create_subscription(Odometry, '/marble/odom', self._odom_cb, 10)

    def _odom_cb(self, msg: Odometry):
        self._marble_z = msg.pose.pose.position.z

    # ── helpers ───────────────────────────────────────────────────────────────

    def _get_tf(self):
        """Block until a plate TF is available. Returns (tf, frame_name)."""
        self.get_logger().info(f'Waiting for plate TF (frames: {CANDIDATE_FRAMES})…')
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

    def _plate_offset_to_world(self, tf, offset_x: float, offset_y: float):
        """
        Convert a plate-frame (x, y) offset to world-frame (X, Y).

        plate_tcp has yaw ≈ 180° at home, so the plate X axis points
        opposite to world X.  We use the plate rotation matrix to correctly
        map the offset into world coordinates.
        """
        q = tf.transform.rotation
        R = _quat_to_rot(q.x, q.y, q.z, q.w)
        # Plate X/Y axes are the first two columns of R
        plate_x_axis = R[:, 0]
        plate_y_axis = R[:, 1]

        world_offset = offset_x * plate_x_axis + offset_y * plate_y_axis
        plate_cx = tf.transform.translation.x
        plate_cy = tf.transform.translation.y

        world_x = plate_cx + world_offset[0]
        world_y = plate_cy + world_offset[1]
        return world_x, world_y

    def _delete_marble(self):
        if not self._delete_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('/delete_entity not available — skipping delete.')
            return
        req = DeleteEntity.Request()
        req.name = 'marble'
        future = self._delete_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info('Deleted existing marble.')
        self.get_logger().info(f'Waiting {DELETE_SETTLE_S:.1f} s for physics flush…')
        deadline = time.monotonic() + DELETE_SETTLE_S
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _spawn_once(self, world_x: float, world_y: float, spawn_z: float) -> bool:
        pkg = get_package_share_directory('marble_balancer')
        with open(os.path.join(pkg, 'urdf', 'marble.sdf'), 'r') as f:
            sdf_xml = f.read()

        req = SpawnEntity.Request()
        req.name                         = 'marble'
        req.xml                          = sdf_xml
        req.robot_namespace              = ''
        req.reference_frame              = 'world'
        req.initial_pose.position.x      = float(world_x)
        req.initial_pose.position.y      = float(world_y)
        req.initial_pose.position.z      = float(spawn_z)
        req.initial_pose.orientation.w   = 1.0

        future = self._spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is not None and result.success:
            return True
        msg = result.status_message if result else 'no response'
        self.get_logger().error(f'Spawn service error: {msg}')
        return False

    # ── main ──────────────────────────────────────────────────────────────────

    def run(self):
        # 1. Delete existing marble
        self._delete_marble()

        # 2. Wait for /spawn_entity
        self.get_logger().info('Waiting for /spawn_entity service…')
        while rclpy.ok() and not self._spawn_client.service_is_ready():
            rclpy.spin_once(self, timeout_sec=0.5)

        for attempt in range(1, MAX_RETRIES + 1):
            # 3. Fresh TF read after settle
            tf, used_frame = self._get_tf()
            if tf is None:
                return

            plate_z   = tf.transform.translation.z
            surface_z = (plate_z if used_frame in TOP_SURFACE_FRAMES
                         else plate_z + PLATE_THICKNESS / 2.0)
            spawn_z   = surface_z + MARBLE_RADIUS + self._drop_height

            # 4. Convert plate-frame offset → world-frame spawn position
            world_x, world_y = self._plate_offset_to_world(
                tf, self._offset_x, self._offset_y)

            self.get_logger().info(
                f'[attempt {attempt}/{MAX_RETRIES}]  '
                f'plate_offset=({self._offset_x:+.3f}, {self._offset_y:+.3f}) m  '
                f'→ world=({world_x:.4f}, {world_y:.4f}, {spawn_z:.4f})')

            # 5. Spawn
            if not self._spawn_once(world_x, world_y, spawn_z):
                self.get_logger().warn('Spawn failed — retrying…')
                deadline = time.monotonic() + 0.5
                while time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=0.05)
                continue

            self.get_logger().info('Spawn accepted — verifying marble stays on plate…')

            # 6. Monitor odom: confirm marble does not fall through plate
            self._marble_z = None
            fell_through   = False
            deadline       = time.monotonic() + MAX_VERIFY_S
            while time.monotonic() < deadline and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                if (self._marble_z is not None
                        and self._marble_z < surface_z + FALL_THROUGH_Z):
                    fell_through = True
                    break

            if not fell_through:
                self.get_logger().info(
                    f'Marble confirmed on plate at offset '
                    f'({self._offset_x:+.3f}, {self._offset_y:+.3f}) m '
                    f'(z={self._marble_z:.4f} m).')
                return

            self.get_logger().warn(
                f'Marble fell through (z={self._marble_z:.4f} m) — retrying…')
            self._delete_marble()

        self.get_logger().error(
            f'Marble failed to stay on plate after {MAX_RETRIES} attempts.')


def main(args=None):
    rclpy.init(args=args)
    node = MarbleSpawnerXY()
    try:
        node.run()
    except Exception as e:
        node.get_logger().error(f'MarbleSpawnerXY exception: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()
