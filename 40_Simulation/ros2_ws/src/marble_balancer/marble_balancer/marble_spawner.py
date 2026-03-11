"""
marble_spawner.py
-----------------
Spawns the marble in Gazebo directly above the marble plate centre.

Reads world → marble_plate (or tool0) from TF, adds a drop offset,
calls /spawn_entity.  No hardcoded coordinates.
"""

import os
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
import tf2_ros
from ament_index_python.packages import get_package_share_directory


MARBLE_RADIUS   = 0.015   # m
PLATE_THICKNESS = 0.005   # m
DROP_HEIGHT     = 0.08    # m above plate surface

# plate_tcp is at the top surface; other frames need a half-thickness correction
CANDIDATE_FRAMES   = ['plate_tcp', 'marble_plate', 'tool0']
TOP_SURFACE_FRAMES = {'plate_tcp'}   # frames already on the top surface


class MarbleSpawner(Node):

    def __init__(self):
        super().__init__('marble_spawner')
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._spawn_client = self.create_client(SpawnEntity, '/spawn_entity')

    def run(self):
        # ── Delete old marble if it exists ────────────────────────────────────
        del_client = self.create_client(DeleteEntity, '/delete_entity')
        if del_client.wait_for_service(timeout_sec=3.0):
            del_req = DeleteEntity.Request()
            del_req.name = 'marble'
            future = del_client.call_async(del_req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() and future.result().success:
                self.get_logger().info('Deleted existing marble.')

        # ── Wait for /spawn_entity ────────────────────────────────────────────
        self.get_logger().info('Waiting for /spawn_entity service…')
        while rclpy.ok() and not self._spawn_client.service_is_ready():
            rclpy.spin_once(self, timeout_sec=0.5)

        # ── Wait for TF — spin_once to let the listener receive messages ──────
        self.get_logger().info(
            f'Waiting for TF (trying: {CANDIDATE_FRAMES})…')
        tf, used_frame = None, None
        while rclpy.ok() and tf is None:
            rclpy.spin_once(self, timeout_sec=0.1)   # pump callbacks first
            for frame in CANDIDATE_FRAMES:
                try:
                    tf = self._tf_buffer.lookup_transform(
                        'world', frame, rclpy.time.Time())  # no timeout — instant
                    used_frame = frame
                    break
                except (tf2_ros.LookupException,
                        tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException):
                    pass

        if not rclpy.ok() or tf is None:
            return

        plate_x = tf.transform.translation.x
        plate_y = tf.transform.translation.y
        plate_z = tf.transform.translation.z
        # plate_tcp is already at the top surface; other frames need half-thickness offset
        surface_z = plate_z if used_frame in TOP_SURFACE_FRAMES else plate_z + PLATE_THICKNESS / 2.0

        spawn_x = plate_x #+ 0.15
        spawn_y = plate_y #+ 0.15
        spawn_z = surface_z + MARBLE_RADIUS + DROP_HEIGHT

        self.get_logger().info(
            f'Plate "{used_frame}": x={plate_x:.4f} y={plate_y:.4f} z={plate_z:.4f}')
        self.get_logger().info(
            f'Spawning marble at:  x={spawn_x:.4f} y={spawn_y:.4f} z={spawn_z:.4f}')

        # ── Load SDF ──────────────────────────────────────────────────────────
        pkg = get_package_share_directory('marble_balancer')
        with open(os.path.join(pkg, 'urdf', 'marble.sdf'), 'r') as f:
            sdf_xml = f.read()

        # ── Call spawn service ────────────────────────────────────────────────
        req = SpawnEntity.Request()
        req.name            = 'marble'
        req.xml             = sdf_xml
        req.robot_namespace = ''
        req.reference_frame = 'world'
        req.initial_pose.position.x    = spawn_x
        req.initial_pose.position.y    = spawn_y
        req.initial_pose.position.z    = spawn_z
        req.initial_pose.orientation.w = 1.0

        future = self._spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result is not None and result.success:
            self.get_logger().info('Marble spawned successfully.')
        else:
            msg = result.status_message if result else 'no response'
            self.get_logger().error(f'Spawn failed: {msg}')


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
