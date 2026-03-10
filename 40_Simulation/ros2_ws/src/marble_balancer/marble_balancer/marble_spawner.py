"""
marble_spawner.py
-----------------
Spawns the marble in Gazebo directly above the marble plate centre.

Looks up the world → marble_plate TF (the plate is a fixed link on the URDF,
so its frame is always in TF), adds a vertical drop offset, then calls the
Gazebo /spawn_entity service.  No hardcoded coordinates.

Launch order: started 6 s after go_to_pose exits so the robot has finished
its 5 s homing trajectory and TF reflects the final plate position.
"""

import os
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import tf2_ros
from ament_index_python.packages import get_package_share_directory


# ── Drop geometry (must match marble.sdf / ur5e_marble_balancer.urdf.xacro) ───
MARBLE_RADIUS   = 0.015   # m  — sphere radius
PLATE_THICKNESS = 0.005   # m  — plate box z-size (plate centre → top = half this)
DROP_HEIGHT     = 0.08    # m  — free-fall gap above plate surface
# ──────────────────────────────────────────────────────────────────────────────

# TF frames to try in order (marble_plate is most precise; tool0 is a fallback)
CANDIDATE_FRAMES = ['marble_plate', 'tool0']


class MarbleSpawner(Node):

    def __init__(self):
        super().__init__('marble_spawner')

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._spawn_client = self.create_client(SpawnEntity, '/spawn_entity')

        self._timer = self.create_timer(0.5, self._try_spawn)
        self.get_logger().info(
            f'MarbleSpawner: waiting for TF and /spawn_entity …  '
            f'(trying frames: {CANDIDATE_FRAMES})')

    def _try_spawn(self):
        if not self._spawn_client.service_is_ready():
            return

        # Try each candidate frame until one resolves
        tf = None
        used_frame = None
        for frame in CANDIDATE_FRAMES:
            try:
                tf = self._tf_buffer.lookup_transform(
                    'world', frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5),
                )
                used_frame = frame
                break
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                continue

        if tf is None:
            return  # not ready yet — try again on next tick

        self._timer.cancel()

        plate_x = tf.transform.translation.x
        plate_y = tf.transform.translation.y
        plate_z = tf.transform.translation.z   # centre of the plate link

        # Marble spawn height: plate centre + half plate thickness (→ plate top)
        #                                  + marble radius (→ marble centre on surface)
        #                                  + drop gap     (→ free-fall start)
        spawn_z = plate_z + PLATE_THICKNESS / 2.0 + MARBLE_RADIUS + DROP_HEIGHT

        self.get_logger().info(
            f'Plate frame "{used_frame}" in world: '
            f'x={plate_x:.4f}  y={plate_y:.4f}  z={plate_z:.4f}')
        self.get_logger().info(
            f'Spawning marble at:  '
            f'x={plate_x:.4f}  y={plate_y:.4f}  z={spawn_z:.4f}  '
            f'(drop gap {DROP_HEIGHT*100:.0f} cm)')

        pkg      = get_package_share_directory('marble_balancer')
        sdf_path = os.path.join(pkg, 'urdf', 'marble.sdf')
        with open(sdf_path, 'r') as f:
            sdf_xml = f.read()

        req = SpawnEntity.Request()
        req.name            = 'marble'
        req.xml             = sdf_xml
        req.robot_namespace = ''
        req.reference_frame = 'world'
        req.initial_pose.position.x    = plate_x
        req.initial_pose.position.y    = plate_y
        req.initial_pose.position.z    = spawn_z
        req.initial_pose.orientation.w = 1.0

        future = self._spawn_client.call_async(req)
        future.add_done_callback(self._on_spawn_done)

    def _on_spawn_done(self, future):
        result = future.result()
        if result.success:
            self.get_logger().info('Marble spawned successfully.')
        else:
            self.get_logger().error(f'Spawn failed: {result.status_message}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MarbleSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
