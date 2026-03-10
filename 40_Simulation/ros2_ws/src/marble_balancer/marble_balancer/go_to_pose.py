"""
go_to_pose.py
-------------
Commands the UR5e to a Cartesian TCP pose using MoveIt's /compute_ik service,
then sends the resulting joint angles directly to the trajectory controller.

Run AFTER the simulation is fully up:
  ros2 run marble_balancer go_to_pose
"""
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# ── Target pose ────────────────────────────────────────────────────────────────
TARGET_X   = 0.433
TARGET_Y   = 0.25
TARGET_Z   = 0.8
# TCP Z axis straight up = tool frame aligned with world frame = identity quaternion
TARGET_QW  = 1.0
TARGET_QX  = 0.0
TARGET_QY  = 0.0
TARGET_QZ  = 0.0

MOVE_TIME_SEC = 5   # seconds to reach the pose
# ──────────────────────────────────────────────────────────────────────────────

JOINTS = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]


class GoToPose(Node):
    def __init__(self):
        super().__init__('go_to_pose')

        self._current_joint_state = None

        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.traj_pub  = self.create_publisher(
            JointTrajectory, '/ur7e_arm_controller/joint_trajectory', 10)

        # Subscribe to joint states so we can seed the IK from current pose
        self._js_sub = self.create_subscription(
            JointState, '/joint_states', self._js_callback, 10)

        self.get_logger().info('Waiting for /compute_ik service...')
        self.ik_client.wait_for_service()

        self.get_logger().info('Waiting for first /joint_states message...')
        while self._current_joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info('Service ready. Requesting IK...')
        self._send_ik_request()

    def _js_callback(self, msg: JointState):
        self._current_joint_state = msg

    def _send_ik_request(self):
        target = PoseStamped()
        target.header.frame_id = 'base_link'
        target.header.stamp    = self.get_clock().now().to_msg()

        target.pose.position.x    = TARGET_X
        target.pose.position.y    = TARGET_Y
        target.pose.position.z    = TARGET_Z
        target.pose.orientation.w = TARGET_QW
        target.pose.orientation.x = TARGET_QX
        target.pose.orientation.y = TARGET_QY
        target.pose.orientation.z = TARGET_QZ

        # Seed the solver from current robot state — critical for KDL to find a solution
        seed = RobotState()
        seed.joint_state = self._current_joint_state

        req = GetPositionIK.Request()
        req.ik_request                  = PositionIKRequest()
        req.ik_request.group_name       = 'ur_arm'
        req.ik_request.pose_stamped     = target
        req.ik_request.robot_state      = seed
        req.ik_request.timeout.sec      = 10
        req.ik_request.avoid_collisions = True

        future = self.ik_client.call_async(req)
        future.add_done_callback(self._ik_callback)

    def _ik_callback(self, future):
        response = future.result()

        error_code = response.error_code.val
        if error_code != 1:
            self.get_logger().error(
                f'IK failed — error code: {error_code}. '
                'The pose may not be reachable with this orientation. '
                'Try adjusting TARGET_Z or the quaternion.')
            rclpy.shutdown()
            return

        positions  = response.solution.joint_state.position
        joint_names = response.solution.joint_state.name

        name_to_pos = dict(zip(joint_names, positions))
        try:
            ordered = [name_to_pos[j] for j in JOINTS]
        except KeyError as e:
            self.get_logger().error(f'Missing joint in IK solution: {e}')
            rclpy.shutdown()
            return

        self.get_logger().info(
            f'IK solution: {[f"{v:.3f}" for v in ordered]}')
        self.get_logger().info(f'Moving to pose in {MOVE_TIME_SEC}s...')

        msg = JointTrajectory()
        msg.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = ordered
        pt.time_from_start = Duration(sec=MOVE_TIME_SEC)
        msg.points = [pt]

        self.traj_pub.publish(msg)
        self.get_logger().info('Trajectory sent. Done.')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = GoToPose()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
