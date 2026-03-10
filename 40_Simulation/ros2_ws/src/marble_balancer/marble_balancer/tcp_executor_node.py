"""
tcp_executor_node.py

ROS2 node: tcp_executor

Subscribes to /target_tcp_pose (geometry_msgs/PoseStamped), solves IK using
the Levenberg-Marquardt method (modern_robotics), and sends the resulting joint
angles to the joint_trajectory_controller via a JointTrajectory message.

On startup the robot automatically moves to the home configuration where the
TCP points straight up (+Z world) — the flat-board equilibrium for balancing.

Topic flow:
  /target_tcp_pose  (geometry_msgs/PoseStamped)
      → IK solver (modern_robotics, Levenberg-Marquardt)
      → /joint_trajectory_controller/joint_trajectory  (trajectory_msgs/JointTrajectory)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from scipy.spatial.transform import Rotation
import modern_robotics as mr


# ── UR5e kinematic parameters ─────────────────────────────────────────────────
H1, L1, L2 = 0.1625, 0.425, 0.3922
W1, W2, H2 = 0.1333, 0.0997, 0.0996

M_HOME = np.array([
    [-1,  0, 0, L1 + L2],
    [ 0,  0, 1, W1 + W2],
    [ 0,  1, 0, H1 - H2],
    [ 0,  0, 0, 1      ]
])

S_LIST = np.array([
    [0,  0,  1,  0,       0,       0     ],
    [0,  1,  0, -H1,      0,       0     ],
    [0,  1,  0, -H1,      0,       L1    ],
    [0,  1,  0, -H1,      0,       L1+L2 ],
    [0,  0, -1, -W1,  L1+L2,       0     ],
    [0,  1,  0,  H2-H1,   0,       L1+L2 ],
]).T

JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

# Home config: TCP pointing straight up (+Z world), board flat for marble balancing
# Rule: q2+q3+q4 = -90 deg, q5 = 90 deg  → tool_Z = [0,0,1]
# FK result: pos=[0.487, 0.133, 0.395]  tool_Z=[0, 0, 1]
# Lower than old config (z=0.395 vs 0.687) → more wrist freedom to tilt board
HOME_THETA = np.array([0.0,
                        np.radians(-75),
                        np.radians(120),
                        np.radians(-135),
                        np.radians(90),
                        0.0])


class TcpExecutor(Node):

    def __init__(self):
        super().__init__('tcp_executor')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('move_duration_sec', 2.0)
        self.declare_parameter('ik_eomg', 0.001)
        self.declare_parameter('ik_ev',   0.0001)
        self.declare_parameter('ik_mu',   0.01)

        self._move_dur = self.get_parameter('move_duration_sec').value
        self._eomg     = self.get_parameter('ik_eomg').value
        self._ev       = self.get_parameter('ik_ev').value
        self._mu       = self.get_parameter('ik_mu').value

        # IK seed — start at home so first solve is close
        self._current_theta = HOME_THETA.copy()

        # ── Pub / Sub ─────────────────────────────────────────────────────────
        self._traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10,
        )

        self.create_subscription(JointState,  '/joint_states',    self._joint_state_cb, 10)
        self.create_subscription(PoseStamped, '/target_tcp_pose', self._pose_cb,        10)

        T_h = mr.FKinSpace(M_HOME, S_LIST, HOME_THETA)
        p   = T_h[:3, 3]
        self.get_logger().info(
            f'TCP executor ready.  Home: x={p[0]:.3f} y={p[1]:.3f} z={p[2]:.3f} '
            f'(tool Z up).  Moving there in 2 s ...'
        )

        # One-shot timer: move to home 3 s after node starts (give controller time to settle)
        self._home_timer = self.create_timer(3.0, self._go_home_once)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _send_trajectory(self, thetas: np.ndarray, duration_sec: float = None):
        """Publish a JointTrajectory message to the trajectory controller."""
        dur = duration_sec if duration_sec is not None else self._move_dur
        traj               = JointTrajectory()
        traj.header.stamp  = self.get_clock().now().to_msg()
        traj.header.frame_id = 'base_link'
        traj.joint_names   = JOINT_NAMES

        point              = JointTrajectoryPoint()
        point.positions    = thetas.tolist()
        point.velocities   = [0.0] * 6
        sec                = int(dur)
        point.time_from_start = Duration(sec=sec, nanosec=int((dur - sec) * 1e9))

        traj.points = [point]
        self._traj_pub.publish(traj)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _go_home_once(self):
        """Send home trajectory once at startup then cancel the timer."""
        self._send_trajectory(HOME_THETA, duration_sec=4.0)
        self.get_logger().info('Home trajectory sent (TCP pointing up).')
        self._home_timer.cancel()

    def _joint_state_cb(self, msg: JointState):
        """Keep current joint angles as warm IK seed."""
        if len(msg.name) < 6 or len(msg.position) < 6:
            return
        try:
            self._current_theta = np.array([
                msg.position[msg.name.index(j)] for j in JOINT_NAMES
            ])
        except ValueError:
            pass

    def _pose_cb(self, msg: PoseStamped):
        """Receive target TCP pose, solve IK, send joint trajectory."""
        p = msg.pose.position
        q = msg.pose.orientation
        rot   = Rotation.from_quat([q.x, q.y, q.z, q.w])
        T_des = np.eye(4)
        T_des[:3, :3] = rot.as_matrix()
        T_des[:3,  3] = [p.x, p.y, p.z]

        # ── Levenberg-Marquardt IK ────────────────────────────────────────────
        thetas    = self._current_theta.copy()
        converged = False

        for _ in range(50):
            T_curr  = mr.FKinSpace(M_HOME, S_LIST, thetas)
            V_b     = mr.Adjoint(mr.TransInv(T_curr)) @ \
                      mr.se3ToVec(mr.MatrixLog6(mr.TransInv(T_curr) @ T_des))
            V_s     = mr.Adjoint(T_curr) @ V_b

            if np.linalg.norm(V_s[:3]) < self._eomg and np.linalg.norm(V_s[3:]) < self._ev:
                converged = True
                break

            Js      = mr.JacobianSpace(S_LIST, thetas)
            thetas += np.linalg.solve(Js.T @ Js + self._mu * np.eye(6), Js.T @ V_s)

        if not converged:
            self.get_logger().warn(
                f'IK did not converge for ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) — skipping.'
            )
            return

        thetas = (thetas + np.pi) % (2 * np.pi) - np.pi
        self._current_theta = thetas
        self._send_trajectory(thetas)

        self.get_logger().info(
            'Joints (deg): ' + '  '.join(f'{np.degrees(t):.1f}' for t in thetas)
        )


def main(args=None):
    rclpy.init(args=args)
    node = TcpExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
