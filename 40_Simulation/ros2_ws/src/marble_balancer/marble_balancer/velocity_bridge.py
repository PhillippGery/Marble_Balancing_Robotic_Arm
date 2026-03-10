import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import PyKDL as kdl
from kdl_parser_py.urdf import treeFromParamServer

class VelocityBridge(Node):
    def __init__(self):
        super().__init__('velocity_bridge')

        # 1. Get URDF from the robot_description parameter
        # This ensures the math matches the simulation 100%
        self.declare_parameter('robot_description', '')
        success, self.tree = treeFromParamServer(self)
        if not success:
            self.get_logger().error("Failed to extract KDL tree from robot_description!")
            return

        self.chain = self.tree.getChain("base_link", "tool0")
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.jac_solver = kdl.ChainJntToJacSolver(self.chain)
        
        # ── Target from your Screenshot ──
        self.target_pos = kdl.Vector(0.326, 0.252, 0.739)

        self.sub_js = self.create_subscription(JointState, '/joint_states', self.js_callback, 10)
        self.pub_traj = self.create_publisher(JointTrajectory, '/ur7e_arm_controller/joint_trajectory', 10)
        
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.q = kdl.JntArray(self.chain.getNrOfJoints())

        self.timer = self.create_timer(0.02, self.control_loop)

    def js_callback(self, msg):
        # Map joint states to KDL JntArray
        positions = dict(zip(msg.name, msg.position))
        for i, name in enumerate(self.joint_names):
            if name in positions:
                self.q[i] = positions[name]

    def control_loop(self):
        # 2. Compute Forward Kinematics
        end_effector_frame = kdl.Frame()
        self.fk_solver.JntToCart(self.q, end_effector_frame)
        
        current_pos = end_effector_frame.p
        self.get_logger().info(f"Actual Position: {current_pos.x()}, {current_pos.y()}, {current_pos.z()}", throttle_duration_sec=1.0)

        # 3. Compute Velocity to reach target
        kp = 2.0
        v_out = kp * (self.target_pos - current_pos)

        # 4. Compute Jacobian and Inverse
        jacobian = kdl.Jacobian(self.chain.getNrOfJoints())
        self.jac_solver.JntToJac(self.q, jacobian)
        
        # Convert KDL Jacobian to Numpy for inversion
        jac_np = np.array([[jacobian[i, j] for j in range(jacobian.columns())] for i in range(jacobian.rows())])
        
        # We only care about linear velocity (x, y, z) for now
        v_vec = np.array([0, 0, 0, v_out.x(), v_out.y(), v_out.z()]) # [Rot, Lin]
        
        q_dot = np.linalg.pinv(jac_np) @ v_vec

        # 5. Publish
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        dt = 0.02
        point.positions = [self.q[i] + q_dot[i] * dt for i in range(len(self.joint_names))]
        point.velocities = q_dot.tolist()
        point.time_from_start = rclpy.duration.Duration(seconds=dt).to_msg()
        traj.points.append(point)
        self.pub_traj.publish(traj)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()