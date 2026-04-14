import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import numpy as np
import modern_robotics as mr
from scipy.spatial.transform import Rotation as R

class marbleIKController(Node):
    def __init__(self):
        super().__init__('marble_ik_controller')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

        # Subscriber for the target position (x, y, z)
        self.subscription = self.create_subscription(Pose, '/target_position', self.target_callback, 10)
        
        # UR7e Kinematic Parameters (from your URDF) ---
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        # Official UR7e/UR5e Geometric Parameters (meters)
        H1, L1, L2 = 0.1625, 0.425, 0.3922
        W1, W2, H2 = 0.1333, 0.0997, 0.0996

        # Home position of end effector (M)

        self.M = np.array([
            [-1, 0, 0, L1 + L2],
            [ 0, 0, 1, W1 + W2],
            [ 0, 1, 0, H1 - H2],
            [ 0, 0, 0, 1]
        ])


        self.Slist = np.array([
            [0, 0,  1, 0,   0,       0],     # S1
            [0, 1,  0, -H1, 0,       0],     # S2
            [0, 1,  0, -H1, 0,       L1],    # S3
            [0, 1,  0, -H1, 0,       L1+L2], # S4
            [0, 0, -1, -W1, L1+L2,   0],     # S5
            [0, 1,  0, H2-H1, 0,     L1+L2]  # S6
        ]).T


        self.current_theta = np.array([0.0, -np.pi/2, 0.0, 0.0, 0.0, 0.0])
        self.get_logger().info("Marble IK Node Started. Send a Point to /target_position")

        T_start = mr.FKinSpace(self.M, self.Slist, self.current_theta)
        
        pos = T_start[0:3, 3]
        self.get_logger().info("\n" + "="*30 + 
                               f"\nINITIAL TCP CONFIGURATION:"
                               f"\nX: {pos[0]:.4f}"
                               f"\nY: {pos[1]:.4f}"
                               f"\nZ: {pos[2]:.4f}"
                               f"\n" + "="*30)

        self.init_timer = self.create_timer(1.0, self.init_publish_callback)

    def init_publish_callback(self):
        # Publish the initial state
        self.get_logger().info("Publishing initial latch state...")
        self.publish_joints(self.current_theta)
        # Destroy the timer so it only runs once
        self.init_timer.destroy()




    def target_callback(self, msg):

        P_des = np.array([msg.position.x, msg.position.y, msg.position.z])

        quat = [
            msg.orientation.x, 
            msg.orientation.y, 
            msg.orientation.z, 
            msg.orientation.w
        ]
        

        rot = R.from_quat(quat)
        R_des = rot.as_matrix()

        T_des = np.eye(4)
        T_des[0:3, 0:3] = R_des
        T_des[0:3, 3] = P_des

        # LM Parameters
        max_iter = 50
        eomg = 0.001  # Angular error tolerance
        ev = 0.0001   # Linear error tolerance (0.1mm)
        mu = 0.01     # Damping factor (lambda^2)

        if np.allclose(self.current_theta, 0):
            thetas = np.array([0.0, -0.2, 0.4, -0.2, 0.0, 0.0])
        else:
            thetas = self.current_theta.copy()
        
        for i in range(max_iter):
            # Forward Kinematics
            T_curr = mr.FKinSpace(self.M, self.Slist, thetas)
            
            # Calculate Error Twist in Space Frame
            V_b = mr.Adjoint(mr.TransInv(T_curr)) @ mr.se3ToVec(mr.MatrixLog6(mr.TransInv(T_curr) @ T_des))
            V_s = mr.Adjoint(T_curr) @ V_b
            
            # Check if error is within tolerance
            err_omg = np.linalg.norm(V_s[0:3])
            err_v = np.linalg.norm(V_s[3:6])
            
            if err_omg < eomg and err_v < ev:
                self.current_theta = (thetas + np.pi) % (2 * np.pi) - np.pi
                self.publish_joints(self.current_theta)
                return

            Js = mr.JacobianSpace(self.Slist, thetas)
            # Solve: (J^T * J + mu * I) * d_theta = J^T * V_s
            d_theta = np.linalg.solve(Js.T @ Js + mu * np.eye(6), Js.T @ V_s)
            
            thetas += d_theta

        self.get_logger().warn(f"LM Solver failed to converge for {msg.position.x, msg.position.y, msg.position.z}")


    def publish_joints(self, thetas):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = thetas.tolist()
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = marbleIKController()
    rclpy.spin(node)
    rclpy.shutdown()