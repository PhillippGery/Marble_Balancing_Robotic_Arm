import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import numpy as np
import modern_robotics as mr

class MarbleIKController(Node):
    def __init__(self):
        super().__init__('marble_ik_controller')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        # Subscriber for the target position (x, y, z)
        self.subscription = self.create_subscription(Point, '/target_position', self.target_callback, 10)
        
        # --- UR7e Kinematic Parameters (from your URDF) ---
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        # Official UR7e/UR5e Geometric Parameters (meters)
        H1, L1, L2 = 0.1625, 0.425, 0.3922
        W1, W2, H2 = 0.1333, 0.0997, 0.0996

        # 2. Home position of end effector (M)
        # This orientation matrix suggests the robot is "reaching forward" 
        # along the X-axis rather than pointing up.
        self.M = np.array([
            [-1, 0, 0, L1 + L2],
            [ 0, 0, 1, W1 + W2],
            [ 0, 1, 0, H1 - H2],
            [ 0, 0, 0, 1]
        ])

        # 3. Space Screw Axes (Slist)
        # Note: S = [omega; v]. v = -omega x q
        self.Slist = np.array([
            [0, 0,  1, 0,   0,       0],     # S1
            [0, 1,  0, -H1, 0,       0],     # S2
            [0, 1,  0, -H1, 0,       L1],    # S3
            [0, 1,  0, -H1, 0,       L1+L2], # S4
            [0, 0, -1, -W1, L1+L2,   0],     # S5
            [0, 1,  0, H2-H1, 0,     L1+L2]  # S6
        ]).T # Transpose to get 6x6 matrix


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
        # 1. Desired Configuration
        R_des = np.eye(3) 
        P_des = np.array([msg.x, msg.y, msg.z])
        T_des = np.eye(4)
        T_des[0:3, 0:3] = R_des
        T_des[0:3, 3] = P_des

        # 2. THE FIX: Avoid the singularity seed
        # If the current state is all zeros, provide a "bent elbow" hint
        if np.allclose(self.current_theta, 0):
            seed_theta = np.array([0.0, -0.2, 0.4, -0.2, 0.0, 0.0])
        else:
            seed_theta = self.current_theta

        # 3. Solve Inverse Kinematics
        # IKinSpace(Slist, M, T, thetalist0, eomg, ev)
        thetas, success = mr.IKinSpace(self.Slist, self.M, T_des, seed_theta, 0.001, 0.0001)

        if success:
            # 4. Normalize and update
            thetas = (thetas + np.pi) % (2 * np.pi) - np.pi
            self.current_theta = thetas
            self.publish_joints(thetas)
        else:
            # Check if the target is simply too far away
            dist = np.linalg.norm(P_des)
            self.get_logger().warn(f"IK Failed. Target Dist: {dist:.2f}m. " 
                                   f"Target: {msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f}")


    def publish_joints(self, thetas):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = thetas.tolist()
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MarbleIKController()
    rclpy.spin(node)
    rclpy.shutdown()