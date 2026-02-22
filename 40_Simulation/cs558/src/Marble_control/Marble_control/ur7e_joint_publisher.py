import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class UR7ePublisher(Node):
    def __init__(self):
        super().__init__('ur7e_joint_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback) # 10Hz
        
        # Define the joint names exactly as they appear in the URDF
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

    def timer_callback(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        # Replace these values with your calculated thetaSol from MATLAB
        # Example: Ball is balancing, so we tilt slightly
        msg.position = [0.0, 1.7933, 0.0657, -0.8229, 0.0000, -0.0361]
        self.get_logger().info(f'Publishing joint states: {msg.position}')
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = UR7ePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()