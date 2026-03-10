import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import numpy as np

class StaticServoPilot(Node):
    def __init__(self):
        super().__init__('marble_servo_controller')

        # This topic feeds MoveIt Servo
        self.servo_pub = self.create_publisher(
            TwistStamped, 
            '/servo_node/delta_twist_cmds', 
            10
        )

        # 100Hz Control Loop
        self.timer = self.create_timer(0.01, self._timer_cb)
        
        self.get_logger().info("Static Pilot Started. Holding Cartesian Zero Drift.")

    def _timer_cb(self):
        # We send a "Zero Twist". 
        # MoveIt Servo interprets this as "Maintain current IK solution" 
        # while checking for collisions and joint limits.
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link" # Commands are relative to the robot base

        # All velocities are 0.0
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.servo_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StaticServoPilot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()