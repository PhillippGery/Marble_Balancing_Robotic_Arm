"""
mux_controller — priority multiplexer between manual and LQR control.

Subscribes to:
  /marble_servo/delta_twist_cmds  (from marble_servo_controller, remapped in launch)
  /manual/delta_twist_cmds        (operator joystick / teleoperation)

Publishes to:
  /servo_node/delta_twist_cmds    (MoveIt Servo input)
  /mux_controller/mode            (std_msgs/String: "manual" | "auto")

Priority rule: if a manual command was received within `manual_timeout` seconds,
forward the manual command; otherwise forward the LQR auto command.
Transitions between modes are logged once each.
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String


class MuxController(Node):
    def __init__(self):
        super().__init__('mux_controller')

        self.declare_parameter('manual_timeout', 0.5)
        self.declare_parameter('publish_rate', 30.0)

        self._manual_timeout = self.get_parameter('manual_timeout').value
        publish_rate = self.get_parameter('publish_rate').value

        self._last_auto_msg: TwistStamped | None = None
        self._last_manual_msg: TwistStamped | None = None
        self._last_manual_time: Time | None = None
        self._in_manual = False

        self.create_subscription(
            TwistStamped,
            '/marble_servo/delta_twist_cmds',
            self._auto_cb,
            10,
        )
        self.create_subscription(
            TwistStamped,
            '/manual/delta_twist_cmds',
            self._manual_cb,
            10,
        )

        self._pub_cmd = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self._pub_mode = self.create_publisher(String, '/mux_controller/mode', 1)

        self.create_timer(1.0 / publish_rate, self._publish_cb)

    def _auto_cb(self, msg: TwistStamped):
        self._last_auto_msg = msg

    def _manual_cb(self, msg: TwistStamped):
        self._last_manual_msg = msg
        self._last_manual_time = self.get_clock().now()

    def _publish_cb(self):
        now = self.get_clock().now()
        manual_active = (
            self._last_manual_time is not None
            and (now - self._last_manual_time).nanoseconds * 1e-9 < self._manual_timeout
        )

        if manual_active != self._in_manual:
            self._in_manual = manual_active
            if manual_active:
                self.get_logger().info('Switching to MANUAL control')
            else:
                self.get_logger().info('Reverting to AUTO control')

        mode_msg = String()
        mode_msg.data = 'manual' if self._in_manual else 'auto'
        self._pub_mode.publish(mode_msg)

        if self._in_manual and self._last_manual_msg is not None:
            self._pub_cmd.publish(self._last_manual_msg)
        elif not self._in_manual and self._last_auto_msg is not None:
            self._pub_cmd.publish(self._last_auto_msg)
        else:
            # Fallback: zero twist so MoveIt Servo halts cleanly
            zero = TwistStamped()
            zero.header.stamp = now.to_msg()
            zero.header.frame_id = 'base_link'
            self._pub_cmd.publish(zero)


def main(args=None):
    rclpy.init(args=args)
    node = MuxController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
