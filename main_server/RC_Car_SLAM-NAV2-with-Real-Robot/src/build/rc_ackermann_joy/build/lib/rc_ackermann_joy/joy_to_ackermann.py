import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoyToAckermann(Node):
    def __init__(self):
        super().__init__('joy_to_ackermann')

        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('cmd_topic', '/rc_car/cmd_vel')
        self.declare_parameter('speed_axis', 1)
        self.declare_parameter('steer_axis', 2)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('max_steer', 1.0)
        self.declare_parameter('speed_deadzone', 0.05)
        self.declare_parameter('steer_deadzone', 0.05)
        self.declare_parameter('invert_speed', False)
        self.declare_parameter('invert_steer', False)

        joy_topic = self.get_parameter('joy_topic').value
        cmd_topic = self.get_parameter('cmd_topic').value

        self._speed_axis = int(self.get_parameter('speed_axis').value)
        self._steer_axis = int(self.get_parameter('steer_axis').value)
        self._max_speed = float(self.get_parameter('max_speed').value)
        self._max_steer = float(self.get_parameter('max_steer').value)
        self._speed_deadzone = float(self.get_parameter('speed_deadzone').value)
        self._steer_deadzone = float(self.get_parameter('steer_deadzone').value)
        self._invert_speed = bool(self.get_parameter('invert_speed').value)
        self._invert_steer = bool(self.get_parameter('invert_steer').value)

        self._pub = self.create_publisher(Twist, cmd_topic, 10)
        self.create_subscription(Joy, joy_topic, self._on_joy, 10)

        self.get_logger().info(
            f'Mapping Joy axes: speed={self._speed_axis} '
            f'steer={self._steer_axis} to {cmd_topic}'
        )

    def _apply_deadzone(self, value, deadzone):
        if abs(value) < deadzone:
            return 0.0
        return value

    def _on_joy(self, msg: Joy):
        if self._speed_axis >= len(msg.axes) or self._steer_axis >= len(msg.axes):
            self.get_logger().warn('Joy message missing expected axes')
            return

        speed = msg.axes[self._speed_axis]
        steer = msg.axes[self._steer_axis]

        if self._invert_speed:
            speed = -speed
        if self._invert_steer:
            steer = -steer

        speed = self._apply_deadzone(speed, self._speed_deadzone)
        steer = self._apply_deadzone(steer, self._steer_deadzone)

        twist = Twist()
        twist.linear.x = float(speed) * self._max_speed
        twist.angular.z = float(steer) * self._max_steer

        self._pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToAckermann()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
