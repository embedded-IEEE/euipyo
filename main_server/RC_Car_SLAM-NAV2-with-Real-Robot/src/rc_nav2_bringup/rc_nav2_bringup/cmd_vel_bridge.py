#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelBridge(Node):
    """Simple bridge that republishes /cmd_vel to a target topic (e.g., /rc_car/cmd_vel)."""

    def __init__(self):
        super().__init__('cmd_vel_bridge')
        self.declare_parameter('target_topic', '/rc_car/cmd_vel')
        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('angular_scale', 1.0)
        target = self.get_parameter('target_topic').get_parameter_value().string_value
        self._linear_scale = float(self.get_parameter('linear_scale').value)
        self._angular_scale = float(self.get_parameter('angular_scale').value)

        self.pub = self.create_publisher(Twist, target, 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)
        self.get_logger().info(
            f"Bridging /cmd_vel -> {target} "
            f"(linear_scale={self._linear_scale}, angular_scale={self._angular_scale})"
        )

    def cb(self, msg: Twist):
        scaled = Twist()
        scaled.linear.x = msg.linear.x * self._linear_scale
        scaled.linear.y = msg.linear.y * self._linear_scale
        scaled.linear.z = msg.linear.z * self._linear_scale
        scaled.angular.x = msg.angular.x * self._angular_scale
        scaled.angular.y = msg.angular.y * self._angular_scale
        scaled.angular.z = msg.angular.z * self._angular_scale
        self.pub.publish(scaled)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
