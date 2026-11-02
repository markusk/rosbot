#!/usr/bin/env python3
"""Convert joystick messages into robot velocity commands."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy


class TeleopJoystickNode(Node):
    """Publish velocity commands from joystick input."""

    def __init__(self):
        """Initialize joystick mappings, parameters, and ROS endpoints."""
        super().__init__('TeleopJoystickNode')
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('forward_axis', 5)
        self.declare_parameter('alternate_forward_axis', 7)
        self.declare_parameter('turn_axis', 4)
        self.declare_parameter('alternate_turn_axis', 6)
        self.declare_parameter('stop_button', 10)
        self.declare_parameter('alternate_stop_button', 1)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.forward_axes = (
            int(self.get_parameter('forward_axis').value),
            int(self.get_parameter('alternate_forward_axis').value),
        )
        self.turn_axes = (
            int(self.get_parameter('turn_axis').value),
            int(self.get_parameter('alternate_turn_axis').value),
        )
        self.stop_buttons = (
            int(self.get_parameter('stop_button').value),
            int(self.get_parameter('alternate_stop_button').value),
        )
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)
        self.get_logger().info('Joystick teleoperation is ready')

    @staticmethod
    def _axis_value(axes, indices):
        """Return the first configured axis value outside the deadzone."""
        for index in indices:
            if 0 <= index < len(axes) and abs(axes[index]) > 0.5:
                return float(axes[index])
        return 0.0

    @staticmethod
    def _button_pressed(buttons, indices):
        """Return whether any configured button is pressed."""
        return any(0 <= index < len(buttons) and buttons[index] for index in indices)

    def joy_callback(self, msg):
        """Publish a Twist command for the latest joystick message."""
        twist = Twist()
        if not self._button_pressed(msg.buttons, self.stop_buttons):
            twist.linear.x = self._axis_value(msg.axes, self.forward_axes) * self.linear_speed
            twist.angular.z = self._axis_value(msg.axes, self.turn_axes) * self.angular_speed
        self.publisher.publish(twist)


def main(args=None):
    """Run the joystick teleoperation node."""
    rclpy.init(args=args)
    node = TeleopJoystickNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
