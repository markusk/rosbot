#!/usr/bin/env python3
"""Route active keyboard or joystick commands to the motor controller."""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class TeleopArbitratorNode(Node):
    """Select the last active teleoperation source without summing commands."""

    def __init__(self):
        """Initialize command subscriptions and the output publisher."""
        super().__init__('TeleopArbitratorNode')
        self.active_source = None
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.keyboard_subscription = self.create_subscription(
            Twist, 'keyboard_cmd_vel', self.keyboard_callback, 10)
        self.joystick_subscription = self.create_subscription(
            Twist, 'joystick_cmd_vel', self.joystick_callback, 10)
        self.stop_subscription = self.create_subscription(
            Bool, 'joystick_stop', self.joystick_stop_callback, 10)

    @staticmethod
    def _is_active(command):
        """Return whether a velocity command requests movement."""
        return any((
            command.linear.x,
            command.linear.y,
            command.linear.z,
            command.angular.x,
            command.angular.y,
            command.angular.z,
        ))

    @staticmethod
    def _resolve_source(active_source, source, is_active, explicit_stop=False):
        """Resolve the active source and whether the output should stop."""
        if explicit_stop or (source == 'keyboard' and not is_active):
            return None, True
        if is_active:
            return source, False
        if source == 'joystick' and active_source == 'joystick':
            return None, True
        return active_source, False

    def _handle_command(self, source, command, explicit_stop=False):
        self.active_source, should_stop = self._resolve_source(
            self.active_source,
            source,
            self._is_active(command),
            explicit_stop,
        )
        if should_stop:
            self.publisher.publish(Twist())
        elif self.active_source == source and self._is_active(command):
            self.publisher.publish(command)

    def keyboard_callback(self, command):
        """Handle keyboard velocity commands."""
        self._handle_command('keyboard', command)

    def joystick_callback(self, command):
        """Handle joystick velocity commands."""
        self._handle_command('joystick', command)

    def joystick_stop_callback(self, message):
        """Stop immediately when a joystick stop button is pressed."""
        if message.data:
            self._handle_command('joystick', Twist(), explicit_stop=True)


def main(args=None):
    """Run the teleoperation arbitrator node."""
    rclpy.init(args=args)
    node = TeleopArbitratorNode()
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
