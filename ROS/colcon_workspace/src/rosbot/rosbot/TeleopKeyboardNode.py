#!/usr/bin/env python3
import sys
import termios
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class TeleopKeyboardNode(Node):
    def __init__(self):
        super().__init__('TeleopKeyboardNode')
        self.get_logger().info('Keyboard teleoperation node started')

        # Publisher for cmd_vel
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Speed parameters
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 1.0  # rad/s
        self.speed_increment = 0.1

        # Current speeds
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular_z = 0.0

        self.print_instructions()

    def print_instructions(self):
        """Print the keyboard control instructions."""
        msg = """
        ╔════════════════════════════════════╗
        ║   ROSBOT KEYBOARD TELEOPERATION    ║
        ╠════════════════════════════════════╣
        ║                                    ║
        ║   Movement (Omniwheel):            ║
        ║                                    ║
        ║        W (forward)                 ║
        ║        ↑                           ║
        ║   A ← + → D                        ║
        ║        ↓                           ║
        ║        S (backward)                ║
        ║                                    ║
        ║   Q: left forward diagonal         ║
        ║   E: right forward diagonal        ║
        ║   Z: left backward diagonal        ║
        ║   C: right backward diagonal       ║
        ║                                    ║
        ║   Rotation:                        ║
        ║   J: turn left                     ║
        ║   L: turn right                    ║
        ║                                    ║
        ║   Speed:                           ║
        ║   +: faster                        ║
        ║   -: slower                        ║
        ║                                    ║
        ║   SPACE: Stop                      ║
        ║   ESC/Ctrl+C: End                  ║
        ║                                    ║
        ╚════════════════════════════════════╝
        """
        print(msg)
        self.print_status()

    def print_status(self):
        """Print the current speed and movement status."""
        print(
            f'\Speed: linear={self.linear_speed:.2f} m/s, '
            f'angular={self.angular_speed:.2f} rad/s')
        print(
            f'Current: vx={self.current_linear_x:.2f}, '
            f'vy={self.current_linear_y:.2f}, '
            f'w={self.current_angular_z:.2f}')

    def get_key(self):
        """Read one character from the keyboard."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def publish_twist(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        """Publish a velocity command."""
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        self.publisher.publish(twist)

        self.current_linear_x = linear_x
        self.current_linear_y = linear_y
        self.current_angular_z = angular_z

    def run(self):
        """Process keyboard input until the node is stopped."""
        try:
            while rclpy.ok():
                key = self.get_key()

                # movement commands
                if key in ('w', 'W'):
                    self.publish_twist(linear_x=self.linear_speed)
                    print('↑ forward')
                elif key in ('s', 'S'):
                    self.publish_twist(linear_x=-self.linear_speed)
                    print('↓ backward')
                elif key in ('a', 'A'):
                    self.publish_twist(linear_y=self.linear_speed)
                    print('← left')
                elif key in ('d', 'D'):
                    self.publish_twist(linear_y=-self.linear_speed)
                    print('→ right')
                # Diagonal movement
                elif key in ('q', 'Q'):
                    self.publish_twist(
                        linear_x=self.linear_speed,
                        linear_y=self.linear_speed)
                    print('↖ left forward')
                elif key in ('e', 'E'):
                    self.publish_twist(
                        linear_x=self.linear_speed,
                        linear_y=-self.linear_speed)
                    print('↗ right forward')
                elif key in ('z', 'Z'):
                    self.publish_twist(
                        linear_x=-self.linear_speed,
                        linear_y=self.linear_speed)
                    print('↙ left backward')
                elif key in ('c', 'C'):
                    self.publish_twist(
                        linear_x=-self.linear_speed,
                        linear_y=-self.linear_speed)
                    print('↘ right backward')
                # Rotation
                elif key in ('j', 'J'):
                    self.publish_twist(angular_z=self.angular_speed)
                    print('↺ left rotate')
                elif key in ('l', 'L'):
                    self.publish_twist(angular_z=-self.angular_speed)
                    print('↻ right rotate')
                # Speed adjustment
                elif key in ('+', '='):
                    self.linear_speed += self.speed_increment
                    self.angular_speed += self.speed_increment
                    print(f'faster: {self.linear_speed:.2f} m/s')
                elif key in ('-', '_'):
                    self.linear_speed = max(
                        0.1, self.linear_speed - self.speed_increment)
                    self.angular_speed = max(
                        0.1, self.angular_speed - self.speed_increment)
                    print(f'slower: {self.linear_speed:.2f} m/s')
                # Stopp
                elif key == ' ':
                    self.publish_twist()
                    print('⏹ STOP')
                # Beenden
                elif key in ('\x1b', '\x03'):  # ESC oder Ctrl+C
                    print('\nEnding...')
                    break
                # Hilfe
                elif key in ('h', 'H'):
                    self.print_instructions()

        except Exception as e:
            self.get_logger().error(f'Fehler: {e}')
        finally:
            # Stop robot when ending
            self.publish_twist()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboardNode()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
