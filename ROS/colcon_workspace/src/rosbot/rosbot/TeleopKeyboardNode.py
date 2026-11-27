#!/usr/bin/env python3
import os
import select
import sys
import termios
import time
import tty

from evdev import InputDevice, ecodes, list_devices
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

from rosbot.TeleopSpeedRamp import key_repeat_timed_out, speed_scale


class TeleopKeyboardNode(Node):
    def __init__(self):
        super().__init__('TeleopKeyboardNode')
        self.get_logger().info('Keyboard teleoperation node started')

        self.publisher = self.create_publisher(
            Twist, 'keyboard_cmd_vel', 10)

        self.declare_parameter('input_mode', 'auto')
        self.declare_parameter('input_device', '')
        self.declare_parameter('key_repeat_timeout', 0.3)
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('start_speed_scale', 0.2)
        self.declare_parameter('end_speed_scale', 1.0)
        self.declare_parameter('acceleration_duration', 2.0)
        self.declare_parameter('speed_increment', 0.1)

        self.input_mode = str(self.get_parameter('input_mode').value)
        self.input_device_path = str(
            self.get_parameter('input_device').value)
        self.key_repeat_timeout = float(
            self.get_parameter('key_repeat_timeout').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(
            self.get_parameter('angular_speed').value)
        self.start_speed_scale = float(
            self.get_parameter('start_speed_scale').value)
        self.end_speed_scale = float(
            self.get_parameter('end_speed_scale').value)
        self.acceleration_duration = float(
            self.get_parameter('acceleration_duration').value)
        self.speed_increment = float(
            self.get_parameter('speed_increment').value)

        if self.input_mode not in ('auto', 'evdev', 'terminal'):
            raise ValueError(
                "input_mode must be 'auto', 'evdev', or 'terminal'")
        if self.key_repeat_timeout <= 0.0:
            raise ValueError('key_repeat_timeout must be greater than zero')
        if not (
            0.0 <= self.start_speed_scale
            <= self.end_speed_scale
            <= 1.0
        ):
            raise ValueError(
                'Speed scales must satisfy 0 <= start <= end <= 1')
        if self.linear_speed <= 0.0 or self.angular_speed <= 0.0:
            raise ValueError('Linear and angular speeds must be positive')
        if self.acceleration_duration <= 0.0 or self.speed_increment <= 0.0:
            raise ValueError(
                'Acceleration duration and speed increment must be positive')

        self.input_device = None
        self.terminal_input = None
        self.terminal_settings = None
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular_z = 0.0
        self.active_command = None
        self.active_key = None
        self.command_started_at = None
        self.last_key_input_at = None
        self.current_speed_scale = 0.0
        self.ctrl_pressed = False
        self.shift_pressed = False

        self.print_instructions()

    def print_instructions(self):
        """Print the keyboard control instructions."""
        msg = """
        ╔════════════════════════════════════╗
        ║   ROSBOT KEYBOARD TELEOPERATION    ║
        ╠════════════════════════════════════╣
        ║              W   (forward)         ║
        ║              ↑                     ║
        ║   Left   A ← + → D   Right         ║
        ║              ↓                     ║
        ║              S   (backward)        ║
        ║                                    ║
        ║   Rotation:                        ║
        ║   J: turn left    L: turn right    ║
        ║                                    ║
        ║   Q: left forward diagonal         ║
        ║   E: right forward diagonal        ║
        ║   Y: left backward diagonal        ║
        ║   C: right backward diagonal       ║
        ║                                    ║
        ║   Speed:                           ║
        ║   +: faster       -: slower        ║
        ║                                    ║
        ║   SPACE: Stop     ESC/Ctrl+C: End  ║
        ╚════════════════════════════════════╝
        """
        print(msg)
        self.print_status()

    def print_status(self):
        """Print the current speed and movement status."""
        print(
            f'\nSpeed: linear={self.linear_speed:.2f} m/s, '
            f'angular={self.angular_speed:.2f} rad/s')
        print(
            f'Current: vx={self.current_linear_x:.2f}, '
            f'vy={self.current_linear_y:.2f}, '
            f'w={self.current_angular_z:.2f}')

    def open_input_device(self):
        """Open the configured keyboard or find a usable keyboard device."""
        if self.input_device_path:
            self.input_device = InputDevice(self.input_device_path)
            self.get_logger().info(
                f'Using configured keyboard: '
                f'{self.input_device.path} ({self.input_device.name})')
            return

        for device_path in list_devices():
            try:
                device = InputDevice(device_path)
                capabilities = device.capabilities().get(ecodes.EV_KEY, [])
            except OSError as error:
                self.get_logger().debug(
                    f'Cannot inspect input device {device_path}: {error}')
                continue

            key_codes = {
                code if isinstance(code, int) else ecodes.ecodes.get(code)
                for code in capabilities
            }
            required_keys = {
                ecodes.KEY_W,
                ecodes.KEY_A,
                ecodes.KEY_S,
                ecodes.KEY_D,
            }
            if required_keys.issubset(key_codes):
                self.input_device = device
                self.get_logger().info(
                    'Using keyboard input device: '
                    f'{device.path} ({device.name})')
                return
            device.close()

        raise RuntimeError(
            'No suitable keyboard found; set the input_device ROS parameter')

    def start_motion(self, key_code):
        """Start a keyboard command associated with a key code."""
        motions = {
            ecodes.KEY_W: (-1.0, 0.0, 0.0, 'forward'),
            ecodes.KEY_S: (1.0, 0.0, 0.0, 'backward'),
            ecodes.KEY_A: (0.0, -1.0, 0.0, 'left'),
            ecodes.KEY_D: (0.0, 1.0, 0.0, 'right'),
            ecodes.KEY_Q: (-1.0, -1.0, 0.0, 'left forward'),
            ecodes.KEY_E: (-1.0, 1.0, 0.0, 'right forward'),
            ecodes.KEY_Z: (1.0, -1.0, 0.0, 'left backward'),
            ecodes.KEY_C: (1.0, 1.0, 0.0, 'right backward'),
            ecodes.KEY_J: (0.0, 0.0, -1.0, 'left rotation'),
            ecodes.KEY_L: (0.0, 0.0, 1.0, 'right rotation'),
        }
        motion = motions.get(key_code)
        if motion is None:
            return False

        now = time.monotonic()
        if self.active_key != key_code:
            self.active_key = key_code
            self.active_command = motion[:3]
            self.command_started_at = now
            self.current_speed_scale = self.start_speed_scale
            print(motion[3])
        self.last_key_input_at = now
        return True

    def publish_active_command(self):
        """Publish the active command with its current acceleration scale."""
        elapsed = time.monotonic() - self.command_started_at
        self.current_speed_scale = speed_scale(
            self.start_speed_scale,
            self.end_speed_scale,
            self.acceleration_duration,
            elapsed,
        )
        linear_x, linear_y, angular_z = self.active_command
        self.publish_twist(
            linear_x=linear_x * self.linear_speed * self.current_speed_scale,
            linear_y=linear_y * self.linear_speed * self.current_speed_scale,
            angular_z=(
                angular_z * self.angular_speed * self.current_speed_scale),
        )

    def stop_motion(self):
        """Stop publishing motion and reset the acceleration ramp."""
        self.publish_twist()
        self.active_key = None
        self.active_command = None
        self.command_started_at = None
        self.last_key_input_at = None
        self.current_speed_scale = 0.0
        print('STOP')

    def handle_key_event(self, event):
        """Handle key-down and key-up events from an evdev device."""
        if event.type != ecodes.EV_KEY:
            return True

        if event.code in (ecodes.KEY_LEFTCTRL, ecodes.KEY_RIGHTCTRL):
            self.ctrl_pressed = event.value != 0
            return True
        if event.code in (ecodes.KEY_LEFTSHIFT, ecodes.KEY_RIGHTSHIFT):
            self.shift_pressed = event.value != 0
            return True
        if event.code == ecodes.KEY_ESC and event.value == 1:
            return False
        if (
            event.code == ecodes.KEY_C
            and event.value == 1
            and self.ctrl_pressed
        ):
            return False

        if event.value == 0:
            if event.code == self.active_key:
                self.stop_motion()
            return True
        if event.value != 1:
            return True

        if event.code == ecodes.KEY_SPACE:
            self.stop_motion()
        elif event.code == ecodes.KEY_EQUAL and self.shift_pressed:
            self.increase_speed()
        elif event.code == ecodes.KEY_MINUS and not self.shift_pressed:
            self.decrease_speed()
        else:
            self.start_motion(event.code)
        return True

    def handle_terminal_character(self, character):
        """Handle a character read from a local or remote terminal."""
        if character in ('\x1b', '\x03'):
            return False
        if character == ' ':
            self.stop_motion()
        elif character in ('+', '='):
            self.increase_speed()
        elif character in ('-', '_'):
            self.decrease_speed()
        else:
            terminal_keys = {
                'w': ecodes.KEY_W,
                'a': ecodes.KEY_A,
                's': ecodes.KEY_S,
                'd': ecodes.KEY_D,
                'q': ecodes.KEY_Q,
                'e': ecodes.KEY_E,
                'y': ecodes.KEY_Z,
                'c': ecodes.KEY_C,
                'j': ecodes.KEY_J,
                'l': ecodes.KEY_L,
            }
            key_code = terminal_keys.get(character.lower())
            if key_code is not None:
                self.start_motion(key_code)
        return True

    def increase_speed(self):
        """Increase the configured linear and angular speed limits."""
        self.linear_speed += self.speed_increment
        self.angular_speed += self.speed_increment
        print(f'Faster: {self.linear_speed:.2f} m/s')

    def decrease_speed(self):
        """Decrease the configured linear and angular speed limits."""
        self.linear_speed = max(
            0.1, self.linear_speed - self.speed_increment)
        self.angular_speed = max(
            0.1, self.angular_speed - self.speed_increment)
        print(f'Slower: {self.linear_speed:.2f} m/s')

    def resolve_input_mode(self):
        """Choose evdev locally or terminal input in SSH sessions."""
        if self.input_mode != 'auto':
            return self.input_mode
        if any(os.environ.get(name) for name in (
            'SSH_CONNECTION', 'SSH_TTY', 'SSH_CLIENT'
        )):
            return 'terminal'
        try:
            self.open_input_device()
            return 'evdev'
        except (OSError, RuntimeError) as error:
            if sys.stdin.isatty():
                self.get_logger().warning(
                    f'Cannot use evdev keyboard ({error}); '
                    'falling back to terminal input')
                return 'terminal'
            raise RuntimeError(
                f'Cannot use evdev keyboard ({error}) and stdin is not a TTY')

    def run_evdev(self):
        """Read key-down/up events and publish until input ends."""
        if self.input_device is None:
            self.open_input_device()
        self.get_logger().info('Keyboard input mode: evdev')
        while rclpy.ok():
            readable, _, _ = select.select(
                [self.input_device.fd], [], [], 0.02)
            if readable:
                for event in self.input_device.read():
                    if not self.handle_key_event(event):
                        return
            if self.active_command is not None:
                self.publish_active_command()

    def run_terminal(self):
        """Read terminal characters and stop when key repeats time out."""
        try:
            self.terminal_input = open('/dev/tty', 'rb', buffering=0)
        except OSError:
            if not sys.stdin.isatty():
                raise RuntimeError(
                    'No controlling terminal available for keyboard input')
            self.terminal_input = sys.stdin.buffer

        fd = self.terminal_input.fileno()
        self.terminal_settings = termios.tcgetattr(fd)
        tty.setraw(fd)
        self.get_logger().info(
            'Keyboard input mode: terminal '
            f'(stop timeout {self.key_repeat_timeout:.2f} seconds)')
        try:
            while rclpy.ok():
                readable, _, _ = select.select([fd], [], [], 0.02)
                if readable:
                    character = os.read(fd, 1).decode(
                        'utf-8', errors='ignore')
                    if not self.handle_terminal_character(character):
                        return
                if self.active_command is not None:
                    if key_repeat_timed_out(
                        self.last_key_input_at,
                        time.monotonic(),
                        self.key_repeat_timeout,
                    ):
                        self.stop_motion()
                    else:
                        self.publish_active_command()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, self.terminal_settings)
            self.terminal_settings = None
            if self.terminal_input is not sys.stdin.buffer:
                self.terminal_input.close()
            self.terminal_input = None

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
        """Select keyboard input mode and publish commands until stopped."""
        try:
            mode = self.resolve_input_mode()
            if mode == 'evdev':
                self.run_evdev()
            else:
                self.run_terminal()
        except Exception as error:
            self.get_logger().error(
                f'Keyboard teleoperation failed: {error}')
        finally:
            self.publish_twist()
            if self.terminal_settings is not None and self.terminal_input:
                termios.tcsetattr(
                    self.terminal_input.fileno(),
                    termios.TCSADRAIN,
                    self.terminal_settings,
                )
            if self.terminal_input is not None:
                self.terminal_input.close()
            if self.input_device is not None:
                self.input_device.close()


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
