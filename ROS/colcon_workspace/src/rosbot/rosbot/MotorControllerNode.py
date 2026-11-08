#!/usr/bin/env python3
import socket

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


FRONT_LEFT_IN_A_GPIO_BCM = 5  # Front-left L298N direction input A
FRONT_LEFT_IN_B_GPIO_BCM = 6  # Front-left L298N direction input B
FRONT_LEFT_ENABLE_GPIO_BCM = 12  # Front-left L298N enable/PWM input

FRONT_RIGHT_IN_A_GPIO_BCM = 20  # Front-right L298N direction input A
FRONT_RIGHT_IN_B_GPIO_BCM = 21  # Front-right L298N direction input B
FRONT_RIGHT_ENABLE_GPIO_BCM = 13  # Front-right L298N enable/PWM input

REAR_LEFT_IN_A_GPIO_BCM = 23  # Rear-left L298N direction input A
REAR_LEFT_IN_B_GPIO_BCM = 24  # Rear-left L298N direction input B
REAR_LEFT_ENABLE_GPIO_BCM = 18  # Rear-left L298N enable/PWM input

REAR_RIGHT_IN_A_GPIO_BCM = 16  # Rear-right L298N direction input A
REAR_RIGHT_IN_B_GPIO_BCM = 26  # Rear-right L298N direction input B
REAR_RIGHT_ENABLE_GPIO_BCM = 19  # Rear-right L298N enable/PWM input

MOTOR_PIN_CONFIG = (
    (
        'front_left',
        FRONT_LEFT_IN_A_GPIO_BCM,
        FRONT_LEFT_IN_B_GPIO_BCM,
        FRONT_LEFT_ENABLE_GPIO_BCM,
    ),
    (
        'front_right',
        FRONT_RIGHT_IN_A_GPIO_BCM,
        FRONT_RIGHT_IN_B_GPIO_BCM,
        FRONT_RIGHT_ENABLE_GPIO_BCM,
    ),
    (
        'rear_left',
        REAR_LEFT_IN_A_GPIO_BCM,
        REAR_LEFT_IN_B_GPIO_BCM,
        REAR_LEFT_ENABLE_GPIO_BCM,
    ),
    (
        'rear_right',
        REAR_RIGHT_IN_A_GPIO_BCM,
        REAR_RIGHT_IN_B_GPIO_BCM,
        REAR_RIGHT_ENABLE_GPIO_BCM,
    ),
)


class MotorChannel:
    """Control one L298N channel with direction pins and enable PWM."""

    PWM_RANGE = 255
    MINIMUM_DUTY = 0.02

    def __init__(self, pi, in_a, in_b, enable, output_mode, pwm_frequency=1000):
        self.pi = pi
        self.in_a = in_a
        self.in_b = in_b
        self.enable = enable

        for pin in (self.in_a, self.in_b, self.enable):
            self.pi.set_mode(pin, output_mode)
            self.pi.write(pin, 0)

        self.pi.set_PWM_range(self.enable, self.PWM_RANGE)
        self.pi.set_PWM_frequency(self.enable, pwm_frequency)
        self.pi.set_PWM_dutycycle(self.enable, 0)

    def set_speed(self, value):
        value = max(-1.0, min(1.0, value))
        if abs(value) < self.MINIMUM_DUTY:
            self.stop()
            return

        self.pi.set_PWM_dutycycle(self.enable, 0)
        if value > 0:
            self.pi.write(self.in_a, 1)
            self.pi.write(self.in_b, 0)
        else:
            self.pi.write(self.in_a, 0)
            self.pi.write(self.in_b, 1)

        duty = round(abs(value) * self.PWM_RANGE)
        self.pi.set_PWM_dutycycle(self.enable, duty)

    def stop(self):
        self.pi.set_PWM_dutycycle(self.enable, 0)
        self.pi.write(self.in_a, 0)
        self.pi.write(self.in_b, 0)


class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('MotorControllerNode')
        self.get_logger().info('Motor controller node started')

        self.hostname = socket.gethostname()
        self.pi = None
        self.motors = []

        if self.hostname == 'rosbot':
            import pigpio

            self.get_logger().info('Connecting to pigpio daemon...')
            self.pi = pigpio.pi()
            if not self.pi.connected:
                self.pi.stop()
                raise RuntimeError(
                    'Cannot connect to pigpiod. Start the daemon and verify '
                    'the installation before launching the motor controller.')

            self.motors = [
                MotorChannel(
                    self.pi,
                    in_a=in_a,
                    in_b=in_b,
                    enable=enable,
                    output_mode=pigpio.OUTPUT,
                )
                for _, in_a, in_b, enable in MOTOR_PIN_CONFIG
            ]
            self.get_logger().info('L298N motor channels initialised')

        else:
            msg = """
            ╔═══════════════════════════════╗
            ║       SIMULATING ROSBOT       ║
            ╠═══════════════════════════════╣
            ║   No Raspberry Pi detected.   ║
            ╚═══════════════════════════════╝
            """
            self.get_logger().info(msg)
#                'Simulation mode - no Raspberry Pi detected')

        # twist subscriber for move commands
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

        # current speed
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0

        self.wheel_speeds = [0.0, 0.0, 0.0, 0.0]

        # robot parameters (can be adjusted based on the actual robot dimensions)
        self.wheel_base = 0.2   # Distance between front and rear axles (m)
        self.track_width = 0.2  # Distance between left and right wheels (m)
        self.max_speed = 1.0    # Maximum speed (m/s)

    def cmd_vel_callback(self, msg):
        self.linear_x = msg.linear.x
        self.linear_y = msg.linear.y
        self.angular_z = msg.angular.z

        # calculating wheel speeds for omniwheel kinematics
        self.calculate_motor_speeds()

        # Set PWM values
        self.set_motor_pwm()

        if self.hostname != 'rosbot':
            self.get_logger().info(
                f'Simulation - vx: {self.linear_x:.2f}, '
                f'vy: {self.linear_y:.2f}, '
                f'w: {self.angular_z:.2f} | Wheel commands: '
                f'FL={self.wheel_speeds[0]:.2f}, '
                f'FR={self.wheel_speeds[1]:.2f}, '
                f'RL={self.wheel_speeds[2]:.2f}, '
                f'RR={self.wheel_speeds[3]:.2f}')

    def calculate_motor_speeds(self):
        """
        Calculate wheel speeds using four-wheel omniwheel kinematics.

        The wheel speeds are calculated as follows:
        - Front Left: vx - vy - w
        - Front Right: vx + vy + w
        - Rear Left: vx + vy - w
        - Rear Right: vx - vy + w
        """
        # normalising inputs
        vx = self.linear_x / self.max_speed
        vy = self.linear_y / self.max_speed
        w = self.angular_z

        # calculating wheel speeds
        fl = vx - vy - w * (self.wheel_base + self.track_width) / 2
        fr = vx + vy + w * (self.wheel_base + self.track_width) / 2
        rl = vx + vy - w * (self.wheel_base + self.track_width) / 2
        rr = vx - vy + w * (self.wheel_base + self.track_width) / 2

        # find maximal value for normalisation
        max_val = max(abs(fl), abs(fr), abs(rl), abs(rr))
        if max_val > 1.0:
            fl /= max_val
            fr /= max_val
            rl /= max_val
            rr /= max_val

        self.wheel_speeds = [fl, fr, rl, rr]

    def set_motor_pwm(self):
        """Apply signed wheel commands to L298N direction and PWM channels."""
        if self.hostname == 'rosbot' and self.pi:
            for motor, speed in zip(self.motors, self.wheel_speeds):
                motor.set_speed(speed)

    def stop_all(self):
        """Stop every motor channel."""
        for motor in self.motors:
            motor.stop()
        self.wheel_speeds = [0.0, 0.0, 0.0, 0.0]

    def stop_motors(self):
        """Stop all motors."""
        self.stop_all()
        self.get_logger().info('All motors stopped')

    def destroy_node(self):
        """Clean up motor channels and pigpio connection."""
        if self.hostname == 'rosbot' and self.pi:
            self.get_logger().info('Stopping motors and shutting down...')
            self.stop_all()
            self.pi.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
