#!/usr/bin/env python3
import socket

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('MotorControllerNode')
        self.get_logger().info('Motor controller node started')

        # GPIO pins for the 4 motors (PWM)
        self.MOTOR_FL_PIN = 4 # front left
        self.MOTOR_FR_PIN = 5 # front right
        self.MOTOR_RL_PIN = 6 # rear left
        self.MOTOR_RR_PIN = 7 # rear right

        # direction pins (optional, if using a H-Bridge)
        # Can be added later

        self.hostname = socket.gethostname()
        self.pi = None

        # Connection to Raspberry Pi
        if self.hostname == 'rosbot':
            import pigpio

            self.pi = pigpio.pi()
            self.get_logger().info('Connecting to Raspberry Pi...')
            if not self.pi.connected:
                self.get_logger().error(
                    'ERR: Connection to pigpio daemon failed. Please ensure pigpio is running on the Raspberry Pi.')
                exit()

            # set PWM frequency (i.e. 1000 Hz)
            self.pi.set_PWM_frequency(self.MOTOR_FL_PIN, 1000)
            self.pi.set_PWM_frequency(self.MOTOR_FR_PIN, 1000)
            self.pi.set_PWM_frequency(self.MOTOR_RL_PIN, 1000)
            self.pi.set_PWM_frequency(self.MOTOR_RR_PIN, 1000)

            self.get_logger().info('Motors initialised')
        else:
            self.get_logger().info(
                'Simulation mode - no Raspberry Pi detected')

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

        # motor speeds (0-255)
        self.motor_fl = 0
        self.motor_fr = 0
        self.motor_rl = 0
        self.motor_rr = 0

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
                f'w: {self.angular_z:.2f} | Motors: FL={self.motor_fl}, '
                f'FR={self.motor_fr}, RL={self.motor_rl}, RR={self.motor_rr}')

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

        # convert to PWM values (0-255)
        self.motor_fl = int(abs(fl) * 255)
        self.motor_fr = int(abs(fr) * 255)
        self.motor_rl = int(abs(rl) * 255)
        self.motor_rr = int(abs(rr) * 255)

        # TODO: take direction into account (sign of fl, fr, rl, rr)
        # This requires additional GPIO pins for direction control

    def set_motor_pwm(self):
        """Set PWM values for all motors."""
        if self.hostname == 'rosbot' and self.pi:
            self.pi.set_PWM_dutycycle(self.MOTOR_FL_PIN, self.motor_fl)
            self.pi.set_PWM_dutycycle(self.MOTOR_FR_PIN, self.motor_fr)
            self.pi.set_PWM_dutycycle(self.MOTOR_RL_PIN, self.motor_rl)
            self.pi.set_PWM_dutycycle(self.MOTOR_RR_PIN, self.motor_rr)

    def stop_motors(self):
        """Stop all motors."""
        self.motor_fl = 0
        self.motor_fr = 0
        self.motor_rl = 0
        self.motor_rr = 0
        self.set_motor_pwm()
        self.get_logger().info('All motors stopped')

    def destroy_node(self):
        """Clean up hardware resources before shutting down."""
        if self.hostname == 'rosbot' and self.pi:
            self.get_logger().info('Stopping motors and shutting down...')
            self.stop_motors()
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
