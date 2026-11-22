#!/usr/bin/env python3
import socket

import rclpy
from rclpy.node import Node


class LEDPWMNode(Node):
    def __init__(self):
        super().__init__('LEDPWMNode')
        self.get_logger().info('Hello ROS 2 world!')

        self.LED_PIN = 18  # GPIO Pin for the LED
        self.hostname = socket.gethostname()
        self.led = None

        if self.hostname == 'rosbot':
            from gpiozero import PWMLED

            self.led = PWMLED(self.LED_PIN, initial_value=0)
        else:
            self.get_logger().info('Simulating LED behavior')

        self.timer = self.create_timer(0.003, self.pwm_callback)
        self.duty_cycle = 0
        self.increasing = True

    def pwm_callback(self):
        if self.hostname == 'rosbot' and self.led:
            if self.increasing:
                self.duty_cycle += 1
                if self.duty_cycle >= 255:
                    self.increasing = False
            else:
                self.duty_cycle -= 1
                if self.duty_cycle <= 0:
                    self.increasing = True

            self.led.value = self.duty_cycle / 255
        else:
            # Simulating the LED PWM behavior
            if self.increasing:
                self.duty_cycle += 1
                if self.duty_cycle >= 255:
                    self.increasing = False
            else:
                self.duty_cycle -= 1
                if self.duty_cycle <= 0:
                    self.increasing = True

            self.get_logger().info(
                f'Simulated LED duty cycle: {self.duty_cycle}')

    def destroy_node(self):
        if self.hostname == 'rosbot' and self.led:
            self.get_logger().info('Turning off LED')
            self.led.off()
            self.led.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LEDPWMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
