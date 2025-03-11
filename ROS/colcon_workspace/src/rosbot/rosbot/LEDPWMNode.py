#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pigpio
import time
import socket

class LEDPWMNode(Node):
    def __init__(self):
        super().__init__('LEDPWMNode')
        self.get_logger().info('Hello ROS 2 world!')

        self.LED_PIN = 18  # GPIO Pin for the LED
        self.hostname = socket.gethostname()
        self.pi = None

        if self.hostname == 'rosbot':
            self.pi = pigpio.pi()  # Connecting to pigpio daemon
            self.get_logger().info("Connecting to Raspi...")
            if not self.pi.connected:
                self.get_logger().error("Failed to connect to pigpio daemon")
                exit()
        else:
            self.get_logger().info("Simulating LED behavior")

        self.timer = self.create_timer(0.003, self.pwm_callback)
        self.duty_cycle = 0
        self.increasing = True

    def pwm_callback(self):
        if self.hostname == 'rosbot' and self.pi:
            if self.increasing:
                self.duty_cycle += 1
                if self.duty_cycle >= 255:
                    self.increasing = False
            else:
                self.duty_cycle -= 1
                if self.duty_cycle <= 0:
                    self.increasing = True

            self.pi.set_PWM_dutycycle(self.LED_PIN, self.duty_cycle)
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

            self.get_logger().info(f"Simulated LED duty cycle: {self.duty_cycle}")

    def destroy_node(self):
        if self.hostname == 'rosbot' and self.pi:
            self.get_logger().info("Turning off LED")
            self.pi.set_PWM_dutycycle(self.LED_PIN, 0)
            self.pi.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LEDPWMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
