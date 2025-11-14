#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pigpio
import socket
import math

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('MotorControllerNode')
        self.get_logger().info('Motor Controller Node gestartet')

        # GPIO Pins für die 4 Motoren (PWM)
        self.MOTOR_FL_PIN = 4   # Front Left
        self.MOTOR_FR_PIN = 5   # Front Right
        self.MOTOR_RL_PIN = 6   # Rear Left
        self.MOTOR_RR_PIN = 7   # Rear Right
        
        # Richtungs-Pins (optional, falls H-Bridge verwendet wird)
        # Können später hinzugefügt werden
        
        self.hostname = socket.gethostname()
        self.pi = None

        # Verbindung zum Raspberry Pi
        if self.hostname == 'rosbot':
            self.pi = pigpio.pi()
            self.get_logger().info("Verbinde mit Raspberry Pi...")
            if not self.pi.connected:
                self.get_logger().error("Verbindung zu pigpio daemon fehlgeschlagen")
                exit()
            
            # PWM Frequenz setzen (z.B. 1000 Hz)
            self.pi.set_PWM_frequency(self.MOTOR_FL_PIN, 1000)
            self.pi.set_PWM_frequency(self.MOTOR_FR_PIN, 1000)
            self.pi.set_PWM_frequency(self.MOTOR_RL_PIN, 1000)
            self.pi.set_PWM_frequency(self.MOTOR_RR_PIN, 1000)
            
            self.get_logger().info("Motoren initialisiert")
        else:
            self.get_logger().info("Simulationsmodus - kein Raspberry Pi erkannt")

        # Twist-Subscriber für Bewegungsbefehle
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Aktuelle Geschwindigkeiten
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        
        # Motor-Geschwindigkeiten (0-255)
        self.motor_fl = 0
        self.motor_fr = 0
        self.motor_rl = 0
        self.motor_rr = 0
        
        # Roboter-Parameter (können angepasst werden)
        self.wheel_base = 0.2  # Abstand zwischen Vorder- und Hinterachse (m)
        self.track_width = 0.2  # Abstand zwischen linken und rechten Rädern (m)
        self.max_speed = 1.0  # Maximale Geschwindigkeit (m/s)

    def cmd_vel_callback(self, msg):
        """Callback für eingehende Twist-Nachrichten"""
        self.linear_x = msg.linear.x
        self.linear_y = msg.linear.y
        self.angular_z = msg.angular.z
        
        # Berechne Motor-Geschwindigkeiten für Omniwheel-Kinematik
        self.calculate_motor_speeds()
        
        # Setze PWM-Werte
        self.set_motor_pwm()
        
        if self.hostname != 'rosbot':
            self.get_logger().info(
                f"Simulation - vx: {self.linear_x:.2f}, vy: {self.linear_y:.2f}, "
                f"w: {self.angular_z:.2f} | Motors: FL={self.motor_fl}, "
                f"FR={self.motor_fr}, RL={self.motor_rl}, RR={self.motor_rr}"
            )

    def calculate_motor_speeds(self):
        """
        Berechnet die Geschwindigkeiten für jeden Motor basierend auf
        Omniwheel-Kinematik (Mecanum/Omni-Räder)
        
        Für einen 4-Rad Omniwheel-Roboter:
        - Front Left:  vx - vy - w
        - Front Right: vx + vy + w
        - Rear Left:   vx + vy - w
        - Rear Right:  vx - vy + w
        """
        # Normalisiere Eingaben
        vx = self.linear_x / self.max_speed
        vy = self.linear_y / self.max_speed
        w = self.angular_z
        
        # Berechne Rad-Geschwindigkeiten
        fl = vx - vy - w * (self.wheel_base + self.track_width) / 2
        fr = vx + vy + w * (self.wheel_base + self.track_width) / 2
        rl = vx + vy - w * (self.wheel_base + self.track_width) / 2
        rr = vx - vy + w * (self.wheel_base + self.track_width) / 2
        
        # Finde maximalen Wert für Normalisierung
        max_val = max(abs(fl), abs(fr), abs(rl), abs(rr))
        if max_val > 1.0:
            fl /= max_val
            fr /= max_val
            rl /= max_val
            rr /= max_val
        
        # Konvertiere zu PWM-Werten (0-255)
        self.motor_fl = int(abs(fl) * 255)
        self.motor_fr = int(abs(fr) * 255)
        self.motor_rl = int(abs(rl) * 255)
        self.motor_rr = int(abs(rr) * 255)
        
        # TODO: Richtung berücksichtigen (Vorzeichen von fl, fr, rl, rr)
        # Dies erfordert zusätzliche GPIO-Pins für die Richtungssteuerung

    def set_motor_pwm(self):
        """Setzt die PWM-Werte für alle Motoren"""
        if self.hostname == 'rosbot' and self.pi:
            self.pi.set_PWM_dutycycle(self.MOTOR_FL_PIN, self.motor_fl)
            self.pi.set_PWM_dutycycle(self.MOTOR_FR_PIN, self.motor_fr)
            self.pi.set_PWM_dutycycle(self.MOTOR_RL_PIN, self.motor_rl)
            self.pi.set_PWM_dutycycle(self.MOTOR_RR_PIN, self.motor_rr)

    def stop_motors(self):
        """Stoppt alle Motoren"""
        self.motor_fl = 0
        self.motor_fr = 0
        self.motor_rl = 0
        self.motor_rr = 0
        self.set_motor_pwm()
        self.get_logger().info("Alle Motoren gestoppt")

    def destroy_node(self):
        """Cleanup beim Beenden"""
        if self.hostname == 'rosbot' and self.pi:
            self.get_logger().info("Stoppe Motoren und beende...")
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
