#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class TeleopKeyboardNode(Node):
    def __init__(self):
        super().__init__('TeleopKeyboardNode')
        self.get_logger().info('Tastatur-Steuerung gestartet')
        
        # Publisher für cmd_vel
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Geschwindigkeitsparameter
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 1.0  # rad/s
        self.speed_increment = 0.1
        
        # Aktuelle Geschwindigkeiten
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular_z = 0.0
        
        self.print_instructions()

    def print_instructions(self):
        """Zeigt die Steuerungsanleitung"""
        msg = """
        ╔════════════════════════════════════════╗
        ║   ROSBOT TASTATUR-STEUERUNG           ║
        ╠════════════════════════════════════════╣
        ║                                        ║
        ║   Bewegung (Omniwheel):                ║
        ║                                        ║
        ║        W (vorwärts)                    ║
        ║        ↑                               ║
        ║   A ← + → D                            ║
        ║        ↓                               ║
        ║        S (rückwärts)                   ║
        ║                                        ║
        ║   Q: links vorwärts diagonal           ║
        ║   E: rechts vorwärts diagonal          ║
        ║   Z: links rückwärts diagonal          ║
        ║   C: rechts rückwärts diagonal         ║
        ║                                        ║
        ║   Rotation:                            ║
        ║   J: links drehen                      ║
        ║   L: rechts drehen                     ║
        ║                                        ║
        ║   Geschwindigkeit:                     ║
        ║   +: schneller                         ║
        ║   -: langsamer                         ║
        ║                                        ║
        ║   SPACE: Stopp                         ║
        ║   ESC/Ctrl+C: Beenden                  ║
        ║                                        ║
        ╚════════════════════════════════════════╝
        """
        print(msg)
        self.print_status()

    def print_status(self):
        """Zeigt den aktuellen Status"""
        print(f"\nGeschwindigkeit: linear={self.linear_speed:.2f} m/s, "
              f"angular={self.angular_speed:.2f} rad/s")
        print(f"Aktuell: vx={self.current_linear_x:.2f}, "
              f"vy={self.current_linear_y:.2f}, w={self.current_angular_z:.2f}")

    def get_key(self):
        """Liest ein Zeichen von der Tastatur"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def publish_twist(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        """Veröffentlicht eine Twist-Nachricht"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        self.publisher.publish(twist)
        
        self.current_linear_x = linear_x
        self.current_linear_y = linear_y
        self.current_angular_z = angular_z

    def run(self):
        """Hauptschleife für Tastatureingaben"""
        try:
            while rclpy.ok():
                key = self.get_key()
                
                # Bewegung
                if key == 'w' or key == 'W':
                    self.publish_twist(linear_x=self.linear_speed)
                    print("↑ Vorwärts")
                elif key == 's' or key == 'S':
                    self.publish_twist(linear_x=-self.linear_speed)
                    print("↓ Rückwärts")
                elif key == 'a' or key == 'A':
                    self.publish_twist(linear_y=self.linear_speed)
                    print("← Links")
                elif key == 'd' or key == 'D':
                    self.publish_twist(linear_y=-self.linear_speed)
                    print("→ Rechts")
                
                # Diagonale Bewegung
                elif key == 'q' or key == 'Q':
                    self.publish_twist(linear_x=self.linear_speed, linear_y=self.linear_speed)
                    print("↖ Links vorwärts")
                elif key == 'e' or key == 'E':
                    self.publish_twist(linear_x=self.linear_speed, linear_y=-self.linear_speed)
                    print("↗ Rechts vorwärts")
                elif key == 'z' or key == 'Z':
                    self.publish_twist(linear_x=-self.linear_speed, linear_y=self.linear_speed)
                    print("↙ Links rückwärts")
                elif key == 'c' or key == 'C':
                    self.publish_twist(linear_x=-self.linear_speed, linear_y=-self.linear_speed)
                    print("↘ Rechts rückwärts")
                
                # Rotation
                elif key == 'j' or key == 'J':
                    self.publish_twist(angular_z=self.angular_speed)
                    print("↺ Links drehen")
                elif key == 'l' or key == 'L':
                    self.publish_twist(angular_z=-self.angular_speed)
                    print("↻ Rechts drehen")
                
                # Geschwindigkeit anpassen
                elif key == '+' or key == '=':
                    self.linear_speed += self.speed_increment
                    self.angular_speed += self.speed_increment
                    print(f"Schneller: {self.linear_speed:.2f} m/s")
                elif key == '-' or key == '_':
                    self.linear_speed = max(0.1, self.linear_speed - self.speed_increment)
                    self.angular_speed = max(0.1, self.angular_speed - self.speed_increment)
                    print(f"Langsamer: {self.linear_speed:.2f} m/s")
                
                # Stopp
                elif key == ' ':
                    self.publish_twist()
                    print("⏹ STOPP")
                
                # Beenden
                elif key == '\x1b' or key == '\x03':  # ESC oder Ctrl+C
                    print("\nBeende...")
                    break
                
                # Hilfe
                elif key == 'h' or key == 'H':
                    self.print_instructions()
                
        except Exception as e:
            self.get_logger().error(f"Fehler: {e}")
        finally:
            # Stoppe Roboter beim Beenden
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
