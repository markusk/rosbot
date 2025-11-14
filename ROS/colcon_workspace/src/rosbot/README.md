# ROS2 Rosbot - Omniwheel Roboter

## Übersicht
Dieses Paket enthält ROS2-Nodes für einen Omniwheel-Roboter mit 4 Motoren auf einem Raspberry Pi.

## Nodes

### 1. MotorControllerNode
Steuert 4 Motoren über PWM für Omniwheel-Kinematik.

**GPIO-Pins:**
- GPIO 4: Front Left Motor
- GPIO 5: Front Right Motor
- GPIO 6: Rear Left Motor
- GPIO 7: Rear Right Motor

**Subscribed Topics:**
- `/cmd_vel` (geometry_msgs/Twist): Bewegungsbefehle

### 2. TeleopKeyboardNode
Tastatursteuerung für den Roboter.

**Tasten:**
- W/A/S/D: Vorwärts/Links/Rückwärts/Rechts
- Q/E/Z/C: Diagonale Bewegungen
- J/L: Links/Rechts drehen
- +/-: Geschwindigkeit erhöhen/verringern
- SPACE: Stopp
- ESC: Beenden

**Published Topics:**
- `/cmd_vel` (geometry_msgs/Twist): Bewegungsbefehle

### 3. LEDPWMNode
Beispiel-Node für LED-Steuerung (GPIO 18).

## Installation

### Voraussetzungen
```bash
# pigpio installieren (auf dem Raspberry Pi)
sudo apt-get install pigpio python3-pigpio

# pigpio daemon starten
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
```

### Build
```bash
cd ~/ROS/colcon_workspace
colcon build --packages-select rosbot
source install/setup.bash
```

## Verwendung

### Alle Nodes starten (mit Launch-Datei)
```bash
ros2 launch rosbot rosbot_launch.py
```

### Einzelne Nodes starten

**Motor Controller:**
```bash
ros2 run rosbot MotorControllerNode
```

**Tastatursteuerung:**
```bash
ros2 run rosbot TeleopKeyboardNode
```

**LED Node:**
```bash
ros2 run rosbot LEDPWMNode
```

### Manuell Bewegungsbefehle senden
```bash
# Vorwärts fahren
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Seitwärts fahren (Omniwheel)
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Drehen
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
```

## Konfiguration

### GPIO-Pins anpassen
Die GPIO-Pins können in `MotorControllerNode.py` angepasst werden:
```python
self.MOTOR_FL_PIN = 4   # Front Left
self.MOTOR_FR_PIN = 5   # Front Right
self.MOTOR_RL_PIN = 6   # Rear Left
self.MOTOR_RR_PIN = 7   # Rear Right
```

### Roboter-Parameter anpassen
In `MotorControllerNode.py`:
```python
self.wheel_base = 0.2      # Abstand Vorder-/Hinterachse (m)
self.track_width = 0.2     # Abstand links/rechts (m)
self.max_speed = 1.0       # Maximale Geschwindigkeit (m/s)
```

## Nächste Schritte

### TODO:
1. **Richtungssteuerung hinzufügen**: Zusätzliche GPIO-Pins für Motorrichtung (H-Bridge)
2. **Joystick-Steuerung**: Node für Joystick-Input erstellen
3. **Encoder-Feedback**: Geschwindigkeitsregelung mit Encodern
4. **Odometrie**: Position und Orientierung berechnen
5. **PID-Regler**: Präzise Geschwindigkeitsregelung

### Joystick-Steuerung (geplant)
```bash
# Joystick-Node wird später hinzugefügt
ros2 run rosbot TeleopJoystickNode
```

## Troubleshooting

### pigpio Fehler
```bash
# Prüfen ob pigpiod läuft
sudo systemctl status pigpiod

# Neu starten
sudo systemctl restart pigpiod
```

### Permissions
```bash
# User zur gpio Gruppe hinzufügen
sudo usermod -a -G gpio $USER
```

## Lizenz
MIT
