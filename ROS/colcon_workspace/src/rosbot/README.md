# ROS2 Rosbot - Omniwheel Roboter

## Übersicht
Dieses Paket enthält ROS2-Nodes für einen Omniwheel-Roboter mit 4 Motoren auf einem Raspberry Pi.

## ROS 2 migration status

The active package is `ROS/colcon_workspace/src/rosbot`. The directory `ROS/rosbot.old__TMP!__migrating_to_ros2_!` is a legacy ROS 1 reference and is not built or installed by colcon. ROS 1 `rospy` nodes, XML launch files, and the custom `Motor.srv` were not copied into the ROS 2 package.

| Legacy functionality | ROS 2 status |
| --- | --- |
| Keyboard control and `cmd_vel` motor control | Existing `TeleopKeyboardNode` and `MotorControllerNode` are the current implementation. |
| Xbox/D-pad joystick input | Migrated to `TeleopJoystickNode`, publishing `cmd_vel` from validated `sensor_msgs/Joy` axes/buttons. |
| ROS 1 motor services | Not migrated: both old service servers are logging/placeholders, and the ROS 2 motor node uses PWM directly. |
| Sphero RVR odometry and battery | Not migrated: legacy odometry is an incomplete prototype and battery code is a standalone SDK script, not a ROS publisher. They require confirmed hardware, SDK, and frame/units behavior before a safe ROS 2 port. |
| GPIO LED test | Covered by the existing `LEDPWMNode`. |
| Empty heading driver and print-only keyboard listener | No independent functionality to migrate. |

GPIO access remains restricted to the host whose hostname is exactly `rosbot`; other hosts use simulation/no hardware access. The joystick device node is likewise only added by the launch file on `rosbot`. On a Mac, joystick commands can be simulated by publishing `sensor_msgs/Joy` to `/joy` or `geometry_msgs/Twist` to `/cmd_vel`.

The existing motor controller currently drives PWM outputs only and discards wheel-command signs. Do not treat reverse/turning hardware behavior as validated until the motor driver's direction wiring and GPIO mapping are confirmed and direction control is implemented.

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

### 4. TeleopJoystickNode
Consumes `sensor_msgs/Joy` on `/joy` and publishes `geometry_msgs/Twist` on `/cmd_vel`. The default mapping supports the legacy controller axes 5/7 for forward/backward, axes 4/6 for turning, and buttons 10/1 for stop. Axis and button indices plus speeds are ROS parameters, and short or differently sized joystick messages are handled without indexing errors.

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

### Nodes starten
```bash
ros2 launch rosbot rosbot_launch.py
```

The joystick is disabled by default. On the Raspberry Pi with a joystick connected, enable it and optionally select another device index:

```bash
ros2 launch rosbot rosbot_launch.py joystick_enabled:=true joy_device_id:=0
```

On a Mac, the launch file intentionally does not start the Linux `joy_node`; run the teleop node and publish test messages to `/joy`, or publish commands directly to `/cmd_vel`.

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
