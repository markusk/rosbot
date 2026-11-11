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
| GPIO LED test | `LEDPWMNode` is retained as a standalone hardware smoke test and is not part of normal robot control. |
| Empty heading driver and print-only keyboard listener | No independent functionality to migrate. |

Motor GPIO access remains restricted to the host whose hostname is exactly `rosbot`; other hosts use simulation/no hardware access. The joystick device node is for Linux with a connected gamepad. On a Mac, publish test velocity commands to `/keyboard_cmd_vel` or `/joystick_cmd_vel`; do not enable the Linux joystick device node without a supported Linux device.

## Raspberry Pi camera

ROS 2 supports the Raspberry Pi camera through the external `camera_ros` driver for libcamera-supported devices. It publishes `sensor_msgs/msg/Image` and `sensor_msgs/msg/CameraInfo`; this project does not contain a custom camera implementation. The actual camera module and Raspberry Pi OS/libcamera compatibility are not yet known and must be checked on the robot.

The main launch includes the camera node only on hostname `rosbot`, and it is disabled by default. Install the `camera_ros` ROS package and enable it with:

```bash
ros2 launch rosbot rosbot_launch.py camera_enabled:=true
```

The driver selects its default camera and stream settings when no camera parameters are supplied. View the stream with a ROS image viewer, for example `ros2 run image_view image_view --ros-args --remap image:=/camera/image_raw`. Camera calibration and exact topic names should be verified on the installed driver before using images for geometry or navigation.

## Hokuyo URG-LX04-UG01

The Hokuyo connects to the Raspberry Pi over USB serial. ROS 2 support uses `urg_node`, which publishes `sensor_msgs/msg/LaserScan`. The launch includes this node only on hostname `rosbot` and keeps it disabled by default. The initial defaults are `/dev/ttyACM0`, `115200` baud, and frame `laser`; confirm the actual port and baud on the robot. Prefer the stable device path under `/dev/serial/by-id/` when available.

Enable the scanner after connecting it:

```bash
ros2 launch rosbot rosbot_launch.py laser_enabled:=true
```

Override the serial device, baud rate, or scan frame if required:

```bash
ros2 launch rosbot rosbot_launch.py laser_enabled:=true laser_device:=/dev/serial/by-id/your-hokuyo laser_baud:=115200 laser_frame_id:=laser
```

The correct fixed transform from the robot base frame to `laser` is still required for RViz/navigation and must be measured from the final mounting position. The exact URG-LX04-UG01 serial baud and device enumeration remain to be confirmed on the Pi.

## L298N motor hardware

`MotorControllerNode` uses pigpio PWM on the L298N enable pins and digital outputs for direction. GPIO values below are BCM GPIO numbers; the ROS 2 wheel kinematics are unchanged.

| Wheel | IN A | IN B | Enable/PWM |
| --- | ---: | ---: | ---: |
| Front left | 5 | 6 | 12 |
| Front right | 20 | 21 | 13 |
| Rear left | 23 | 24 | 18 |
| Rear right | 16 | 26 | 19 |

Remove the ENA/ENB jumpers on both L298N boards when using GPIO-controlled PWM. GPIO 18 is allocated to the rear-left motor enable. `LEDPWMNode` is only a standalone hardware test and must not run while the motor controller is using this pin. The main launch does not start the LED node.

The current wheel mixing assumes the existing four-wheel omniwheel/Mecanum-X arrangement. Confirm the actual wheel type, motor order, polarity, and wiring before driving. Test first with the wheels raised and low velocity commands; ROS 2 build and physical motor behavior still need validation on the Raspberry Pi.

## Nodes

### 1. MotorControllerNode
Steuert 4 Motoren über PWM für Omniwheel-Kinematik.

**BCM GPIO-Pins:** See the L298N motor pin table in the `L298N motor hardware` section below.

**Subscribed Topics:**
- `/cmd_vel` (geometry_msgs/Twist): Bewegungsbefehle

The node stops the motors if active velocity commands stop arriving. The `cmd_vel_timeout` parameter defaults to `0.5` seconds and must be greater than zero. Override it at launch, for example:

```bash
ros2 launch rosbot rosbot_launch.py cmd_vel_timeout:=0.75
```

This watchdog is a software fallback, not a replacement for a physical emergency stop.

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
Standalone ROS 2 GPIO smoke test for the LED PWM hardware. It is deliberately excluded from normal robot control. Run only when the motor controller is stopped because its test GPIO 18 is assigned to the rear-left L298N enable input.

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

On a Mac, the launch file does not start the camera node or access GPIO hardware. The Linux `joy_node` requires a supported Linux joystick device; velocity-command simulation can be tested using `/keyboard_cmd_vel` or `/joystick_cmd_vel`.

Pressing `ESC` in the keyboard teleoperation terminal stops the robot and shuts down the entire launch, including the motor, joystick, and arbitration nodes. `Ctrl+C` in the launch terminal also shuts down all launched nodes.

### Einzelne Nodes starten

**Motor Controller:**
```bash
ros2 run rosbot MotorControllerNode
```

**Tastatursteuerung:**
```bash
ros2 run rosbot TeleopKeyboardNode
```

**LED PWM hardware smoke test (run with the motor controller stopped):**
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
Die L298N BCM GPIO-Zuordnung kann in `MotorControllerNode.py` in der `self.motors`-Liste angepasst werden. ENA/ENB sind Enable/PWM-Eingänge; Jumper müssen dafür entfernt sein. Der separate LED-Hardwaretest ist kein Teil dieser Pin-Zuordnung und darf nicht gleichzeitig mit dem Motorcontroller laufen.

### Roboter-Parameter anpassen
In `MotorControllerNode.py`:
```python
self.wheel_base = 0.2      # Abstand Vorder-/Hinterachse (m)
self.track_width = 0.2     # Abstand links/rechts (m)
self.max_speed = 1.0       # Maximale Geschwindigkeit (m/s)
```

## Nächste Schritte

### TODO:
1. **Encoder-Feedback**: Geschwindigkeitsregelung mit Encodern
2. **Odometrie**: Position und Orientierung berechnen
3. **PID-Regler**: Präzise Geschwindigkeitsregelung

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
