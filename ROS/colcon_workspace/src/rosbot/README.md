# ROS 2 Rosbot - Omniwheel Robot

## Overview

This package contains ROS 2 nodes for a four-motor omniwheel robot running on a Raspberry Pi.

## ROS 2 migration status

The active package is `ROS/colcon_workspace/src/rosbot`. The directory `ROS/rosbot.old__TMP!__migrating_to_ros2_!` is a legacy ROS 1 reference and is not built or installed by colcon. ROS 1 `rospy` nodes, XML launch files, and the custom `Motor.srv` were not copied into the ROS 2 package.

| Legacy functionality | ROS 2 status |
| --- | --- |
| Keyboard control and `cmd_vel` motor control | Existing `TeleopKeyboardNode` and `MotorControllerNode` are the current implementation. |
| Xbox/D-pad joystick input | Migrated to `TeleopJoystickNode`, publishing `cmd_vel` from validated `sensor_msgs/Joy` axes/buttons. |
| ROS 1 motor services | Not migrated: both old service servers are logging/placeholders, and the ROS 2 motor node uses PWM directly. |
| Robot odometry and battery | Not migrated: legacy odometry is an incomplete prototype and battery code is a standalone SDK script, not a ROS publisher. They require confirmed hardware, SDK, and frame/units behavior before a safe ROS 2 port. |
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

`MotorControllerNode` uses gpiozero PWM outputs on the L298N enable pins and digital outputs for direction. The central pin assignment table, including BCM GPIO and physical header pin numbers for the Raspberry Pi 5, is in the [global README](../../../../README.md).

Remove the corresponding jumpers on the L298N modules when using PWM on ENA/ENB. GPIO 18 is assigned to the rear-left wheel's enable input. `LEDPWMNode` is a separate hardware test and must not run at the same time as motor control because it uses the same GPIO. The standard launch does not start the LED node.

The current wheel mixing assumes the existing four-wheel omniwheel/Mecanum-X arrangement. Confirm the actual wheel type, motor order, polarity, and wiring before driving. Test first with the wheels raised and low velocity commands. The ROS 2 package has been built and keyboard movement has been verified on the Raspberry Pi 5 robot. Camera, laser, joystick, and watchdog hardware behavior may still need separate validation.

## Nodes

### 1. MotorControllerNode

Controls four motors using PWM for omniwheel kinematics.

**BCM GPIO pins:** See the central Raspberry Pi 5 pin assignment table in the [global README](../../../../README.md).

**Subscribed topics:**
- `/cmd_vel` (`geometry_msgs/Twist`): velocity commands

The node stops the motors if active velocity commands stop arriving. The `cmd_vel_timeout` parameter defaults to `0.5` seconds and must be greater than zero. Override it at launch, for example:

```bash
ros2 launch rosbot rosbot_launch.py cmd_vel_timeout:=0.75
```

This watchdog is a software fallback, not a replacement for a physical emergency stop.

### 2. TeleopKeyboardNode

Keyboard control reads Linux key-down and key-up events from an `evdev` input device, so an interactive terminal is not needed for keyboard input. Hold a movement key to publish commands continuously while they linearly ramp from `start_speed_scale` to `end_speed_scale` over `acceleration_duration`; release it to publish a stop command. On a German QWERTZ keyboard, the physical Y key is handled through Linux key code `KEY_Z`. Use the space bar for an immediate stop and ESC or Ctrl+C to exit.

The default ramp starts at 20% of the configured maximum speed, reaches 100% after 2 seconds, and applies to translation and rotation. These values are ROS parameters loaded from `config/teleop.yaml` by the standard launch file.

**Keys:**
- W/A/S/D: forward/left/backward/right
- Q/E/Y/C: diagonal movement (German QWERTZ layout; hold to move, release to stop)
- J/L: rotate left/right
- +/-: increase/decrease speed
- SPACE: stop
- ESC: exit

**Published topics:**
- `/cmd_vel` (`geometry_msgs/Twist`): velocity commands

### 3. LEDPWMNode

Standalone ROS 2 GPIO smoke test for the LED PWM hardware. It is deliberately excluded from normal robot control. Run only when the motor controller is stopped because its test GPIO 18 is assigned to the rear-left L298N enable input.

### 4. TeleopJoystickNode

Consumes `sensor_msgs/Joy` on `/joy` and publishes `geometry_msgs/Twist` on `/cmd_vel`. The default mapping supports the legacy controller axes 5/7 for forward/backward, axes 4/6 for turning, and buttons 10/1 for stop. Axis and button indices plus speeds are ROS parameters, and short or differently sized joystick messages are handled without indexing errors.

## Installation

### Prerequisites

```bash
sudo apt-get update
sudo apt-get install python3-gpiozero python3-evdev
```

gpiozero does not require a separate GPIO daemon. The keyboard node uses `evdev` to detect key release. Grant the robot user access to input devices, for example by adding it to the `input` group and then logging out and back in:

```bash
sudo usermod -aG input "$USER"
```

Input device permissions vary by Linux installation. List stable keyboard device paths with `ls -l /dev/input/by-id/`. If automatic keyboard selection does not choose the desired device, configure its `/dev/input/eventX` path using the `input_device` parameter in `config/teleop.yaml`.

### Build

```bash
cd ~/ROS/colcon_workspace
colcon build --packages-select rosbot
source install/setup.bash
```

## Usage

### Launch the nodes

```bash
ros2 launch rosbot rosbot_launch.py
```

The joystick is disabled by default. On the Raspberry Pi with a joystick connected, enable it and optionally select another device index:

```bash
ros2 launch rosbot rosbot_launch.py joystick_enabled:=true joy_device_id:=0
```

On a Mac, the launch file does not start the camera node or access GPIO hardware. The Linux `joy_node` requires a supported Linux joystick device; velocity-command simulation can be tested using `/keyboard_cmd_vel` or `/joystick_cmd_vel`.

Pressing `ESC` on the configured keyboard stops the robot and shuts down the entire launch, including the motor, joystick, and arbitration nodes. `Ctrl+C` in the launch terminal also shuts down all launched nodes.

### Launch individual nodes

**Motor controller:**

```bash
ros2 run rosbot MotorControllerNode
```

**Keyboard control:**

```bash
ros2 run rosbot TeleopKeyboardNode
```

**LED PWM hardware smoke test (run with the motor controller stopped):**

```bash
ros2 run rosbot LEDPWMNode
```

### Publish velocity commands manually

```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Move sideways (omniwheel)
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Rotate
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
```

## Configuration

### Change GPIO pin assignments

The L298N BCM GPIO assignments can be changed in the `self.motors` list in `MotorControllerNode.py`. ENA/ENB are enable/PWM inputs; remove the jumpers when using them for PWM. The separate LED hardware test is not part of this pin assignment and must not run at the same time as the motor controller.

### Configure keyboard teleoperation

Edit `config/teleop.yaml` in the package to configure the keyboard device, maximum linear and angular speeds, start/end speed scales, acceleration duration, and speed-key increment. The standard launch loads this YAML file automatically.

```yaml
/**:
  ros__parameters:
    input_device: ''
    linear_speed: 0.5
    angular_speed: 1.0
    start_speed_scale: 0.2
    end_speed_scale: 1.0
    acceleration_duration: 2.0
    speed_increment: 0.1
```

The scale values must satisfy `0 <= start_speed_scale <= end_speed_scale <= 1`. To apply a different persistent configuration, edit this YAML file and rebuild the package. For a one-time override without rebuilding, pass launch arguments, for example `ros2 launch rosbot rosbot_launch.py start_speed_scale:=0.1 end_speed_scale:=0.7 acceleration_duration:=3.0`. Parameters are read when the keyboard node starts.

### Change robot kinematic parameters

In `MotorControllerNode.py`:

```python
self.wheel_base = 0.2      # Distance between front and rear axles (m)
self.track_width = 0.2     # Distance between left and right wheels (m)
self.max_speed = 1.0       # Maximum speed (m/s)
```

## Planned extensions

These features are possible future extensions and require suitable hardware as well as confirmed units and coordinate frames:

- Wheel-speed feedback using encoders
- Odometry for position and orientation
- PID control for wheel speed
- Battery status as a ROS topic is out of scope because the robot is powered by a power bank

## Troubleshooting

### gpiozero errors

Check that the gpiozero package is installed and that the selected GPIO pin factory is available for the Raspberry Pi. The user running the node must have permission to access GPIO.

## License

MIT
