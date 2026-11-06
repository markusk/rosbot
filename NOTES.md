# Project Notes

## Goal

ROS 2 control software for a four-wheel omniwheel robot running on a Raspberry Pi. The active package targets ROS 2 Jazzy and is built with `colcon` as an `ament_python` package.

## Architecture

- Active package: `ROS/colcon_workspace/src/rosbot/`.
- `MotorControllerNode` subscribes to `geometry_msgs/Twist` on `cmd_vel` and writes PWM through pigpio on GPIO 4–7.
- `TeleopKeyboardNode` and `TeleopJoystickNode` publish `Twist`; joystick input is `sensor_msgs/Joy` on `joy`.
- `LEDPWMNode` uses GPIO 18 through pigpio.
- `launch/rosbot_launch.py` starts the motor and LED nodes; joystick teleoperation is optional (`joystick_enabled`, default `false`). The Linux joystick device node is only included when the hostname is exactly `rosbot`.
- Hardware access in the motor and LED nodes is guarded by `self.hostname == 'rosbot'`; other hostnames use simulation/no hardware access.
- `ROS/rosbot.old__TMP!__migrating_to_ros2_!/` is a ROS 1 reference archive, not the active colcon package.

## Conventions and Decisions

- ROS 2 Python nodes are registered as console scripts in the package `setup.py`; launch files are installed from `launch/*.py`.
- ROS interfaces use `geometry_msgs/Twist` for velocity commands and `sensor_msgs/Joy` for joystick input.
- Preserve hostname-based hardware gating so development on non-robot machines does not access GPIO hardware.
- User-provided convention: source comments and commit messages should be in English.
- The legacy ROS 1 motor services were not ported: the archived servers are placeholders and current ROS 2 motor control uses direct PWM.
- The legacy odometry and battery code has no ROS 2 counterpart; the archived implementations are incomplete or standalone utilities.

## Known Technical Limitations

- Motor PWM currently uses absolute wheel speeds and does not implement direction GPIO; reverse and turning behavior on hardware is not validated.
- ROS 2 build and Raspberry Pi hardware operation have not been verified in this project session. The ROS 2 toolchain and pytest were unavailable in the development environment during the prior checks.
