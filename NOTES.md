# Project Notes

## Goal

ROS 2 control software for a four-wheel omniwheel robot running on a Raspberry Pi. The active package targets ROS 2 Jazzy and is built with `colcon` as an `ament_python` package.

## Architecture

- Active package: `ROS/colcon_workspace/src/rosbot/`.
- `MotorControllerNode` subscribes to `geometry_msgs/Twist` on `cmd_vel` and controls four L298N channels with pigpio: BCM direction/enable pins FL 5/6/12, FR 20/21/13, RL 23/24/18, RR 16/26/19. Its configurable `cmd_vel_timeout` watchdog stops the motors after missed commands (default 0.5 seconds).
- `TeleopKeyboardNode` and `TeleopJoystickNode` publish `Twist`; joystick input is `sensor_msgs/Joy` on `joy`.
- `LEDPWMNode.py` is retained as a standalone ROS hardware smoke test, not as part of the robot motor GPIO mapping. Its test output pin is reserved for that test only and must not be driven while the L298N motor node is active.
- `launch/rosbot_launch.py` starts the motor controller, teleoperation arbitrator, and keyboard node; joystick, CSI/libcamera camera, and USB-serial Hokuyo support are optional and default off. Camera and laser drivers start only on hostname `rosbot`; `LEDPWMNode.py` is not included in the main launch.
- Motor GPIO access is guarded by `self.hostname == 'rosbot'`; other hostnames use simulation/no hardware access.
- `ROS/rosbot.old__TMP!__migrating_to_ros2_!/` is a ROS 1 reference archive, not the active colcon package.

## Conventions and Decisions

- ROS 2 Python nodes are registered as console scripts in the package `setup.py`; launch files are installed from `launch/*.py`.
- ROS interfaces use `geometry_msgs/Twist` for velocity commands and `sensor_msgs/Joy` for joystick input.
- Preserve hostname-based hardware gating so development on non-robot machines does not access GPIO hardware.
- User-provided convention: source comments and commit messages should be in English.
- The legacy ROS 1 motor services were not ported: the archived servers are placeholders and current ROS 2 motor control uses direct PWM.
- The legacy odometry and battery code has no ROS 2 counterpart; the archived implementations are incomplete or standalone utilities.

## Known Technical Limitations

- Motor directions are now driven through L298N IN pins, but motor polarity, wheel order, the assumed omniwheel/Mecanum-X layout, and physical behavior still require validation on the robot.
- The ROS 2 build and package tests have been completed successfully in a Jazzy virtual machine, as reported by the project owner. Raspberry Pi hardware behavior remains unverified because the robot is currently unavailable.
