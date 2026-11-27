# Project Notes

## Goal

ROS 2 control software for a four-wheel omniwheel robot running on a Raspberry Pi. The active package targets ROS 2 Jazzy and is built with `colcon` as an `ament_python` package.

## Architecture

- Active package: `ROS/colcon_workspace/src/rosbot/`.
- `MotorControllerNode` subscribes to `geometry_msgs/Twist` on `cmd_vel` and controls four L298N channels with gpiozero: BCM direction/enable pins FL 5/6/12, FR 20/21/13, RL 23/24/18, RR 16/26/19. Its configurable `cmd_vel_timeout` watchdog stops the motors after missed commands (default 0.5 seconds).
- `TeleopKeyboardNode` and `TeleopJoystickNode` publish `Twist`; joystick input is `sensor_msgs/Joy` on `joy`. Keyboard bindings are W/A/S/D for cardinal movement, Q/E/Y/C for diagonals on a German QWERTZ keyboard, J/L for rotation, plus/minus for speed, space to stop, and ESC/Ctrl+C to exit. Linux `evdev` key-down/up events let the keyboard node ramp continuously while a movement key is held and stop on release. Start/end scale and acceleration duration are ROS parameters loaded from installed `config/teleop.yaml`; the keyboard node requires `python3-evdev` and input-device permissions.
- `TeleopArbitratorNode` selects keyboard or joystick velocity commands and publishes the selected command on `cmd_vel`.
- `LEDPWMNode.py` is retained as a standalone ROS hardware smoke test, not as part of the robot motor GPIO mapping. Its test output pin GPIO 18 is shared with the rear-left motor enable and must not be driven while the L298N motor node is active.
- `launch/rosbot_launch.py` starts the motor controller, teleoperation arbitrator, and keyboard node; joystick, CSI/libcamera camera, and USB-serial Hokuyo support are optional and default off. Camera and laser drivers start only on hostname `rosbot`; `LEDPWMNode.py` is not included in the main launch.
- Motor GPIO access is guarded by `self.hostname == 'rosbot'`; other hostnames use simulation/no hardware access.
- `ROS/rosbot.old__TMP!__migrating_to_ros2_!/` is a ROS 1 reference archive, not the active colcon package.

## Conventions and Decisions

- ROS 2 Python nodes are registered as console scripts in the package `setup.py`; launch files are installed from `launch/*.py`.
- ROS interfaces use `geometry_msgs/Twist` for velocity commands and `sensor_msgs/Joy` for joystick input.
- Preserve hostname-based hardware gating so development on non-robot machines does not access GPIO hardware.
- Source comments, documentation, and commit messages should be in English.
- The legacy ROS 1 motor services were not ported: the archived servers are placeholders and current ROS 2 motor control uses direct PWM.
- The legacy odometry code has no ROS 2 counterpart; its archived implementation is an incomplete prototype. Battery-state publishing is out of scope because the robot is powered by a power bank.
- The global `README.md` contains the Raspberry Pi GPIO pinout and L298N wiring diagrams. The package README is also in English.

## Verified on the Robot

- Before the speed-ramp change, the project owner confirmed that keyboard teleoperation drove the robot correctly in all tested directions, including forward, backward, left, right, diagonals, and rotation. The new `evdev` key-release handling and configurable speed ramp have not yet been tested on the robot.
- The motor driver GPIO assignments and both L298N modules' ENA/ENB and IN1-IN4 connections were checked against the wiring diagrams. The L298N enable jumpers are removed for GPIO-controlled PWM.

## Known Technical Limitations and Remaining Validation

- The successful physical movement test covers keyboard teleoperation. Joystick direction mapping and stop behavior, camera and laser operation, and physical watchdog behavior still need separate validation if those features are used.
- The ROS 2 package has been built successfully on the actual Raspberry Pi 5 robot. Automated package test execution on that device has not been confirmed.
- Encoder feedback, odometry, and PID speed control are not implemented in the active ROS 2 package. Hardware, units, and coordinate frames need confirmation before implementation. Battery-state publishing is not planned because the robot is powered by a power bank.
