# Handoff

## Current State

The active ROS 2 Jazzy package is `ROS/colcon_workspace/src/rosbot`. It provides motor control using gpiozero, keyboard and joystick teleoperation, command arbitration, and a command-velocity watchdog. Optional launch support is available for a Raspberry Pi camera through `camera_ros` and a Hokuyo scanner through `urg_node`. The ROS 1 directory is an archive and is not part of the active colcon build.

The repository's main README includes the Raspberry Pi GPIO pinout and L298N wiring diagrams. The package README is in English and documents package behavior and usage.

Before the recent speed-ramp and dual-input changes, keyboard teleoperation was confirmed on the Raspberry Pi in all tested directions. The bindings are W/A/S/D for cardinal movement, Q/E/Y/C for diagonals on a German QWERTZ keyboard, and J/L for rotation. Direction command signs were adapted to the actual wiring and observed physical directions. Both L298N modules' input and enable GPIO connections were checked against the diagrams; their enable jumpers are removed for PWM control. The latest untested changes add configurable speed ramps, local `evdev` key-release handling, and an SSH-compatible terminal fallback that stops after a configurable key-repeat timeout (0.3 seconds, accepted by the project owner). The confirmed Cherry keyboard's stable `/dev/input/by-id/usb-Cherry_GmbH_CHERRY_Wired_Keyboard-event-kbd` path is configured in YAML for local use.

The ROS 2 package has been built successfully on the actual Raspberry Pi 5 robot. Automated package test execution on that device has not been confirmed. Camera and Hokuyo hardware operation and the physical command watchdog remain unverified.

## Relevant Files

- `README.md`: repository setup, build, launch instructions, and GPIO/L298N wiring diagrams.
- `NOTES.md`: project architecture, decisions, and verified robot status.
- `ROS/colcon_workspace/src/rosbot/README.md`: package behavior, migration status, usage, and hardware limitations.
- `ROS/colcon_workspace/src/rosbot/rosbot/`: active ROS 2 nodes.
- `ROS/colcon_workspace/src/rosbot/launch/rosbot_launch.py`: package launch entry point.
- `ROS/colcon_workspace/src/rosbot/test/`: package tests, including speed-ramp helper tests.
- `ROS/colcon_workspace/src/rosbot/config/teleop.yaml`: ROS parameters for input mode/device, SSH repeat timeout, and speed ramp (separate translation/angular scales, 20% start and 100% end by default, 2-second ramp); currently configured with the confirmed Cherry keyboard path and angular speed of 1.3 rad/s.
- `circuit/GPIO-Pinout-Diagram.png`: Raspberry Pi GPIO wiring diagram.
- `circuit/L298N Motor Controller.png`: L298N module and motor wiring diagram.
- `ROS/rosbot.old__TMP!__migrating_to_ros2_!/`: archived ROS 1 implementation; its ROS 1 launch and service files are not active ROS 2 components.

## Remaining Work

- If further hardware validation is needed, test the `cmd_vel_timeout` stop/recovery behavior on the robot. It is a software fallback, not a physical emergency stop.
- Verify joystick direction mapping and stop behavior if joystick control will be used.
- Verify the installed camera module and libcamera compatibility, Hokuyo serial path and baud rate, and measure the base-to-laser transform before using those optional devices.
- Encoder feedback, odometry, and PID speed control are not implemented in the active ROS 2 package. Revisit these after confirming the required hardware, units, and coordinate frames.
- Battery-state publishing is out of scope because the robot is powered by a power bank.

## Git / Working Tree

At the time of this handoff, `HEAD` and `origin/master` are both at `c2da422` (`updated teleop keyboard layout togerman QWERTZ`). The only untracked files observed are `circuit/GPIO-Pinout-Diagram.af` and `circuit/L298N Motor Controller.af`; they were not changed as part of this handoff update.
