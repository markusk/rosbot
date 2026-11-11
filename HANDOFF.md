# Handoff

## Current State

The active ROS 2 Jazzy package is `ROS/colcon_workspace/src/rosbot`. It provides motor control, keyboard and joystick teleoperation, command arbitration, and a command-velocity watchdog. Optional launch support is available for a Raspberry Pi camera through `camera_ros` and a Hokuyo scanner through `urg_node`. The ROS 1 directory is an archive and is not part of the active colcon build.

The ROS 2 build and package tests have been completed successfully in a Jazzy virtual machine, as reported by the project owner. Physical motor and sensor behavior has not been verified because the robot is currently unavailable.

## Relevant Files

- `README.md`: repository setup, build, and launch instructions.
- `ROS/colcon_workspace/src/rosbot/README.md`: package behavior, migration status, usage, and hardware limitations.
- `ROS/colcon_workspace/src/rosbot/rosbot/`: active ROS 2 nodes.
- `ROS/colcon_workspace/src/rosbot/launch/rosbot_launch.py`: package launch entry point.
- `ROS/colcon_workspace/src/rosbot/test/`: package tests.
- `ROS/rosbot.old__TMP!__migrating_to_ros2_!/`: archived ROS 1 implementation; its ROS 1 launch and service files are not active ROS 2 components.

## Remaining Work

- On the robot, verify L298N wiring, motor order and polarity, BCM pin assignments, wheel layout, and low-speed movement with the wheels raised. The standalone LED GPIO test must not run while the motor controller is active because GPIO 18 is shared.
- Test the 0.5-second `cmd_vel_timeout` behavior on hardware, including stop and recovery. It is a software fallback, not a physical emergency stop.
- Verify the installed camera module and libcamera compatibility, Hokuyo serial path and baud rate, and measure the base-to-laser transform. The code supports these devices, but their physical connection and operation are unverified.
- Encoder feedback, odometry, PID speed control, and battery publishing are not implemented in the active ROS 2 package. Revisit these after confirming the required hardware, units, and coordinate frames.
