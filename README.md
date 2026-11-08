# rosbot

ROS 2 control software for a four-wheel omniwheel robot running on a Raspberry Pi. The active package targets ROS 2 Jazzy and is located at `ROS/colcon_workspace/src/rosbot`. The `ROS/rosbot.old__TMP!__migrating_to_ros2_!/` directory is a legacy ROS 1 archive; its nodes and launch files are not used by the ROS 2 build.

## Requirements

- ROS 2 Jazzy installed on Ubuntu on the Raspberry Pi. See the [ROS 2 Raspberry Pi installation guide](https://docs.ros.org/en/jazzy/How-To-Guides/Installing-on-Raspberry-Pi.html).
- `colcon` and `rosdep`.
- pigpio and its Python package for Raspberry Pi GPIO control.
- For joystick control: a supported gamepad and the ROS 2 `joy` package.

## Step 1: Clone and build

Clone this repository:

```bash
git clone https://github.com/markusk/rosbot.git
cd rosbot
```

Source ROS 2 and install the package dependencies, build, then source the workspace:

```bash
source /opt/ros/jazzy/setup.bash
cd ROS/colcon_workspace
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select rosbot
source install/setup.bash
```

After a reboot or in every new terminal, source both environments again before using ROS 2 commands:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/rosbot/ROS/colcon_workspace
source install/setup.bash
```

This project builds directly in its repository workspace; no symlink to a separate ROS workspace is required.

## Step 2: Configure the Raspberry Pi

### Set the robot hostname

Motor GPIO access is enabled only when the system hostname is exactly `rosbot`. Set and verify it on the Raspberry Pi, then reboot if the hostname changed:

```bash
sudo hostnamectl set-hostname rosbot
hostname
```

If the hostname is anything other than `rosbot`, the motor node runs without accessing GPIO hardware.

### Step 2.1: Install and autostart pigpio

Install pigpio and its Python binding:

```bash
sudo apt-get update
sudo apt-get install pigpio python3-pigpio
```

This repository includes a systemd unit for installations where the daemon is at `/usr/local/bin/pigpiod`:

```bash
sudo cp raspi/etc__systemd__system__rosbot-boot.service /etc/systemd/system/rosbot-boot.service
sudo systemctl daemon-reload
sudo systemctl enable --now rosbot-boot.service
sudo systemctl status rosbot-boot.service
pigs t
```

The unit starts pigpiod at boot and restarts it if it fails. If pigpio was installed from the Ubuntu package and its daemon is not at `/usr/local/bin/pigpiod`, use the package-provided `pigpiod` systemd service instead:

```bash
sudo systemctl enable --now pigpiod
sudo systemctl status pigpiod
pigs t
```

### Step 2.2: Set up a Linux joystick/gamepad

Install the Linux joystick utilities, connect the gamepad, and verify that Linux sees it:

```bash
sudo apt-get install joystick
jstest --normal /dev/input/js0
```

The ROS 2 `joy` package is a dependency of the `rosbot` package and is installed by `rosdep`. Joystick launch is optional and disabled by default; enable it on Linux with:

```bash
ros2 launch rosbot rosbot_launch.py joystick_enabled:=true joy_device_id:=0
```

`joy_device_id` selects the joystick index (`0` uses the first device, normally `/dev/input/js0`). The launch starts the ROS 2 `joy_node` and joystick teleoperation, alongside the motor controller, velocity-command arbitrator, and keyboard teleoperation. Press `ESC` in the keyboard teleoperation terminal or `Ctrl+C` in the launch terminal to stop the launch.

For keyboard-only operation, leave joystick support disabled:

```bash
ros2 launch rosbot rosbot_launch.py
```

## Step 3: Run and test

On a machine with ROS 2 sourced and the workspace built, run the normal launch. On non-robot hosts, motor output is simulated rather than written to GPIO. To publish a test velocity command:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

The ROS 2 package README documents node topics, L298N BCM pin assignments, joystick mappings, and hardware limitations: `ROS/colcon_workspace/src/rosbot/README.md`.
