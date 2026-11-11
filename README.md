# rosbot

ROS 2 control software for a four-wheel omniwheel robot running on a Raspberry Pi. The active package targets ROS 2 Jazzy and is located at `ROS/colcon_workspace/src/rosbot`. The `ROS/rosbot.old__TMP!__migrating_to_ros2_!/` directory is a legacy ROS 1 archive; its nodes and launch files are not used by the ROS 2 build.

## Requirements

- ROS 2 Jazzy installed on Ubuntu on the Raspberry Pi. See the [ROS 2 Raspberry Pi installation guide](https://docs.ros.org/en/jazzy/How-To-Guides/Installing-on-Raspberry-Pi.html).
- `colcon` and `rosdep`.
- pigpio and its Python package for Raspberry Pi GPIO control.
- For joystick control: a supported gamepad and the ROS 2 `joy` package.
- For the Raspberry Pi camera: a camera module supported by libcamera and the ROS 2 `camera_ros` driver.
- Hokuyo URG-LX04-UG01 over USB serial: ROS 2 `urg_node`, optional in the robot launch; confirm serial port and baud rate on the Pi.

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

On the Raspberry Pi with a supported libcamera camera connected, optionally start its ROS 2 image and camera-info topics with:

```bash
ros2 launch rosbot rosbot_launch.py camera_enabled:=true
```

The camera launch is disabled by default and only included when the hostname is exactly `rosbot`. Install the `camera_ros` driver and verify the camera module is supported by the Pi's libcamera version first. The camera connects to the Raspberry Pi's dedicated camera connector.

### Step 2.3: Enable the Hokuyo USB laser

The Hokuyo URG-LX04-UG01 connects to the Raspberry Pi over USB serial. The ROS package dependency installs `urg_node` during the documented `rosdep install` step. The laser launch is disabled by default and is only available when the hostname is exactly `rosbot`. Enable it with:

```bash
ros2 launch rosbot rosbot_launch.py laser_enabled:=true
```

The default serial device is `/dev/ttyACM0`; override it with a stable path from `/dev/serial/by-id/` if available:

```bash
ros2 launch rosbot rosbot_launch.py laser_enabled:=true laser_device:=/dev/serial/by-id/your-hokuyo laser_baud:=115200 laser_frame_id:=laser
```

The baud rate defaults to `115200`, and the scan frame defaults to `laser`. Confirm the detected port and scanner-supported baud rate on the Raspberry Pi before use. A fixed transform from the robot base to the `laser` frame will be needed for RViz/navigation.

## Step 3: Run and test

On a machine with ROS 2 sourced and the workspace built, run the normal launch. On non-robot hosts, motor output is simulated rather than written to GPIO. To publish a test velocity command:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

The ROS 2 package README documents node topics, optional camera and Hokuyo launches, L298N BCM pin assignments, joystick mappings, and hardware limitations: `ROS/colcon_workspace/src/rosbot/README.md`.
