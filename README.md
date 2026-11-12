# rosbot

ROS 2 control software for a four-wheel omniwheel robot running on a Raspberry Pi. The active package targets ROS 2 Jazzy and is located at `ROS/colcon_workspace/src/rosbot`. The `ROS/rosbot.old__TMP!__migrating_to_ros2_!/` directory is a legacy ROS 1 archive; its nodes and launch files are not used by the ROS 2 build.

## Requirements

- ROS 2 Jazzy on Ubuntu. For Raspberry Pi installation, see the [ROS 2 Raspberry Pi installation guide](https://docs.ros.org/en/jazzy/How-To-Guides/Installing-on-Raspberry-Pi.html).
- `colcon` and `rosdep`.
- On the robot: pigpio and its Python package for GPIO control.
- Optional devices: a Linux-compatible gamepad, a libcamera-supported Raspberry Pi camera with `camera_ros`, and/or a Hokuyo URG-LX04-UG01 with `urg_node`.

## One-time setup

Do these steps once for each computer or Raspberry Pi where you want to build or run the package.

### 1. Get the source

```bash
git clone https://github.com/markusk/rosbot.git
cd rosbot
```

If the repository is already cloned, use its existing directory instead.

### 2. Install ROS dependencies and build

```bash
source /opt/ros/jazzy/setup.bash
cd ROS/colcon_workspace
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select rosbot
```

`rosdep install` installs the package dependencies, including the ROS joystick, camera, and laser drivers. Optional devices still need to be physically connected and configured. Re-run dependency installation if package dependencies change; rebuild after changing source code.

### 3. Configure the Raspberry Pi for motor GPIO (robot only)

Motor GPIO access is enabled only when the system hostname is exactly `rosbot`. Set the hostname once on the robot and reboot if it changed:

```bash
sudo hostnamectl set-hostname rosbot
hostname
```

Install pigpio and its Python binding:

```bash
sudo apt-get update
sudo apt-get install pigpio python3-pigpio
```

Configure **one** pigpio service option, depending on where `pigpiod` is installed. If it is at `/usr/local/bin/pigpiod`, use the repository service:

```bash
sudo cp ~/rosbot/raspi/etc__systemd__system__rosbot-boot.service /etc/systemd/system/rosbot-boot.service
sudo systemctl daemon-reload
sudo systemctl enable --now rosbot-boot.service
```

If pigpio was installed from the Ubuntu package and its daemon is not at `/usr/local/bin/pigpiod`, use the package-provided service instead:

```bash
sudo systemctl enable --now pigpiod
```

Check the selected service with `sudo systemctl status rosbot-boot.service` or `sudo systemctl status pigpiod`, then verify pigpio responds with `pigs t`.

On any host whose name is not `rosbot`, the motor node does not access GPIO and simulates motor output. Do not set the robot hostname on a development computer.

### 4. Optional device setup

- **Joystick:** Connect a supported gamepad to Linux. To diagnose the Linux device, install `joystick` and run `jstest --normal /dev/input/js0`. The ROS `joy` driver is installed through `rosdep`.
- **Camera:** Connect a camera supported by the Pi's libcamera version. The `camera_ros` driver is installed through `rosdep`.
- **Hokuyo laser:** Connect the URG-LX04-UG01 over USB. Confirm its serial device and supported baud rate on the robot; prefer a stable path under `/dev/serial/by-id/` when available.

Camera and laser launches are restricted to the host whose name is exactly `rosbot`. Their physical operation has not been verified; the scanner's base-to-laser transform must also be measured before use for navigation.

## Start the robot later

Do these steps in each new terminal session. No dependency installation or rebuild is needed unless dependencies or source code have changed.

### 1. Source ROS 2 and the workspace

```bash
source /opt/ros/jazzy/setup.bash
cd ~/rosbot/ROS/colcon_workspace
source install/setup.bash
```

If the repository is in a different location, replace `~/rosbot` with its path.

### 2. Start the standard launch

```bash
ros2 launch rosbot rosbot_launch.py
```

This starts the motor controller, command arbitrator, and keyboard teleoperation. The keyboard node needs an interactive terminal. Press `ESC` in that terminal or `Ctrl+C` in the launch terminal to stop all launched nodes. On the robot, make the first movement tests with the wheels raised and at low speed; the software watchdog is not a physical emergency stop.

### Start with optional devices

Add launch arguments to the same command when you want to use an optional device:

```bash
ros2 launch rosbot rosbot_launch.py joystick_enabled:=true joy_device_id:=0
ros2 launch rosbot rosbot_launch.py camera_enabled:=true
ros2 launch rosbot rosbot_launch.py laser_enabled:=true
```

The laser defaults to `/dev/ttyACM0`, baud `115200`, and frame `laser`. Override the serial settings if needed:

```bash
ros2 launch rosbot rosbot_launch.py laser_enabled:=true laser_device:=/dev/serial/by-id/your-hokuyo laser_baud:=115200 laser_frame_id:=laser
```

Combine multiple options in one launch command, for example `joystick_enabled:=true camera_enabled:=true`. The joystick requires a supported Linux device. Camera and laser nodes only start on the robot hostname `rosbot`.

## After changing the source

From the workspace directory, rebuild and source the updated package in each terminal that will use it:

```bash
cd ~/rosbot/ROS/colcon_workspace
colcon build --symlink-install --packages-select rosbot
source install/setup.bash
```

If `package.xml` dependencies changed, run `rosdep install --from-paths src --ignore-src -r -y` before building.

## Further details

The package README documents node topics, L298N BCM pin assignments, joystick mappings, and hardware limitations: `ROS/colcon_workspace/src/rosbot/README.md`.
