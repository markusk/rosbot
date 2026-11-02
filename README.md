# rosbot

A ROS Python package for my rosbot - running on a Raspberry Pi.

_Please note: this code is still in the middle of the development process!_

[![GitHub issues](https://img.shields.io/github/issues/markusk/rosbot)](https://github.com/markusk/rosbot/issues) [![GitHub stars](https://img.shields.io/github/stars/markusk/rosbot)](https://github.com/markusk/rosbot/stargazers) [![GitHub license](https://img.shields.io/github/license/markusk/rosbot)](https://github.com/markusk/rosbot/blob/master/LICENSE)

---

## Step 1: Setup Raspberry with Ubuntu

- Install Ubuntu Desktop ([Instruction](https://ubuntu.com/desktop/)).


### Download and install pigpio
https://abyz.me.uk/rpi/pigpio/download.html
(See detailed instructions there - will be installed to /usr/local/lib and /usr/local/bin)

### clone this repository
https://github.com/markusk/rosbot

```bash
git clone https://github.com/markusk/rosbot.git
cd rosbot
```

## Step 2: Setting up pigpiod for autostart with systemd
```bash
sudo cp raspi/etc__systemd__system__rosbot-boot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable rosbot-boot.service
sudo systemctl start rosbot-boot.service
```

## Step 3: Joystick/Gamepad OS support
```bash
sudo apt-get install joystick
```

### Microsoft XBOX Wireless Controller
- Supported by the Kernel - if used with the USB connector.

### Gamepad/Joystick test
- Connect a Gamepad to the Pi, start the following command and press any buttons or move some axes:
```bash
jstest --normal /dev/input/js0
```


## Step 3: ROS 2

- Install ROS  on your Raspberry Pi ([Instruction](https://docs.ros.org/en/jazzy/How-To-Guides/Installing-on-Raspberry-Pi.html)):

- Install ROS packages (+++ TO BE DEFINED !!! +++):

- Do not forget to "autostart" ros; i.e. in your terminal, when logging in:
```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
```

## Step 5: Create a central place for this repository

- Create your own development directory "develop"

```bash
mkdir ~/develop
cd ~/develop
```

- Clone this repository

```bash
git clone https://github.com/markusk/rvr.git
```

- Create a colcon workspace (ROS 2) _with_ 'src' subfolder:

```bash
mkdir -p ~/ros2_ws/src
```

- Create symbolic link with the name of each ros package from this repository, pointing into the 'src' folder from your ROS/colcon workspace (ros_ws):

```bash
cd ~/ros2_ws/src
ln -s ~/develop/rosbot/ROS/colcon_workspace/src/rosbot rosbot
ln -s ~/develop/rosbot/ROS/colcon_workspace/src/testbot testbot
```

- Now also source your ROS workspace you created:
```bash
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
```

- Build the ROS rosbot package
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```


## Step 4: Build and run ROS 2

Install ROS 2 Jazzy and the Raspberry Pi dependencies using the official [ROS 2 Raspberry Pi guide](https://docs.ros.org/en/jazzy/How-To-Guides/Installing-on-Raspberry-Pi.html). The active ROS 2 package is `ROS/colcon_workspace/src/rosbot`; `ROS/rosbot.old__TMP!__migrating_to_ros2_!` is a non-buildable ROS 1 archive and its launch files must not be used with ROS 2.

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
cd ROS/colcon_workspace
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select rosbot
source install/setup.bash
```

On the Raspberry Pi, install/start pigpio before launching hardware nodes:

```bash
sudo apt-get install pigpio python3-pigpio
sudo systemctl enable --now pigpiod
ros2 launch rosbot rosbot_launch.py
```

The joystick is optional and disabled by default. On the Pi with a controller attached, use:

```bash
ros2 launch rosbot rosbot_launch.py joystick_enabled:=true joy_device_id:=0
```

The launch setup checks that the hostname is exactly `rosbot` before starting the Linux joystick device node. Motor and LED GPIO access is also enabled only on that hostname; on a Mac, the hardware nodes simulate/no-op. To exercise joystick mapping without a device, publish `sensor_msgs/Joy` to `/joy`; to exercise the motor command interface, publish `geometry_msgs/Twist` to `/cmd_vel`.

For package details, migrated functionality, parameters, and hardware limitations, see `ROS/colcon_workspace/src/rosbot/README.md`. The ROS 1 `roslaunch`, `catkin_ws`, and `ROS_MASTER_URI` instructions formerly in this README are obsolete and are not supported by the active package.
