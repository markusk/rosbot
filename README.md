# rvr

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

- Create a colcon workspace (ROS 2) _without_ 'src' subfolder:

```bash
mkdir ~/ros2_ws
cd ~/ros2_ws
```

- Create symbolic link with the name 'src', pointing to the 'src' folder in the ROS directory from this repository:

```bash
ln -s ~/develop/rosbot/ROS/colcon_workspace/src/ src
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


## Step 7: Run ROS 2

### Test the rosbot ROS package

- Start the ROS launch file:

```bash
cd ~/ros2_ws
ros2 launch rosbot test.launch
```

- The output should look like above.
- Exit the program with CTRL-C.

### The main launch file

- Run the main launch file on the robot (Raspberry Pi):

```bash
cd ~/catkin_ws
roslaunch rvr rvr.launch
```

_**to do:**_ On another computer (the ground control center):

```bash
export ROS_MASTER_URI=http://<hostname>:11311
rosparam set joy_node/dev "/dev/input/js1"
roslaunch rvr ground_control_center.launch
```

## _**to do:**_ Step 8: Setting up ROS for autostart

### systemd under Ubuntu

```bash
sudo cp raspi/etc__systemd__system__rvr-ros-start.service /etc/systemd/system/rvr-ros-start.service
sudo systemctl daemon-reload
sudo systemctl start rvr-ros-start.service
sudo systemctl enable rvr-ros-start.service
```

---

## The ROS launch files

### K

#### keyboard_control_test

Listens to a teleop_twist_keyboard node and prints out the data/messages. Uses:

- _teleop_twist_keyboard_
- _nodes/keyboard_listener.py_

### M

#### motor_server

Controls the motors on the robot. Uses:

- _motor_server.py_

### R

#### rvr

Controls the whole robot. To be started on the robot. Uses:

- _motor_server.py_

_**to do:**_
- _tf_broadcaster.py_
- _battery_publisher.py_
- _imu_bno055.py_
- _base_controller.py_
- _minibot_camera_
- _urg_node_
