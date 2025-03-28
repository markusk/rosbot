#!/usr/bin/env python3
# coding=utf-8


"""
This is my listener for the joy_node. It listens on the topic 'joy' and prints out some information.
It then switches on motors on the robot (Raspberry Pi) when the D-Pad is used on the gamepad.

This needs the motor_server to be run on the robot. Like this:

Usage
-----
Robot (Raspberry Pi):
1. roslaunch rosbot motor_server. (This starts also the roscore on this computer automatically).

Another Ubuntu machine:
1. export ROS_MASTER_URI=http://hostname-of-your-pi:11311/ from the robot. I.E.:
   export ROS_MASTER_URI=http://rosbot:11311/
2. Set joystick device (if different) to js0. I.E.:
   rosparam set joy_node/dev "/dev/input/js2"
3. roslaunch rosbot joystick_control.launch


Author:  Markus Knapp
Website: https://direcs.de
"""

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
# name of the package(!).srv
from rosbot.srv import *


# Getting robot parameters
rospy.loginfo('Getting parameters for robot.')
# speed of the motors (0-255).
drivingSpeed = rospy.get_param('/rosbot/drivingSpeed')
rospy.loginfo('Using drivingSpeed %s.', drivingSpeed)
# the speed when turning the bot can be higher if needed (higher friction)
turnSpeed = rospy.get_param('/rosbot/turnSpeed')
rospy.loginfo('Using turnSpeed %s.', turnSpeed)

# Service 'motor' from motor_server.py ready?
rospy.loginfo("Waiting for service 'motor'")
rospy.wait_for_service('motor')


#  this will execute the "drive" command
def drive(direction, speed):
    # Send driving direction to motor
    try:
        # Create the handle 'motor_switcher' with the service type 'Motor'.
        # The latter automatically generates the MotorRequest and MotorResponse objects.
        motor_switcher = rospy.ServiceProxy('motor', Motor)

         # the handle can be called like a normal function
        rospy.loginfo("Switching motors to %s @ %s.", direction, speed)
        response = motor_switcher(direction, speed)

        # show result
        rospy.loginfo(rospy.get_caller_id() + ' says result is %s.', response.result)

    except rospy.ServiceException, e:
        rospy.logerr("Service call for 'motor' failed: %s", e)


def callback(joy):
    """
    # debug messages
    rospy.loginfo("+++ Found Axes: %s joystick axes+++", len(joy.axes))
    rospy.loginfo("+++ Found %s joystick buttons +++", len(joy.buttons))

    rospy.loginfo("axis 0: %s", joy.axes[0])
    rospy.loginfo("axis 1: %s", joy.axes[1])
    rospy.loginfo("axis 2: %s", joy.axes[2])
    rospy.loginfo("axis 3: %s", joy.axes[3])
    rospy.loginfo("axis 4: %s", joy.axes[4])
    rospy.loginfo("axis 5: %s", joy.axes[5])
    rospy.loginfo("-------------------")
    """

    # which button was pressed?
    # D-Pad, vertikal up  or  XBOX controller cross up

    # check if we really using my xbox controller with >10 axes
    # otherwise get an exception accessing the array out of bound
    # @todo: implement a better solution!!  if (len(joy.axes) > 10):
    if (joy.axes[5] == 1.0) or (joy.axes[7] == 1.0):
      rospy.loginfo("FORWARD button pressed.")
      drive("FORWARD", drivingSpeed)
    # D-Pad, vertikal down
    elif (joy.axes[5] == -1.0) or (joy.axes[7] == -1.0):
      rospy.loginfo("BACKWARD button pressed.")
      drive("BACKWARD", drivingSpeed)
    # D-Pad, horizontal left  or  XBOX controller cross left
    elif (joy.axes[4] ==  1.0) or (joy.axes[6] ==  1.0):
      rospy.loginfo("LEFT button pressed.")
      drive("LEFT", turnSpeed)
    # D-Pad, horizontal right  or  XBOX controller cross right
    elif (joy.axes[4] ==  -1.0) or (joy.axes[6] ==  -1.0):
      rospy.loginfo("RIGHT button pressed.")
      drive("RIGHT", turnSpeed)
    # red button on my gamepad  or  XBOX controller red button B
    elif (joy.buttons[10] == 1.0) or (joy.buttons[1] == 1.0):
      rospy.loginfo("RED button pressed.")
      drive("STOP", 0)
    #else:
    #  rospy.logwarn("Wrong joystick/gamepad: not enough axes!")


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'joy_listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('joy_listener', anonymous=True)

    # subscribe the joy(stick) topic
    rospy.Subscriber('joy', Joy, callback)

    # Ready
    rospy.loginfo("Ready. Press a button on your joystick D-Pad now.")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
