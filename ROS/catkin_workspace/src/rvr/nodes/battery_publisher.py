#!/usr/bin/env python3
# coding=utf-8

"""
This node generates battery messages from the RVR.

It publishes battery messages into ROS.
Author:  Markus Knapp, 2020
Website: https://direcs.de
"""


# RVR stuff
# path to find the RVR lib from the public SDK
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../lib/')))


from math import pi
import logging
import asyncio
from sphero_sdk import SpheroRvrAsync
from sphero_sdk import SerialAsyncDal
from sphero_sdk import RvrStreamingServices
import traceback

import tf
import rospy
import time # for ROS message header timestamp
import math # for NaN values (Not a Number)
# we need a data type to publish the voltage
from std_msgs.msg import Header, Float32
# ros default battery messages
from sensor_msgs.msg import BatteryState


# initialise the node
rospy.init_node('battery_publisher')

# name of topic is 'voltage'
pubBattery = rospy.Publisher('voltage', BatteryState, queue_size=1)

# sleep time for this node in seconds
sleepTime = 1

# for reading the voltages
voltage = 0.0
value = 0
demoVoltage = 11.11


""" ROS message header """
# define battery message
battery_msg = BatteryState()
frame_id = 'battery'
seq = 0


loop = asyncio.get_event_loop()

rvr = SpheroRvrAsync(
    dal=SerialAsyncDal(
        loop
        # device='/dev/ttyTHS1',
    )
)


# logging.basicConfig(level=logging.DEBUG)


num_msgs_received = {}


# broadcast message
br = None

received_components = set()

def check_if_need_to_send_msg(component):
    print(f"got component {component}")
    received_components.add(component)
    if received_components >= {'locator','quaternion','gyroscope','velocity'}:
        received_components.clear()
        odom.header.stamp = rospy.Time.now()
        print(f"publishing! {odom}")
        try:
            pub_odom.publish(odom)
            br.sendTransform(
                (pose.position.x, pose.position.y, pose.position.z),
                (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                odom.header.stamp,
                '/base_link',
                '/odom'
            )
            print("published")
        except Exception:
            traceback.print_exc()
        print("blah")


async def locator_handler(locator_data):
    position.x = locator_data['Locator']['X'] * 16000.0 / MAX_SENSOR_VALUE
    position.y = locator_data['Locator']['Y'] * 16000.0 / MAX_SENSOR_VALUE
    position.z = 0
    check_if_need_to_send_msg('locator')


async def quaternion_handler(quaternion_data):
    orientation.w = quaternion_data['Quaternion']['W']
    orientation.x = quaternion_data['Quaternion']['X']
    orientation.y = quaternion_data['Quaternion']['Y']
    orientation.z = quaternion_data['Quaternion']['Z']
    check_if_need_to_send_msg('quaternion')


async def gyroscope_handler(gyroscope_data):
    angular.x = gyroscope_data['Gyroscope']['X'] * 2 * pi / 360
    angular.y = gyroscope_data['Gyroscope']['Y'] * 2 * pi / 360
    angular.z = gyroscope_data['Gyroscope']['Z'] * 2 * pi / 360
    check_if_need_to_send_msg('gyroscope')


async def velocity_handler(velocity_data):
    linear.x = velocity_data['Velocity']['X'] * 5.0 / MAX_SENSOR_VALUE
    linear.y = velocity_data['Velocity']['Y'] * 5.0 / MAX_SENSOR_VALUE
    check_if_need_to_send_msg('velocity')


async def main():
    """
    Enable a single sensor to stream.
    """
    rospy.init_node('publish_odom')
    global pub_odom
    global br
    pub_odom = rospy.Publisher('odom', Odometry, queue_size=10)
    br = tf.TransformBroadcaster()

    await rvr.wake()

    # Give RVR time to wake up
    await asyncio.sleep(2)

    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.locator,
        handler=locator_handler,
    )
    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.quaternion,
        handler=quaternion_handler,
    )
    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.gyroscope,
        handler=gyroscope_handler,
    )
    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.velocity,
        handler=velocity_handler,
    )

    await rvr.sensor_control.start(interval=250)


# The asyncio loop will run forever to allow infinite streaming.
if __name__ == '__main__':
    try:
        asyncio.ensure_future(
            main()
        )
        loop.run_forever()

    except KeyboardInterrupt:
        print('\nProgram terminated with keyboard interrupt.')

        loop.run_until_complete(
            asyncio.gather(
                rvr.sensor_control.clear(),
                rvr.close()
            )
        )

    finally:
        if loop.is_running():
            loop.close()
