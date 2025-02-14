#!/usr/bin/env python3
# coding=utf-8


""" Some first GPIO stuff with the Raspberry Pi """

import pigpio
import time

MotorA1In3 = 4     # GPIO
MotorA1In4 = 27    # GPIO


pi = pigpio.pi()  # connect to pigpio daemon
print("Connecting to Raspi...")
if not pi.connected:
    exit()


try:
    while True:
        # Motor RIGHT
        print("Motors RIGHT")
        pi.write(MotorA1In3, 1)
        pi.write(MotorA1In4, 0)
        time.sleep(1)

        # Motor LEFT
        print("Motors LEFT")
        pi.write(MotorA1In3, 0)
        pi.write(MotorA1In4, 1)
        time.sleep(1)
except KeyboardInterrupt:
    # Motor OFF
    print("Motors OFF")
    pi.write(MotorA1In3, 0)
    pi.write(MotorA1In4, 0)
    pi.stop()  # close connection
