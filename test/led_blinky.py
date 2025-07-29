#!/usr/bin/env python3
# coding=utf-8


""" Some first GPIO stuff with the Raspberry Pi """

import pigpio
import time


PIN = 18  # GPIO number for a LED

pi = pigpio.pi()  # connect to pigpio daemon


print("Connecting to Raspi...")
if not pi.connected:
    print("ERROR")
    exit()

try:
    while True:
        print("ON")
        pi.write(PIN, 1)  # set to HIGH
        time.sleep(1)
        print("OFF")
        pi.write(PIN, 0)  # set to LOW
        time.sleep(1)
except KeyboardInterrupt:
    print("OFF")
    pi.write(PIN, 0)  # set to LOW again
    pi.stop()  # close connection
