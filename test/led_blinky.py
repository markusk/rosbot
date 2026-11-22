#!/usr/bin/env python3
# coding=utf-8

""" Some first GPIO stuff with the Raspberry Pi """

import time
from gpiozero import LED

PIN = 18  # GPIO number for a LED

led = LED(PIN)

try:
    while True:
        print("ON")
        led.on()  # set to HIGH
        time.sleep(1)
        print("OFF")
        led.off()  # set to LOW
        time.sleep(1)
except KeyboardInterrupt:
    print("OFF")
    led.off()  # set to LOW again
    led.close()
