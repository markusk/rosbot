#!/usr/bin/env python3
# coding=utf-8

""" Some first GPIO stuff with the Raspberry Pi """

import time
from gpiozero import DigitalOutputDevice, PWMOutputDevice

MotorA1In3 = 4     # GPIO
MotorA1In4 = 27    # GPIO
MotorA1En = 17     # GPIO (Enable / PWM)
MotorA1PWM = 200 / 255  # PWM duty cycle (0-1) aka "max motor speed"

in3 = DigitalOutputDevice(MotorA1In3, initial_value=False)
in4 = DigitalOutputDevice(MotorA1In4, initial_value=False)
enable = PWMOutputDevice(MotorA1En, initial_value=0)

try:
    while True:
        # turn Motor RIGHT
        print("Motors RIGHT")
        enable.value = MotorA1PWM
        in3.on()
        in4.off()
        time.sleep(1)

        # turn Motor LEFT
        print("Motors LEFT")
        enable.value = MotorA1PWM
        in3.off()
        in4.on()
        time.sleep(1)

        # turn Motor OFF
        print("Motors OFF")
        enable.value = 0
        time.sleep(1)
except KeyboardInterrupt:
    # turn Motor OFF
    print("Motors OFF")
    enable.value = 0
    in3.off()
    in4.off()
    enable.close()
    in3.close()
    in4.close()
