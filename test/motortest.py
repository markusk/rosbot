#!/usr/bin/env python3
# coding=utf-8


""" Some first GPIO stuff with the Raspberry Pi """

import pigpio
import time

MotorA1In3 = 4     # GPIO
MotorA1In4 = 27    # GPIO
MotorA1En  = 17    # GPIO (Enable / PWM)
MotorA1PWM = 200   # PWM duty cycle (0-255) aka "motor max speed"


pi = pigpio.pi()  # connect to pigpio daemon
print("Connecting to Raspi...")
if not pi.connected:
    exit()


try:
    while True:
        # Motor RIGHT
        print("Motors RIGHT")
        pi.set_PWM_dutycycle(MotorA1En, MotorA1PWM)
        pi.write(MotorA1In3, 1)
        pi.write(MotorA1In4, 0)
        time.sleep(1)

        # Motor LEFT
        print("Motors LEFT")
        pi.set_PWM_dutycycle(MotorA1En, MotorA1PWM)
        pi.write(MotorA1In3, 0)
        pi.write(MotorA1In4, 1)
        time.sleep(1)

        # Motor OFF
        print("Motors OFF")
        pi.set_PWM_dutycycle(MotorA1En, 0)
        time.sleep(1)
except KeyboardInterrupt:
    # Motor OFF
    print("Motors OFF")
    pi.set_PWM_dutycycle(MotorA1En, 0)
    pi.write(MotorA1In3, 0)
    pi.write(MotorA1In4, 0)
    pi.stop()  # close connection
