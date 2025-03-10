#!/usr/bin/env python3
# coding=utf-8

""" LED soft on and off with PWM using pigpio """

import pigpio
import time

LED_PIN = 18  # GPIO Pin for LED

pi = pigpio.pi()  # Connecting to pigpio daemon
print("Connecting to Raspi...")
if not pi.connected:
    exit()

try:
    while True:
        # increase LED brightness
        for duty_cycle in range(0, 256):
            pi.set_PWM_dutycycle(LED_PIN, duty_cycle)
            time.sleep(0.003)

        # decrease LED brightness
        for duty_cycle in range(255, -1, -1):
            pi.set_PWM_dutycycle(LED_PIN, duty_cycle)
            time.sleep(0.003)
except KeyboardInterrupt:
    # turn off LED
    print("Turning off LED")
    pi.set_PWM_dutycycle(LED_PIN, 0)
    pi.stop()  # closeing connection
