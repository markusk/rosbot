#!/usr/bin/env python3
# coding=utf-8

""" LED soft on and off with PWM using gpiozero """

import time
from gpiozero import PWMLED

LED_PIN = 18  # GPIO Pin for LED

led = PWMLED(LED_PIN, initial_value=0)

try:
    while True:
        # increase LED brightness
        for duty_cycle in range(0, 255):
            led.value = duty_cycle / 255
            time.sleep(0.003)

        # decrease LED brightness
        for duty_cycle in range(255, -1, -1):
            led.value = duty_cycle / 255
            time.sleep(0.003)
except KeyboardInterrupt:
    # turn off LED
    print("Turning off LED")
    led.off()
    led.close()
