#!/usr/bin/env python3
# coding=utf-8


""" Some first GPIO stuff with the Raspberry Pi """

import RPi.GPIO as GPIO
import time

MotorA1In3 = 4     # GPIO
MotorA1In4 = 27    # GPIO

print("Wake up Raspi...")

GPIO.setmode(GPIO.BCM)  # Use BCM numbering (GPIO-Numbers instead of Pin-Numbers on the PCB)
GPIO.setup(MotorA1In3, GPIO.OUT)  # Setze den Pin als Ausgang
GPIO.setup(MotorA1In4, GPIO.OUT)  # Setze den Pin als Ausgang

try:
    while True:
        # Motor ON
        GPIO.output(MotorA1In3, GPIO.HIGH)
        GPIO.output(MotorA1In4, GPIO.HIGH)
        time.sleep(1)  # 1 Sekunde warten

        # Motor OFF
        GPIO.output(MotorA1In3, GPIO.LOW)
        GPIO.output(MotorA1In4, GPIO.LOW)
        time.sleep(1)  # 1 Sekunde warten
except KeyboardInterrupt:
    # Motor OFF
    GPIO.output(MotorA1In3, GPIO.LOW)
    GPIO.output(MotorA1In4, GPIO.LOW)
    GPIO.cleanup()  # GPIOs sauber freigeben
