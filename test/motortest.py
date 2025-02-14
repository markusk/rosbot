#!/usr/bin/env python3
# coding=utf-8


""" Some first GPIO stuff with the Raspberry Pi """

import pigpio
import time

MotorA1In3 = 4     # GPIO
MotorA1In4 = 27    # GPIO


pi = pigpio.pi()  # Verbindung zum pigpio-Daemon
print("Connecting to Raspi...")
if not pi.connected:
    exit()


try:
    while True:
        # Motor ON
        pi.write(MotorA1In3, 1)
        pi.write(MotorA1In4, 1)
        time.sleep(1)  # 1 Sekunde warten

        # Motor OFF
        pi.write(MotorA1In3, 0)
        pi.write(MotorA1In4, 0)
        time.sleep(1)  # 1 Sekunde warten
except KeyboardInterrupt:
    # Motor OFF
    pi.write(MotorA1In3, 0)
    pi.write(MotorA1In4, 0)
    pi.stop()  # Verbindung schließen
