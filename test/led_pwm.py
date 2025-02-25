#!/usr/bin/env python3
# coding=utf-8

""" LED soft on and off with PWM using pigpio """

import pigpio
import time

LED_PIN = 18  # GPIO Pin für die LED

pi = pigpio.pi()  # Verbindung zum pigpio Daemon herstellen
print("Connecting to Raspi...")
if not pi.connected:
    exit()

try:
    while True:
        # LED Helligkeit erhöhen
        for duty_cycle in range(0, 256):
            pi.set_PWM_dutycycle(LED_PIN, duty_cycle)
            time.sleep(0.01)
        
        # LED Helligkeit verringern
        for duty_cycle in range(255, -1, -1):
            pi.set_PWM_dutycycle(LED_PIN, duty_cycle)
            time.sleep(0.01)
except KeyboardInterrupt:
    # LED ausschalten
    print("Turning off LED")
    pi.set_PWM_dutycycle(LED_PIN, 0)
    pi.stop()  # Verbindung schließen