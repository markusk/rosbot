#!/usr/bin/env python3
# coding=utf-8


""" Some first GPIO stuff with the Raspberry Pi """

import pigpio
import time


PIN = 18  # Wähle eine GPIO-Nummer

pi = pigpio.pi()  # Verbindung zum pigpio-Daemon


print("Connecting to Raspi...")
if not pi.connected:
    exit()

try:
    while True:
        pi.write(PIN, 1)  # HIGH setzen
        time.sleep(1)
        pi.write(PIN, 0)  # LOW setzen
        time.sleep(1)
except KeyboardInterrupt:
    pi.stop()  # Verbindung schließen
