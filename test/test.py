#!/usr/bin/env python3
# coding=utf-8


""" Some first GPIO stuff with the Raspberry Pi """

import RPi.GPIO as GPIO
import time

PIN = 18  # Wähle eine GPIO-Nummer

print("Wake up Raspi...")

GPIO.setmode(GPIO.BCM)  # Verwende BCM-Nummerierung
GPIO.setup(PIN, GPIO.OUT)  # Setze den Pin als Ausgang

try:
    while True:
        GPIO.output(PIN, GPIO.HIGH)  # Pin auf HIGH setzen
        time.sleep(1)  # 1 Sekunde warten
        GPIO.output(PIN, GPIO.LOW)  # Pin auf LOW setzen
        time.sleep(1)  # 1 Sekunde warten
except KeyboardInterrupt:
    GPIO.cleanup()  # GPIOs sauber freigeben
