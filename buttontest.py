#!/usr/bin/env python3
# coding=utf-8

import pigpio
import time
from subprocess import call

#------
# init
#------
pi = pigpio.pi()  # Verbindung zum pigpio Daemon herstellen
print("Connecting to Raspi...")
if not pi.connected:
    exit()

# pins	    BCM
ledPin     = 27
switchPin  = 17

# buttonCounter
buttonPressed = 0

# setup
print('setup...')
pi.set_mode(ledPin, pigpio.OUTPUT)
pi.set_mode(switchPin, pigpio.INPUT)
pi.set_pull_up_down(switchPin, pigpio.PUD_UP)  # waits for LOW

# switch detection by interrupt, falling edge, with debouncing
def my_callback(gpio, level, tick):
    global buttonPressed
    buttonPressed += 1
    print(f'Button on GPIO {gpio} pushed the {buttonPressed} time.')

# add button pressed event detector
pi.callback(switchPin, pigpio.FALLING_EDGE, my_callback)

# timing
secs = 5

#------
# loop
#------
print('Press button now (5 secs)...!')

# turn LED on
pi.write(ledPin, 0)

# delay
time.sleep(secs)

# turn LED off
pi.write(ledPin, 1)

# cleanup
pi.stop()
