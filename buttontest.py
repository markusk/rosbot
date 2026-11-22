#!/usr/bin/env python3
# coding=utf-8

import time
from gpiozero import Button, LED
# pins BCM
ledPin = 27
switchPin = 17

# buttonCounter
buttonPressed = 0

# setup
print('setup...')
led = LED(ledPin, active_high=False)
button = Button(switchPin, pull_up=True)  # waits for LOW

# switch detection by interrupt, falling edge, with debouncing
def my_callback():
    global buttonPressed
    buttonPressed += 1
    print(f'Button on GPIO {switchPin} pushed the {buttonPressed} time.')


# add button pressed event detector
button.when_pressed = my_callback

# timing
secs = 5

# loop
print('Press button now (5 secs)...!')

# turn LED on
led.on()

# delay
time.sleep(secs)

# turn LED off
led.off()

# cleanup
button.close()
led.close()
