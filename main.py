# main.py: the first file the runs on startup. does the following:
# 1. initialize WiFi so that it persists between runs/tests
# 2. blink LED, wait for button to be pressed
# 3. import sar_main.py, run from that whichever test/program is set below
# 4. goto 2 (repeat)

from machine import Pin, reset
from time import sleep
import network

led = Pin("LED")

# Physical button on my SAR bot wired to a spare GPIO pin, used for rerunning and stopping code
button = Pin(1, mode=Pin.IN, pull=Pin.PULL_UP)

# Connect to WiFi. NOTE this is a slow process and often straight
# up fails multiple times in a row. I commented it out sometimes for testing
print("Connecting to WiFi...")
wifi = network.WLAN(network.STA_IF)
wifi.active(True)
wifi.connect("electronics-workshop", "elecwork123")
#wifi.connect("Room 310 Private Wifi", "password123")
while not wifi.isconnected():
    led.on()
    sleep(0.05)
    if button.value() == 0:
        led.off()
        sleep(0.5)
        reset()
print("Finished connecting to WiFi!")
led.off()

# Uncomment to debug the WiFi connection
# import urequests
# astronauts = urequests.get("http://api.open-notify.org/astros.json").json()
# print("Testing Internet Connection: ", astronauts)

# UNUSED (ended up ditching webrepl)
# import webrepl
# import webrepl_setup
# webrepl_setup.main()
# webrepl.start()

import sar_main as sm

# In between tests, wait for button press while blinking LED. NOTE: In a
# perfect world I wanted to use python's asyncio to check for button presses
# *while* the sar_main code is running, but I didn't have time to set that up
while True:
    while True:
        led.on()
        sleep(1)
        if button.value() == 0:
            break
        led.off()
        sleep(1)
        if button.value() == 0:
            break
    sleep(0.5)

    sm.print_and_log("Running sar_main.py!")
    try: # To test certain things I'd replace sm.main() here
        # sm.spam_dist_readings(spin=True)
        # sm.spam_hall_readings()
        sm.main()
        # sm.get_imu_calibrations(spin_motors=False, mag=False)
        # sm.get_imu_calibrations(spin_motors=True, spin_clockwise=True, mag=False)
        # sm.get_imu_calibrations(spin_motors=True, spin_clockwise=False, mag=False)
    except Exception as e:
        sm.print_and_log(str(e))

    sm.turn_everything_off()
    sleep(0.5)
