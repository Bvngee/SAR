from machine import Pin, reset
from time import sleep
import network

led = Pin("LED")

button = Pin(1, mode=Pin.IN, pull=Pin.PULL_UP)

print("Connecting to WiFi...")
wifi = network.WLAN(network.STA_IF)
wifi.active(True)
wifi.connect("electronics-workshop", "elecwork123")
# wifi.connect("iPhone (7)", "P4SSWORD")
# wifi.connect("Room 310 Private Wifi", "password123")
while not wifi.isconnected():
    led.on()
    sleep(0.05)
    if button.value() == 0:
        reset()
print("Finished connecting to WiFi!")
led.off()


# import urequests
# astronauts = urequests.get("http://api.open-notify.org/astros.json").json()
# print("Testing Internet Connection: ", astronauts)


# import webrepl
# import webrepl_setup
#
# webrepl_setup.main()
# webrepl.start()

from sar_main import (
    main,
    get_imu_calibrations,
    print_and_log,
    turn_everything_off,
    spam_dist_readings
)


# get_imu_calibrations(spin_motors=False, mag=False)
# get_imu_calibrations(spin_motors=True, spin_clockwise=True, mag=False)
# get_imu_calibrations(spin_motors=True, spin_clockwise=False, mag=False)
while True:
    try:
        # spam_dist_readings(spin=True)
        main()
    except Exception as e:
        print_and_log(str(e))
    turn_everything_off()
    sleep(0.5)
    while True:
        led.on()
        sleep(1)
        if button.value() == 0:
            break
        led.off()
        sleep(1)
        if button.value() == 0:
            break
    print("Rerunning sar_main.py!")
