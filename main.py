from machine import Pin
from time import sleep
import network

led = Pin("LED")

print("Connecting to WiFi...")
wifi = network.WLAN(network.STA_IF)
wifi.active(True)
wifi.connect("electronics-workshop", "elecwork123")
while not wifi.isconnected():
    led.on()
    sleep(0.05)
print("Finished connecting to WiFi!")
led.off()


# import webrepl
# import webrepl_setup
#
# webrepl_setup.main()
# webrepl.start()

from sar_main import main
while True:
    main()
    print("Rerunning sar_main.py!")
    sleep(1)
