from machine import Pin
from time import sleep
import network

led = Pin("LED")

wifi = network.WLAN(network.STA_IF)
wifi.active(True)
wifi.connect("electronics-workshop", "elecwork123")
while not wifi.isconnected():
    led.on()
    sleep(0.05)
led.off()


# import webrepl
# import webrepl_setup
#
# webrepl_setup.main()
# webrepl.start()

while True:
    import sar_main
    print("Rerunning sar_main.py!")
    sleep(1)
