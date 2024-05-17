from machine import Pin
from time import sleep
import network

led = Pin("LED")

# print("Connecting to WiFi...")
# wifi = network.WLAN(network.STA_IF)
# wifi.active(True)
# wifi.connect("electronics-workshop", "elecwork123")
# # wifi.connect("iPhone (7)", "P4SSWORD")
# # wifi.connect("Room 310 Private Wifi", "password123")
# # wifi.connect("AUSDlab", "StuDevRuckus")
# while not wifi.isconnected():
#     led.on()
#     sleep(0.05)
# print("Finished connecting to WiFi!")
# led.off()


# import urequests
# astronauts = urequests.get("http://api.open-notify.org/astros.json").json()
# print("Testing Internet Connection: ", astronauts)


# import webrepl
# import webrepl_setup
#
# webrepl_setup.main()
# webrepl.start()

from sar_main import main
# grid_test()

while True:
    main()
    print("Rerunning sar_main.py!")
    sleep(1)
