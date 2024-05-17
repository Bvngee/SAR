from time import sleep, time
from machine import SoftI2C, Pin, PWM, ADC

led = Pin("LED")
for _ in range(4):
    led.on()
    sleep(0.1)
    led.off()
    sleep(0.1)

button = Pin(1, mode=Pin.IN, pull=Pin.PULL_UP)

from vl53l4cd import VL53L4CD
i2c_vl53l4cd = SoftI2C(sda=Pin(16), scl=Pin(17))
vl53 = VL53L4CD(i2c_vl53l4cd)
vl53.inter_measurement = 0
vl53.timing_budget = 20

from as7341 import AS7341
i2c_as7341 = SoftI2C(sda=Pin(20), scl=Pin(21))
as7341 = AS7341(i2c_as7341)

from drv8833 import DRV8833
frequency = 40000
ain1 = PWM(Pin(14, Pin.OUT), freq=frequency)
ain2 = PWM(Pin(15, Pin.OUT), freq=frequency)
bin1 = PWM(Pin(13, Pin.OUT), freq=frequency)
bin2 = PWM(Pin(12, Pin.OUT), freq=frequency)
drv = DRV8833(ain1, ain2, bin1, bin2)

### DRV5053VAQLPGM
drv5053 = ADC(Pin(26))
def adc_to_gauss(adc_value):
    sensitivity = 1.65 # Calibration factor (mV/Gauss) for DRV5053VAQLPGM
    vref = 3.3 # Vref (reference voltage) of ADC (3.3V for ESP32)
    voltage = adc_value / 4095 * vref # Convert ADC reading to voltage
    return voltage / sensitivity # Convert voltage to Gauss using sensitivity

import web_dashboard as wd
print("Trying to connect to webserver...")
wd.connect_web_server()
wd.id = "99"
wd.init_log()
wd.log(f"Connection established to webserver! @ {time()}")
print("Connected established to webserver!")


def avg(nums) -> float:
    sum = 0
    for i in nums:
        sum += i
    return sum / len(nums)

def reset_sar():
    led.off()
    drv.stop_a()
    drv.stop_b()
    wd.log(f"Resetting! Time: {time()}")

def grid_test():
    wd.init_grid(3)
    wd.set_square(2, 2, "red")
    wd.set_square(1, 1, "blue")

def main():
    # as7341.led = True

    motor_b_adjustment  = 0.805
    throttle = 0.4
    drv.throttle_a(throttle)
    drv.throttle_b(-1*throttle*motor_b_adjustment)

    vl53.start_ranging()

    iter = 0
    numLastDists = 5
    lastDists = [vl53.get_distance() for _ in range(numLastDists)]
    streak = 0
    while True:
        # if iter % 10 == 0:
        #     color_readings = as7341.get_readings()
        #     maxK = ""
        #     maxV = 0
        #     for k, v in color_readings.items():
        #         if v > maxV:
        #             maxV = v
        #             maxK = k
        #     # wd.log(f"Max color: {maxK} {maxV}")

        dist = vl53.get_distance()
        print(f"Dist cm: {dist}")
        wd.log(f"Dist cm: {dist}")

        lastDists.pop(0)
        lastDists.append(dist)

        if dist > avg(lastDists):
            streak += 1
        else:
            streak = 0

        if streak >= 10:
            drv.stop_a(hard=True)
            drv.stop_b(hard=True)
            reset_sar()
            break

        if button.value() == 0:
            reset_sar()
            break
            
        iter += 1

    # as7341.led = False

    while True:
        led.on()
        sleep(1)
        if button.value() == 0:
            break
        led.off()
        sleep(1)
        if button.value() == 0:
            break
