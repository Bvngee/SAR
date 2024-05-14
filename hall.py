from machine import ADC, Pin

### DRV5053VAQLPGM
drv5053 = ADC(Pin(26))

def adc_to_gauss(adc_value):
    sensitivity = 2.3 # Calibration factor (mV/Gauss) for DRV5053CAQLPGM # 1.65 / 1.3
    vref = 3.3 # Vref (reference voltage) of ADC (3.3V for ESP32/RP2040)
    voltage = (adc_value * vref) / 2**16 # Convert ADC reading to voltage
    # print("voltage:", voltage)
    return (voltage - 1) / sensitivity # Convert voltage to Gauss using sensitivity

# def adc_to_gauss_but_good(adc):
#     return (adc - (2**16 - 1) / 3) / 23.9

while True:
    # print(adc_to_gauss(drv5053.read_u16()))
    print(drv5053.read_u16())
