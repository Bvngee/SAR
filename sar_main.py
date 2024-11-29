# sar_main.py: the main programs and tests for my SAR bot.

from time import sleep, ticks_ms, ticks_diff, time
import math
from machine import SoftI2C, Pin, PWM, ADC, reset

# Physical button on my SAR bot wired to a spare GPIO pin, used for rerunning and stopping code
button = Pin(1, mode=Pin.IN, pull=Pin.PULL_UP)

led = Pin("LED")
for _ in range(4):
    led.on()
    sleep(0.1)
    led.off()
    sleep(0.1)

# Distance sensor
from vl53l4cd import VL53L4CD
i2c_vl53l4cd = SoftI2C(sda=Pin(16), scl=Pin(17))
vl53 = VL53L4CD(i2c_vl53l4cd)
vl53.inter_measurement = 0
vl53.timing_budget = 20

# Color sensor
from as7341 import AS7341
i2c_as7341 = SoftI2C(sda=Pin(20), scl=Pin(21))
as7341 = AS7341(i2c_as7341)

# Motor Driver
from drv8833 import DRV8833
frequency = 40000
ain1 = PWM(Pin(14, Pin.OUT), freq=frequency)
ain2 = PWM(Pin(15, Pin.OUT), freq=frequency)
bin1 = PWM(Pin(13, Pin.OUT), freq=frequency)
bin2 = PWM(Pin(12, Pin.OUT), freq=frequency)
drv = DRV8833(ain1, ain2, bin1, bin2)
motor_b_adjustment = 0.91 # 0.805
min_throttle = 0.19 # 0.23 0.18

# IMU (Accelerometer, Gyroscope, Magnetometer)
from mpu9250 import MPU9250 # wrapper around ak8963 and mpu6500
from ak8963 import AK8963 # handles magnetometer
from mpu6500 import MPU6500, SF_DEG_S # handles gyroscope & accelerometer
# These are my gyroscope and magnetometer calibration values. This works by
# spinning the IMU in all directions while recording values and then taking the
# averages. Determined by multiple calls to get_imu_calibrations()
gyro_offset = (-6.28721, 2.475767, 0.1901755) # 0.09552677
mag_offset = (10.45459, 52.97549, -11.75391)
mag_scale = (0.7664631, 0.7468262, 0.8948628)
i2c_imu = SoftI2C(sda=Pin(6), scl=Pin(7))
# NOTE: the MPU9250 IMU constructor must run BEFORE internal-sensors
# (ak8963/mpu6500) are initialized, which is why we set these in this order
imu = MPU9250(i2c=i2c_imu)
imu.ak8963 = AK8963( # magnetometer
    i2c=i2c_imu,
    offset=mag_offset,
    scale=mag_scale,
)
imu.mpu6500 = MPU6500( # gyro & accel
    i2c=i2c_imu,
    gyro_offset=gyro_offset,
    gyro_sf=SF_DEG_S # report degrees instead of radians
)

# Hall Effect Sensor (DRV5053VAQLPGM)
drv5053 = ADC(Pin(26))
def adc_to_gauss(adc_value):
    sensitivity = 1.65 # Calibration factor (mV/Gauss) for DRV5053VAQLPGM
    vref = 3.3 # Vref (reference voltage) of ADC (3.3V for ESP32)
    voltage = adc_value / 4095 * vref # Convert ADC reading to voltage
    return voltage / sensitivity # Convert voltage to Gauss using sensitivity


# See SAR Documentation for explanation of the Web Dashboard
import web_dashboard as wd
print("Trying to connect to webserver...")
wd.connect_web_server()
wd.id = "jpn24"
wd.init_grid(4)
# I use the global variable plotX so that every time I plot a point on the
# graph it's one unit to the right of the previous point. Ideally I would use
# timestamps instead but for all practical purposes this is fine
plotX = 0
wd.init_plot()
wd.init_log()
wd.log(f"Connection established to webserver! @ {time()}")
print("Connected established to webserver!")

def avg(nums) -> float:
    sum = 0
    for i in nums:
        sum += i
    return sum / len(nums)

def print_and_log(message: str):
    print(message)
    # uncomment wd.log if wifi is misbehaving or for quick tests
    wd.log(message)

# WORKING, BUT CURRENTLY UNUSED
# This waits until the slope of the distance sensor graph changes signs. That
# is, it waits until the distance readings go from getting closer to getting
# farther or vice versa. Bot should already be spinning
# Parameters:
# - curr_slope_positive: whether to wait for (+)=>(-) or (-)=>(+) slope change
# - last_dists_size: Compare the average of the last n readings to determine a slope change
# - streak_needed: how many times in a row the last readings needs to be >or< than the
#           previous readings to be considered a slope change
def wait_until_dist_slope_change(
    curr_slope_positive: bool,
    streak_needed: int = 5,
    last_dists_size: int = 5,
):
    global plotX
    # NOTE: This doesn't start motors or anything. Caller has to do that
    streak = 0
    last_dists = [vl53.get_distance(wait_for_new_data=False) for _ in range(last_dists_size)]
    while True:
        dist = vl53.get_distance()
        # wd.plot(plotX, dist, "red")
        print_and_log(f"Dist: {dist}")
        plotX += 1
        if curr_slope_positive:
            if dist < avg(last_dists):
                streak += 1
            else:
                streak = 0
        else:
            if dist > avg(last_dists):
                streak += 1
            else:
                streak = 0

        if streak >= streak_needed:
            break

        last_dists.pop(0)
        last_dists.append(dist)


# This waits until the slope of the distance sensor graph approaches zero. It
# has the same effect as wait_until_dist_slope_change(), but instead of waiting
# until after a slope change, it waits until just before the slope chnage. Bot
# should already be spinning
# Parameters:
# - curr_slope_positive: whether to wait for (+)=>(-) or (-)=>(+) slope change
# - max_stopping_diff: the slope delta thats considered close enough to zero
# - list_size: keep track of this many readings, average them to determine if
#           slope is close enough to zero
def wait_until_dist_slope_near_zero(
    curr_slope_positive: bool,
    max_stopping_diff: float = 1,
    list_size: int = 5,
):
    def get_and_plot_dist():
        global plotX
        dist = vl53.get_distance()
        # wd.plot(plotX, dist, "magenta")
        plotX += 1
        return dist

    last_n_dists = [get_and_plot_dist() for _ in range(list_size)]
    while True:
        n_dists = [get_and_plot_dist() for _ in range(list_size)]
        if curr_slope_positive:
            if 0 < avg(n_dists) - avg(last_n_dists) < max_stopping_diff:
                break
        else:
            if -max_stopping_diff < avg(n_dists) - avg(last_n_dists) < 0:
                break
                
        last_n_dists = n_dists

        if button.value() == 0:
            reset()


# WORKING, BUT CURRENTLY UNUSED
# This uses the IMU's gyroscope to turn approximately n degrees in one
# direction. It does this by integrating the gyro's Z-axis over time, which is
# angular velocity, to get the current angle turned and stops when close to n
# deg. It also scales the speed of the motors quadratically, slowing them down
# as it approaches the full n degree turn as an attempt to increase precision.
# It's surprisingly close to perfect, however I did not end up using it because
# the error builds up super fast over when used multiple times in a row.
def turn_n_degrees_gyro(degrees: float):
    max_throttle = 0.35
    # drv.throttle_a(max_throttle)
    # drv.throttle_b(-max_throttle)
    #initial_mag_deg = mag_to_deg(imu.magnetic)
    gyro_z_deg_traveled = 0
    last_tick_us = ticks_ms()
    while gyro_z_deg_traveled < degrees-0.05:
        _, _, gyro_z = imu.gyro
        gyro_z_deg_traveled += abs(ticks_diff(ticks_ms(), last_tick_us)*0.001*gyro_z)
        last_tick_us = ticks_ms()

        # clamp from 0.35 to 0.18, quadratically
        speed = (min_throttle - 0.05) + (1.0 - math.pow(gyro_z_deg_traveled/90, 2))*(max_throttle - (min_throttle - 0.05))

        #print_and_log(f"speed: {speed}")
        drv.throttle_a(speed)
        drv.throttle_b(-speed)

        #print_and_log(f"mag: {mag_to_deg(imu.magnetic) - initial_mag_deg}, gyro: {gyro_z_deg_traveled}")
    drv.stop_a()
    drv.stop_b()


# Ramps down throttle from `from_throttle` to 0 using the given parameters to control speed
def slow_down_from(from_throttle: float, delay: float = 0.02, delta: float = 0.05):
    throttle = from_throttle
    while throttle > 0:
        drv.throttle_a(throttle)
        drv.throttle_b(throttle)
        throttle -= delta
        sleep(delay)
    drv.stop_a()
    drv.stop_b()

# Takes in a tuple of magnetometer readings on the x/y/z axis and spits out
# heading aka yaw (degrees rotated around the verticle axis). Output is from
# -180 to 180 degrees where 0deg is geomagnetic north
def mag_to_deg(mag: tuple) -> float:
    mag_x, mag_y, _ = mag
    deg = math.degrees(math.atan2(mag_y, mag_x)) - 90
    # deg is from -180 to 180 where 0 is magnetic north
    if deg < -180: deg += 360
    return deg

# Print and log hall effect sensor readings in a loop, used for testing
def spam_hall_readings():
    while True:
        print_and_log(str(drv5053.read_u16()))
        sleep(0.05)

# Print and log distance sensor readings in a loop, used for testing
def spam_dist_readings(spin: bool = False):
    global plotX
    if spin:
        drv.throttle_a(-min_throttle)
        drv.throttle_b(min_throttle)
    vl53.start_ranging()
    while True:
        dist = vl53.get_distance()
        print_and_log(str(dist))
        # w.plot(plotX, dist, "blue")
        plotX += 1
        if button.value() == 0:
            break
    if spin:
        drv.stop_a()
        drv.stop_b()

# Returns computed calibration values from the IMU library's internal
# calibration functions
def get_imu_calibrations(spin_motors: bool = True, spin_clockwise: bool = True, mag: bool = True, gyro: bool = True):
    if spin_motors:
        if spin_clockwise:
            drv.throttle_a(-1*0.7)
            drv.throttle_b(0.7)
        else: 
            drv.throttle_a(0.7)
            drv.throttle_b(-1*0.7)
    if mag:
        print_and_log("IMU: Calibrating ak8963 (mag)!")
        mag_offset, mag_scale = imu.ak8963.calibrate(count=1000, delay=20) # magnetometer
        print_and_log(f"ak8963 (mag): offset {mag_offset}, scale {mag_scale}")
    if gyro:
        print_and_log("IMU: Calibrating mpu6500 (gyro)!")
        gyro_offset = imu.mpu6500.calibrate(count=1000, delay=20) # gyro & accel
        print_and_log(f"mpu650 (gyro): offset {gyro_offset}")
    sleep(0.5)
    if spin_motors:
        drv.stop_a()
        drv.stop_b()

# Stop motors, turn off LED, turn off color sensor LED, stop distance sensor readings
def turn_everything_off():
    drv.stop_a()
    drv.stop_b()
    led.off()
    as7341.led = False
    vl53.stop_ranging()


# This drives the bot forward until it's `dist` cm's away the wall. While doing
# so, it uses the Z-axis of the gyroscope (angular velocity) to dynically
# correct for swerving and motor inconsistencies. This is, if it's detected
# that the bot # is swerving right, it increases the throttle to the right
# motor to account for it. If it's instead detected that the bot is swerving
# left, it increase throttle to the left motor. This could be thought of as a
# rudamentary PID alrogithm; in fact, I tried incorporating Integral
# calculations as well (to determine the correction factor) but never had time
# to make it work well.
# Additionally, if the hall effect sensor value is within a certain magnetic
# field strength, it turns the LED on.
def drive_until_distance_corrected(dist: float, current_row: int):
    last_hall_values = [drv5053.read_u16() for _ in range(5)]
    throttle = 0.9
    n = 1 # need to balance delay to correction with smoothing of outliers/noise - not sure on this yet
    last_gyro_zs = [imu.gyro[2] for _ in range(n)]
    last_degs = [mag_to_deg(imu.magnetic) for _ in range(n)]
    # correction_integral = 0
    # last_correction = 0
    drv.throttle_a(throttle)
    drv.throttle_b(throttle)
    while True:
        gyro_z = imu.gyro[2]
        last_gyro_zs.pop(0)
        last_gyro_zs.append(gyro_z)

        deg = mag_to_deg(imu.magnetic)
        last_degs.pop(0)
        last_degs.append(deg)

        ### P
        # 220->100 makes it wobble (maybe useful for ful PID?)
        gyro_z_throttle_correction = max(min(avg(last_gyro_zs)/210, 0.3), -0.3)

        # ### I
        # correction_integral += gyro_z_throttle_correction
        # print_and_log(f"throttle_correction_integral: {throttle_correction_integral}")

        final_correction = gyro_z_throttle_correction
        a = throttle + final_correction
        b = throttle - final_correction
        # If a or b is out of [-1, 1], add the difference to the other (instead of just clamping)
        if a > 1.0:
            b -= a - 1.0
            a = 1.0
        elif a < -1.0:
            b += -1.0 - a
            a = -1.0
        if b > 1.0:
            a -= b - 1.0
            b = 1.0
        elif b < -1.0:
            b = -1.0
            a += -1.0 - b
        drv.throttle_a(a)
        drv.throttle_b(b)

        if button.value() == 0:
            reset()

        d = vl53.get_distance(wait_for_new_data=False)
        if d < dist + 15:
            break

        last_hall_values.pop(0)
        last_hall_values.append(drv5053.read_u16())

        avg_hall_value = avg(last_hall_values)
        # print_and_log(f"hall: {avg_hall_value}")
        # wd.log(f"hall: {avg_hall_value}")
        global plotX
        # wd.plot(plotX, avg_hall_value, "green")
        plotX += 1
        if avg_hall_value > 31000:
            # print_and_log("magnet")
            # this is not reliable yet...
            # x = int(d / 15)
            # wd.set_square(x, current_row, "magnet")
            led.on()
        else:
            led.off()

    slow_down_from(throttle)


# Turns the bot 90 degrees by waiting for the slope of the distance sensor
# graph to approach zero two separate times. This takes advantage of the
# box arena having flat walls and the observation we made that while
# turning in a circle the distance sensor readings graph has two clear,
# easily measurable peaks.
def turn_90_degrees_dist(spin_direction: int):
    sleep(0.5)
    drv.throttle_a(spin_direction*min_throttle)
    drv.throttle_b(-spin_direction*min_throttle*motor_b_adjustment)
    wait_until_dist_slope_near_zero(
        curr_slope_positive=True,
        max_stopping_diff=1.7,
        list_size=3,
    )
    wait_until_dist_slope_near_zero(
        curr_slope_positive=False,
        max_stopping_diff=1.0, #0.8
        list_size=5,
    )
    drv.stop_a(hard=True)
    drv.stop_b(hard=True)
    sleep(0.05)
    # # Old code: waits for the distance graph's slope to *change sings* twice.
    # wait_until_dist_slope_change(curr_slope_positive=True)
    # wait_until_dist_slope_change(curr_slope_positive=False)
    # # Old code: the bot would over-turn past 90degrees; this would
    # # attempt to correct that by spinning briefly in the opposite direction
    # drv.throttle_a(-spin_direction*min_throttle)
    # drv.throttle_b(spin_direction*min_throttle*motor_b_adjustment)
    # sleep(0.26) # 0.18
    # drv.stop_a(hard=True)
    # drv.stop_b(hard=True)
    # sleep(0.5)


# Main loop.
# Spins towards magnetic north (which the arena is aligned to), then drives in
# a snake pattern over all squares in the arena while searching for the magnet
# hidden underneath.
def main():
    as7341.led = False # True

    vl53.start_ranging()

    # Spin towards magnetic north (I commented this out for testing)
    drv.throttle_a(min_throttle)
    drv.throttle_b(-1*min_throttle*motor_b_adjustment)
    while True:
        deg = mag_to_deg(imu.magnetic)
        if abs(deg) < 3:
            break
    drv.stop_a(hard=True)
    drv.stop_b(hard=True)
    print_and_log("Found magnetic north!")

    sleep(1)

    # Arbitrary hardcoded centimeter values that worked generally ok for the
    # rough draft arena we had at the time
    dists_to_wall = [35, 18, 4]
    short_wall_dist = dists_to_wall[2]
    for i in range(4):
        spin_direction = i % 2 == 0 and 1 or -1
        print_and_log(f"driving forward {i+1}")
        drive_until_distance_corrected(short_wall_dist, 3-i)
        
        print_and_log(f"turning 90 {i+1}")
        turn_90_degrees_dist(spin_direction)

        if i < 3:
            print_and_log(f"driving forward {i+1}")
            drive_until_distance_corrected(dists_to_wall[i], 3-i)

            print_and_log(f"turning 90 {i+1}")
            turn_90_degrees_dist(spin_direction)

    turn_everything_off()
    print_and_log("sar_main completed!")

