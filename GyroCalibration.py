from pololu_3pi_2040_robot import robot
import time
import math

motors = robot.Motors()
encoders = robot.Encoders()
display = robot.Display()
imu = robot.IMU()
imu.reset()
imu.enable_default()


programFinished = False

# Robot parameters
ticks_per_rotation = 12
gear_ratio = 29.86
wheel_diameter = 3.2004  # cm
wheel_circumference = math.pi * wheel_diameter
wheel_base = 5.6

GYRO_BIAS_Z = -0.52  # degrees per second, avg, expected nominal drift 0.06 degrees/min

def gyroReading():
    imu.read()  # get fresh gyro data
    current_time = time.ticks_ms()
    
    # time difference in seconds
    dt = (time.ticks_diff(current_time, last_time)) / 1000.0
    last_time = current_time

    # corrected yaw rate
    corrected_rate = imu.gyro.last_reading_dps[2] - GYRO_BIAS_Z

    # integrate to get heading
    angle += corrected_rate * dt

    # keep angle between 0–360°
    angle = angle % 360

    # display
    display.text("Angle: {:.2f}".format(angle), 0, 10)
    display.show()




def print_gyro_calibration():
    while True:
        total = 0.0
        for _ in range(2000):
            imu.read()  # or imu.update(), whatever forces a new read
            total += imu.gyro.last_reading_dps[2]
            time.sleep_ms(1)

        display.text("{:.5f}".format(total / 2000.0), 0, 10)
        display.show()
        time.sleep_ms(2000)
        display.fill(0)





while not programFinished:
    display.fill(0)
    gyroReading()
    # insert 
    # functions here

    programFinished = True
    display.fill(0)

	




