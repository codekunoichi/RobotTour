from pololu_3pi_2040_robot import robot
import time
import _thread
import math

motors = robot.Motors()
encoders = robot.Encoders()
display = robot.Display()
imu = robot.IMU()
imu.reset()
imu.enable_default()

# Global variables
GYRO_BIAS_Z = 0.0
angle = 0.0
angle_lock = _thread.allocate_lock()
gyro_ready = False

# Constants
SAMPLES = 2000  # More samples
WARMUP_TIME = 15_000  # Longer warmup - 15 seconds
UPDATE_PERIOD_MS = 20

def calibrate_gyro():
    global GYRO_BIAS_Z, gyro_ready
    
    display.fill(0)
    display.text("Warmup 15...", 0, 10)
    display.text("KEEP STILL!", 0, 20)
    display.show()
    
    print("Warming up gyro for 15...")
    time.sleep_ms(WARMUP_TIME)

    # Collect samples
    samples = []
    display.fill(0)
    display.text("Calibrating...", 0, 10)
    display.show()
    
    print("Collecting samples...")
    for i in range(SAMPLES):
        imu.read()
        samples.append(imu.gyro.last_reading_dps[2])
        time.sleep_ms(1)
        
        # Progress indicator
        if i % 300 == 0:
            display.fill(0)
            display.text("Calibrating...", 0, 10)
            display.text("{}%".format(int(100*i/SAMPLES)), 0, 20)
            display.show()
    
    # Outlier rejection: remove top/bottom 5%
    samples.sort()
    trim_count = int(SAMPLES * 0.05)
    trimmed = samples[trim_count:-trim_count]
    
    GYRO_BIAS_Z = sum(trimmed) / len(trimmed)
    
    # Calculate standard deviation for diagnostics
    mean = GYRO_BIAS_Z
    variance = sum((x - mean) ** 2 for x in trimmed) / len(trimmed)
    std_dev = math.sqrt(variance)
    
    print("Bias measured:", GYRO_BIAS_Z, "dps")
    print("Std dev:", std_dev, "dps")
    print("Min/Max (trimmed):", trimmed[0], "/", trimmed[-1])
    
    display.fill(0)
    display.text("Bias: {:.3f}".format(GYRO_BIAS_Z), 0, 10)
    display.text("Ready!", 0, 20)
    display.show()
    time.sleep_ms(2000)
    
    gyro_ready = True

def gyro_thread():
    global angle
    last_time = time.ticks_ms()

    while not gyro_ready:
        time.sleep_ms(10)

    while True:
        imu.read()
        current_time = time.ticks_ms()
        dt = time.ticks_diff(current_time, last_time) / 1000.0
        last_time = current_time

        corrected_rate = imu.gyro.last_reading_dps[2] - GYRO_BIAS_Z
        
        with angle_lock:
            angle += corrected_rate * dt
            angle = angle % 360

        time.sleep_ms(UPDATE_PERIOD_MS)

def display_loop():
    while not gyro_ready:
        time.sleep_ms(100)

    while True:
        with angle_lock:
            display_angle = angle
        display.fill(0)
        display.text("Angle: {:.2f}".format(display_angle), 0, 10)
        display.show()
        time.sleep_ms(50)

def main():
    _thread.start_new_thread(gyro_thread, ())
    calibrate_gyro()
    display_loop()

main()