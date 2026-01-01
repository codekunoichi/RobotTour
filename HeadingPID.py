from pololu_3pi_2040_robot import robot
import time
import _thread

# --------------------------
# Hardware setup
# --------------------------
motors = robot.Motors()
encoders = robot.Encoders()
imu = robot.IMU()
imu.reset()
imu.enable_default()

# Calibrated gyro bias
GYRO_BIAS_Z = -0.52  # Replace with your measured value

# Global heading variables
angle = 0.0
angle_lock = _thread.allocate_lock()
UPDATE_PERIOD_MS = 20  # Gyro integration period

# --------------------------
# Gyro integration thread
# --------------------------
def gyro_thread():
    global angle
    last_time = time.ticks_ms()
    
    # Short warmup
    print("Gyro warming up 5s...")
    time.sleep_ms(5000)
    print("Gyro ready!")

    while True:
        imu.read()
        current_time = time.ticks_ms()
        dt = time.ticks_diff(current_time, last_time) / 1000.0
        last_time = current_time

        # Corrected Z rate
        corrected_rate = imu.gyro.last_reading_dps[2] - GYRO_BIAS_Z
        
        with angle_lock:
            angle += corrected_rate * dt
            angle %= 360

        time.sleep_ms(UPDATE_PERIOD_MS)

# Thread-safe access
def get_heading():
    with angle_lock:
        return angle

def reset_heading(new_angle=0.0):
    global angle
    with angle_lock:
        angle = new_angle



def DS2(distance_cm, base_speed=580, Kp_enc=2.5, Kp_gyro=0.5,
                   wheel_circ_left=10.15, wheel_circ_right=10.40):
    """
    Drive straight for a specified distance using:
    - Separate left/right wheel circumferences
    - Encoder P-control to correct wheel mismatch
    - Gyro heading correction
    - 10ms update loop
    """

    # Reset encoder counts
    encoders.get_counts(reset=True)
    distance_cm_left = 0.0
    distance_cm_right = 0.0

    # Reset heading
    reset_heading(0.0)
    target_heading = 0.0
    ticks_per_rotation = 12
    gear_ratio = 29.86
    max_speed = 1500

    # Conversion: ticks -> distance in cm
    ticks_per_cm_left = (ticks_per_rotation * gear_ratio) / wheel_circ_left
    ticks_per_cm_right = (ticks_per_rotation * gear_ratio) / wheel_circ_right

    # Determine direction
    forward = distance_cm >= 0

    while True:
        # Read signed encoder deltas
        delta_left, delta_right = encoders.get_counts(reset=True)

        # Convert to cm
        distance_cm_left  += delta_left / ticks_per_cm_left
        distance_cm_right += delta_right / ticks_per_cm_right

        # --- Encoder P-correction ---
        error_enc = distance_cm_left - distance_cm_right
        correction_enc = Kp_enc * error_enc

        # --- Gyro correction ---
        current_heading = get_heading()
        error_heading = target_heading - current_heading
        if error_heading > 180: error_heading -= 360
        if error_heading < -180: error_heading += 360
        correction_gyro = Kp_gyro * error_heading

        # --- Motor speeds ---
        speed_sign = 1 if forward else -1
        left_speed  = speed_sign * base_speed - correction_enc - correction_gyro
        right_speed = speed_sign * base_speed + correction_enc + correction_gyro

        # Clamp motor speeds to max allowed
        left_speed  = max(min(left_speed, max_speed), -max_speed)
        right_speed = max(min(right_speed, max_speed), -max_speed)

        motors.set_speeds(int(left_speed), int(right_speed))

        # --- Check target distance ---
        avg_distance = (distance_cm_left + distance_cm_right) / 2
        if forward and avg_distance >= distance_cm:
            break
        elif not forward and avg_distance <= distance_cm:
            break

        time.sleep_ms(10)

    motors.set_speeds(0, 0)

# --------------------------
# Example usage
# --------------------------
def main():
    # Start gyro thread
    _thread.start_new_thread(gyro_thread, ())

    # Wait for gyro warmup
    time.sleep_ms(6000)

    print("Ready to go!")

    # Drive 50 cm straight
    DS2(25, base_speed=600, Kp_enc=2.5, Kp_gyro=0.7)
    time.sleep_ms(1000)
    DS2(-25, base_speed=550, Kp_enc=2.5, Kp_gyro=0.7)
    time.sleep_ms(1000)
    DS2(25, base_speed=600, Kp_enc=2.5, Kp_gyro=0.7)
    time.sleep_ms(1000)
    DS2(-25, base_speed=550, Kp_enc=2.5, Kp_gyro=0.7)
    time.sleep_ms(1000)
    DS2(25, base_speed=600, Kp_enc=2.5, Kp_gyro=0.7)
    time.sleep_ms(1000)
    DS2(-25, base_speed=550, Kp_enc=2.5, Kp_gyro=0.7)
    time.sleep_ms(1000)
    DS2(25, base_speed=600, Kp_enc=2.5, Kp_gyro=0.7)
    time.sleep_ms(1000)


    print("Drive complete. Current heading:", get_heading())

main()
