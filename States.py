from pololu_3pi_2040_robot import robot
import time
import math

motors = robot.Motors()
encoders = robot.Encoders()
display = robot.Display()

programFinished = False

# Robot parameters
ticks_per_rotation = 12
gear_ratio = 29.86
wheel_diameter = 3.2004  # cm
wheel_circumference = math.pi * wheel_diameter
wheel_base = 5.6  # distance between wheels in cm (adjust to your bot)

# PID constants (tune these)
Kp_heading = 2.0
Kd_heading = 0.1
Kp_dist = 1.5
Kd_dist = 0.1

# Low-level variables
prev_left = 0
prev_right = 0
prev_heading_error = 0
prev_dist_error = 0

# Convert encoder ticks to cm
def ticks_to_cm(ticks):
    return (ticks / (ticks_per_rotation * gear_ratio)) * wheel_circumference

# Read encoders and return distances in cm
def read_distances():
    counts = encoders.get_counts(reset=True)
    left_cm = ticks_to_cm(counts[0])
    right_cm = ticks_to_cm(counts[1])
    return left_cm, right_cm

# Move straight with encoder-only heading PID
def move_distance(target_cm, base_speed=1000):
    global prev_left, prev_right, prev_heading_error, prev_dist_error

    distance_left = 0
    distance_right = 0
    prev_time = time.ticks_ms()

    finished = False

    while not finished:
        # Read distances
        dl, dr = read_distances()
        distance_left += dl
        distance_right += dr
        avg_distance = (distance_left + distance_right) / 2

        # Heading error: difference between left and right wheels
        heading_error = distance_right - distance_left  # positive → veering left
        dist_error = target_cm - avg_distance

        # Time delta
        current_time = time.ticks_ms()
        dt = max((current_time - prev_time)/1000.0, 0.001)
        prev_time = current_time

        # PID derivative
        heading_deriv = (heading_error - prev_heading_error)/dt
        dist_deriv = (dist_error - prev_dist_error)/dt

        # PID output
        correction = Kp_heading*heading_error + Kd_heading*heading_deriv
        speed = Kp_dist*dist_error + Kd_dist*dist_deriv

        # Combine distance and heading control
        left_speed = speed - correction
        right_speed = speed + correction

        # Clamp motor speeds
        left_speed = max(min(left_speed, base_speed), -base_speed)
        right_speed = max(min(right_speed, base_speed), -base_speed)

        # Apply speeds
        motors.set_speeds(left_speed, right_speed)

        # Update previous errors
        prev_heading_error = heading_error
        prev_dist_error = dist_error

        # Display debug info
        display.fill(0)
        display.text(f"DL: {distance_left:.2f}", 0, 20)
        display.text(f"DR: {distance_right:.2f}", 0, 30)
        display.text(f"Err: {dist_error:.2f}", 0, 40)
        display.show()

        # Stop condition
        if abs(dist_error) < 0.2:
            motors.set_speeds(0,0)
            finished = True

        time.sleep_ms(10)

# Turn in place using encoder differences only
def turn_angle(target_deg, base_speed=800):
    global prev_left, prev_right
    distance_per_deg = (math.pi * wheel_base) / 360  # cm per degree
    target_cm = target_deg * distance_per_deg

    distance_left = 0
    distance_right = 0
    prev_time = time.ticks_ms()

    finished = False

    while not finished:
        dl, dr = read_distances()
        distance_left += dl
        distance_right += dr

        error = target_cm - ((distance_right - distance_left)/2)

        # Simple P control for turning
        correction = Kp_heading * error

        left_speed = -correction
        right_speed = correction

        # Clamp speeds
        left_speed = max(min(left_speed, base_speed), -base_speed)
        right_speed = max(min(right_speed, base_speed), -base_speed)

        motors.set_speeds(left_speed, right_speed)

        # Stop condition
        if abs(error) < 0.5:
            motors.set_speeds(0,0)
            finished = True

        time.sleep_ms(10)

# Example usage:
# move_distance(50)   # move forward 50 cm
# turn_angle(90)      # turn right 90 degrees


while not programFinished:
    start = time.ticks_ms()

    move_distance(50)
    turn_angle(90)
    # insert functions here

    programFinished = True
    display.fill(0)

	




