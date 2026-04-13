# Optimized Fast Stepper Test for Raspberry Pi Pico
# Motor 1: DIR = GP27, STEP = GP26 (Note: check your pin numbers, comments differ from variables below)
# Motor 2: DIR = GP13, STEP = GP12

from machine import Pin
import utime

# === CONFIG ===
# --- Motor 1 Pins & Settings ---
DIR1_PIN = 12
STEP1_PIN = 11
STEPS_VERT = 100000
MIN_DELAY_US_1 = 2000  # Start speed for Motor 1
MAX_SPEED_US_1 = 60    # Target speed for Motor 1
ACCEL_STEPS_1 = 1500   # Acceleration ramp length for Motor 1

# --- Motor 2 Pins & Settings ---
DIR2_PIN = 10
STEP2_PIN = 9
STEPS_HOR = 27500
MIN_DELAY_US_2 = 3000  # Start speed for Motor 2 (example: starts slower)
MAX_SPEED_US_2 = 200   # Target speed for Motor 2 (example: lower max speed)
ACCEL_STEPS_2 = 1000   # Acceleration ramp length for Motor 2


# === Setup Pins ===
dir1_pin = Pin(DIR1_PIN, Pin.OUT)
step1_pin = Pin(STEP1_PIN, Pin.OUT, value=0)

dir2_pin = Pin(DIR2_PIN, Pin.OUT)
step2_pin = Pin(STEP2_PIN, Pin.OUT, value=0)

def single_pulse(step_pin_obj, delay_us):
    """Sends a single step pulse."""
    step_pin_obj.value(1)
    utime.sleep_us(2)  # Most drivers only need 1-2us.
    step_pin_obj.value(0)
    utime.sleep_us(delay_us)

def move_steps(step_pin_obj, dir_pin_obj, n_steps, direction, min_delay_us, max_speed_us, accel_steps):
    """Moves motor using a pre-calculated constant-acceleration speed ramp."""
    
    dir_pin_obj.value(direction)
    utime.sleep_ms(5)

    if n_steps < 2 * accel_steps:
        steps_for_ramp = n_steps // 2
    else:
        steps_for_ramp = accel_steps

    # --- Pre-calculate Linear Speed Ramp ---
    # Converting delay to frequency (speed), interpolating, and back to delay
    ramp_delays = []
    v_start = 1000000 / min_delay_us
    v_target = 1000000 / max_speed_us
    
    for i in range(steps_for_ramp):
        current_v = v_start + (v_target - v_start) * (i / steps_for_ramp)
        ramp_delays.append(int(1000000 / current_v))

    # 1. Acceleration ramp
    for delay in ramp_delays:
        single_pulse(step_pin_obj, delay)

    # 2. Constant speed section
    constant_steps = n_steps - 2 * steps_for_ramp
    for _ in range(constant_steps):
        single_pulse(step_pin_obj, max_speed_us)

    # 3. Deceleration ramp (play the ramp backwards)
    for delay in reversed(ramp_delays):
        single_pulse(step_pin_obj, delay)

# === MAIN EXECUTION ===

print("Moving Motor 1 forward...")
# Pass Motor 1's specific speed variables
move_steps(step1_pin, dir1_pin, STEPS_VERT, 1, MIN_DELAY_US_1, MAX_SPEED_US_1, ACCEL_STEPS_1)
utime.sleep_ms(1000)

print("Moving Motor 1 backward...")
move_steps(step1_pin, dir1_pin, STEPS_VERT, 0, MIN_DELAY_US_1, MAX_SPEED_US_1, ACCEL_STEPS_1)
utime.sleep_ms(1000)

print("Moving Motor 2 forward...")
# Pass Motor 2's specific speed variables
move_steps(step2_pin, dir2_pin, STEPS_HOR, 0, MIN_DELAY_US_2, MAX_SPEED_US_2, ACCEL_STEPS_2)
utime.sleep_ms(1000)

print("Moving Motor 2 backward...")
move_steps(step2_pin, dir2_pin, STEPS_HOR, 1, MIN_DELAY_US_2, MAX_SPEED_US_2, ACCEL_STEPS_2)
utime.sleep_ms(1000)

print("Done.")
