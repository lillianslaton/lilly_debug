import sys
from utime import sleep_ms, ticks_us
import select
from machine import freq, Pin
from diff_drive_controller import DiffDriveController
from stepper_motor import StepperMotor

# SETUP
freq(240_000_000)  # orignal frequency: 150MHz
# Modules config
ddc = DiffDriveController(
    left_ids=((3, 2, 4), (21, 20)),
    right_ids=((6, 7, 8), (11, 10)),
)
s = StepperMotor(
    dir_id=27,
    step_id=26,
    en_id=22,
)
a = StepperMotor(
    dir_id=13,
    step_id=12,
    en_id=28,
)
ol = Pin("WL_GPIO0", Pin.OUT)
in_msg_poll = select.poll()
in_msg_poll.register(sys.stdin, select.POLLIN)
event = in_msg_poll.poll()
# Variables
dev_name = "Pico"
out_msg = f"{dev_name} {ticks_us()}\n"
targ_lin_vel = 0.0
targ_ang_vel = 0.0
step_dir = 0
tic = ticks_us()

# LOOP
while True:
    # read data from serial
    for msg, _ in event:
        buffer = msg.readline().strip().split(",")
        if len(buffer) == 4:
            targ_lin_vel = float(buffer[0])
            targ_ang_vel = float(buffer[1])
            step_dir = int(buffer[2])
            step2_dir = int(buffer[3])
            ddc.set_vels(targ_lin_vel, targ_ang_vel)
            s.set_dir(step_dir)
            a.set_dir(step2_dir)

    toc = ticks_us()
    if toc - tic >= 10_000:  # 10 ms period, 100 Hz
        # out_msg = f"[{dev_name} {toc}]: {ddc.get_vels()}\n"
        # out_msg = f"[{dev_name} {toc}]: {targ_lin_vel}, {targ_ang_vel}\n"
        meas_lin_vel, meas_ang_vel = ddc.get_vels()
        
        #Adding Deadband - xero out tiny noise values when the robot should be still
        DEADBAND = 0.01 #this is in meters/sec... may need to tune later if neccessary
        if abs(meas_lin_vel) < DEADBAND:
            meas_lin_vel = 0.0
        if abs(meas_ang_vel) < DEADBAND:
            meas_ang_vel = 0.0
        out_msg = f"{meas_lin_vel}, {meas_ang_vel}\n"
        sys.stdout.write(out_msg)
        tic = ticks_us()
        ol.toggle()
