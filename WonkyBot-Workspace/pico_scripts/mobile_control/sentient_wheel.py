from mobile_control.encoded_motor import EncodedMotor
from machine import Timer
from math import pi


class SentientWheel(EncodedMotor):
    def __init__(self, driver_ids, encoder_ids):
        # Pin configuration
        super().__init__(driver_ids, encoder_ids)
        # Variables
        self.prev_counts = 0
        self.meas_lin_vel = 0.0
        self.meas_ang_vel = 0.0
        # Properties
        self.cpr = 64
        self.gear_ratio = 70
        self.wheel_radius = 0.075  # diameter in mm -> radius in m
        self.freq_meas = 100  # Hz

        # Init timer for velocity probing
        self.vel_meas_timer = Timer(
            mode=Timer.PERIODIC,
            freq=self.freq_meas,
            callback=self._measure_velocity,
        )

    def _measure_velocity(self, timer):
        curr_counts = self.encoder_counts
        delta_counts = curr_counts - self.prev_counts
        counts_per_sec = delta_counts * self.freq_meas  # delta_c / delta_t
        orig_rev_per_sec = counts_per_sec / self.cpr
        orig_rad_per_sec = orig_rev_per_sec * 2 * pi  # original motor shaft velocity
        self.meas_ang_vel = orig_rad_per_sec / self.gear_ratio
        self.meas_lin_vel = self.meas_ang_vel * self.wheel_radius
        self.prev_counts = curr_counts  # UPDATE prev_counts


if __name__ == "__main__":
    from utime import sleep

    # SETUP
    sw = SentientWheel((16, 17, 18), (26, 27))  # left
    # sw = SentientWheel((21, 20, 19), (6, 7))  # right

    # Rotate 1 rev
    # while sw.encoder_counts < sw.gear_ratio * sw.cpr:
    #     sw.forward(0.2)
    # sw.stop

    # LOOP
    for i in range(100):
        sw.forward((i + 1) / 100)
        print(f"angular velocity={sw.meas_ang_vel}, linear velocity={sw.meas_lin_vel}")
        sleep(4 / 100)  # 4 seconds to ramp up
    for i in reversed(range(100)):
        sw.forward((i + 1) / 100)
        print(f"angular velocity={sw.meas_ang_vel}, linear velocity={sw.meas_lin_vel}")
        sleep(4 / 100)  # 4 seconds to ramp down
    for i in range(100):
        sw.backward((i + 1) / 100)
        print(f"angular velocity={sw.meas_ang_vel}, linear velocity={sw.meas_lin_vel}")
        sleep(4 / 100)  # 4 seconds to ramp up
    for i in reversed(range(100)):
        sw.backward((i + 1) / 100)
        print(f"angular velocity={sw.meas_ang_vel}, linear velocity={sw.meas_lin_vel}")
        sleep(4 / 100)  # 4 seconds to ramp down

    # TERMINATE
    sw.disable()
    print("Wheel driver disabled")

