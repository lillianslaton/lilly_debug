from mobile_control.sentient_wheel import SentientWheel
from machine import Timer


def clamp(x, min_val, max_val):
    return max(min_val, min(x, max_val))


class RegulatedWheel(SentientWheel):
    def __init__(self, driver_ids: list | tuple, encoder_ids: list | tuple) -> None:
        super().__init__(driver_ids, encoder_ids)
        # Constants
        self.k_p = 0.25
        self.k_i = 0.0
        self.k_d = 0.05
        self.reg_freq = 50  # Hz
        # Variables
        self.reg_vel_counter = 0
        self.duty = 0.0
        self.error = 0.0
        self.prev_error = 0.0
        self.error_inte = 0.0  # integral
        self.error_diff = 0.0  # differentiation
        self.ref_lin_vel = 0.0
        # PID controller config
        self.vel_reg_timer = Timer(
            freq=self.reg_freq,
            mode=Timer.PERIODIC,
            callback=self.regulate_velocity,
        )

    def regulate_velocity(self, timer):
        if self.ref_lin_vel == 0.0 or self.reg_vel_counter > self.reg_freq:
            self.stop()
            self.prev_error = 0.0
        else:
            self.error = self.ref_lin_vel - self.meas_lin_vel  # ang_vel also works
            self.error_inte += self.error
            self.error_diff = self.error - self.prev_error
            self.prev_error = self.error  # UPDATE previous error
            inc_duty = (
                self.k_p * self.error
                + self.k_i * self.error_inte
                + self.k_d * self.error_diff
            )
            inc_duty = clamp(inc_duty, -0.03, 0.03)
            self.duty = self.duty + inc_duty
            if self.duty > 0:
                if self.duty > 1.0:
                    self.duty = 1.0
                self.forward(self.duty)
            else:
                if self.duty < -1.0:
                    self.duty = -1.0
                self.backward(-self.duty)
            self.reg_vel_counter += 1

    def set_velocity(self, ref_lin_vel):
        ref_lin_vel = clamp(ref_lin_vel, -0.8, 0.8)  # restrict top speed
        if ref_lin_vel is not self.ref_lin_vel:
            self.ref_lin_vel = ref_lin_vel
            self.prev_error = 0.0
            self.error_inte = 0.0
            self.reg_vel_counter = 0


if __name__ == "__main__":
    """ Set target velocity to different values to tune PID"""
    from utime import sleep
    from machine import Pin

    # regw = RegulatedWheel(
    #     driver_ids=(16, 17, 18),
    #     encoder_ids=(26, 27),
    # )  # left
    regw = RegulatedWheel(
        driver_ids=(21, 20, 19),
        encoder_ids=(6, 7),
    )  # right

    # LOOP
    for i in range(100):
        if i == 25:  # step up @ t=0.5s
            regw.set_velocity(0.6)
        elif i == 80:  # step down @ t=1.6s
            regw.set_velocity(0.0)
        print(
            f"Reference velocity={regw.ref_lin_vel} m/s, Measured velocity={regw.meas_lin_vel} m/s"
        )
        sleep(0.02)
    # Terminate
    regw.disable()
    print("Wheel disabled")


