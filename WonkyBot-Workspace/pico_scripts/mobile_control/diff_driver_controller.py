from mobile_control.regulated_wheel import RegulatedWheel


class DiffDriveController:
    def __init__(self, left_ids, right_ids) -> None:
        self.left_wheel = RegulatedWheel(*left_ids)
        self.right_wheel = RegulatedWheel(*right_ids)

        # Variables
        self.meas_lin_vel = 0.0
        self.meas_ang_vel = 0.0

        # Constants
        self.WHEEL_SEP = 0.477  # wheel separation in meters

    def get_vels(self):
        """
        Get actual velocities
        """
        self.meas_lin_vel = 0.5 * (
            self.left_wheel.meas_lin_vel + self.right_wheel.meas_lin_vel
        )  # robot's linear velocity
        self.meas_ang_vel = (
            self.right_wheel.meas_lin_vel - self.left_wheel.meas_lin_vel
        ) / self.WHEEL_SEP  # robot's angular velocity
        return self.meas_lin_vel, self.meas_ang_vel

    def set_vels(self, target_lin_vel, target_ang_vel):
        """
        Set reference velocities
        """
        left_ref_lin_vel = target_lin_vel - 0.5 * (target_ang_vel * self.WHEEL_SEP)
        right_ref_lin_vel = target_lin_vel + 0.5 * (target_ang_vel * self.WHEEL_SEP)
        self.left_wheel.set_velocity(left_ref_lin_vel)
        self.right_wheel.set_velocity(right_ref_lin_vel)


if __name__ == "__main__":
    from utime import sleep
    from math import pi
    from machine import freq

    # freq(300_000_000)
    # SETUP
    ddc = DiffDriveController(
        left_ids=((16, 17, 18), (26, 27)), right_ids=((21, 20, 19), (6, 7))
    )

    for i in range(500):
        if i >= 24:  # step up @ t=0.5 s
            ddc.set_vels(.3, -0.2)
        meas_lin_vel, meas_ang_vel = ddc.get_vels()
        print(f"Velocity={meas_lin_vel} m/s, {meas_ang_vel} rad/s")
        sleep(0.02)
    ddc.set_vels(0.0, 0.0)

