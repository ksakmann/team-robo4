from math import atan

from pid import PID

class YawRateController(object):
    def __init__(self, max_angle_steer, steer_ratio, wheel_base):
        self.max_angle_steer = max_angle_steer
        self.steer_ratio = steer_ratio
        self.wheel_base = wheel_base

    def feedforward_contol(self, v_car, n_yaw):
        if v_car > 0:
            steer = atan(self.wheel_base * n_yaw / v_car) * self.steer_ratio
            steer = min(max(steer, -self.max_angle_steer), self.max_angle_steer)
        else:
            steer = 0

        return steer

    def control(self, v_car_sp, n_yaw_sp):
        return self.feedforward_contol(v_car_sp, n_yaw_sp)