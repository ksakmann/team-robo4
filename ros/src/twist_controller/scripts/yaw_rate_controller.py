from math import atan

from pid import PID

class YawRateController(object):
    def __init__(self, max_angle_steer, steer_ratio, wheel_base):
        self.max_angle_steer = max_angle_steer
        self.steer_ratio = steer_ratio
        self.wheel_base = wheel_base
        kp = 0
        ki = 0
        kd = 0
        self.pid = PID(kp, kp, ki, kd, min=-self.max_angle_steer, max=self.max_angle_steer)

    def feedforward_contol(self, v_car, n_yaw):
        if v_car > 0:
            steer = atan(self.wheel_base * n_yaw / v_car) * self.steer_ratio
            steer = min(max(steer, -self.max_angle_steer), self.max_angle_steer)
        else:
            steer = 0

        return steer


    def feedback_control(self, demand, x, time_step):
        error = demand - x
        steer = self.pid.step(error, time_step)

        return steer


    def control(self, v_car_sp, n_yaw_sp):
        return self.feedforward_contol(v_car_sp, n_yaw_sp)


# from math import atan

# class YawController(object):
    # def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
    #     self.wheel_base = wheel_base
    #     self.steer_ratio = steer_ratio
    #     self.min_speed = min_speed
    #     self.max_lat_accel = max_lat_accel

    #     self.min_angle = -max_steer_angle
    #     self.max_angle = max_steer_angle


    # def get_angle(self, radius):
    #     angle = atan(self.wheel_base / radius) * self.steer_ratio
    #     return max(self.min_angle, min(self.max_angle, angle))

    # def get_steering(self, linear_velocity, angular_velocity, current_velocity):
    #     angular_velocity = current_velocity * angular_velocity / linear_velocity if abs(linear_velocity) > 0. else 0.

    #     if abs(current_velocity) > 0.1:
    #         max_yaw_rate = abs(self.max_lat_accel / current_velocity);
    #         angular_velocity = max(-max_yaw_rate, min(max_yaw_rate, angular_velocity))

    #     return self.get_angle(max(current_velocity, self.min_speed) / angular_velocity) if abs(angular_velocity) > 0. else 0.0;
