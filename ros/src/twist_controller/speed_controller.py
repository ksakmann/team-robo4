
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class SpeedController(object):
    def __init__(self):
        kp = 1
        ki = 0
        kd = 0
        is_initialized = False # TODO Implement intialization (handle dt)
        self.pid = pid(kp, ki, kd, min=-1, max=1)

    def control(self, demand, x, time_step):
        error = demand - x
        val = self.pid.step(error, time_step)
        
        if val > 0:
            throttle = val
            brake = 0
        else:
            throttle = 0
            brake = -val * self.brake_torque_gain 

        return throttle, brake
