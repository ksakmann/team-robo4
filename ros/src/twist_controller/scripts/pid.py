MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp_pos, kp_neg, ki, kd, min=MIN_NUM, max=MAX_NUM):
        self.kp_pos = kp_pos
        self.kp_neg = kp_neg
        self.ki = ki
        self.kd = kd
        self.min = min
        self.max = max

        # Initialization
        self.reset()

    def reset(self):
        self.error      = 0.0
        self.i_error    = 0.0
        self.d_error    = 0.0
        self.last_error = 0.0

    def step(self, error, sample_time):
        # self.last_i_error = self.i_error
        self.error = error

        i_error = self.i_error + self.error * sample_time;
        self.d_error = (self.error - self.last_error) / sample_time;

        if error > 0:
            actuator = self.kp_pos * self.error + self.ki * self.i_error + self.kd * self.d_error;
        else:
            actuator = self.kp_neg * self.error + self.ki * self.i_error + self.kd * self.d_error;

        # TODO Implement anti-windup
        if actuator > self.max:
            self.actuator = self.max
        elif actuator < self.min:
            self.actuator = self.min
        else:
            self.actuator = actuator
            self.i_error = i_error

        self.last_error = self.error

        return self.actuator
