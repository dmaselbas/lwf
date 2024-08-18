import asyncio
from time import sleep

import numpy as np

from devices.pwm_controller import PWMClient


class ServoController:

    def __init__(self, pwm_channel, sleep_time=0.01):
        self.sleep_time = sleep_time
        self.pwm_channel = pwm_channel
        self.min_position = 75
        self.max_position = 750
        self.current_angle = 90
        self.duty_cycles = np.linspace(self.min_position, self.max_position, 180)
        self.pwm = PWMClient()

    def _set_angle(self, angle):
        angle = max(min(angle, 180), 0) - 1
        for angle in np.linspace(self.current_angle, angle, 16, dtype=int):
            duty_cycle = self.duty_cycles[angle]
            self.pwm.set_pwm(self.pwm_channel, int(duty_cycle))
            self.current_angle = angle
            sleep(self.sleep_time)

    def set_angle(self, angle):
        self._set_angle(angle)

    def get_angle(self):
        return self.current_angle
