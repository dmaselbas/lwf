from devices.pwm_controller import PWMController


class DriveController:
    def __init__(self,  pwm_controller:PWMController):
        self.pwm_controller = pwm_controller
        self.l1_pwm = 10 + 16
        self.l1_forward = 11 + 16
        self.l1_reverse = 12 + 16
        self.l2_pwm = 1
        self.l2_forward = 2
        self.l3_pwm = 9
        self.l3_forward = 10
        self.r1_pwm = 14 + 16
        self.r1_forward = 15 + 16
        self.r1_reverse = 16 + 16
        self.r2_pwm = 13
        self.r2_forward = 13
        self.r3_pwm = 13
        self.r3_forward = 13
        self.speed = 0
        self.stop()
        self.direction = "forward"
        self.forward()

    def _set_direction(self, left_forward: bool, right_forward: bool):
        self.pwm_controller.set_pwm(self.l1_forward, 4096 if left_forward else 0)
        self.pwm_controller.set_pwm(self.l1_reverse, 0 if left_forward else 4096)
        self.pwm_controller.set_pwm(self.l2_forward, 4096 if left_forward else 0)
        self.pwm_controller.set_pwm(self.l3_forward, 4096 if left_forward else 0)
        self.pwm_controller.set_pwm(self.r1_forward, 4096 if right_forward else 0)
        self.pwm_controller.set_pwm(self.r1_reverse, 0 if right_forward else 4096)
        self.pwm_controller.set_pwm(self.r2_forward, 4096 if right_forward else 0)
        self.pwm_controller.set_pwm(self.r3_forward, 4096 if right_forward else 0)

    def _set_speed(self, value):
        value = min(max(value, 0), 4096)
        self.pwm_controller.set_pwm(self.l1_pwm, value)
        self.pwm_controller.set_pwm(self.l2_pwm, value)
        self.pwm_controller.set_pwm(self.l3_pwm, value)
        self.pwm_controller.set_pwm(self.r1_pwm, value)
        self.pwm_controller.set_pwm(self.r2_pwm, value)
        self.pwm_controller.set_pwm(self.r3_pwm, value)

    def set_speed(self, speed):
        self.speed = speed
        self._set_speed(self.speed)

    def left(self):
        self.direction = "left"
        self._set_direction(False, True)

    def right(self):
        self.direction = "right"
        self._set_direction(True, False)

    def reverse(self):
        self.direction = "reverse"
        self._set_direction(False, False)

    def forward(self):
        self.direction = "forward"
        self._set_direction(True, True)

    def stop(self):
        self.set_speed(0)
