from devices.pwm_controller import PWMController


class DriveController:
    def __init__(self,  pwm_controller:PWMController):
        self.pwm_controller = pwm_controller
        self.l1_pwm = 13
        self.l1_forward = 13
        self.l1_reverse = 13
        self.l2_pwm = 13
        self.l2_forward = 13
        self.l2_reverse = 13
        self.l3_pwm = 13
        self.l3_forward = 13
        self.l3_reverse = 13
        self.r1_pwm = 13
        self.r1_forward = 13
        self.r1_reverse = 13
        self.r2_pwm = 13
        self.r2_forward = 13
        self.r2_reverse = 13
        self.r3_pwm = 13
        self.r3_forward = 13
        self.r3_reverse = 13
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
        self.pwm_controller.set_pwm(self.l1_pwm, value)
        self.pwm_controller.set_pwm(self.l2_pwm, value)
        self.pwm_controller.set_pwm(self.l3_pwm, value)
        self.pwm_controller.set_pwm(self.r1_pwm, value)
        self.pwm_controller.set_pwm(self.r2_pwm, value)
        self.pwm_controller.set_pwm(self.r3_pwm, value)

    def set_speed(self, speed):
        self.speed = speed
        self._set_speed(self.speed)

    def move_left(self):
        self.direction = "left"
        self._set_direction(False, True)

    def move_right(self):
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
