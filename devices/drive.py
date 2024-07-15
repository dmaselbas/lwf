from devices.pwm_controller import PWMController


class DriveController:
    def __init__(self,  pwm_controller:PWMController):
        self.pwm_controller = pwm_controller
        self.l1_forward = 9 + 16
        self.l1_reverse = 10 + 16
        self.l2_pwm = 9
        self.l2_forward = 10
        self.l3_pwm = 1
        self.l3_forward = 12
        self.r1_forward = 11 + 16
        self.r1_reverse = 12 + 16
        self.r2_pwm = 13
        self.r2_forward = 13
        self.r3_pwm = 13
        self.r3_forward = 13
        self.speed = 0
        self.stop()
        self.direction = "forward"
        self.forward()

    def _set_direction(self, left_forward: bool, right_forward: bool):
        self.pwm_controller.set_pwm(self.l2_forward, 0 if left_forward else 4095)
        self.pwm_controller.set_pwm(self.l3_forward, 0 if left_forward else 4095)
        self.pwm_controller.set_pwm(self.r2_forward, 4095 if right_forward else 0)
        self.pwm_controller.set_pwm(self.r3_forward, 4095 if right_forward else 0)

    def _set_speed(self, value):
        value = min(max(value, 0), 4095)
        self.pwm_controller.set_pwm(self.l2_pwm, value)
        self.pwm_controller.set_pwm(self.l3_pwm, value)
        self.pwm_controller.set_pwm(self.r2_pwm, value)
        self.pwm_controller.set_pwm(self.r3_pwm, value)
        if self.direction == "reverse":
            self.pwm_controller.set_pwm(self.l1_forward, 0)
            self.pwm_controller.set_pwm(self.l1_reverse, value)
            self.pwm_controller.set_pwm(self.r1_forward, 0)
            self.pwm_controller.set_pwm(self.r1_reverse, value)
        if self.direction == "forward":
            self.pwm_controller.set_pwm(self.l1_forward, value)
            self.pwm_controller.set_pwm(self.l1_reverse, 0)
            self.pwm_controller.set_pwm(self.r1_forward, value)
            self.pwm_controller.set_pwm(self.r1_reverse, 0)
        if self.direction == "left":
            self.pwm_controller.set_pwm(self.l1_forward, 0)
            self.pwm_controller.set_pwm(self.l1_reverse, value)
            self.pwm_controller.set_pwm(self.r1_forward, value)
            self.pwm_controller.set_pwm(self.r1_reverse, 0)
        if self.direction == "right":
            self.pwm_controller.set_pwm(self.l1_forward, value)
            self.pwm_controller.set_pwm(self.l1_reverse, 0)
            self.pwm_controller.set_pwm(self.r1_forward, 0)
            self.pwm_controller.set_pwm(self.r1_reverse, value)
        if self.direction == "stop":
            self.pwm_controller.set_pwm(self.l1_forward, 0)
            self.pwm_controller.set_pwm(self.l1_reverse, 0)
            self.pwm_controller.set_pwm(self.r1_forward, 0)
            self.pwm_controller.set_pwm(self.r1_reverse, 0)


    def set_speed(self, speed):
        self.speed = speed
        self._set_speed(self.speed)

    def left(self):
        self.direction = "left"
        self._set_direction(False, True)
        self._set_speed(self.speed)

    def right(self):
        self.direction = "right"
        self._set_direction(True, False)
        self._set_speed(self.speed)

    def reverse(self):
        self.direction = "reverse"
        self._set_direction(False, False)
        self._set_speed(self.speed)

    def forward(self):
        self.direction = "forward"
        self._set_direction(True, True)
        self._set_speed(self.speed)

    def stop(self):
        self.direction = "stop"
        self.set_speed(0)
