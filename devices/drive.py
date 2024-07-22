from devices.pwm_controller import PWMController


class DriveController:
    def __init__(self,  pwm_controller:PWMController):
        self.pwm_controller = pwm_controller
        self.lf_forward = 1 + 16
        self.lf_reverse = 2 + 16
        self.lr_forward = 3 + 16
        self.lr_reverse = 4 + 16
        self.rf_forward = 5 + 16
        self.rf_reverse = 6 + 16
        self.rr_forward = 7 + 16
        self.rr_reverse = 8 + 16
        self.speed = 0
        self.stop()
        self.direction = "stop"

    def _set_speed(self, value):
        value = min(max(value, 0), 4095)
        if self.direction == "forward":
            self.pwm_controller.set_pwm(self.lf_forward, 0)
            self.pwm_controller.set_pwm(self.lf_reverse, value)
            self.pwm_controller.set_pwm(self.lr_forward, 0)
            self.pwm_controller.set_pwm(self.lr_reverse, value)
            self.pwm_controller.set_pwm(self.rf_forward, 0)
            self.pwm_controller.set_pwm(self.rf_reverse, value)
            self.pwm_controller.set_pwm(self.rr_forward, 0)
            self.pwm_controller.set_pwm(self.rr_reverse, value)
        if self.direction == "reverse":
            self.pwm_controller.set_pwm(self.lf_forward, value)
            self.pwm_controller.set_pwm(self.lf_reverse, 0)
            self.pwm_controller.set_pwm(self.lr_forward, value)
            self.pwm_controller.set_pwm(self.lr_reverse, 0)
            self.pwm_controller.set_pwm(self.rf_forward, value)
            self.pwm_controller.set_pwm(self.rf_reverse, 0)
            self.pwm_controller.set_pwm(self.rr_forward, value)
            self.pwm_controller.set_pwm(self.rr_reverse, 0)
        if self.direction == "right":
            self.pwm_controller.set_pwm(self.lf_forward, 0)
            self.pwm_controller.set_pwm(self.lf_reverse, value)
            self.pwm_controller.set_pwm(self.lr_forward, 0)
            self.pwm_controller.set_pwm(self.lr_reverse, value)
            self.pwm_controller.set_pwm(self.rf_forward, value)
            self.pwm_controller.set_pwm(self.rf_reverse, 0)
            self.pwm_controller.set_pwm(self.rr_forward, value)
            self.pwm_controller.set_pwm(self.rr_reverse, 0)
        if self.direction == "left":
            self.pwm_controller.set_pwm(self.lf_forward, value)
            self.pwm_controller.set_pwm(self.lf_reverse, 0)
            self.pwm_controller.set_pwm(self.lr_forward, value)
            self.pwm_controller.set_pwm(self.lr_reverse, 0)
            self.pwm_controller.set_pwm(self.rf_forward, 0)
            self.pwm_controller.set_pwm(self.rf_reverse, value)
            self.pwm_controller.set_pwm(self.rr_forward, 0)
            self.pwm_controller.set_pwm(self.rr_reverse, value)


    def set_speed(self, speed):
        self.speed = speed
        self._set_speed(self.speed)

    def left(self):
        self.direction = "left"
        self._set_speed(self.speed)

    def right(self):
        self.direction = "right"
        self._set_speed(self.speed)

    def reverse(self):
        self.direction = "reverse"
        self._set_speed(self.speed)

    def forward(self):
        self.direction = "forward"
        self._set_speed(self.speed)

    def stop(self):
        self.direction = "stop"
        self.set_speed(0)
