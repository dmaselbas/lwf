from devices.pwm_controller import PWMController


class NavigationCameraController:

    def __init__(self, pwm_controller: PWMController) -> None:
        self.pwm_controller = pwm_controller

    def set_pan_position(self, position: int) -> None:
        self.pwm_controller.set_pwm(16, position)
