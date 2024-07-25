from devices.camera_ptz import CameraController
from devices.pwm_controller import PWMController


class CameraService:

    def __init__(self, pwm_controller: PWMController) -> None:
        self.pwm_controller = pwm_controller
        self.nav_cam = CameraController(pwm_controller, 26, 27)
        self.classification_cam = CameraController(pwm_controller, 2, 3)
