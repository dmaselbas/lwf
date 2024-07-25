import asyncio
from threading import Thread

from devices.pwm_controller import PWMController
import numpy as np
from time import sleep

from devices.pwm_controller import PWMController


class CameraPTZ:

    def __init__(self, pwm_controller: PWMController, pan_chan: int, tilt_chan: int,
                 pan_min: int = 0, pan_max: int = 4095, tilt_min: int = 0, tilt_max: int = 4095) -> None:
        self.pwm_controller = pwm_controller
        self.pan_chan = pan_chan
        self.tilt_chan = tilt_chan
        self.pan_min = pan_min
        self.pan_max = pan_max
        self.tilt_min = tilt_min
        self.tilt_max = tilt_max

    def set_pan_position(self, position: int) -> None:
        position = max(self.pan_min, min(position, self.pan_max))
        self.pwm_controller.set_pwm(self.pan_chan, position)

    def set_tilt_position(self, position: int) -> None:
        position = max(self.tilt_min, min(position, self.tilt_max))
        self.pwm_controller.set_pwm(self.tilt_chan, position)


class CameraController:

    def __init__(self, pwm_controller: PWMController, pan_ch, tilt_ch) -> None:
        self.pwm_controller = pwm_controller
        self.camera_ptz = CameraPTZ(pwm_controller, pan_chan=pan_ch, tilt_chan=tilt_ch)
        self.pan_position = 0
        self.tilt_position = 0

        Thread(target=self.home).start()

    def home(self) -> None:
        positions = np.linspace(0, 4095, 200, dtype=np.int16)
        for position in positions:
            self.camera_ptz.set_pan_position(position)
            self.camera_ptz.set_tilt_position(position)
        for position in reversed(positions):
            self.camera_ptz.set_pan_position(position)
            self.camera_ptz.set_tilt_position(position)
        self.pan_position = positions[50]
        self.tilt_position = positions[50]
        self.camera_ptz.set_pan_position(self.pan_position)
        self.camera_ptz.set_tilt_position(self.tilt_position)

    def set_pan_position(self, position: int) -> None:
        self.pan_position = position
        self.camera_ptz.set_pan_position(self.pan_position)

    def set_tilt_position(self, position: int) -> None:
        self.tilt_position = position
        self.camera_ptz.set_tilt_position(self.tilt_position)
