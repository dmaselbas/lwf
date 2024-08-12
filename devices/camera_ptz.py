import asyncio
from threading import Thread

from devices.pwm_controller import PWMClient, PWMController
import numpy as np
from time import sleep

from devices.pwm_controller import PWMController


class CameraPTZ:

    def __init__(self, pan_chan: int, tilt_chan: int,
                 pan_min: int = 0, pan_max: int = 4095, tilt_min: int = 0, tilt_max: int = 4095) -> None:
        self.pwm_controller = PWMClient()
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
    def __init__(self, pan_ch, tilt_ch, move_callback, cam_name) -> None:
        self.pwm_controller = PWMClient()
        self.pan_ch = pan_ch
        self.tilt_ch = tilt_ch
        self.move_callback = move_callback
        self.cam_name = cam_name
        self.pan_position: int = 0
        self.tilt_position: int = 0
        self.set_pan_position(256)
        self.set_tilt_position(256)

    def set_pan_position(self, position: int) -> None:
        self.pan_position = position
        self.pwm_controller.set_pwm(self.pan_ch, position)
        self.move_callback(self.cam_name, "pan", position)

    def set_tilt_position(self, position: int) -> None:
        self.tilt_position = position
        self.pwm_controller.set_pwm(self.tilt_ch, position)
        self.move_callback(self.cam_name, "tilt", position)
