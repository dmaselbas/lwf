import asyncio
from threading import Thread

from devices.pwm_controller import PWMClient, PWMController
import numpy as np
from time import sleep

from devices.pwm_controller import PWMController
from devices.servo import ServoController


class CameraPTZ:

    def __init__(self, pan_chan: int, tilt_chan: int,
                 pan_min: int = 0, pan_max: int = 180, tilt_min: int = 0, tilt_max: int = 180) -> None:
        self.pan_chan = pan_chan
        self.tilt_chan = tilt_chan
        self.pan_min = pan_min
        self.pan_max = pan_max
        self.tilt_min = tilt_min
        self.tilt_max = tilt_max
        self.tilt_servo = ServoController(tilt_chan)
        self.pan_servo = ServoController(pan_chan)

    def set_pan_position(self, position: int) -> None:
        position = max(self.pan_min, min(position, self.pan_max))
        self.pan_servo.set_angle(position)

    def set_tilt_position(self, position: int) -> None:
        position = max(self.tilt_min, min(position, self.tilt_max))
        self.tilt_servo.set_angle(position)

    def set_position(self, pan: int, tilt: int) -> None:
        self.set_pan_position(pan)
        self.set_tilt_position(tilt)


class CameraController:
    def __init__(self, pan_ch, tilt_ch, move_callback, cam_name) -> None:
        self.ptz = CameraPTZ(pan_ch, tilt_ch)
        self.move_callback = move_callback
        self.cam_name = cam_name
        self.ptz.set_pan_position(90)

    def set_pan_position(self, position: int) -> None:
        self.ptz.set_pan_position(position)
        self.move_callback(self.cam_name, "pan", position)

    def set_tilt_position(self, position: int) -> None:
        self.ptz.set_tilt_position(position)
        self.move_callback(self.cam_name, "tilt", position)
