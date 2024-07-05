from picamera2 import Picamera2
import cv2

class CameraCaptureController:
    def __init__(self):
        self.camera = Picamera2()
        self.camera.configure(self.camera.create_preview_configuration())
        self.camera.start()

    def get_frame(self):
        frame = self.camera.capture_array()
        return frame
