from datetime import datetime

import cv2
from time import sleep
from pathlib import Path
from devices.laser import LaserController


class TrainingDataCollector:

    def __init__(self, laser_controller: LaserController):
        self.video_capture = cv2.VideoCapture(10)
        self.laser_controller = laser_controller

        self.positive_images_dir = Path("/var/training/img/positive")
        self.negative_images_dir = Path("/var/training/img/negative")

        self.negative_images_dir.mkdir(parents=True, exist_ok=True)
        self.positive_images_dir.mkdir(parents=True, exist_ok=True)
        self.running = True


    def shutdown(self):
        self.running = False
        self.video_capture.release()


    def run(self):
        while self.running:
            ret, frame = self.video_capture.read()

            if not ret:
                print("Error: Unable to read frame from video stream")
                break
            frame = cv2.flip(frame, -1)
            jpeg_image = cv2.imencode('.jpg', frame)
            if self.laser_controller.laser_on_status:
                self.laser_controller.off()
                cv2.imwrite(self.positive_images_dir / f"{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg", jpeg_image)
                self.laser_controller.on()
            else:
                cv2.imwrite(self.negative_images_dir / f"{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg", jpeg_image)

            sleep(1)
