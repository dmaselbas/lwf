import atexit
import threading
from time import sleep

from devices.compass import CompassClient
from devices.drive import DriveClient


class CourseCorrectionService:

    def __init__(self):
        self.correcting_heading = False
        self.running = True
        self.last_direction = None
        self.locked_heading = None
        self.pre_heading_speed = 0
        self.drive_controller = DriveClient()
        self.compass = CompassClient()
        atexit.register(self.shutdown)  # Ensure the service is properly shut down when the
        self.thread = threading.Thread(target=self.run, daemon=True, name="CourseCorrectionService")
        self.thread.start()

    def shutdown(self):
        print("Shutting down CourseCorrectionService...")
        self.running = False

    def run(self):
        while self.running:
            if "forward" not in self.drive_controller.get_direction() and not self.correcting_heading:
                self.locked_heading = None
            elif "stop" in self.drive_controller.get_direction() or "reverse" in self.drive_controller.get_direction():
                self.locked_heading = None
                self.correcting_heading = False
            else:
                current_heading = self.compass.get_bearing()
                if self.locked_heading is None:
                    self.locked_heading = self.compass.get_bearing()
                    self.pre_heading_speed = self.drive_controller.get_speed()
                    print(f"Locked heading: {self.locked_heading}")
                    print(f"Pre-heading speed: {self.pre_heading_speed}")
                heading_difference = current_heading - self.locked_heading

                # Normalize the heading difference to the range [-180, 180]
                if heading_difference > 180:
                    heading_difference -= 360
                elif heading_difference < -180:
                    heading_difference += 360

                speed = self.drive_controller.get_speed()
                # Check if we need to correct the heading
                if abs(heading_difference) > 2:  # Consider a small threshold to avoid constant corrections
                    self.correcting_heading = True
                    print(f"Heading difference: {heading_difference}")
                    print(f"Speed: {speed}")
                    # Determine the direction and apply the scaled correction
                    if heading_difference > 0:  # Need to turn right
                        if heading_difference < 5:
                            self.drive_controller.forward()
                            self.drive_controller.set_right_speed(speed - (abs(heading_difference) * 0.5))
                        else:
                            self.drive_controller.right()
                    elif heading_difference < 0:  # Need to turn left
                        if heading_difference > -5:
                            self.drive_controller.forward()
                            self.drive_controller.set_left_speed(speed - (abs(heading_difference) * 0.5))
                        else:
                            self.drive_controller.left()
                    print(self.drive_controller.left_speed)
                    print(self.drive_controller.right_speed)
                else:
                    self.drive_controller.forward()
                    self.correcting_heading = False
            sleep(1)


if __name__ == "__main__":
    course_correction_service = CourseCorrectionService()
    while course_correction_service.running:
        sleep(1)
