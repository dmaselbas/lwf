from devices.drive import DriveController

class NavigationService:

    def __init__(self, mqtt_broker_address="10.0.0.1"):
        self.state = {}
        self.drive_controller = DriveController(on_update_callback=self.drive_update_callback, mqtt_broker_address=mqtt_broker_address)

    def drive_update_callback(self, status):
        self.state.update(status)

    def set_speed(self, speed):
        self.drive_controller.set_speed(speed)

    def move_left(self):
        self.drive_controller.move_left()

    def move_right(self):
        self.drive_controller.move_right()

    def reverse(self):
        self.drive_controller.reverse()

    def stop(self):
        self.drive_controller.stop()
