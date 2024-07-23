from servicess.collision_avoidance_service import CollisionAvoidanceSystem
from devices.drive import DriveController
from devices.lidar import LidarController
from devices.gps import GPSController


class NavigationService:

    def __init__(self, drive: DriveController,
                 lidar: LidarController, gps: GPSController):
        self.collision_avoidance_system = CollisionAvoidanceSystem(
                on_update_callback=self.handle_collision_avoidance_update,
                drive=drive, lidar=lidar, gps=gps )
        self.lidar_controller: LidarController = lidar
        self.gps_controller: GPSController = gps
        self.drive_controller: DriveController = drive

    def handle_collision_avoidance_update(self, data):
        pass

    def get_lidar_data(self):
        return self.lidar_controller.get_last_reading()

    def drive_forward(self):
        if not self.collision_avoidance_system.taking_avoidance_action:
            self.drive_controller.forward()

    def drive_backward(self):
        if not self.collision_avoidance_system.taking_avoidance_action:
            self.drive_controller.reverse()

    def drive_left(self):
        if not self.collision_avoidance_system.taking_avoidance_action:
            self.drive_controller.left()

    def drive_right(self):
        if not self.collision_avoidance_system.taking_avoidance_action:
            self.drive_controller.right()

    def stop_driving(self):
        self.drive_controller.stop()

    def set_drive_speed(self, speed):
        if not self.collision_avoidance_system.taking_avoidance_action:
            self.drive_controller.set_speed(speed)
