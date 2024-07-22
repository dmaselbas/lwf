from servicess.collision_avoidance_service import CollisionAvoidanceSystem
from devices.drive import DriveController
from devices.compass import HMC5883L
from devices.lidar import LidarController
from devices.gps import GPSController


class NavigationService:

    def __init__(self, compass: HMC5883L, drive: DriveController,
                 lidar: LidarController, gps: GPSController):
        self.collision_avoidance_system = CollisionAvoidanceSystem(
                on_update_callback=self.handle_collision_avoidance_update,
                compass=compass,
                drive=drive,
                lidar=lidar,
                gps=gps
                )
        self.lidar_controller: LidarController = lidar
        self.compass_controller: HMC5883L = compass
        self.gps_controller: GPSController = gps
        self.drive_controller: DriveController = drive

    def handle_collision_avoidance_update(self, data):
        # Implement collision avoidance update handling logic here
        pass

    def drive_forward(self):
        self.drive_controller.forward()

    def drive_backward(self):
        self.drive_controller.reverse()

    def drive_left(self):
        self.drive_controller.left()

    def drive_right(self):
        self.drive_controller.right()

    def stop_driving(self):
        self.drive_controller.stop()
