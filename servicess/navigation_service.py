from servicess.collision_avoidance_service import CollisionAvoidanceSystem

class NavigationService:

    def __init__(self, compass, drive, lidar, gps):
        self.collision_avoidance_system = CollisionAvoidanceSystem(
            on_update_callback=self.handle_collision_avoidance_update,
            compass=compass,
            drive=drive,
            lidar=lidar,
            gps=gps
        )
        self.lidar_controller = lidar
        self.compass_controller = compass
        self.gps_controller = gps
        self.drive_controller = drive

    def handle_collision_avoidance_update(self, data):
        # Implement collision avoidance update handling logic here
        pass

    def drive_forward(self, speed):
        self.drive_controller.set_speed(speed)
        self.drive_controller.forward()

    def drive_backward(self, speed):
        self.drive_controller.set_speed(speed)
        self.drive_controller.reverse()

    def drive_left(self, speed):
        self.drive_controller.set_speed(speed)
        self.drive_controller.left()

    def drive_right(self, speed):
        self.drive_controller.set_speed(speed)
        self.drive_controller.right()

    def stop_driving(self):
        self.drive_controller.stop()
