import json
import threading
import time
from typing import Callable

from devices.compass import HMC5883L
from devices.drive import DriveController
from devices.gps import GPSController
from devices.lidar import LidarController
from devices.pwm_controller import PWMController


class CollisionAvoidanceSystem:

    def __init__(self, on_update_callback: Callable,
                 compass: HMC5883L,
                 drive: DriveController,
                 lidar: LidarController,
                 gps: GPSController):
        self.collision_probability = 1
        self.on_update_callback = on_update_callback

        self.lidar = lidar
        self.gps = gps
        self.compass = compass
        self.drive = drive
        self.lidar_data = None
        self.gps_data = None
        self.running = True

        self.thread = threading.Thread(target=self.run, daemon=True, name="CollisionAvoidanceSystem")
        self.thread.start()

    def shutdown(self):
        self.running = False

    def handle_lidar_update(self, scan_data):
        print(f"Received lidar update: {scan_data}")
        self.lidar_data = json.loads(scan_data)
        self.calculate_collision_probability()

    def handle_gps_update(self, gps_data):
        print(f"Received gps update: {gps_data}")
        self.gps_data = json.loads(gps_data)

    def avoid_collision(self):
        self.drive.set_speed(0)

    def calculate_collision_probability(self):
        # Implement collision probability calculation logic here
        return 0.5  # Placeholder value for demonstration purposes

    def run(self):
        while self.running:
            if self.lidar_data and self.gps_data:
                self.calculate_collision_probability()
                self.on_update_callback({
                    "lidar": self.lidar_data,
                    "gps": self.gps_data,
                    "compass": self.compass.read_magnetic_field(),
                    "collision_probability": self.collision_probability
                })

            time.sleep(1)
