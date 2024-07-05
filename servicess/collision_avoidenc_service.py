import json
from typing import Callable

from paho import mqtt

from devices.drive import DriveController
from devices.gps import GPSController
from devices.lidar import LidarController


class CollisionAvoidanceSystem:

    def __init__(self,
                 on_update_callback=Callable,
                 on_online_callback=Callable,
                 on_offline_callback=Callable):
        self.on_update_callback = on_update_callback
        self.on_online_callback = on_online_callback
        self.on_offline_callback = on_offline_callback

        self.lidar = LidarController(self.handle_lidar_update)
        self.gps = GPSController(self.handle_gps_update)
        self.drive = DriveController(self.handle_drive_update)

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect("10.0.0.2", 1883)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.loop_start()

        self.lidar_data = dict()
        self.gps_data = dict()
        self.is_online = False
        self.drive_data = {"speed": 0, "direction": "forward"}


    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected with result code {rc}")

    def on_message(self, client, userdata, message):
        message = message.payload.decode()
        print(f"Message received on topic {message.topic}: {message}")

    def set_online_status(self, status: bool):
        self.is_online = status
        self.mqtt_client.publish("/dev/system/collision/online", str(status))
        if status:
            self.on_online_callback()
        else:
            self.on_offline_callback()

    def publish_collision_update(self, message: dict):
        self.mqtt_client.publish("/dev/system/collision/update", json.loads(message))

    def handle_drive_update(self, message):
        print(f"Received drive update: {message}")
        self.drive_data = json.loads(message)
        self.calculate_collision_probability()

    def handle_lidar_update(self, scan_data):
        print(f"Received lidar update: {scan_data}")
        self.lidar_data = json.loads(scan_data)
        self.calculate_collision_probability()

    def handle_gps_update(self, gps_data):
        print(f"Received gps update: {gps_data}")
        self.lidar_data = json.loads(gps_data)

    def avoid_collision(self):
        self.drive.set_speed(0)

    def calculate_collision_probability(self):
        # Implement collision probability calculation logic here
        pass
