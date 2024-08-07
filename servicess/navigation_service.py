import json

import numpy as np

from devices.imu import MPU6050
from servicess.collision_avoidance_service import CollisionAvoidanceSystem
from devices.drive import DriveController
from devices.lidar import LidarController
from devices.gps import GPSController
from devices.compass import HMC5883L

class NavigationService:

    def __init__(self, drive: DriveController,
                 lidar: LidarController, gps: GPSController,
                 imu: MPU6050, compass: HMC5883L):
        self.collision_avoidance_system = CollisionAvoidanceSystem(
                on_update_callback=self.handle_collision_avoidance_update,
                drive=drive, lidar=lidar, gps=gps, imu=imu)
        self.lidar_controller: LidarController = lidar
        self.gps_controller: GPSController = gps
        self.drive_controller: DriveController = drive
        self.imu = imu
        self.compass = compass
        self.autopilot_online = False
        self.state = "CAS"

    def handle_collision_avoidance_update(self, data):
        pass

    def get_lidar_data(self):
        lidar_data = self.lidar_controller.get_last_reading()
        if np.min(lidar_data) < 16:
            self.update_state("CAS")
        else:
            if self.autopilot_online:
                self.update_state("AUTO_PILOT")
            else:
                self.update_state("CAS")

    def update_state(self, state):
        self.state = state
        print(f"State updated: {self.state}")

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

    def on_connect(self, client, userdata, flags, rc):
        print("Connected to MQTT broker with result code " + str(rc))
        client.subscribe("/service/autopilot/command")

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode('utf-8')

        if topic == "/service/autopilot/command":
            self.handle_autopilot_command(json.loads(payload))
        if topic == "/service/autopilot/status":
            if json.loads(payload).get("status") == "offline":
                self.state = "CAS"
                self.autopilot_online = False
            if json.loads(payload).get("status") == "online":
                self.state = "AUTO_PILOT"
                self.autopilot_online = True

    def handle_autopilot_command(self, command):
        # Process the autopilot command received from the topic
        print(f"Received autopilot command: {command}")
        if self.state != "AUTO_PILOT":
            print("Autopilot mode not active. Ignoring command.")
            return
        action = command.get("command")
        if action == 1:
            self.drive_forward()
        elif action == 2:
            self.drive_backward()
        elif action == 3:
            self.drive_left()
        elif action == 4:
            self.drive_right()
        elif action == 0:
            self.stop_driving()
