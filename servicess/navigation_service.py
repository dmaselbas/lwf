import json

import numpy as np
import paho.mqtt.client as mqtt

from devices.imu import IMUClient, MPU6050
from servicess.collision_avoidance_service import CollisionAvoidanceSystem
from devices.drive import DriveController
from devices.lidar import LidarController
from devices.gps import GPSClient, GPSController
from devices.compass import CompassClient, HMC5883L


class NavigationService:

    def __init__(self, lidar: LidarController, gps: GPSController):
        self.cas = CollisionAvoidanceSystem(
                on_update_callback=self.handle_collision_avoidance_update, lidar=lidar)
        self.lidar_controller: LidarController = lidar
        self.gps_controller: GPSClient = GPSClient()
        self.drive_controller: DriveController = DriveController()
        self.imu = IMUClient()
        self.compass = CompassClient()
        self.autopilot_online = False
        self.state = "CAS"
        self.pre_stopping_speed = 0.0


        # Initialize MQTT client
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        # Connect to the MQTT broker
        self.client.connect("mqtt.weedfucker.local", 1883, 60)
        self.client.loop_start()

    def handle_collision_avoidance_update(self, data):
        # if self.cas.taking_avoidance_action:
        #     self.update_state("CAS")
        # else:
            self.update_state("AUTO_PILOT")

    def update_state(self, state):
        self.state = state
        self.client.publish("/service/navigation/update/state", self.state)

    def drive_forward(self):
        # if not self.cas.taking_avoidance_action:
        self.drive_controller.forward()

    def drive_backward(self):
        # if not self.cas.taking_avoidance_action:
        self.drive_controller.reverse()

    def drive_left(self):
        # if not self.cas.taking_avoidance_action:
        self.drive_controller.left()

    def drive_right(self):
        # if not self.cas.taking_avoidance_action:
        self.drive_controller.right()

    def stop_driving(self):
        self.drive_controller.stop()

    def set_drive_speed(self, speed):
        self.drive_controller.set_speed(speed)

    def on_connect(self, client, userdata, flags, rc):
        print("Connected to MQTT broker with result code " + str(rc))
        client.subscribe("/service/autopilot/command")

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode('utf-8')

        if topic == "/service/autopilot/command":
            self.handle_autopilot_command(payload)
            self.autopilot_online = True
        if topic == "/service/autopilot/status":
            if "offline" in payload:
                self.state = "CAS"
                self.stop_driving()
                self.autopilot_online = False

    def handle_autopilot_command(self, command):
        # Process the autopilot command received from the topic
        if self.state != "AUTO_PILOT":
            return
        action = int(command)
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
