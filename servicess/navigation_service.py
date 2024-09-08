import atexit
import json
import threading
from time import sleep

import numpy as np
import paho.mqtt.client as mqtt

from devices.imu import IMUClient
from servicess.collision_avoidance_service import CollisionAvoidanceSystem
from devices.drive import DriveClient
from devices.lidar import Lidar, LidarController
from devices.gps import GPSClient
from devices.compass import CompassClient


class NavigationService:

    def __init__(self, lidar: Lidar):
        # self.cas = CollisionAvoidanceSystem(
        #         on_update_callback=self.handle_collision_avoidance_update, lidar=lidar)
        self.lidar_controller: LidarController = lidar
        self.gps_controller: GPSClient = GPSClient()
        self.drive_controller: DriveClient = DriveClient()
        self.imu = IMUClient()
        self.compass = CompassClient()
        self.autopilot_online = False
        self.state = "CAS"
        self.pre_stopping_speed = 0.0

        self.locked_heading = None
        self.running = True
        thread = threading.Thread(target=self.update_stuff, daemon=True)
        thread.start()
        self.client = mqtt.Client(protocol=mqtt.MQTTv5)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        atexit.register(self.shutdown)  # Ensure the service is properly shut down when the
        # Connect to the MQTT broker
        self.client.connect("mqtt.weedfucker.local", 1883, 60)
        self.client.loop_start()

    def shutdown(self):
        self.running = False
        self.client.loop_stop()
        self.client.disconnect()

    def update_stuff(self):
        while self.running:
            current_heading = self.compass.get_bearing()
            if self.locked_heading is None:
                sleep(0.5)
            else:
                # Calculate the difference between the current heading and the locked heading
                heading_difference = current_heading - self.locked_heading

                # Normalize the heading difference to the range [-180, 180]
                if heading_difference > 180:
                    heading_difference -= 360
                elif heading_difference < -180:
                    heading_difference += 360

                # Check if we need to correct the heading
                if abs(heading_difference) > 1:  # Consider a small threshold to avoid constant corrections
                    speed = self.drive_controller.get_speed()
                    # Determine the direction and apply the scaled correction
                    if heading_difference > 0:  # Need to turn right
                        self.drive_controller.set_right_speed(speed - (abs(heading_difference) * 2))
                    elif heading_difference < 0:  # Need to turn left
                        self.drive_controller.set_left_speed(speed - (abs(heading_difference) * 2))
                else:
                    speed = self.drive_controller.get_speed()
                    self.drive_controller.set_left_speed(speed)
                    self.drive_controller.set_right_speed(speed)

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
        self.locked_heading = self.compass.get_bearing()
        # self.drive_controller.forward()

    def drive_backward(self):
        # if not self.cas.taking_avoidance_action:
        self.locked_heading = None
        # self.drive_controller.reverse()

    def drive_left(self):
        # if not self.cas.taking_avoidance_action:
        self.locked_heading = None
        # self.drive_controller.left()

    def drive_right(self):
        # if not self.cas.taking_avoidance_action:
        self.locked_heading = None
        # self.drive_controller.right()

    def stop_driving(self):
        self.locked_heading = None
        # self.drive_controller.stop()

    def set_drive_speed(self, speed):
        self.drive_controller.set_speed(speed)

    def on_connect(self, client, userdata, flags, rc, properties):
        print("Connected to MQTT broker with result code ")
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
