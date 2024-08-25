import threading
from time import sleep

import sys
import numpy as np
import pandas as pd
import paho.mqtt.client as mqtt
import json
import math
import atexit  # Import the atexit module
from stable_baselines3 import PPO

from devices.compass import CompassClient
from devices.drive import DriveClient
from real_env import RealRobotEnv


class AutoPilotMode:
    DISABLED = 0
    HEADING_LOCK = 1


class AutoPilotService:

    def __init__(self):
        super().__init__()
        self.running = True
        self.mqtt_broker = "mqtt.weedfucker.local"
        self.mqtt_port = 1883
        self.last_gps: np.array = None
        self.last_compass: np.array = None
        self.last_lidar: np.array = None
        self.longitude = None
        self.latitude = None
        self.angle_longest_reading = None
        self.angle_shortest_reading = None
        self.online_status_sent = False  # Flag to track if the "online" message has been sent
        self.last_observation = None
        self.mode = AutoPilotMode.DISABLED

        custom_objs = {"clip_range": 0.2, "lr_schedule": lambda _: 0.0003}
        self.env = RealRobotEnv(self)  # Create the environment
        self.model = PPO.load("ppo_robot.zip",
                              env=self.env,
                              custom_objects=custom_objs,
                              device="auto")  # Load the pre-trained model
        self.compas = CompassClient()  # Create the compass client
        self.drive_client = DriveClient()  # Create the drive client
        self.thread = threading.Thread(target=self.run, daemon=True, name="AutoPilotService")
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(self.mqtt_broker, self.mqtt_port, 60)
        self.thread.start()
        self.client.loop_forever()


    def run(self):
        while self.running:
            if self.last_observation is None:
                continue

            action = self.model.predict(self.last_observation.reshape(1, -1))[0]
            reward, done, info = self.env.step(action)
            self.last_observation = self.env.get_obs()

            if done:
                print("Episode finished!")
                self.env.reset()
                self.last_observation = self.env.get_obs()

            if self.mode == AutoPilotMode.HEADING_LOCK:
                self.compas.lock_heading(self.angle_longest_reading)


    def on_connect(self, client, userdata, flags, rc):
        print("Connected to MQTT broker with result code " + str(rc))
        # Subscribe to the relevant topics
        client.subscribe("/dev/lidar/update")
        client.subscribe("/dev/compass/bearing")
        client.subscribe("/dev/gps/update")

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode('utf-8')

        if topic == "/dev/lidar/update":
            self.process_lidar_update(json.loads(payload))
        elif topic == "/dev/compass/bearing":
            self.process_compass_update(payload)
        elif topic == "/dev/gps/update":
            self.process_gps_update(json.loads(payload))

    def process_lidar_update(self, data):
        # Convert the LiDAR data from inches to meters
        lidar_data = pd.json_normalize(data)
        lidar_data = lidar_data.fillna(20).astype(np.float32).mul(0.0254).to_numpy()[-1]
        if len(lidar_data) != 360:
            return

        self.angle_longest_reading = lidar_data.argmax()
        self.angle_shortest_reading = lidar_data.argmin()
        self.last_lidar = lidar_data

        print(f"Angle with longest reading: {self.angle_longest_reading} degrees")
        print(f"Angle with shortest reading: {self.angle_shortest_reading} degrees")
        self.check_and_send_online_status()
        self.try_get_next_action()

    def process_compass_update(self, bearing):
        bearing_in_radians = math.radians(float(bearing))
        self.last_compass = bearing_in_radians
        self.check_and_send_online_status()
        self.try_get_next_action()

    def process_gps_update(self, data):
        self.longitude = data.get('longitude')
        self.latitude = data.get('latitude')
        self.check_and_send_online_status()
        self.try_get_next_action()

    def check_and_send_online_status(self):
        # Check if all necessary data is available and send the "online" message if applicable
        if not self.online_status_sent and self.last_lidar is not None and self.longitude is not None and self.latitude is not None and self.last_compass is not None:
            self.client.publish("/service/autopilot/status", json.dumps({"status": "online"}))
            self.online_status_sent = True
            print("Sent 'online' status message")

    def prepare_observation(self):
        # Combine the most recent LiDAR, GPS, and compass data into a single observation
        if self.last_lidar is not None and self.longitude is not None and self.latitude is not None and self.last_compass is not None:
            observation = np.concatenate((self.last_lidar,
                                          [self.longitude, self.latitude,
                                           self.last_compass, self.angle_longest_reading,
                                           self.angle_shortest_reading]))
            return observation
        return np.zeros(365, )

    def get_next_action(self):
        # Prepare the observation data
        observation = self.prepare_observation()
        self.last_observation = observation
        if observation is not None:
            # Use the model to predict the next action
            action, _states = self.model.predict(observation, deterministic=False)
            self.env.step(action)  # Execute the chosen action in the environment
            self.client.publish("/service/autopilot/command", str(action).strip())

    def try_get_next_action(self):
        if self.last_lidar is not None and self.longitude is not None and self.latitude is not None and self.last_compass is not None:
            self.get_next_action()

    def shutdown(self):
        # Shutdown the MQTT client and perform any necessary cleanup
        print("Shutting down AutoPilotService...")
        self.client.publish("/service/autopilot/status", json.dumps({"status": "offline"}))
        self.client.loop_stop()
        self.client.disconnect()

    def get_last_observation(self):
        return self.last_observation


if __name__ == "__main__":
    sys.path.append("/var/training/code")
    service = AutoPilotService()
    print("AutoPilotService started")
    atexit.register(service.shutdown)  # Register the shutdown method to be called on exit
    try:
        while True:
            sleep(.25)
    except Exception as ex:
        quit()
