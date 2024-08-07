import numpy as np
import paho.mqtt.client as mqtt
import json
import math
import atexit  # Import the atexit module
from stable_baselines3 import DQN  # Import the desired model from stable_baselines3
from stable_baselines3.common.env_util import make_vec_env  # Utility to create vectorized environments


class AutoPilotService:

    def __init__(self):
        super().__init__()
        self.mqtt_broker = "mqtt.weedfucker.local"
        self.mqtt_port = 1883
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(self.mqtt_broker, self.mqtt_port, 60)
        self.client.loop_start()
        self.last_gps: np.array = None
        self.last_compass: np.array = None
        self.last_lidar: np.array = None
        self.longitude = None
        self.latitude = None
        self.angle_longest_reading = None
        self.angle_shortest_reading = None
        self.online_status_sent = False  # Flag to track if the "online" message has been sent

        # Load the pre-trained stable_baselines3 model
        self.env = make_vec_env('dqn_robot', n_envs=1)  # Replace 'CartPole-v1' with your custom environment
        self.model = DQN.load("path_to_pretrained_model", env=self.env)  # Load the pre-trained model

        # Register the shutdown method to be called on exit
        atexit.register(self.shutdown)

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
        lidar_data = np.array(data).astype(np.float32) * 0.0254
        self.last_lidar = lidar_data

        # Calculate the angles with the longest and shortest readings
        angle_longest_reading = np.argmax(lidar_data)
        angle_shortest_reading = np.argmin(lidar_data)

        self.angle_longest_reading = angle_longest_reading
        self.angle_shortest_reading = angle_shortest_reading

        print("Lidar data received (meters):", lidar_data)
        print(f"Angle with longest reading: {angle_longest_reading} degrees")
        print(f"Angle with shortest reading: {angle_shortest_reading} degrees")

        # Check if all necessary data is available and send the "online" message if applicable
        self.check_and_send_online_status()

        # Try to get the next action if all necessary data is available
        self.try_get_next_action()

    def process_compass_update(self, bearing):
        # Convert the compass bearing from degrees to radians
        bearing_in_radians = math.radians(float(bearing))
        print("Compass bearing received (radians):", bearing_in_radians)
        self.last_compass = bearing_in_radians

        # Check if all necessary data is available and send the "online" message if applicable
        self.check_and_send_online_status()

        # Try to get the next action if all necessary data is available
        self.try_get_next_action()

    def process_gps_update(self, data):
        # Extract longitude and latitude from the GPS data
        self.longitude = data.get('longitude')
        self.latitude = data.get('latitude')
        print(f"GPS data received: longitude={self.longitude}, latitude={self.latitude}")

        # Check if all necessary data is available and send the "online" message if applicable
        self.check_and_send_online_status()

        # Try to get the next action if all necessary data is available
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
            observation = np.concatenate((self.last_lidar, [self.longitude, self.latitude, self.last_compass]))
            return observation
        return None

    def get_next_action(self):
        # Prepare the observation data
        observation = self.prepare_observation()
        if observation is not None:
            # Use the model to predict the next action
            action, _states = self.model.predict(observation, deterministic=True)
            print(f"Predicted action: {action}")
            self.send_autopilot_command(action)

    def try_get_next_action(self):
        # Check if all necessary data is available and get the next action
        if self.last_lidar is not None and self.longitude is not None and self.latitude is not None and self.last_compass is not None:
            self.get_next_action()

    def send_autopilot_command(self, action):
        # Send the autopilot command to the robot
        print("Sending autopilot command:", action)
        if self.last_gps is not None and self.last_compass is not None and self.last_lidar is not None:
            self.client.publish("/service/autopilot/command", json.dumps({"command": action}))

    def shutdown(self):
        # Shutdown the MQTT client and perform any necessary cleanup
        print("Shutting down AutoPilotService...")
        self.client.publish("/service/autopilot/status", json.dumps({"status": "offline"}))
        self.client.loop_stop()
        self.client.disconnect()
