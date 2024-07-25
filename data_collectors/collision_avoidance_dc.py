import atexit
import json

import paho.mqtt.client as mqtt
import pandas as pd

class COSDataCollector:
    def __init__(self, mqtt_broker="mqtt.weedfucker.local", mqtt_port=1883):
        self.recording = False
        self.last_compass_bearing = None
        self.last_drive_speed = None
        self.last_drive_direction = None
        self.last_lidar_reading = None
        self.recording_df = pd.DataFrame(columns=["compass_bearing", "drive_speed", "drive_direction"])
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(mqtt_broker, mqtt_port, 60)
        self.client.loop_start()

        atexit.register(self.stop)

    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code " + str(rc))
        client.subscribe("/dev/compass/bearing")

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode('utf-8')

        if topic == "/dev/compass/bearing":
            self.last_compass_bearing = payload
        if topic == "/dev/drive/speed":
            self.last_drive_speed = payload
        if topic == "/dev/drive/direction":
            self.last_drive_direction = payload
        if topic == "/dev/lidar/update":
            self.last_lidar_reading = json.loads(payload)

    def stop(self):
        print("Stopping COS data collector")
        self.client.loop_stop()
        self.client.disconnect()
