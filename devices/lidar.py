import atexit
import json
import os
import threading
import traceback
from datetime import datetime

import numpy as np
import pandas as pd
import time
from rplidar import RPLidar, RPLidarException
import paho.mqtt.client as mqtt

from devices.pwm_controller import PWMClient, PWMController
from devices.servo import ServoController


class LidarController:

    def __init__(self, on_update_callback=None, tilt_pwm_channel: int = 7) -> None:
        self.last_scan = None
        self.running = True
        self.callback = on_update_callback
        try:
            self.lidar = RPLidar("/dev/ttyUSB0", baudrate=460800)
        except:
            self.lidar = None
        self.lidar_on = True
        self.measurements = []
        self.last_reading = pd.DataFrame({
            "angle":    np.zeros(360),
            "distance": np.zeros(360),
            "quality":  np.zeros(360),
            "tilt":     np.zeros(360),
            })
        # self.tilt_servo = ServoController(tilt_pwm_channel)
        self.tilt_servo = ServoController(7)
        self.min_tilt = 90 - 25
        self.max_tilt = 90 + 25
        self.current_tilt = 90
        self.tilt_incrementor = 1
        self.tilt_servo.set_angle(self.current_tilt)
        atexit.register(self.shutdown)
        thread = threading.Thread(target=self.publish_lidar_data, daemon=True, name="Lidar Thread")
        if self.lidar is not None:
            thread.start()

        # Initialize MQTT client
        self.mqtt_client = mqtt.Client(protocol=mqtt.MQTTv5)
        self.mqtt_client.connect("mqtt.weedfucker.local", 1883, 60)
        self.mqtt_client.loop_forever()

    def shutdown_lidar(self):
        print("Shutting down Lidar...")
        if self.lidar is not None:
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.reset()
            self.lidar.disconnect()
            self.lidar = None
            self.lidar_on = False
        self.running = False

    def shutdown(self):
        self.lidar_on = False
        self.shutdown_lidar()
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()

    def store_lidar_data(self, last_scan_df):
        lidar_data_path = f"/var/training/code/data/lidar/lidar_data_{datetime.now().strftime('%Y%m%d')}.csv"
        if not os.path.exists(os.path.dirname(lidar_data_path)):
            os.makedirs(os.path.dirname(lidar_data_path))
            last_scan_df.to_parquet(lidar_data_path)
        else:
            try:
                existing_df = pd.read_parquet(lidar_data_path)
                new_df = pd.concat([existing_df, last_scan_df])
                new_df.to_csv(lidar_data_path)
            except Exception as e:
                last_scan_df.to_csv(lidar_data_path)


    def publish_lidar_data(self):
        print("Collecting Lidar Data...")
        self.lidar.start_motor()
        while self.lidar_on:
            try:
                self.lidar.clear_input()
                for scan in self.lidar.iter_scans(max_buf_meas=2000, min_len=360):
                    scan_qualities, scan_angles, scan_distances = zip(*scan)
                    scan_df = pd.DataFrame({
                        "quality":  scan_qualities,
                        "angle":    scan_angles,
                        "distance": scan_distances,
                        })
                    new_index = pd.Index(np.arange(0, 360, 1), name="angle")
                    scan_df = (scan_df.assign(
                            angle=lambda df: df["angle"].astype(int),
                            distance=lambda df: df["distance"].div(25.4))
                               .drop_duplicates(subset="angle", keep="last")
                               .sort_values(by="angle", ascending=True)
                               .set_index("angle")
                               .reindex(new_index)
                               .reset_index()
                               .ffill()
                               .ewm(alpha=0.4)
                               .mean()
                               .assign(timestamp=pd.to_datetime(datetime.utcnow(), utc=True)))
                    self.update_tilt()
                    # Publish the Lidar data to the MQTT topic
                    if scan_df is None:
                        continue
                    if self.last_reading is None or "distance" not in self.last_reading.columns:
                        self.last_reading = scan_df.copy()
                    scan_df["distance"] = np.where(pd.notnull(scan_df["distance"]),
                                                   scan_df["distance"],
                                                   self.last_reading["distance"])
                    scan_df["tilt_angle"] = self.current_tilt
                    self.last_reading = scan_df.copy()
                    lidar_data = self.last_reading[["distance"]].to_json()
                    self.mqtt_client.publish("/dev/lidar/update", lidar_data)
                    self.store_lidar_data(self.last_reading)
                    # Call the update callback if provided
                    if self.callback:
                        self.callback(lidar_data)
                    if self.tilt_servo.get_angle() <= self.min_tilt:
                        self.tilt_servo.set_angle(self.max_tilt)
                    if self.tilt_servo.get_angle() >= self.max_tilt:
                        self.tilt_servo.set_angle(self.min_tilt)
            except RPLidarException as e:
                continue
            except Exception as e:
                print(f"Error in lidar controller: {traceback.format_exc()}")
                continue

    def update_tilt(self):
        self.current_tilt += self.tilt_incrementor
        self.tilt_servo.set_angle(self.current_tilt)
        if self.current_tilt >= self.max_tilt:
            self.current_tilt = self.max_tilt
            self.tilt_incrementor = -1
        elif self.current_tilt <= self.min_tilt:
            self.current_tilt = self.min_tilt
            self.tilt_incrementor = 1

    def get_scan_data(self):
        return self.last_reading["distance"].tolist()

    def get_angles(self):
        return self.last_reading["angle"].tolist()

    def get_last_reading(self):
        return self.last_reading.copy()


class Lidar:
    def __init__(self, mqtt_broker="mqtt.weedfucker.local", mqtt_port=1883, topic="/dev/lidar/update"):
        self.mqtt_broker = mqtt_broker
        self.mqtt_port = mqtt_port
        self.topic = topic
        self.latest_data = pd.DataFrame()

        # Initialize MQTT client
        self.mqtt_client = mqtt.Client(protocol=mqtt.MQTTv5)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
        self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected to MQTT broker with result code {rc}")
        client.subscribe(self.topic)

    def on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode('utf-8')
            data = json.loads(payload)
            self.latest_data = pd.DataFrame.from_dict(data)
            print("Updated latest Lidar data")
        except Exception as e:
            print(f"Error processing message: {e}")

    def get_latest_data(self):
        return self.latest_data.copy()

    def stop(self):
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()

# Example usage
if __name__ == "__main__":
    lidar = LidarController()
    while lidar.running:
        time.sleep(1)
