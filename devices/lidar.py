import atexit
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

    def __init__(self, on_update_callback=None, tilt_pwm_channel: int = 3) -> None:
        self.last_scan = None
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
        self.tilt_servo = ServoController(tilt_pwm_channel)
        self.min_tilt = 90 - 25
        self.max_tilt = 90 + 25
        self.tilt_servo.set_angle(90)
        atexit.register(self.shutdown)
        thread = threading.Thread(target=self.publish_lidar_data, daemon=True, name="Lidar Thread")
        if self.lidar is not None:
            thread.start()

        # Initialize MQTT client
        self.mqtt_client = mqtt.Client(protocol=mqtt.MQTTv5)
        self.mqtt_client.connect("mqtt.weedfucker.local", 1883, 60)
        self.mqtt_client.loop_start()

    def shutdown_lidar(self):
        print("Shutting down Lidar...")
        if self.lidar is not None:
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.reset()
            self.lidar.disconnect()
            self.lidar = None
            self.lidar_on = False

    def shutdown(self):
        self.lidar_on = False
        self.shutdown_lidar()
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()

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

                    # Publish the Lidar data to the MQTT topic
                    if scan_df is None:
                        continue
                    if self.last_reading is None or "distance" not in self.last_reading.columns:
                        self.last_reading = scan_df.copy()
                    scan_df["distance"] = np.where(pd.notnull(scan_df["distance"]),
                                                   scan_df["distance"],
                                                   self.last_reading["distance"])
                    self.last_reading = scan_df.copy()
                    lidar_data = self.last_reading[["distance"]].to_json()
                    self.mqtt_client.publish("/dev/lidar/update", lidar_data)

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

    def get_scan_data(self):
        return self.last_reading["distance"].tolist()

    def get_angles(self):
        return self.last_reading["angle"].tolist()

    def get_last_reading(self):
        return self.last_reading.copy()
