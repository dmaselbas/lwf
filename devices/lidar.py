import atexit
import json
import threading
import time
import traceback
from datetime import datetime

import numpy as np
import pandas as pd
from rplidar import RPLidar
import paho.mqtt.client as mqtt


class LidarController:

    def __init__(self, on_update_callback=None):
        self.last_scan = None
        self.callback = on_update_callback
        self.lidar = RPLidar("/dev/ttyUSB0", baudrate=460800)
        self.lidar_on = True
        self.measurements = []
        self.last_reading = pd.DataFrame({
            "angle":    np.zeros(360),
            "distance": np.zeros(360),
            "quality":  np.zeros(360),
            })
        self.semaphore = threading.Semaphore(0)
        atexit.register(self.shutdown)
        thread = threading.Thread(target=self.publish_lidar_data, daemon=True, name="Lidar Thread")
        thread.start()

        # Initialize MQTT client
        self.mqtt_client = mqtt.Client()
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
                for scan in self.lidar.iter_scans(max_buf_meas=2500, min_len=1000):
                    scan_qualities, scan_angles, scan_distances = zip(*scan)
                    scan_df = pd.DataFrame({
                        "quality":  scan_qualities,
                        "angle":    scan_angles,
                        "distance": scan_distances,
                        })
                    new_index = pd.Index(np.arange(0, 360, 1), name="angle")
                    self.last_reading = (scan_df.assign(
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
                    lidar_data = self.last_reading.to_dict(orient='list')
                    self.mqtt_client.publish("/dev/lidar/update", json.dumps(lidar_data))
                    
                    # Call the update callback if provided
                    if self.callback:
                        self.callback(lidar_data)
            except:
                continue

    def get_scan_data(self):
        return self.last_reading["distance"].tolist()

    def get_angles(self):
        return self.last_reading["angle"].tolist()

    def get_last_reading(self):
        return self.last_reading.copy()
