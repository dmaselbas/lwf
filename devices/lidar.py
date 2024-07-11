import atexit
import json
import struct
import threading
import time
import traceback
from datetime import timedelta

import numpy as np
import pandas as pd
import paho.mqtt.client as mqtt
from rplidar import RPLidar


class LidarController:

    def __init__(self, on_update_callback=None):
        self.callback = on_update_callback
        self.lidar = RPLidar("/dev/ttyUSB0", baudrate=460800)
        self.lidar.start_motor()
        self.lidar.connect()
        atexit.register(self.shutdown)

    def shutdown_lidar(self):
        if self.lidar is not None:
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.reset()
            self.lidar.disconnect()
            self.lidar = None
            self.lidar_on = False

    def shutdown(self):
        self.shutdown_lidar()
        if self.publish_thread is not None:
            self.publish_thread.join()

        self.mqtt_client.disconnect()

    def publish_lidar_data(self):
        while self.lidar_on:
            try:
                for scan in self.lidar.iter_scans():
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
                               .ffill())
                    if self.callback is not None:
                        self.callback(scan_df)
            except Exception as e:
                print(f"Error publishing lidar data: {e}")
                traceback.print_exc()
                print(self.lidar.get_health())
                self.lidar.stop()
                time.sleep(1)
                self.lidar.reset()
                time.sleep(10)
        self.shutdown_lidar()
