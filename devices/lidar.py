import atexit
import threading
import time
import traceback
from datetime import datetime

import numpy as np
import pandas as pd
from rplidar import RPLidar


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
        self.semephore = threading.Semaphore(0)
        atexit.register(self.shutdown)
        thread = threading.Thread(target=self.publish_lidar_data, daemon=True, name="Lidar Thread")
        thread.start()

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

    def publish_lidar_data(self):
        print("Collecting Lidar Data...")
        while self.lidar_on:
            # noinspection TryExceptContinue
            try:
                self.lidar.clear_input()
                for scan in self.lidar.iter_scans(max_buf_meas=2000, min_len=100):
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
            except:
                continue

    def get_scan_data(self):
        return self.last_reading["distance"].tolist()

    def get_angles(self):
        return self.last_reading["angle"].tolist()

    def get_last_reading(self):
        return self.last_reading.copy()
