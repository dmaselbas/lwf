import atexit
import threading
import time
import traceback

import numpy as np
import pandas as pd
from rplidar import RPLidar


class LidarController:

    def __init__(self, on_update_callback=None):
        self.callback = on_update_callback
        self.lidar = RPLidar("/dev/ttyUSB0", baudrate=460800)
        self.lidar.start_motor()
        self.lidar.connect()
        self.lidar_on = True
        self.last_reading = pd.DataFrame({
            "angle":    np.zeros(360),
            "distance": np.zeros(360),
            "quality":  np.zeros(360),
            })
        atexit.register(self.shutdown)
        thread = threading.Thread(target=self.publish_lidar_data, daemon=True, name="Lidar Thread")
        thread.start()

    def shutdown_lidar(self):
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
                                         .mean())
                    if self.callback is not None:
                        self.callback(self.last_reading)
            except Exception as e:
                print(f"Error publishing lidar data: {e}")
                # traceback.print_exc()
                print(self.lidar.get_health())
                self.lidar.reset()
        self.shutdown_lidar()

    def get_scan_data(self):
        return self.last_reading["distance"].tolist()

    def get_angles(self):
        return self.last_reading["angle"].tolist()
