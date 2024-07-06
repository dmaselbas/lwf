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
        self.publish_thread = None
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect("192.168.4.59", 1883)  # Replace with your MQTT broker address and port
        self.lidar_on = True
        if on_update_callback is not None:
            self.mqtt_client.on_message = self.on_message
            self.mqtt_client.subscribe("/dev/lidar/update")

        if on_update_callback is None:
            self.lidar = RPLidar("/dev/ttyUSB0", baudrate=460800)
            self.lidar.start_motor()
            self.lidar.connect()
            self.publish_thread = threading.Thread(target=self.publish_lidar_data)
            self.publish_thread.start()
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

    def on_message(self, client, userdata, message):
        if self.callback is not None:
            scan_data = message.payload.decode()
            self.callback(scan_data)

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
                    self.mqtt_client.publish("/dev/lidar/update", scan_df.to_json(orient="records"))
            except Exception as e:
                print(f"Error publishing lidar data: {e}")
                traceback.print_exc()
                print(self.lidar.get_health())
                self.lidar.stop()
                time.sleep(1)
                self.lidar.reset()
                time.sleep(10)
        self.shutdown_lidar()


if __name__ == "__main__":
    lidar = LidarController(on_update_callback=None)
