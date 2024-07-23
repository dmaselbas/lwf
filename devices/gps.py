import json
import threading
from datetime import datetime, timezone

import pandas as pd
import serial
import pynmea2

from devices.compass import HMC5883L


class GPSController:

    def __init__(self, on_update_callback=None):
        self.gps_data = pd.DataFrame(columns=["datetime",
                                              "latitude",
                                              "longitude",
                                              "altitude",
                                              "altitude_units",
                                              "num_sats",
                                              "heading"])
        self.compass = HMC5883L()
        self.on_update_callback = on_update_callback
        self.gps = serial.Serial("/dev/ttyS2", baudrate=9600, timeout=3)
        self.process_thread = threading.Thread(target=self.process_gps_data)
        self.process_thread.start()

    def read_line(self):
        line = self.gps.readline().decode("utf-8", errors="ignore").strip()
        return line

    def process_gps_data(self):
        h = 360.0 - self.compass.get_bearing()
        while True:
            line = self.gps.readline().decode("utf-8", errors="ignore").strip()
            if line:
                try:
                    if "GGA" in line:  # RMC (Recommended Minimum Navigation Information)
                        msg = pynmea2.parse(line)
                        gps_data = dict(datetime=datetime.now(timezone.utc).timestamp(),
                                        latitude=msg.latitude,
                                        longitude=msg.longitude,
                                        altitude=msg.altitude,
                                        altitude_units=msg.altitude_units,
                                        num_sats=msg.num_sats,
                                        heading=h)
                        gps_data = pd.DataFrame(gps_data, index=[0])
                        self.gps_data = pd.concat([self.gps_data, gps_data], ignore_index=True)
                except serial.SerialException as e:
                    print('Device error: {}'.format(e))
                    continue
                except pynmea2.ParseError as e:
                    print('Parse error: {}'.format(e))
                    continue

    def get_gps_reading(self):
        return self.gps_data.tail(1)

    def get_heading(self):
        return self.compass.get_bearing()
