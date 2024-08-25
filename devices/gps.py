import atexit
import json
import threading
from datetime import datetime, timezone
from pathlib import Path
from time import sleep

import pandas as pd
import serial
import pynmea2
import paho.mqtt.client as mqtt

from devices.compass import CompassClient, HMC5883L


class GPSController:

    def __init__(self):
        self.running = True
        self.gps = serial.Serial("/dev/ttyS2", baudrate=9600, timeout=3)
        self.logging_path = Path(f"/var/training/data/gps/{datetime.utcnow().strftime('%Y-%m-%d_%H-%M-%S')}_gps.txt")
        if not self.logging_path.parent.exists():
            self.logging_path.parent.mkdir(parents=True)
        self.process_thread = threading.Thread(target=self.process_gps_data, daemon=True, name="GPS Data Processing Thread")
        self.process_thread.start()

        # Initialize MQTT client
        self.mqtt_client = mqtt.Client(protocol=mqtt.MQTTv5)
        self.mqtt_client.connect("mqtt.weedfucker.local", 1883, 60)
        self.mqtt_client.loop_start()

    def _shutdown(self):
        self.running = False
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()

    def read_line(self):
        line = self.gps.readline().decode("utf-8", errors="ignore").strip()
        return line

    def process_gps_data(self):
        while self.running:
            line = self.gps.readline().decode("utf-8", errors="ignore").strip()
            if line:
                try:
                    with open(self.logging_path, "a") as f:
                        f.write(line + "\n")
                    if "GGA" in line:  # RMC (Recommended Minimum Navigation Information)
                        msg = pynmea2.parse(line)
                        # Publish the GPS data to the MQTT topic
                        self.mqtt_client.publish("/dev/gps/latitude", msg.latitude)
                        self.mqtt_client.publish("/dev/gps/longitude", msg.longitude)
                        self.mqtt_client.publish("/dev/gps/altitude", msg.altitude)
                        self.mqtt_client.publish("/dev/gps/altitude_units", msg.altitude_units)
                        self.mqtt_client.publish("/dev/gps/connected_sats", msg.num_sats)
                    sleep(0.5)
                except serial.SerialException as e:
                    print('Device error: {}'.format(e))
                    continue
                except pynmea2.ParseError as e:
                    print('Parse error: {}'.format(e))
                    continue
                except Exception as ex:
                    print(f'Unexpected error: {ex}')
                    continue


class GPSClient:
    def __init__(self):
        self.gps_data = dict(
                latitude=0.0,
                longitude=0.0,
                altitude=0.0,
                altitude_units="x",
                connected_sats=-1,
                )
        self.compass = CompassClient()  # Initialize compass client
        self.mqtt_client = mqtt.Client(protocol=mqtt.MQTTv5)
        self.mqtt_client.on_connect = self._on_connect
        self.mqtt_client.on_message = self._on_message
        self.mqtt_client.connect("mqtt.weedfucker.local", 1883, 60)
        self.mqtt_client.loop_start()

    def _on_message(self, client, userdata, message):
        topic = message.topic
        value = message.payload.decode("utf-8")
        if value == '':
            return
        if topic == "/dev/gps/latitude":
            self.gps_data["latitude"] = float(value)
        elif topic == "/dev/gps/longitude":
            self.gps_data["longitude"] = float(value)
        elif topic == "/dev/gps/altitude":
            self.gps_data["altitude"] = float(value)
        elif topic == "/dev/gps/altitude_units":
            self.gps_data["altitude_units"] = value
        elif topic == "/dev/gps/connected_sats":
            self.gps_data["connected_sats"] = int(value)

    def _on_connect(self, client, userdata, flags, rc, properties):
        client.subscribe("/dev/gps/#")

    def get_gps_reading(self):
        df = pd.DataFrame(self.gps_data, index=[datetime.now(timezone.utc)])
        return df

    def get_heading(self):
        return self.compass.get_bearing()


if __name__ == "__main__":
    gps_controller = GPSController()
    while gps_controller.running:
        sleep(1)
