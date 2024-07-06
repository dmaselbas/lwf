import json
import threading

import serial
import pynmea2
import paho.mqtt.client as mqtt
from daemon import DaemonContext


class GPSController:

    def __init__(self, on_update_callback=None):
        self.gps_data = None
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect("192.168.4.59", 1883)  # Replace with your MQTT broker address and port

        if on_update_callback is not None:
            self.mqtt_client.on_message = self.on_message
            self.mqtt_client.subscribe("/dev/gps")

        self.mqtt_client.loop_start()
        if on_update_callback is None:
            self.gps = serial.Serial("/dev/ttyS5", baudrate=9600, timeout=10)
            self.process_thread = threading.Thread(target=self.process_gps_data)
            self.process_thread.start()

    def read_line(self):
        line = self.gps.readline().decode("utf-8", errors="ignore").strip()
        return line

    def publish_gps_data(self, data):
        self.mqtt_client.publish("/dev/gps/update", data)

    def publish_gps_health(self, health):
        self.mqtt_client.publish("/dev/gps/health", health)

    def on_message(self, client, userdata, message):
        if self.on_update_callback is not None:
            payload = message.payload.decode()
            self.on_update_callback(payload)

    def process_gps_data(self):
        while True:
            line = self.read_line()
            if line:
                try:
                    msg = pynmea2.parse(line)
                    if msg.sentence_type == "GGA":
                        self.publish_gps_data(str(msg))
                    else:
                        self.publish_gps_health(line)
                except pynmea2.nmea.ParseError:
                    print("Invalid GPS data")


if __name__ == "__main__":
    daemon = GPSController(on_update_callback=None)
    daemon.mqtt_client.loop_forever()
