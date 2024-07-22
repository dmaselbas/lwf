import paho.mqtt.client as mqtt
import json
from devices.gps import GPSController  # Assuming the GPSController class is in a file named gps_controller.py

class GPSService:
    def __init__(self, mqtt_broker="localhost", mqtt_port=1883):
        self.gps_controller = GPSController(on_update_callback=self.on_gps_update)
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(mqtt_broker, mqtt_port, 60)
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code " + str(rc))
        client.subscribe("/dev/gps/#")

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode('utf-8')
        print(f"Received message on topic {topic}: {payload}")

        if topic == "/dev/gps/get_latitude":
            latitude = self.gps_controller.get_latitude()
            client.publish("/dev/gps/latitude", json.dumps({"latitude": latitude}))
        elif topic == "/dev/gps/get_longitude":
            longitude = self.gps_controller.get_longitude()
            client.publish("/dev/gps/longitude", json.dumps({"longitude": longitude}))
        else:
            print("Unknown command")

    def on_gps_update(self, gps_data):
        self.client.publish("/dev/gps/update", json.dumps(gps_data))

    def stop(self):
        self.client.loop_stop()
        self.client.disconnect()
        self.gps_controller.gps.close()

if __name__ == "__main__":
    service = GPSService()
    try:
        while True:
            pass  # Keep the main thread alive
    except KeyboardInterrupt:
        service.stop()
        print("Service stopped")
