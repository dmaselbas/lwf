import paho.mqtt.client as mqtt
import json
from devices.laser import LaserController  # Assuming the LaserController class is in a file named laser_controller.py

class LaserService:
    def __init__(self, mqtt_broker="localhost", mqtt_port=1883):
        self.laser_controller = LaserController()
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(mqtt_broker, mqtt_port, 60)
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code " + str(rc))
        client.subscribe("/dev/laser/#")

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode('utf-8')
        print(f"Received message on topic {topic}: {payload}")

        if topic == "/dev/laser/on":
            self.laser_controller.on()
        elif topic == "/dev/laser/off":
            self.laser_controller.off()
        elif topic == "/dev/laser/move_to_position":
            try:
                position = int(payload)
                self.laser_controller.move_to_position(position)
            except ValueError:
                print("Invalid position value")
        else:
            print("Unknown command")

    def stop(self):
        self.client.loop_stop()
        self.client.disconnect()
        self.laser_controller.shutdown()

if __name__ == "__main__":
    service = LaserService()
    try:
        while True:
            pass  # Keep the main thread alive
    except KeyboardInterrupt:
        service.stop()
        print("Service stopped")
