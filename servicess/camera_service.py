import paho.mqtt.client as mqtt
from devices.camera_ptz import CameraController


class CameraService:
    def __init__(self) -> None:
        self.mqtt_client = mqtt.Client(protocol=mqtt.MQTTv5)
        self.mqtt_client.connect("mqtt.weedfucker.local", 1883, 60)
        self.mqtt_client.loop_start()

        self.nav_cam = CameraController(26, 27, self.on_camera_move, "navigation")
        self.classification_cam = CameraController(28, 29, self.on_camera_move, "targeting")

    def on_camera_move(self, cam_name, direction, value):
        topic = f"/svc/cam/{cam_name}/{direction}"
        self.mqtt_client.publish(topic, str(value))
