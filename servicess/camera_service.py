import paho.mqtt.client as mqtt
from devices.camera_ptz import CameraController
from devices.pwm_controller import PWMController


class CameraService:
    def __init__(self, pwm_controller: PWMController) -> None:
        self.pwm_controller = pwm_controller
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect("mqtt.weedfucker.local", 1883, 60)
        self.mqtt_client.loop_start()

        self.nav_cam = CameraController(pwm_controller, 26, 27, self.on_camera_move, "navigation")
        self.classification_cam = CameraController(pwm_controller, 28, 29, self.on_camera_move, "targeting")

    def on_camera_move(self, cam_name, direction, value):
        topic = f"/svc/cam/{cam_name}/{direction}"
        self.mqtt_client.publish(topic, str(value))
