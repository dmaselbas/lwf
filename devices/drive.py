from devices.pwm_controller import PWMController
import paho.mqtt.client as mqtt
import json


class DriveController:
    def __init__(self, controller, on_update_callback=None):
        self.controller = controller
        self.mqtt_client = mqtt.Client()
        self.on_update_callback = on_update_callback
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect("10.0.0.2", port=1883)
        self.mqtt_client.loop_start()

        self.speed = 0
        self.direction = "forward"

    def on_connect(self, client, userdata, flags, rc):
        if self.is_client:
            print("Connected to MQTT broker")
            client.subscribe("/dev/drive/status/update")
        else:
            client.subscribe("/dev/drive")

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        message = msg.payload.decode()

        if topic == "/dev/drive" and "update" not in topic:
            if "speed" in message:
                self.speed = int(message.split(":")[-1])
                self.speed = min(self.speed, 0)
                self.speed = max(self.speed, 100)
                scaled_speed = int(self.speed * 4096 / 100)
                self.controller.set_pwm(13, scaled_speed, 0)
                self.controller.set_pwm(15, scaled_speed, 0)
            elif message == "left":
                self.direction = "left"
                self.controller.set_pwm(14, 4095, 0)
                self.controller.set_pwm(12, 0, 0)
            elif message == "right":
                self.direction = "right"
                self.controller.set_pwm(14, 0, 0)
                self.controller.set_pwm(12, 4095, 0)
            elif message == "reverse":
                self.direction = "reverse"
                self.controller.set_pwm(14, 4095, 0)
                self.controller.set_pwm(12, 4095, 0)
            elif message == "stop":
                self.speed = 0
                self.controller.set_pwm(12, 0, 0)
                self.controller.set_pwm(13, 0, 0)
                self.controller.set_pwm(14, 0, 0)
                self.controller.set_pwm(15, 0, 0)
            status = {"speed": self.speed, "direction": self.direction}
            self.mqtt_client.publish("/dev/drive/status/update", json.dumps(status))
        if self.on_update_callback and topic == "/dev/drive/status/update":
            status = json.loads(message)
            self.on_update_callback(status)

    def set_speed(self, speed):
        self.mqtt_client.publish("/dev/drive/speed", str(speed))

    def move_left(self):
        self.mqtt_client.publish("/dev/drive", "left")

    def move_right(self):
        self.mqtt_client.publish("/dev/drive", "right")

    def reverse(self):
        self.mqtt_client.publish("/dev/drive", "reverse")

    def stop(self):
        self.mqtt_client.publish("/dev/drive", "stop")
