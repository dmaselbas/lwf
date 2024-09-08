import atexit
from datetime import datetime
from pathlib import Path
from time import sleep

import numpy as np
import paho.mqtt.client as mqtt
from devices.pwm_controller import PWMClient


class DriveController:
    def __init__(self):
        self.left_drive_forward_mode = True
        self.right_drive_forward_mode = True
        self.right_drive_speed = 0
        self.left_drive_speed = 0
        self.pwm_controller = PWMClient()
        self.lf_forward = 1 + 16
        self.lf_reverse = 2 + 16
        self.lm_forward = 13
        self.lm_reverse = 14
        self.lr_forward = 3 + 16
        self.lr_reverse = 4 + 16
        self.rf_forward = 5 + 16
        self.rm_forward = 15
        self.rm_reverse = 16
        self.rf_reverse = 6 + 16
        self.rr_forward = 7 + 16
        self.rr_reverse = 8 + 16
        self.speed = 0
        self.direction = "stop"
        self.running = True
        self.speed_curve = np.linspace(0, 4095, 100, dtype=int)

        # Initialize MQTT client
        self.mqtt_client = mqtt.Client(protocol=mqtt.MQTTv5)
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.connect("mqtt.weedfucker.local", 1883, 60)
        self.mqtt_client.subscribe("/dev/drive/#")
        self.mqtt_client.loop_forever()

    def shutdown(self):
        self.stop()
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        self.running = False

    def on_connect(self, client, userdata, flags, rc, properties):
        self.mqtt_client.subscribe("/dev/drive/#")

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode('utf-8')
        if "update" in topic:
            return

        if "/dev/drive/speed" in topic:
            value = int(payload)
            value = min(max(value, 0), 100)
            normalized_speed = value / 100.0
            speed = int(4095 * (normalized_speed ** 2))
            if "left" in topic:
                self.set_left_drive_speed(speed)
            elif "right" in topic:
                self.set_right_drive_speed(speed)
            else:
                self.speed = speed
                self.set_speed()

        elif topic == "/dev/drive/direction":
            if payload == "left":
                self.left()
            elif payload == "right":
                self.right()
            elif payload == "forward":
                self.forward()
            elif payload == "reverse":
                self.reverse()
            elif payload == "stop":
                self.stop()

    def set_left_drive_forward_mode(self, mode):
        self.left_drive_forward_mode = mode

    def set_right_drive_forward_mode(self, mode):
        self.right_drive_forward_mode = mode

    def set_left_drive_speed(self, speed):
        self.left_drive_speed = speed
        self.pwm_controller.set_pwm(self.lf_forward, 0)
        self.pwm_controller.set_pwm(self.lf_reverse, 0)
        self.pwm_controller.set_pwm(self.lr_forward, 0)
        self.pwm_controller.set_pwm(self.lr_reverse, 0)
        # forward and backward are wired wrong
        if not self.left_drive_forward_mode:
            self.pwm_controller.set_pwm(self.lf_forward, speed)
            self.pwm_controller.set_pwm(self.lf_forward, speed)
        else:
            self.pwm_controller.set_pwm(self.lf_reverse, speed)
            self.pwm_controller.set_pwm(self.lf_reverse, speed)
        self.mqtt_client.publish("/dev/drive/update/speed/left", self.left_drive_speed)

    def set_right_drive_speed(self, speed):
        self.right_drive_speed = speed
        self.pwm_controller.set_pwm(self.rf_forward, 0)
        self.pwm_controller.set_pwm(self.rf_reverse, 0)
        self.pwm_controller.set_pwm(self.rr_forward, 0)
        self.pwm_controller.set_pwm(self.rr_reverse, 0)
        # forward and backward are wired wrong
        if not self.right_drive_forward_mode:
            self.pwm_controller.set_pwm(self.rf_forward, speed)
            self.pwm_controller.set_pwm(self.rr_forward, speed)
        else:
            self.pwm_controller.set_pwm(self.rf_reverse, speed)
            self.pwm_controller.set_pwm(self.rr_reverse, speed)
        self.mqtt_client.publish("/dev/drive/update/speed/right", self.right_drive_speed)

    def set_speed(self):
        value = self.speed
        print(f"Setting speed to {value}")
        print(f"Direction: {self.direction}")
        inside_value = value
        if self.direction == "forward":
            self.set_left_drive_forward_mode(True)
            self.set_right_drive_forward_mode(True)
            self.set_left_drive_speed(value)
            self.set_right_drive_speed(value)
        if self.direction == "reverse":
            self.set_left_drive_forward_mode(False)
            self.set_right_drive_forward_mode(False)
            self.set_left_drive_speed(value)
            self.set_right_drive_speed(value)
        if self.direction == "right":
            self.set_left_drive_forward_mode(True)
            self.set_right_drive_forward_mode(False)
            self.set_left_drive_speed(value)
            self.set_right_drive_speed(inside_value)
        if self.direction == "left":
            self.set_left_drive_forward_mode(False)
            self.set_right_drive_forward_mode(True)
            self.set_left_drive_speed(inside_value)
            self.set_right_drive_speed(value)
        if self.direction == "stop":
            self.set_left_drive_speed(0)
            self.set_right_drive_speed(0)
        self.mqtt_client.publish("/dev/drive/update/direction", self.direction)
        # log_path = Path(f"/var/training/data/drive/{datetime.utcnow().strftime('%Y-%m-%d')}.txt")
        # if not log_path.parent.exists():
        #     log_path.parent.mkdir(parents=True)
        # with open(log_path, "a") as f:
        #     f.write(f"{datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S')},{value},{self.direction}\n")

    def left(self):
        self.direction = "left"
        self.set_speed()

    def right(self):
        self.direction = "right"
        self.set_speed()

    def reverse(self):
        self.direction = "reverse"
        self.set_speed()

    def forward(self):
        self.direction = "forward"
        self.set_speed()

    def stop(self):
        self.direction = "stop"
        self.set_speed()

    def get_speed(self):
        return self.speed

    def get_direction(self):
        return self.direction

    def set_direction(self, last_direction):
        self.direction = last_direction
        # self.mqtt_client.publish("/dev/drive/direction", self.direction)

class DriveClient:
    def __init__(self, broker_address="mqtt.weedfucker.local", port=1883, topic_prefix="/dev/drive"):
        self.left_speed = 0
        self.right_speed = 0
        self.client = mqtt.Client(protocol=mqtt.MQTTv5)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(broker_address, port, 60)
        self.topic_prefix = topic_prefix
        self.speed = 0
        self.direction = "stop"
        self.message_handlers = {
            f"{self.topic_prefix}/update/speed":     self._update_speed,
            f"{self.topic_prefix}/update/direction": self._update_direction,
            }
        self.client.loop_start()
        atexit.register(self.shutdown)

    def _update_speed(self, speed):
        self.speed = speed

    def _update_direction(self, direction):
        self.direction = direction

    def get_speed(self):
        return self.speed

    def get_direction(self):
        return self.direction

    def on_connect(self, client, userdata, flags, rc, properties=None):
        print(f"Connected to MQTT broker with result code {rc}")
        self.client.subscribe(f"{self.topic_prefix}/#")

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode('utf-8')
        if topic in self.message_handlers:
            self.message_handlers[topic](payload)
        if f"{self.topic_prefix}/update/speed" in topic:
            if "left" in topic:
                self.left_speed = int(payload)
            elif "right" in topic:
                self.speed = int(payload)
                self.right_speed = int(payload)
            else:
                self.left_speed = int(payload)
                self.right_speed = int(payload)
            self.speed = max(self.left_speed, self.right_speed)

    def register_handler(self, topic, handler):
        full_topic = f"{self.topic_prefix}/update/{topic}"
        self.message_handlers[full_topic] = handler

    def publish(self, topic, payload):
        full_topic = f"{self.topic_prefix}/{topic}"
        self.client.publish(full_topic, payload, qos=2)

    def shutdown(self):
        self.client.loop_stop()
        self.client.disconnect()

    def forward(self):
        self.publish("direction", "forward")

    def reverse(self):
        self.publish("direction", "reverse")

    def left(self):
        self.publish("direction", "left")

    def right(self):
        self.publish("direction", "right")

    def stop(self):
        self.publish("direction", "stop")

    def set_speed(self, speed):
        self.publish("speed", int(speed))

    def set_left_speed(self, speed):
        self.publish("speed/left", int(speed))

    def set_right_speed(self, speed):
        self.publish("speed/right", int(speed))


if __name__ == "__main__":
    drive_controller = DriveController()
    while drive_controller.running:
        sleep(1)
