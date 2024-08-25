import asyncio
import atexit
import threading
import subprocess
from time import sleep

import numpy as np
import paho.mqtt.client as mqtt
import time

from devices.pwm_controller import PWMClient


class LaserController:
    def __init__(self):
        self.motor_on = None
        self.wiggle_future = None
        self.cycle_motion_sleep = None
        self.direction_channel = 30
        self.pulse_channel = 31
        self.laser_on_channel = 32
        self.left_limit_pin = 25
        self.right_limit_pin = 23
        self.ok_to_move_left = True
        self.ok_to_move_right = True
        self.position = 0
        self.max_position = 100
        self.target_position = 10
        self.running = True
        self.manual_mode = False
        self.pulse_time = 1
        self.pulse_laser_enabled = False
        self.pwm = PWMClient()
        self.setup_gpio(self.direction_channel, "out")
        self.setup_gpio(self.left_limit_pin, "in")
        self.setup_gpio(self.right_limit_pin, "in")
        self.incrementor = 1
        self.cycle_motion = False
        self.laser_on_status = False
        self.mqtt_client = mqtt.Client(protocol=mqtt.MQTTv5)
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect("mqtt.weedfucker.local", 1883, 60)
        self.mqtt_client.subscribe("/dev/laser/#")
        self.thread = threading.Thread(target=self.run, daemon=True, name="LaserController")
        self.thread.start()
        atexit.register(self.shutdown)
        self.mqtt_client.loop_forever()

    def shutdown(self):
        self.off()
        self.motor_stop()
        self.running = False
        self.mqtt_client.loop_stop()


    def check_limits(self):
        while self.running:
            if self.get_gpio_input(self.left_limit_pin):
                self.position = 0
                self.move_right()

            if self.get_gpio_input(self.right_limit_pin):
                self.position = self.max_position
                self.move_left()
            sleep(1)


    def enable_cycle_motion(self):
        self.cycle_motion = True

    def disable_cycle_motion(self):
        self.cycle_motion = False

    def move_left(self):
        self.manual_mode = True
        self.pwm.set_pwm(self.direction_channel, "on")
        self.mqtt_client.publish("/dev/laser/move", "left")

    def move_right(self):
        self.manual_mode = True
        self.pwm.set_pwm(self.direction_channel, "off")
        self.mqtt_client.publish("/dev/laser/move", "right")

    def set_gpio_output(self, channel_number, value):
        subprocess.run(["gpio", "write", str(channel_number), str(value)], check=True)

    def get_gpio_input(self, channel_number):
        result = subprocess.run(["gpio", "read", str(channel_number)], capture_output=True, text=True, check=True)
        result = result.stdout.strip() == '1'
        return result

    def setup_gpio(self, channel_number, mode):
        mode_str = mode
        subprocess.run(["gpio", "mode", str(channel_number), mode_str], check=True)

    async def pulse_laser(self):
        for _ in range(1000):
            self.pwm.set_pwm(self.laser_on_channel, 2056)
            await asyncio.sleep(0.25)
            self.off()
            await asyncio.sleep(0.75)

    def wiggle_motor(self):
        self.left_mode = True
        frequency = 5  # Frequency of the sine wave
        sampling_rate = 1000  # Number of samples per second
        duration = 2  # Duration in seconds
        # Time array
        t = np.linspace(0, duration, int(sampling_rate * duration), endpoint=False)

        # Sine wave with diminishing amplitude
        amplitude = np.exp(-t)  # Exponential decay
        sine_wave = amplitude * np.sin(2 * np.pi * frequency * t)
        while self.cycle_motion:
            for value in sine_wave:
                if self.cycle_motion is False:
                    break
                if self.get_gpio_input(self.left_limit_pin):
                    self.move_right()
                if self.get_gpio_input(self.right_limit_pin):
                    self.move_left()
                self.pulse_motor()
                self.check_limits()
                sleep(min((0.0001 * value), 0.5))
                self.check_limits()
                self.motor_stop()
                left_mode = not self.left_mode
                if left_mode:
                    self.move_left()
                else:
                    self.move_right()

    def run(self):
        while self.running:
            if self.pulse_laser_enabled:
                asyncio.run(self.pulse_laser())
            self.position += self.incrementor
            if self.position >= self.max_position:
                self.incrementor = -1
            elif self.position <= 0:
                self.incrementor = 1
            if self.get_gpio_input(self.left_limit_pin):
                self.position = 0
                self.move_right()
            if self.get_gpio_input(self.right_limit_pin):
                self.position = self.max_position
                self.move_left()
            if self.cycle_motion:
                self.wiggle_motor()


    def pulse_motor(self, is_init=False):
        self.pwm.set_pwm(self.pulse_channel, 1024)

    def motor_stop(self):
        self.pwm.set_pwm(self.pulse_channel, 0)

    def motor_start(self):
        self.pulse_motor()

    def on(self):
        self.motor_on = True
        self.pwm.set_pwm(self.laser_on_channel, 4095)

    def off(self):
        self.motor_on = False
        self.pwm.set_pwm(self.laser_on_channel, 0)

    def move_to_position(self, position):
        if position < 500:
            self.move_left()
            self.pulse_motor()
            sleep(position / 1000)
            self.motor_stop()
        elif position >= 500:
            self.move_right()
            self.pulse_motor()
            sleep(position - 500 / 1000)
            self.motor_stop()

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode()

        if topic == "/dev/laser/enable_cycle_motion":
            self.enable_cycle_motion()
        elif topic == "/dev/laser/disable_cycle_motion":
            self.disable_cycle_motion()
        elif topic == "/dev/laser/move_left":
            self.move_left()
        elif topic == "/dev/laser/move_right":
            self.move_right()
        elif topic == "/dev/laser/on":
            self.pulse_laser_enabled = False
            self.on()
        elif topic == "/dev/laser/off":
            self.pulse_laser_enabled = False
            self.off()
        elif topic == "/dev/laser/pulse_laser":
            self.pulse_laser_enabled = True
        elif topic == "/dev/laser/move_to_position":
            try:
                position = int(payload)
                self.move_to_position(position)
            except ValueError:
                print(f"Invalid position value: {payload}")
        elif topic == "/dev/laser/motor_start":
            self.motor_start()
        elif topic == "/dev/laser/motor_stop":
            self.motor_stop()
        else:
            print(f"Unknown command: {topic}")


class LaserClient:
    def __init__(self, broker_address="mqtt.weedfucker.local", port=1883):
        self.mqtt_client = mqtt.Client(protocol=mqtt.MQTTv5)
        self.mqtt_client.connect(broker_address, port, 60)
        self.mqtt_client.loop_start()
        atexit.register(self.disconnect)

    def send_message(self, topic, payload):
        self.mqtt_client.publish(topic, payload)

    def enable_cycle_motion(self):
        self.send_message("/dev/laser/enable_cycle_motion", "")

    def disable_cycle_motion(self):
        self.send_message("/dev/laser/disable_cycle_motion", "")

    def move_left(self):
        self.send_message("/dev/laser/move_left", "")

    def move_right(self):
        self.send_message("/dev/laser/move_right", "")

    def turn_on(self):
        self.send_message("/dev/laser/on", "")

    def turn_off(self):
        self.send_message("/dev/laser/off", "")

    def pulse_laser(self):
        self.send_message("/dev/laser/pulse", "")

    def move_to_position(self, position):
        self.send_message("/dev/laser/move_to_position", str(position))

    def motor_start(self):
        self.send_message("/dev/laser/motor_start", "")

    def motor_stop(self):
        self.send_message("/dev/laser/motor_stop", "")

    def disconnect(self):
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()


if __name__ == "__main__":
    laser_controller = LaserController()
    laser_controller.off()
    laser_controller.motor_stop()
    while laser_controller.running:
        time.sleep(1)
