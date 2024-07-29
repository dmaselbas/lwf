import atexit
import threading
import subprocess
from time import sleep
import paho.mqtt.client as mqtt

class LaserController:
    def __init__(self):
        self.direction_channel = 5
        self.pulse_channel = 7
        self.laser_on_channel = 8
        self.left_limit_pin = 25
        self.right_limit_pin = 23
        self.ok_to_move_left = True
        self.ok_to_move_right = True
        self.position = None
        self.max_position = None
        self.target_position = 10
        self.running = True
        self.pulse_time = 0
        self.setup_gpio(self.laser_on_channel, "out")
        self.setup_gpio(self.direction_channel, "out")
        self.setup_gpio(self.pulse_channel, "out")
        self.setup_gpio(self.left_limit_pin, "in")
        self.setup_gpio(self.right_limit_pin, "in")
        self.incrementor = 1
        self.cycle_motion = False
        self.laser_on_status = False
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect("mqtt.weedfucker.local", 1883, 60)
        self.mqtt_client.loop_start()
        atexit.register(self.shutdown)
        self.thread = threading.Thread(target=self.run, daemon=True, name="LaserController")
        self.thread.start()

    def shutdown(self):
        self.running = False
        self.running.mqtt_client.loop_stop()

    def enable_cycle_motion(self):
        self.cycle_motion = True

    def disable_cycle_motion(self):
        self.cycle_motion = False

    def move_left(self):
        self.set_gpio_output(self.direction_channel, 1)
        self.mqtt_client.publish("/dev/laser/move", "left")

    def move_right(self):
        self.set_gpio_output(self.direction_channel, 0)
        self.mqtt_client.publish("/dev/laser/move", "right")

    def set_gpio_output(self, channel_number, value):
        subprocess.run(["gpio", "write", str(channel_number), str(value)], check=True)

    def get_gpio_input(self, channel_number):
        result = subprocess.run(["gpio", "read", str(channel_number)], capture_output=True, text=True, check=True)
        return result.stdout.strip() == '1'

    def setup_gpio(self, channel_number, mode):
        mode_str = mode
        subprocess.run(["gpio", "mode", str(channel_number), mode_str], check=True)

    def run(self):
        while self.running:
            while self.position != self.target_position or self.position is None or self.max_position is None:
                if self.position is None:
                    if self.get_gpio_input(self.left_limit_pin):
                        self.position = 0
                        self.move_right()
                    else:
                        self.move_left()
                    self.set_gpio_output(self.pulse_channel, 1)
                    self.set_gpio_output(self.pulse_channel, 0)
                if self.position is not None and self.max_position is None:
                    if self.get_gpio_input(self.right_limit_pin):
                        self.max_position = self.position
                        self.move_left()
                        self.target_position = self.max_position // 2
                    else:
                        self.move_right()
                        self.position += 1
                    self.set_gpio_output(self.pulse_channel, 1)
                    self.set_gpio_output(self.pulse_channel, 0)
                if self.position is not None and self.max_position is not None:
                    if self.target_position < self.position:
                        self.move_left()
                        self.set_gpio_output(self.pulse_channel, 1)
                        self.set_gpio_output(self.pulse_channel, 0)
                        self.position -= 1
                    if self.target_position > self.position:
                        self.move_right()
                        self.set_gpio_output(self.pulse_channel, 1)
                        self.set_gpio_output(self.pulse_channel, 0)
                        self.position += 1
            if self.get_gpio_input(self.left_limit_pin):
                self.position = 0
                self.move_right()
            if self.get_gpio_input(self.right_limit_pin):
                self.position = self.max_position
                self.move_left()
            self.mqtt_client.publish("/dev/laser/position", str(self.position))
            if self.cycle_motion:
                if self.position == 0:
                    self.target_position = self.max_position
                if self.position == self.max_position:
                    self.target_position = 0
            sleep(0.001)  # Avoid blocking the CPU

    def on(self):
        self.set_gpio_output(self.laser_on_channel, 1)
        self.laser_on_channel = True

    def off(self):
        self.set_gpio_output(self.laser_on_channel, 0)
        self.laser_on_status = False

    def move_to_position(self, position):
        self.target_position = max(0, min(position, self.max_position))
