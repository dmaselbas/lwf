from devices.pwm_controller import PWMController
import paho.mqtt.client as mqtt

class LaserController:
    def __init__(self, controller:PWMController, client_mode):
        self.controller = controller
        self.client_mode = client_mode
        self.mqtt_client = None

    def setup_mqtt_client(self):
        self.mqtt_client = mqtt.Client()
        if not self.client_mode:
            self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.connect("10.0.0.2", port=1883)
        self.mqtt_client.loop_start()

    def on_mqtt_message(self, client, userdata, msg):
        topic = msg.topic
        message = msg.payload.decode()

        if topic == "/dev/laser":
            if message == "1":
                self.on()
            elif message == "0":
                self.off()
        elif topic == "/dev/laser/move":
            if message == "left":
                self.move_left()
            elif message == "right":
                self.move_right()
        elif topic == "/dev/laser/stop":
            self.stop()

    def on(self):
        if self.client_mode:
            self.mqtt_client.publish("/dev/laser", "1")
        else:
            self.controller.set_pwm(6, 4095, 0)  # Set channel 6 to 100% duty cycle

    def off(self):
        if self.client_mode:
            self.mqtt_client.publish("/dev/laser", "0")
        else:
            self.controller.set_pwm(6, 0, 0)  # Set channel 6 to 0% duty cycle

    def move_left(self):
        if self.client_mode:
            self.mqtt_client.publish("/dev/laser/move", "left")
        else:
            self.controller.set_pwm(5, 4095, 0)  # Set channel 5 to 100% duty cycle
            self.controller.set_pwm(4, 1023, 0)  # Set channel 4 to 25% duty cycle

    def move_right(self):
        if self.client_mode:
            self.mqtt_client.publish("/dev/laser/move", "right")
        else:
            self.controller.set_pwm(5, 0, 0)  # Set channel 5 to 0% duty cycle
            self.controller.set_pwm(4, 1023, 0)  # Set channel 4 to 25% duty cycle

    def stop(self):
        if self.client_mode:
            self.mqtt_client.publish("/dev/laser/stop", "")
        else:
            self.controller.set_pwm(4, 0, 0)  # Set channel 4 to 0% duty cycle

    def __del__(self):
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
