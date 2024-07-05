import paho.mqtt.client as mqtt
from smbus3 import SMBus


class PCA9685:
    def __init__(self, address=0x40):
        self.address = address
        self.bus = SMBus(1)  # Use I2C bus 1
        self.bus.write_byte(self.address, 0x00)  # Reset the PCA9685

    def set_pwm_frequency(self, frequency):
        prescale = 25000000.0 / (4096.0 * float(frequency)) - 1.0
        prescale = int(round(prescale))
        self.bus.write_byte_data(self.address, 0xFE, prescale)

    def set_pwm(self, channel, on_value, off_value):
        self.bus.write_word_data(self.address, 0x06 + 4 * channel, on_value)
        self.bus.write_word_data(self.address, 0x08 + 4 * channel, off_value)


class PWMController:
    def __init__(self):
        self.broker_address = "10.0.0.2"
        self.topics = ["/dev/pwm/{}".format(i) for i in range(17)]  # 0-16
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected with result code {rc}")
        for topic in self.topics:
            self.client.subscribe(topic)

    def on_message(self, client, userdata, msg):
        message = msg.payload.decode()
        print(f"Message received on topic {msg.topic}: {message}")
        if msg.topic in self.topics:
            channel = int(msg.topic.split("/")[-1])
            on_value, off_value = int(message.split(",")[0]), int(message.split(",")[1])
            self.pwm.set_pwm(channel, on_value, off_value)

    def run(self):
        self.client.connect(self.broker_address, port=1883)
        self.client.loop_start()
