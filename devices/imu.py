import atexit

import pandas as pd
import smbus3
import time
import threading
import paho.mqtt.client as mqtt
import json


class MPU6050:
    # MPU-6050 Registers
    PWR_MGMT_1 = 0x6B
    SMPLRT_DIV = 0x19
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C
    INT_ENABLE = 0x38
    ACCEL_XOUT_H = 0x3B
    ACCEL_YOUT_H = 0x3D
    ACCEL_ZOUT_H = 0x3F
    GYRO_XOUT_H = 0x43
    GYRO_YOUT_H = 0x45
    GYRO_ZOUT_H = 0x47

    def __init__(self, bus=3, address=0x68, mqtt_broker="mqtt.weedfucker.local", mqtt_port=1883):
        self.bus = smbus3.SMBus(bus)
        self.address = address
        self.initialize_sensor()
        self.running = True
        atexit.register(self.shutdown)
        # Start the thread to publish data
        self.publish_thread = threading.Thread(target=self.publish_data, daemon=True)
        self.publish_thread.start()
        # Initialize MQTT client
        self.mqtt_client = mqtt.Client(protocol=mqtt.MQTTv5)
        self.mqtt_client.connect(mqtt_broker, mqtt_port, 60)
        self.mqtt_client.loop_forever()


    def shutdown(self):
        print("Shutting down MPU6050...")
        self.running = False
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        self.bus.close()

    def initialize_sensor(self):
        # Wake up the MPU-6050 since it starts in sleep mode
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        # Set sample rate to 1kHz
        self.bus.write_byte_data(self.address, self.SMPLRT_DIV, 0x07)
        time.sleep(0.1)
        # Set accelerometer configuration
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)
        time.sleep(0.1)
        # Set gyroscope configuration
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)
        time.sleep(0.1)
        # Set interrupt enable
        self.bus.write_byte_data(self.address, self.INT_ENABLE, 0x01)
        time.sleep(0.1)

    def read_raw_data(self, reg):
        # Read two bytes of data from the specified register
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        value = (high << 8) | low
        # Convert to signed value
        if value > 32768:
            value -= 65536
        return value

    def publish_data(self):
        while self.running:
            gyro_x = self.read_raw_data(self.GYRO_XOUT_H) / 131.0
            self.mqtt_client.publish("/dev/imu/gyro/x", gyro_x)
            time.sleep(.5)
            gyro_y = self.read_raw_data(self.GYRO_YOUT_H) / 131.0
            self.mqtt_client.publish("/dev/imu/gyro/y", gyro_y)
            time.sleep(.5)
            gyro_z = self.read_raw_data(self.GYRO_ZOUT_H) / 131.0
            self.mqtt_client.publish("/dev/imu/gyro/z", gyro_z)
            time.sleep(.5)
            accel_x = self.read_raw_data(self.ACCEL_XOUT_H) / 16384.0
            self.mqtt_client.publish("/dev/imu/accel/x", accel_x)
            time.sleep(.5)
            accel_y = self.read_raw_data(self.ACCEL_YOUT_H) / 16384.0
            self.mqtt_client.publish("/dev/imu/accel/y", accel_y)
            time.sleep(.5)
            accel_z = self.read_raw_data(self.ACCEL_ZOUT_H) / 16384.0
            self.mqtt_client.publish("/dev/imu/accel/z", accel_z)
            time.sleep(.5)


class IMUClient:

    def __init__(self):
        self.accel_x = 0
        self.accel_y = 0
        self.accel_z = 0
        self.gyro_x = 0
        self.gyro_y = 0
        self.gyro_z = 0

        self.mqtt_client = mqtt.Client(protocol=mqtt.MQTTv5)
        self.mqtt_client.connect("mqtt.weedfucker.local", 1883, 60)
        self.mqtt_client.loop_start()

        self.mqtt_client.on_message = self._on_message

    def _on_connect(self, client, userdata, flags, rc):
        client.subscribe("/dev/imu/#")

    def _on_message(self, client, userdata, message):
        type = message.topic.split("/")[-2]
        axis = message.topic.split("/")[-1]
        value = float(message.payload.decode("utf-8"))
        if type == "accel":
            setattr(self, f"accel_{axis}", value)
        if type == "gyro":
            setattr(self, f"gyro_{axis}", value)

    def get_accel_data(self):
        return {
            "x": self.accel_x,
            "y": self.accel_y,
            "z": self.accel_z
            }

    def get_gyro_data(self):
        return {
            "x": self.gyro_x,
            "y": self.gyro_y,
            "z": self.gyro_z
            }

    def get_all_data(self):
        accel_data = self.get_accel_data()
        gyro_data = self.get_gyro_data()
        return {
            "accel": accel_data,
            "gyro":  gyro_data
            }

    def get_all_data_as_df(self):
        accel_data = self.get_accel_data()
        gyro_data = self.get_gyro_data()
        return pd.DataFrame([{"x": accel_data["x"], "y": accel_data["y"], "z": accel_data["z"], "idx": "Accelerometer"},
                             {"x": gyro_data["x"], "y": gyro_data["y"], "z": gyro_data["z"], "idx": "Gyroscope"
                              }]).set_index("idx", drop=True)


if __name__ == "__main__":
    imu = MPU6050()
