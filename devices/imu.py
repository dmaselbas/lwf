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
        # Initialize MQTT client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect(mqtt_broker, mqtt_port, 60)
        self.mqtt_client.loop_start()

        # Start the thread to publish data
        self.publish_thread = threading.Thread(target=self.publish_data, daemon=True)
        self.publish_thread.start()

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
        # Set accelerometer configuration
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)
        # Set gyroscope configuration
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)
        # Set interrupt enable
        self.bus.write_byte_data(self.address, self.INT_ENABLE, 0x01)

    def read_raw_data(self, reg):
        # Read two bytes of data from the specified register
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        value = (high << 8) | low
        # Convert to signed value
        if value > 32768:
            value -= 65536
        return value

    def get_accel_data(self):
        accel_x = self.read_raw_data(self.ACCEL_XOUT_H)
        accel_y = self.read_raw_data(self.ACCEL_YOUT_H)
        accel_z = self.read_raw_data(self.ACCEL_ZOUT_H)
        return {
            "x": accel_x / 16384.0,
            "y": accel_y / 16384.0,
            "z": accel_z / 16384.0
            }

    def get_gyro_data(self):
        gyro_x = self.read_raw_data(self.GYRO_XOUT_H)
        gyro_y = self.read_raw_data(self.GYRO_YOUT_H)
        gyro_z = self.read_raw_data(self.GYRO_ZOUT_H)
        return {
            "x": gyro_x / 131.0,
            "y": gyro_y / 131.0,
            "z": gyro_z / 131.0
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
                             {"x":   gyro_data["x"], "y": gyro_data["y"], "z": gyro_data["z"], "idx": "Gyroscope"
                              }]).set_index("idx", drop=True)

    def publish_data(self):
        while self.running:
            data = self.get_all_data()
            self.mqtt_client.publish("/dev/imu/data", json.dumps(data))
            time.sleep(1)
