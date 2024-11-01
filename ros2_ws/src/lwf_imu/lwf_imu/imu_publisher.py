import pandas as pd
import numpy as np
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import math
import serial
import struct
import rclpy
from rclpy.node import Node
from time import sleep


class BNO085IMUPublisher(Node):
    def __init__(self):
        super().__init__('bno085_imu_publisher')

        # Initialize serial port for UART communication
        self.serial_port = '/dev/ttyUSB0'  # Replace with the appropriate port
        self.baud_rate = 115200  # Baud rate for BNO085 in UART-RVC mode
        self.serial_connection = serial.Serial(self.serial_port, self.baud_rate, timeout=1)

        # Publisher for IMU data
        self.imu_publisher = self.create_publisher(Imu, 'imu/data', 10)

        # Set a timer to publish data at 10 Hz (0.1 seconds)
        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.publish_imu_data)

        # Data collection setup
        self.data_collected = False
        self.df = pd.DataFrame(columns=['yaw', 'pitch', 'roll', 'accel_x', 'accel_y', 'accel_z'])
        self.collection_count = 0
        self.covariance_matrices = {
            "orientation":         np.zeros((3, 3)),
            "linear_acceleration": np.zeros((3, 3))
            }

    def read_rvc_data(self):
        """
        Read and parse the RVC data from BNO085 in UART-RVC mode.
        """
        # Try to read a full 19-byte frame as described previously
        while True:
            try:
                byte1 = self.serial_connection.read(1)
                if byte1 == b'\xaa':
                    byte2 = self.serial_connection.read(1)
                    if byte2 == b'\xaa':
                        # Read the remaining 17 bytes of the frame
                        frame = self.serial_connection.read(17)
                        break
                self.get_logger().warn("Resynchronizing on sync bytes")
            except serial.SerialException as e:
                self.get_logger().warn(f"Serial read error: {e}")
                self.serial_connection.close()
                sleep(1)  # Wait a moment before reconnecting
                self.serial_connection.open()
                continue

        # Construct the frame and unpack it
        frame = b'\xaa\xaa' + frame
        if len(frame) != 19:
            self.get_logger().warn(f"Incomplete frame received: expected 19 bytes, got {len(frame)}")
            return None

        header, index, yaw, pitch, roll, accel_x, accel_y, accel_z, reserved1, reserved2, reserved3 = struct.unpack_from(
            '<HBhhhhhhBBB',
            frame,
            0)

        # Convert yaw, pitch, roll from 0.01 degree units to radians
        yaw = (yaw * 0.01) * (math.pi / 180.0)
        pitch = (pitch * 0.01) * (math.pi / 180.0)
        roll = (roll * 0.01) * (math.pi / 180.0)

        # Convert acceleration from mg to m/s²
        accel_x = accel_x * 9.80665 / 1000
        accel_y = accel_y * 9.80665 / 1000
        accel_z = accel_z * 9.80665 / 1000

        return yaw, pitch, roll, accel_x, accel_y, accel_z

    def collect_data(self, data):
        """Collect the first 1000 parsed messages in a DataFrame."""
        yaw, pitch, roll, accel_x, accel_y, accel_z = data
        self.df = self.df.append(
                {'yaw':     yaw, 'pitch': pitch, 'roll': roll,
                 'accel_x': accel_x, 'accel_y': accel_y, 'accel_z': accel_z
                 },
                ignore_index=True
                )
        self.collection_count += 1

        if self.collection_count >= 1000:
            self.data_collected = True
            self.calculate_covariance_matrices()

    def calculate_covariance_matrices(self):
        """Calculate covariance matrices for orientation and acceleration from the collected data."""
        # Covariance for orientation (yaw, pitch, roll)
        orientation_cov = self.df[['yaw', 'pitch', 'roll']].cov().values
        self.covariance_matrices["orientation"] = orientation_cov.flatten()

        # Covariance for linear acceleration (accel_x, accel_y, accel_z)
        accel_cov = self.df[['accel_x', 'accel_y', 'accel_z']].cov().values
        self.covariance_matrices["linear_acceleration"] = accel_cov.flatten()

        self.get_logger().info("Covariance matrices calculated. Starting to publish messages.")

    def publish_imu_data(self):
        data = self.read_rvc_data()

        if data is None:
            return  # Skip publishing if data is invalid

        # If data collection is not complete, collect the data without publishing
        if not self.data_collected:
            self.collect_data(data)
            return

        # Extract each measurement
        yaw, pitch, roll, accel_x, accel_y, accel_z = data

        # IMU message for orientation and linear acceleration
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Set orientation in quaternion format (yaw-pitch-roll to quaternion)
        qx, qy, qz, qw = self.ypr_to_quaternion(yaw, pitch, roll)
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw

        # Set covariance for orientation and acceleration
        imu_msg.orientation_covariance = list(self.covariance_matrices["orientation"])
        imu_msg.linear_acceleration_covariance = list(self.covariance_matrices["linear_acceleration"])

        # Linear acceleration (m/s²)
        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z

        # Publish the IMU message
        self.imu_publisher.publish(imu_msg)
        self.get_logger().info("Published IMU data with covariance.")

    def ypr_to_quaternion(self, yaw, pitch, roll):
        """Convert yaw, pitch, and roll to quaternion representation."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw

    def destroy(self):
        """Close the serial connection on node destruction."""
        if self.serial_connection.is_open:
            self.serial_connection.close()


def main(args=None):
    rclpy.init(args=args)
    imu_publisher = BNO085IMUPublisher()

    try:
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        imu_publisher.get_logger().info("IMU Publisher node shutting down")
    finally:
        imu_publisher.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
