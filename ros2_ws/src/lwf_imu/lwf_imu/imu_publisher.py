#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import struct
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header
import math


class BNO085IMUPublisher(Node):
    def __init__(self):
        super().__init__('bno085_imu_publisher')

        # Initialize serial port for UART communication
        self.serial_port = '/dev/ttyUSB0'  # Replace with the appropriate port
        self.baud_rate = 115200  # Typical baud rate for BNO085 in UART mode
        self.serial_connection = serial.Serial(self.serial_port, self.baud_rate, timeout=1)

        # Publishers for IMU data and magnetometer data
        self.imu_publisher = self.create_publisher(Imu, 'imu/data', 10)
        self.magnetic_field_publisher = self.create_publisher(MagneticField, 'imu/mag', 10)

        # Set a timer to read data at 50 Hz (20ms)
        self.timer_period = 0.02  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_imu_data)

    def read_rvc_data(self):
        """
        Read and parse the RVC data from BNO085.
        The RVC frame format (for BNO085) is:
        [0xAA 0xAA (yaw_pitch_roll) x 3 bytes each + (ang_vel_x, ang_vel_y, ang_vel_z) + (accel_x, accel_y, accel_z) +
        (mag_x, mag_y, mag_z) + checksum]
        Returns all 9 DOFs: orientation (yaw, pitch, roll), angular velocity, linear acceleration, and magnetometer data.
        """
        # Read 38 bytes: 2 sync + 3 float32 (orientation) + 3 float32 (angular velocity) + 3 float32 (acceleration) +
        # 3 float32 (magnetometer) + 1 checksum
        frame = self.serial_connection.read(38)

        if len(frame) < 38 or frame[0:2] != b'\xaa\xaa':
            self.get_logger().warn("Invalid frame received")
            return None, None, None, None, None, None, None, None, None

        # Unpack orientation (yaw, pitch, roll), angular velocity, linear acceleration, and magnetometer data
        yaw, pitch, roll, ang_vel_x, ang_vel_y, ang_vel_z, accel_x, accel_y, accel_z, mag_x, mag_y, mag_z = struct.unpack_from(
                '<ffffffffffff', frame, offset=2
                )

        # Convert from degrees to radians for orientation
        yaw = yaw * math.pi / 180.0
        pitch = pitch * math.pi / 180.0
        roll = roll * math.pi / 180.0

        return yaw, pitch, roll, ang_vel_x, ang_vel_y, ang_vel_z, accel_x, accel_y, accel_z, mag_x, mag_y, mag_z

    def publish_imu_data(self):
        data = self.read_rvc_data()

        if data is None:
            return  # Skip publishing if data is invalid

        # Extract each measurement
        yaw, pitch, roll, ang_vel_x, ang_vel_y, ang_vel_z, accel_x, accel_y, accel_z, mag_x, mag_y, mag_z = data

        # IMU message for orientation, angular velocity, and linear acceleration
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

        # Angular velocity (rad/s)
        imu_msg.angular_velocity.x = ang_vel_x
        imu_msg.angular_velocity.y = ang_vel_y
        imu_msg.angular_velocity.z = ang_vel_z

        # Linear acceleration (m/s^2)
        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z

        # Publish the IMU message
        self.imu_publisher.publish(imu_msg)
        self.get_logger().info("Published IMU data")

        # Magnetometer data in MagneticField message
        mag_msg = MagneticField()
        mag_msg.header = imu_msg.header  # Use the same timestamp and frame id as IMU data
        mag_msg.magnetic_field.x = mag_x * 1e-6  # Convert from microteslas to teslas
        mag_msg.magnetic_field.y = mag_y * 1e-6  # Convert from microteslas to teslas
        mag_msg.magnetic_field.z = mag_z * 1e-6  # Convert from microteslas to teslas

        # Publish the MagneticField message
        self.magnetic_field_publisher.publish(mag_msg)
        self.get_logger().info("Published Magnetometer data")

    def ypr_to_quaternion(self, yaw, pitch, roll):
        """
        Convert yaw, pitch, and roll to quaternion representation.
        """
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
        # Close the serial connection on node destruction
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
