import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time


class TwistToSerialDutyCycle(Node):
    def __init__(self, wheel_separation, wheel_radius, max_rpm, serial_port, baud_rate):
        super().__init__('twist_to_serial_duty_cycle')

        # Set the wheel parameters
        self.wheel_separation = wheel_separation
        self.wheel_radius = wheel_radius
        self.max_rpm = max_rpm

        # Set up the serial connection to the microcontroller
        self.serial_connection = serial.Serial(serial_port, baud_rate, timeout=1)
        time.sleep(2)  # Wait for the serial connection to initialize

        # Subscriber to the Twist message (cmd_vel)
        self.subscription = self.create_subscription(
                Twist,
                'cmd_vel',
                self.twist_callback,
                10)

    def twist_callback(self, msg):
        # Extract linear and angular velocities from Twist
        linear_velocity = msg.linear.x  # forward/backward speed in m/s
        angular_velocity = msg.angular.z  # rotation around z-axis in rad/s

        # Calculate wheel velocities (in m/s)
        v_left = (linear_velocity - (angular_velocity * self.wheel_separation / 2))
        v_right = (linear_velocity + (angular_velocity * self.wheel_separation / 2))

        # Convert wheel velocities from m/s to RPM
        left_rpm = (v_left / (2 * 3.1416 * self.wheel_radius)) * 60
        right_rpm = (v_right / (2 * 3.1416 * self.wheel_radius)) * 60

        # Map RPM to duty cycle (-1023 to 1023) based on max_rpm
        left_duty_cycle = int(max(-1023, min(1023, (left_rpm / self.max_rpm) * 1023)))
        right_duty_cycle = int(max(-1023, min(1023, (right_rpm / self.max_rpm) * 1023)))

        # Send the duty cycles to the microcontroller
        self.send_duty_cycle_to_microcontroller(left_duty_cycle, right_duty_cycle)

        # Log the values for debugging
        self.get_logger().info(f'Sent Left Duty Cycle: {left_duty_cycle}, Right Duty Cycle: {right_duty_cycle}')

    def send_duty_cycle_to_microcontroller(self, left_duty_cycle, right_duty_cycle):
        # Format the message as a string, e.g., "L512R-512\n"
        message = f'L{left_duty_cycle}R{right_duty_cycle}\n'

        # Send the message over the serial connection
        self.serial_connection.write(message.encode())

    def destroy_node(self):
        # Close the serial connection when shutting down
        self.serial_connection.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    # Define your robot's parameters (in meters)
    wheel_separation = 0.4572  # Distance between wheels in meters
    wheel_radius = 0.0762  # Radius of each wheel in meters
    max_rpm = 10  # Max RPM of the motors

    # Define serial connection parameters
    serial_port = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B00465V5-if00-port0'  # Change to your actual serial port
    baud_rate = 115200  # Make sure this matches the microcontroller's baud rate

    # Create and spin the node
    twist_to_serial_duty_cycle_node = TwistToSerialDutyCycle(
            wheel_separation, wheel_radius, max_rpm, serial_port, baud_rate)

    rclpy.spin(twist_to_serial_duty_cycle_node)

    # Shutdown
    twist_to_serial_duty_cycle_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
