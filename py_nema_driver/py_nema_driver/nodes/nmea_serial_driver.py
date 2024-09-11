# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Defines the main method for the nmea_serial_driver executable."""

import serial
from py_nema_driver.driver import RosNMEADriver
import rclpy
from rclpy.node import Node

class NmeaSerialDriverNode(Node):

    def __init__(self):
        super().__init__('nmea_serial_driver')

        self.subscription = self.create_publisher( 'std_msgs/String', 'nmea_sentence', 10)
        self.driver = RosNMEADriver()

        self.serial_port = "/dev/tty5"
        self.serial_baud = 9600
        self.frame_id = RosNMEADriver.get_frame_id()
        self.serial_port = None

    def spin(self):
        try:
            self.serial_port = serial.Serial(port=self.serial_port, baudrate=self.serial_baud, timeout=2)
        except serial.SerialException as e:
            self.get_logger().error("Failed to open serial port: %s", str(e))
            self.context.shutdown()
        for line in iter(self.serial_port.readline, b''):
            sentence = line.decode('ascii').strip()
            if sentence:
                self.subscription.publish(sentence)
                try:
                    self.driver.add_sentence(sentence, frame_id=self.frame_id)
                except ValueError as e:
                    self.get_logger().warn("Invalid NMEA sentence: %s", str(e))
def main(args=None):
    rclpy.init(args=args)
    node = NmeaSerialDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
