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

"""Defines the main method for the nmea_topic_serial_reader executable."""

import serial

from nmea_msgs.msg import Sentence
import rclpy
from prompt_toolkit.key_binding.bindings.named_commands import self_insert
from rclpy.node import Node
from py_nema_driver.driver import RosNMEADriver
class SerialNMEAReader(Node):

    def __init__(self):
        self.nmea_pub = self.create_publisher("nmea_sentence", Sentence, queue_size=1)

        self.serial_port = '/dev/ttyUSB0'
        self.serial_baud =  9600

        # Get the frame_id
        self.frame_id = RosNMEADriver.get_frame_id()

        self.GPS = serial.Serial(port=self.serial_port, baudrate=self.serial_baud, timeout=2)
        while self.context.ok():
            data = self.GPS.readline().strip()

            sentence = Sentence()
            sentence.header.stamp = self.get_clock().now()
            sentence.header.frame_id = self.frame_id

            try:
                sentence.sentence = data.decode('ascii')
            except UnicodeError as e:
                self.get_logger().warn("Skipped reading a line from the serial device because it could not be "
                              "decoded as an ASCII string. The bytes were {0}".format(data))
            else:
                self.nmea_pub.publish(sentence)

    def shutdown(self):
        self.GPS.close()
        rclpy.shutdown()

def main():
    """Create and run the nmea_topic_serial_reader ROS node.

    Opens a serial device and publishes data from the device as nmea_msgs.msg.Sentence messages.

    :ROS Parameters:
        - ~port (str)
            Path of the serial device to open.
        - ~baud (int)
            Baud rate to configure the serial device.

    :ROS Publishers:
        - nmea_sentence (nmea_msgs.msg.Sentence)
            Publishes each line from the openserial device as a new message. The header's stamp is
            set to the rostime when the data is read from the serial device.
    """
    rclpy.init_node('nmea_topic_serial_reader')
