# Software License Agreement (BSD License)
#
# Copyright (c) 2016, Rein Appeldoorn
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

"""Defines the main method for the nmea_socket_driver executable."""


import select
import traceback

try:
    import socketserver
except ImportError:
    import SocketServer as socketserver  # Python 2.7

import rclpy
from rclpy.node import Node
from py_nema_driver.driver import RosNMEADriver


class NMEAMessageHandler(socketserver.DatagramRequestHandler):
    def handle(self):
        for line in self.rfile:
            line = line.strip()
            if not line:
                continue

            try:
                nmea_str = line.decode('ascii')
                self.server.driver.add_sentence(nmea_str, self.server.frame_id)
            except UnicodeError as e:
                rclpy.logwarn("Skipped reading a line from the UDP socket because it could not be "
                              "decoded as an ASCII string. The bytes were {0}".format(line))
            except ValueError:
                rclpy.logwarn(
                    "ValueError, likely due to missing fields in the NMEA "
                    "message. Please report this issue at "
                    "https://github.com/ros-drivers/nmea_navsat_driver"
                    ", including the following:\n\n"
                    "```\n" +
                    repr(line) + "\n\n" +
                    traceback.format_exc() +
                    "```")

class NMEASocketDriverServer(Node):
    def __init__(self):
        super().__init__('pynema_socket_driver_server')
        self.driver = None
        self.frame_id = None

        self.local_ip =  '0.0.0.0'
        self.local_port =  10110
        self.timeout =  2

        # Create a socket
        self.server = socketserver.UDPServer((self.local_ip, self.local_port), NMEAMessageHandler,
                                        bind_and_activate=False)
        self.server.frame_id = RosNMEADriver.get_frame_id()
        self.server.driver = RosNMEADriver()

        # Start listening for connections
        self.server.server_bind()
        self.server.server_activate()

        # Handle incoming connections until ROS shuts down
        try:
            while self.context.ok():
                rlist, _, _ = select.select([self.server], [], [], self.timeout)
                if self.server in rlist:
                    self.server.handle_request()
        except Exception:
            self.get_logger().error(traceback.format_exc())
        finally:
            self.server.server_close()

def main(args=None):
    rclpy.init_node(args=args)
    server = NMEASocketDriverServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()
