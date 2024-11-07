# Copyright 2018 Lucas Walter
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
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
#  * Neither the name of Lucas Walter nor the names of its
#    contributors may be used to endorse or promote products derived
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

import argparse
import os
from pathlib import Path  # noqa: E402
import sys

# Hack to get relative import of .camera_config file working
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)

from launch import LaunchDescription  # noqa: E402
from launch.actions import GroupAction  # noqa: E402
from launch_ros.actions import Node  # noqa: E402


def generate_launch_description():
    ld = LaunchDescription()

    front_cam_params = [
        {"video_device": "/dev/video1"},
        {"framerate": 30.0},
        {"io_method": "mmap"},
        {"frame_id": "front_camera"},
        {"pixel_format": "mjpeg2rgb"},
        {"av_device_format": "YUV422P"},
        {"image_width": 1280},
        {"image_height": 720},
        {"camera_name": "front_cam"},
        {"camera_info_url": "package://lwf_launch/config/front_camera_info.yaml"},
        {"brightness": -1},
        {"contrast": -1},
        {"saturation": -1},
        {"sharpness": -1},
        {"gain": -1},
        {"auto_white_balance": True},
        {"white_balance": 4000},
        {"autoexposure": True},
        {"exposure": 100},
        {"autofocus": False},
        {"focus": -1},
        ]
    rear_cam_params = [
        {"video_device": "/dev/video0"},
        {"framerate": 30.0},
        {"io_method": "mmap"},
        {"frame_id": "rear_camera"},
        {"pixel_format": "mjpeg2rgb"},
        {"av_device_format": "YUV422P"},
        {"image_width": 1280},
        {"image_height": 720},
        {"camera_name": "front_cam"},
        {"camera_info_url": "package://lwf_launch/config/rear_camera_info.yaml"},
        {"brightness": -1},
        {"contrast": -1},
        {"saturation": -1},
        {"sharpness": -1},
        {"gain": -1},
        {"auto_white_balance": True},
        {"white_balance": 4000},
        {"autoexposure": True},
        {"exposure": 100},
        {"autofocus": False},
        {"focus": -1},
        ]
    cam_remappings = []
    camera_nodes = [
        Node(
                package='usb_cam', executable='usb_cam_node_exe', output='screen',
                name="front_cam_node",
                namespace="front_cam",
                parameters=front_cam_params,
                remappings=cam_remappings
                ),
        Node(
                package='usb_cam', executable='usb_cam_node_exe', output='screen',
                name="rear_cam_node",
                namespace="rear_cam",
                parameters=rear_cam_params,
                remappings=cam_remappings
                )
        ]

    camera_group = GroupAction(camera_nodes)

    ld.add_action(camera_group)
    return ld
