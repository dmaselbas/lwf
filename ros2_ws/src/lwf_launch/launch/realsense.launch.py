# Requirements:
#   A realsense D435i
#   Install realsense2 ros2 package (ros-$ROS_DISTRO-realsense2-camera)
# Example:
#   $ ros2 launch rtabmap_examples realsense_d435i_color.launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    return LaunchDescription([

        # Make sure IR emitter is enabled
        SetParameter(name='depth_module.emitter_enabled', value=1),

        # Launch camera driver
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('realsense2_camera'), 'launch'),
                    '/rs_launch.py']),
                launch_arguments={'camera_namespace':         '',
                                  # 'depth_camera.profile': '640x360x30',
                                  # 'rgb_camera.profile':   '640x360x30',
                                  'align_depth.enable':       'true',
                                  # 'enable_sync':              'true',
                                  # 'enable_depth':             'true',
                                  # 'enable_color':             'true',
                                  # 'initial_reset':            'true',
                                  }.items(),
                ),

        ])
