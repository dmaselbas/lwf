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
    parameters = [{
        'frame_id':            'base_link',
        'subscribe_depth':     True,
        'subscribe_scan':      True,
        'subscribe_odom_info': True,
        'approx_sync':         False,
        'wait_imu_to_init':    False,
        'queue_size':          10,
        'pub_rate':            1,
        'qos':                 2,
        'rviz':                True,
        'visual_odometry':     False,
        'sync_queue_size':     20,
        'topic_queue_size':    10
        }]

    remappings = [
        ('/imu', '/imu/data'),
        ('/rgb/image', '/camera/color/image_raw'),
        ('/rgb/camera_info', '/camera/color/camera_info'),
        ('/gps/fix', '/fix'),
        ('/depth/image', '/camera/aligned_depth_to_color/image_raw'),
        ('/depth/camera_info', '/camera/aligned_depth_to_color/camera_info')]

    return LaunchDescription([

        # Make sure IR emitter is enabled
        SetParameter(name='depth_module.emitter_enabled', value=1),

        # Launch camera driver
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('realsense2_camera'), 'launch'),
                    '/rs_launch.py']),
                launch_arguments={'camera_namespace':   '',
                                  'align_depth.enable': 'true',
                                  'enable_sync':        'true',
                                  'enable_depth':       'true',
                                  'enable_color':       'true',
                                  'spatial_filter.enable':       'true',
                                  'temporal_filter.enable':       'true',
                                  'decimation_filter.enable':       'true',
                                  'initial_reset':      'true',
                                  }.items(),
                ),

        # Node(
        #         package='rtabmap_odom', executable='rgbd_odometry', output='screen',
        #         parameters=parameters,
        #         remappings=remappings),
        #
        # Node(
        #         package='rtabmap_slam', executable='rtabmap', output='screen',
        #         parameters=parameters,
        #         remappings=remappings,
        #         arguments=['-d']),
        #
        # Node(
        #         package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        #         parameters=parameters,
        #         remappings=remappings),
        ])
