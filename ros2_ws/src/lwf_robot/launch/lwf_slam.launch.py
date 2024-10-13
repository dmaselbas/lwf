# Requirements:
#   A realsense D435i
#   Install realsense2 ros2 package (ros-$ROS_DISTRO-realsense2-camera)
# Example:
#   $ ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=1 enable_sync:=true
#
#   $ ros2 launch rtabmap_examples realsense_d435i_color.launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    parameters = [{
        'frame_id':            'rgbd_camera',
        'subscribe_depth':     True,
        'subscribe_odom_info': True,
        'approx_sync':         False,
        'wait_imu_to_init':    True,
        'subscribe_rgb':       True,
        'subscribe_imu':       True,
        }]

    remappings = [
        ('imu', '/imu/data'),
        ('/camera/camera/rgb/image', '/camera/color/image_raw'),
        ('/camera/camera/depth/image_rect_raw', '/camera/depth/image_rect_raw'),
        ('/camera/camera/rgb/camera_info', '/camera/color/camera_info'),
        ('/camera/camera/depth/camera_info', '/camera/color/camera_info'),
        ('/camera/camera/depth/camera_info', '/camera/color/camera_info'),
        ('/camera/camera/rgbd', '/camera/rgbd'),
        ('/camera/camera/aligned_depth/image', '/camera/aligned_depth_to_color/image_raw'),
        ('/rtabmap/map', '/map'),
        ('/rtabmap/odom', '/odom'), ]

    return LaunchDescription([

        # Make sure IR emitter is enabled
        SetParameter(name='depth_module.emitter_enabled', value=1),

        # Launch camera driver
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('realsense2_camera'), 'launch'),
                    '/rs_launch.py']),
                launch_arguments={'enable_gyro':                'true',
                                  'enable_accel':               'true',
                                  'enable_color':               'true',
                                  'enable_depth':               'true',
                                  'unite_imu_method':           '2',
                                  'align_depth.enable':         'true',
                                  'enable_sync':                'true',
                                  'rgb_camera.color_profile':   '640x360x30',
                                  'depth_module.infra_profile': '640x360x30',
                                  'depth_module.depth_profile': '640x360x30',
                                  'colorizer.enable':           'true',
                                  'initial_reset':              'true',
                                  'base_frame_id':              'rgbd_camera'
                                  }.items(),
                ),

        Node(
                package='rtabmap_odom', executable='rgbd_odometry', output='screen',
                parameters=parameters,
                remappings=remappings),

        Node(
                package='rtabmap_slam', executable='rtabmap', output='screen',
                parameters=parameters,
                remappings=remappings,
                arguments=['-d']),

        Node(
                package='rtabmap_viz', executable='rtabmap_viz', output='screen',
                parameters=parameters,
                remappings=remappings),

        # Compute quaternion of the IMU
        Node(
                package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
                parameters=[{'use_mag':     False,
                             'world_frame': 'enu',
                             'publish_tf':  True
                             }],
                remappings=[('imu/data_raw', '/camera/imu')]),

        # The IMU frame is missing in TF tree, add it:
        Node(
                package='tf2_ros', executable='static_transform_publisher', output='screen',
                arguments=['0', '0', '0', '0', '0', '0', 'camera_gyro_optical_frame', 'camera_imu_optical_frame']),
        ])
