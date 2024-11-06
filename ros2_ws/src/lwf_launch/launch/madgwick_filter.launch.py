import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(get_package_share_directory("lwf_launch"), "config", "madgwick_filter_config.yaml")
    return LaunchDescription([
        Node(
                package='imu_filter_madgwick',
                executable='imu_filter_madgwick_node',
                name='imu_filter_madgwick_node',
                output='screen',
                remappings=[
                    ('/imu/data', '/imu/data_raw'),
                    ('/imu/clean_data', '/filtered_imu/data')],
                parameters=[config_file]  # Replace with your config path
                )
        ])
