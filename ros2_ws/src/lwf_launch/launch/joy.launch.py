import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(get_package_share_directory("lwf_launch"), "config", "joy_teleop.yaml")
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': '/dev/input/js0',  # Path to the joystick device
                'deadzone': 0.1,
                'autorepeat_rate': 20.0
            }]
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            output='screen',
            parameters=[config_file]  # Full path to your YAML file
        )
    ])
