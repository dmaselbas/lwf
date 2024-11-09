import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 90 degrees in radians for Y-axis rotation (pitch)
    rotation_pitch = '1.5708'  # 90 degrees in radians

    static_transforms = [
        ('camera_link', 'camera_color_optical_frame'),
        ('camera_link', 'camera_depth_optical_frame'),
        ('camera_link', 'camera_infra1_optical_frame'),
        ('camera_link', 'camera_infra2_optical_frame'),
        ('camera', 'camera_link')
        ]

    # Create a node for each static transform
    nodes = [
        Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'static_tf_{child_frame.replace("_", "")}',
                output='screen',
                arguments=['0', '0', '0', '0', rotation_pitch, '0', parent_frame, child_frame]
                )
        for parent_frame, child_frame in static_transforms
        ]

    return LaunchDescription(nodes)
