from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    parameters = [{
        'frame_id':            'base_link',
        'subscribe_depth':     True,
        'subscribe_scan':      True,
        'subscribe_odom_info': False,
        'database_path':       '/opt/maps/rtabmap.db',
        'approx_sync':         True
        }]

    remappings = [
        ('rgb/image', '/camera/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/camera/color/camera_info'),
        ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw')]

    return LaunchDescription([

        Node(
                package='rtabmap_odom', executable='rgbd_odometry', output='screen',
                parameters=parameters,
                remappings=remappings),

        Node(
                package='rtabmap_slam', executable='rtabmap', output='screen',
                parameters=parameters,
                remappings=remappings,
                arguments=['-d']),
        ])
