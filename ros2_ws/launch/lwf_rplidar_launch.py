from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare serial port argument
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',  # Default serial port
            description='Serial port for the LIDAR'
        ),

        Node(
            package='rplidar_ros',  # Use the existing rplidar_ros package
            executable='rplidar_composition',
            name='rplidar',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),  # Pass dynamic serial port
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
        ),
    ])
