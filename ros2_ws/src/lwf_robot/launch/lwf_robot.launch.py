import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import launch
import launch_ros

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'lwf_robot.urdf'
    default_model_path = os.path.join(get_package_share_directory('lwf_robot'), urdf_file_name)
    default_rviz_config_path = os.path.join(get_package_share_directory('lwf_robot'), 'urdf_config.rviz')

    with open(default_model_path, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher_node = launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
            )
    launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s',
                                       'libgazebo_ros_factory.so'], output='screen'),
    joint_state_publisher_node = launch_ros.actions.Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            arguments=[default_model_path]
            )
    rviz_node = launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
            )
    spawn_entity = launch_ros.actions.Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'lwf_robot', '-topic', 'robot_description'],
            output='screen'
            )
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                             description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                             description='Absolute path to rviz config file'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        rviz_node
    ])
