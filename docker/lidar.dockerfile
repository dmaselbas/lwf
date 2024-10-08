FROM 192.168.5.239:5000/robo_base:latest
#https://index.ros.org/p/rplidar_ros/

CMD . /opt/ros/humble/setup.sh && ros2 launch sllidar_ros2 view_sllidar_c1_launch.py
