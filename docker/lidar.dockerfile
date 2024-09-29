FROM robo-base:latest

RUN source /opt/ros/humble/setup.bash \
    && apt install -y \
    ros-humble-rplidar


CMD source /opt/ros/humble/setup.bash && roslaunch rplidar_ros rplidar.launch
