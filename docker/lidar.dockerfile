FROM robo-base:latest
#https://index.ros.org/p/rplidar_ros/

RUN source /opt/ros/humble/setup.bash \
    && apt install -y \
    ros-humble-rplidar \
    && sudo chmod 777 /dev/ttyUSB0


CMD source /opt/ros/humble/setup.bash && roslaunch rplidar_ros rplidar.launch
