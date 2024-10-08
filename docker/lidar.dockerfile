FROM 192.168.5.239:5000/robo-base:latest
#https://index.ros.org/p/rplidar_ros/

RUN . /opt/ros/humble/setup.sh \
    && apt install -y \
    ros-humble-rplidar \
    && sudo chmod 777 /dev/ttyUSB0


CMD . /opt/ros/humble/setup.sh && roslaunch rplidar_ros rplidar.launch
