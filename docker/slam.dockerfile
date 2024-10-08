FROM 192.168.5.239:5000/robo_base:latest
RUN . /opt/ros/humble/setup.sh && ros2 launch lwf_robot lwf_slam.launch.py
