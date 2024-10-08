FROM 192.168.5.239:5000/robo_base:latest

RUN . /opt/ros/humble/setup.sh \
    && apt install -y \
    ros-humble-librealsense2* \
    ros-humble-realsense2*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0
CMD  ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true enable_gyro:=true enable_accel:=true unite_imu_method:=2
