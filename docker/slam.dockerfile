FROM 192.168.5.239:5000/lwf_arm_base:latest

RUN apt install -y \
    ros-humble-librealsense2* \
    ros-humble-realsense2* \
    ros-humble-rtabmap \
    ros-humble-rtabmap-ros \
    ros-humble-spatio-temporal-voxel-layer

ENTRYPOINT ["/bin/bash", "-c", "/ros_entrypoint.sh && source install/setup.bash && ros2 launch lwf_launch lwf_slam.launch.py"]
