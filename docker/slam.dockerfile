FROM lwf_base:latest

RUN apt install -y \
    ros-humble-rtabmap \
    ros-humble-rtabmap-ros \
    ros-humble-spatio-temporal-voxel-layer

ENTRYPOINT ["/bin/bash", "-c", "/ros_entrypoint.sh && source install/setup.bash && ros2 launch lwf_launch lwf_slam.launch.py"]
