FROM lwf_base

RUN apt install ros-humble-foxglove-bridge ros-humble-foxglove-compressed-video-transport -y
ENTRYPOINT ["/bin/bash", "-c", "/ros_entrypoint.sh && source install/setup.bash && ros2 launch foxglove_bridge foxglove_bridge_launch.xml"]
