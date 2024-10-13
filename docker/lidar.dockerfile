FROM  192.168.5.239:5000/lwf_base:latest

ENTRYPOINT ["/bin/bash", "-c", "/ros_entrypoint.sh && source install/setup.bash && ros2 launch lwf_launch lidar.launch.py"]
