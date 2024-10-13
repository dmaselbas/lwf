FROM 192.168.5.239:5000/lwf:latest


CMD  ros2 launch lwf_launch rtabmap_vio.launch.py
