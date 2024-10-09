FROM 192.168.5.239:5000/lwf:latest


# https://index.ros.org/p/teleop_twist_joy/
CMD ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
