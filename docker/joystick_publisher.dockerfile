FROM robo-base:latest


# https://index.ros.org/p/teleop_twist_joy/
CMD ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
