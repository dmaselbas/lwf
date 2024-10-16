FROM 192.168.5.239:5000/lwf_arm_base:latest

#https://index.ros.org/r/robot_state_publisher/github-ros-robot_state_publisher/
#https://index.ros.org/p/robot_state_publisher
ENTRYPOINT ["/bin/bash", "-c", "/ros_entrypoint.sh && source install/setup.bash && launch lwf_robot lwf_robot.launch.py gui:=false"]
