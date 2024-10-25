FROM  lwf_base

ENTRYPOINT ["/bin/bash", "-c", "/ros_entrypoint.sh && source install/setup.bash && ros2 run lwf_imu imu_publisher"]
