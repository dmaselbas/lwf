FROM 192.168.5.239:5000/lwf_arm_base

#https://index.ros.org/p/camera_ros/
#https://index.ros.org/r/image_pipeline/

ENTRYPOINT ["/bin/bash", "-c", "/ros_entrypoint.sh && source install/setup.bash && ros2 launch lwf_launch usb_cam.launch.py"]
