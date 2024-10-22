FROM lwf_base:latest

RUN apt install -y \
    va-driver-all \
    libva-glx2 \
    libva2 \
    libdrm2 \
    vainfo \
    ros-humble-spatio-temporal-voxel-layer

#ENTRYPOINT ["/bin/bash", "-c", "/ros_entrypoint.sh && source install/setup.bash && ros2 launch lwf_launch lwf_slam.launch.py"]
ENTRYPOINT ["/bin/bash", "-c", "/ros_entrypoint.sh && source install/setup.bash && ros2 launch lwf_launch dcam.launch.py frame_id:=rgbd_camera_frame"]
