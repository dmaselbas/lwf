FROM 192.168.5.239:5000/robo_base:latest


CMD  source /opt/ros/humble/setup.sh && \
     ros2 launch rtabmap_ros rtabmap.launch.py frame_id:=rgbd_camera odom_frame_id:=odom rgbd_sync:=true approx_sync:=true
