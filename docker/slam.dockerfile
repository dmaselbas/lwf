FROM 192.168.5.239:5000/robo_base:latest


CMD  ros2 launch rtabmap_ros rtabmap.launch.py localization:=true frame_id:=rgbd_camera odom_frame_id:=odom rgbd_sync:=true approx_sync:=true
