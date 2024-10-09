FROM 192.168.5.239:5000/lwf:latest


CMD ros2 launch realsense2_camera rs_launch.py name:=rgbd_camera  \
    enable_sync:=true enable_rgbd:=true pointcloud.enable:=true enable_depth:=true \
    enable_infra:=true enable_infra1:=true enable_infra2:=true enable_color:=true \
    enable_accel:=true enable_gyro:=true colorizer.enable:=true \
    align_depth.enable:=true initial_reset:=true base_frame_id:=rgbd_camera

