FROM 192.168.5.239:5000/lwf:latest


CMD ros2 launch realsense2_camera rs_launch.py name:=rgbd_camera  \
    enable_sync:=true pointcloud.enable:=true enable_depth:=true \
    enable_color:=true depth_module.depth_profile:=640x480x15 \
    depth_module.infra_profile:=640x480x15 rgb_camera.color_profile:=1280x720x15 \
    enable_accel:=true enable_gyro:=true colorizer.enable:=true \
    align_depth.enable:=true initial_reset:=true base_frame_id:=rgbd_camera
