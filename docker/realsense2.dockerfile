FROM 192.168.5.239:5000/lwf:latest


CMD ros2 launch realsense2_camera rs_launch.py name:=rgbd_camera depth_module.hdr_enabled:=true \
    enable_sync:=true enable_rgbd:=true pointcloud.enable:=true enable_depth:=true \
    enable_accel:=true enable_gyro:=true colorizer.enable:=true
