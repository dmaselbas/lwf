FROM  lwf_base

ENTRYPOINT ["/bin/bash", "-c", "/ros_entrypoint.sh && source install/setup.bash && ros2 launch lwf_launch lidar.launch.py serial_port:=/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B00465V5-if00-port0"]
