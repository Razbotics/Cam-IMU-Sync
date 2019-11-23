# Cam-IMU-Sync
This ROS repository contains packages for accurate timestamp syncing of MPU9250 IMU with Arducam MT9V034 global shutter camera image frames. Time stamping is done using Arduino by triggering the camera and sending the trigger time to the host computer.  This repository is use for running Visual Inertial Odometry algorithms.

Run command: 
roslaunch arducam_usb2_ros trigger_capture_node.launch
