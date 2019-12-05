# Cam-IMU-Sync
This ROS repository contains packages for accurate timestamp syncing of MPU9250 IMU with Arducam MT9V034 global shutter camera image frames. Time stamping is done using Arduino by triggering the camera and sending the trigger time to the host computer. Also, it contains modified driver to run mavros PX4 IMU camera sync API. This repository is exclusive use for running Visual Inertial Odometry algorithms.

Run command:   
`roslaunch arducam_usb2_ros arducam_trigger_sync_mpu.launch` ## For camera sync with MPU9250  
`roslaunch arducam_usb2_ros arducam_trigger_sync_px4.launch` ## For camera sync with mavros PX4 API


