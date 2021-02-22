#!/bin/bash

TERMINAL=gnome-terminal

# store rosbag in home directory
cd ~

# filter some compressed image topic which lead rosbag record error
# filter raw rgb image which lead to big rosbag 
rosbag record /camera/color/camera_info /camera/color/image_raw/compressed  /camera/color/image_raw/compressed/parameter_descriptions /camera/color/image_raw/compressed/parameter_updates /camera/depth/camera_info /camera/depth/image_rect_raw /camera/extrinsics/depth_to_color /camera/realsense2_camera_manager/bond /camera/rgb_camera/auto_exposure_roi/parameter_descriptions /camera/rgb_camera/auto_exposure_roi/parameter_updates /camera/rgb_camera/parameter_descriptions /camera/rgb_camera/parameter_updates /camera/stereo_module/auto_exposure_roi/parameter_descriptions /camera/stereo_module/auto_exposure_roi/parameter_updates /camera/stereo_module/parameter_descriptions /camera/stereo_module/parameter_updates /clicked_point /diagnostics /initialpose /move_base_simple/goal /rosout /rosout_agg /tf /tf_static /chassis/chassis_communication/decoded_chassis_signal /chassis/chassis_communication/raw_chassis_signal
