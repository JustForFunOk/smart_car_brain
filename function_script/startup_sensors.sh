#!/bin/bash

TERMINAL=gnome-terminal

# realsense d435
${TERMINAL} --title "camera" -- bash -c "roslaunch realsense2_camera rs_camera.launch; " --maximize 
