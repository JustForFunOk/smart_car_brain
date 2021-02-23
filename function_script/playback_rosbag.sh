#!/bin/bash

# roscore first
roscore &

# sleep for 1 second
sleep 1

# rviz
rviz -d $(rospack find startup_entry)/config/playback.rviz \
    -s $(rospack find startup_entry)/config/rviz_splash.jpg