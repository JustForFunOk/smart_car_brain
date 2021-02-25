#!/bin/bash

source ~/smart_car/smart_car_brain/devel/setup.bash

# change mode for lidar usb port
sudo chmod 666 /dev/ttyUSB0

# start all from all_in_one.launch
roslaunch startup_entry all_in_one.launch &
