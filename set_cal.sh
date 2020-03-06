#!/bin/bash

source /opt/aws/deepracer/setup.sh
source ~/catkin_ws/devel/setup.sh

# calibrating
# rosservice call /set_car_cal calType max mid min polarity
rosservice call /set_car_cal 0 1830000 1540000 1290000 1
rosservice call /set_car_cal 1 1603500 1423500 1293000 -1

# reset to 0
rostopic pub -1 /manual_drive ctrl_pkg/ServoCtrlMsg -- 0.9 0.9