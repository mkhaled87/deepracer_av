#!/bin/bash

source /opt/aws/deepracer/setup.sh
source ~/catkin_ws/devel/setup.sh

cd scripts
sudo chmod +rwx *.py

cd ../../..
catkin_make
catkin_make install
