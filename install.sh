#!/bin/bash

cd scripts
sudo chmod +rwx *.py

cd ../../..
catkin_make
catkin_make install
