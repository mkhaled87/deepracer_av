#!/bin/bash

cd scripts
sudoo chmod +rwx *.py

cd ../../..
catkin_make
catkin_make install
