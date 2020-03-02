#!/usr/bin/env python

import os
import rospy
import numpy
import math
from simple_pid import PID
pid = PID(1, 0.1, 0.05, setpoint=1)


def get_control(line_lane_left, line_lane_right):
    both_lines = numpy.array([line_lane_left, line_lane_right])
    center_line =  numpy.average(both_lines, axis=0)

    # the center line is our control indicator
    # we need to keep it vertical (angle_error == 0)
    angle_error = math.atan2(center_line[2] - center_line[0], center_line[1] - center_line[3])
    angle_error = math.degrees(angle_error)
    if(angle_error <= 2):
        angle_error = 0
    control = pid(angle_error)
    return control


def control_callback():
    if 'myVar' in locals():
    old_time = 
    t_start = time.time()
