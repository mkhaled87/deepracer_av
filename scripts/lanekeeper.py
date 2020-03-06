#!/usr/bin/env python

import rospy
import numpy
import math
from simple_pid import PID
from deepracer_av.msg import RoadLaneInfo
from deepracer_av.msg import ControlInfo

cam_emulation = False
try:
    from ctrl_pkg.msg import ServoCtrlMsg
except ImportError:
    print("Warning: failed to import ServoCtrlMsg. it seems we" +
        "are not insidee the AWS DeepRacer.")
    cam_emulation = True

pid = PID(3, 0.1, 0.05, setpoint=0)


# this function applies the input to the AWS DeepRacer
# angle should be within -0.9 to 0.9
# throttle should be within -0.9 to 0.9
def apply_actions(angle, throttle):
    msg = ServoCtrlMsg()
    msg.angle = float(angle)
    msg.throttle = float(throttle)
    md_publisher.publish(msg)
    rospy.loginfo('applied angle: %f ; throttle: %f', angle, throttle)


def compute_control(line_lane_left, line_lane_right):
    both_lines = numpy.array([line_lane_left, line_lane_right])
    center_line = numpy.average(both_lines, axis=0)

    # the center line is our control indicator
    # we need to keep it vertical (angle_error == 0)
    # and we need to keep it in the center (crosstrack_error = 0)
    x_bottom = center_line.reshape(4)[0]
    crosstrack_error = x_bottom - (640/2)
    angle_error = math.degrees(math.atan2(
        center_line[2] - center_line[0],
        center_line[1] - center_line[3]))

    # scaling/thresholding the cross_track error
    # [max value = half-width of the image]
    crosstrack_error = crosstrack_error/(640/2)
    if crosstrack_error > 1.0:
        crosstrack_error = 1.0
    if crosstrack_error < -1.0:
        crosstrack_error = -1.0

    # scaling/thresholding the angle error [max value = 45]
    angle_error = angle_error/45.0
    if angle_error > 1.0:
        angle_error = 1.0
    if angle_error < -1.0:
        angle_error = -1.0

    # combine the two errors with wiights
    w_crosstrack = 0.7
    w_angle = 0.3
    error = w_crosstrack*crosstrack_error + w_angle*angle_error

    control = pid(error)
    return control, angle_error, crosstrack_error, center_line


def control_cb(data):
    right = data.line_latest_valid_right_border
    left = data.line_latest_valid_left_border
    control, angle_error, crosstrack_error, center_line = compute_control(
        left, right)

    # publish on a topic
    ret_msg = ControlInfo()
    ret_msg.lane_center_line = center_line
    ret_msg.error_angle = angle_error
    ret_msg.error_crosstrack = crosstrack_error
    ret_msg.pid_control = control
    control_pub.publish(ret_msg)

    # apply control
    # fixed throttle: 0.4
    if not cam_emulation:
        apply_actions(control, 0.5)


if __name__ == '__main__':
    try:
        control_pub = rospy.Publisher('av_control', ControlInfo, queue_size=10)

        if not cam_emulation:
            md_publisher = rospy.Publisher('manual_drive', ServoCtrlMsg, queue_size=10)

        rospy.init_node('lanekeeper', anonymous=False)
        image_sub = rospy.Subscriber("road_lanes", RoadLaneInfo, control_cb)
        rospy.loginfo("Started the lanekeeper controller node. We wait" +
            "for lane info from /road_lanes and publish control info to" +
            "/av_control for anyone interessted to see it. We also send" +
            "the control signal directly to the /actionproxy node.")

        rospy.spin()
        apply_actions(0.0, 0.0)
    except rospy.ROSInterruptException:
        pass
