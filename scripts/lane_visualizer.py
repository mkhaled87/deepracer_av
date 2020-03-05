#!/usr/bin/env python

import numpy
import cv2
import rospy
from threading import Lock
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from deepracer_av.msg import RoadLaneInfo
from deepracer_av.msg import ControlInfo

# from lane detector (perception)
img_mutex = Lock()
lanes_mutex = Lock()
image_detected = False
lanes_detected = False
source_image = []
two_lanes = []
OldLanes = False

# from lane keeper (control)
has_control = True
control_mutex = Lock()
center_line = []
error_angle = 0
error_crosstrack = 0
control_value = 0


# put text on image
def put_text(image, text, pos_x, pos_y, color):
    font = cv2.FONT_HERSHEY_SIMPLEX
    org = (pos_x, pos_y)
    fontScale = 0.7
    # Line thickness of 2 px
    thickness = 1

    image = cv2.putText(image, text, org, font, fontScale, color, thickness, cv2.LINE_AA)

    return image

# plots lines on the image with some color
def draw_lines(image, lines, color):
    lane_image = numpy.zeros_like(image)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            cv2.line(lane_image, (int(x1), int(y1)), (int(x2), int(y2)), color, 5)

    return cv2.addWeighted(image, 0.8, lane_image, 1.0, 1.0)


def source_img_callback(data):
    global source_image
    global image_detected
    global img_mutex
    img_mutex.acquire()
    source_image = bridge.imgmsg_to_cv2(data, "bgr8")
    image_detected = True
    img_mutex.release()
    return


def control_callback(data):
    global has_control
    global control_mutex
    global center_line
    global error_angle
    global error_crosstrack
    global control_value

    control_mutex.acquire()

    center_line = data.lane_center_line
    error_angle = data.error_angle
    error_crosstrack = data.error_crosstrack
    control_value = data.pid_control
    has_control = True

    control_mutex.release()
    return


def detected_lanes_callback(data):
    global two_lanes
    global OldLanes
    global lanes_detected
    global lanes_mutex
    lanes_mutex.acquire()

    OldLanes = False

    if data.found_right_border:
        right_lane = data.line_right_border
    else:
        OldLanes = True
        right_lane = data.line_latest_valid_right_border

    if data.found_left_border:
        left_lane = data.line_left_border
    else:
        OldLanes = True
        left_lane = data.line_latest_valid_left_border

    two_lanes = numpy.array([left_lane, right_lane])

    lanes_detected = True
    lanes_mutex.release()

    return


def visualize_lanes():
    global OldLanes
    global image_detected
    global lanes_detected
    global img_mutex
    global lanes_mutex

    global has_control
    global control_mutex
    global center_line
    global error_angle
    global error_crosstrack
    global control_value

    while(not rospy.is_shutdown()):
        needs_refresh = False
        image_combined = source_image

        if image_detected and lanes_detected:
            image_detected = False
            lanes_detected = False

            img_mutex.acquire()
            lanes_mutex.acquire()

            if OldLanes:
                image_combined = draw_lines(image_combined, two_lanes, (0, 0, 255))
            else:
                image_combined = draw_lines(image_combined, two_lanes, (0, 255, 0))

            img_mutex.release()
            lanes_mutex.release()

            needs_refresh = True

        if has_control:
            control_mutex.acquire()

            # image center
            image_center = numpy.array([640/2, 480, 640/2, 0])
            image_combined = draw_lines(image_combined, numpy.array([image_center]), (128, 128, 128))

            # center line
            image_combined = draw_lines(image_combined, numpy.array([center_line]), (255, 0, 0))

            # errors from center line to image_center
            error_line = numpy.array([center_line[0], 480, 640/2, 480])
            image_combined = draw_lines(image_combined, numpy.array([error_line]), (0, 0, 255))

            # write error values
            image_combined = put_text(image_combined, "Cont: %+06.2f" % control_value,
            int(640/2)+19, 480-70, (255, 0, 0))
            image_combined = put_text(image_combined, "E_ct: %+06.2f" % error_crosstrack,
            int(640/2)+19, 480-40, (0, 0, 255))
            image_combined = put_text(image_combined, "E_ang: %+06.2f" % error_angle,
            int(640/2), 480-10, (0, 0, 255))

            has_control = False
            control_mutex.release()

            needs_refresh = True

        if needs_refresh:
            cv2.imshow('laneviz', image_combined)
            cv2.waitKey(1)



def rospy_has_topic(topic):
    topics = rospy.get_published_topics()
    for t in topics:
        if(topic in t):
            return True

    return False


if __name__ == '__main__':
    try:
        if(rospy_has_topic('/video_mjpeg') and rospy_has_topic('/road_lanes')):
            image_sub = rospy.Subscriber("video_mjpeg", Image, source_img_callback)
            lanes_sub = rospy.Subscriber("road_lanes", RoadLaneInfo, detected_lanes_callback)
            control_sub = rospy.Subscriber("av_control", ControlInfo, control_callback)
            bridge = CvBridge()
            rospy.init_node('lane_visualizer', anonymous=False)
            rospy.loginfo("Started the lane visualizer")
            visualize_lanes()
        
        else:
            rospy.loginfo("The topics /video_mjpeg and/or /road_lanes \
                are not available. Please launch the corresponding \
                nodes first.")

    except rospy.ROSInterruptException:
        pass
