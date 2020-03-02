#!/usr/bin/env python

import os
import numpy
import cv2
import math
import rospy
from threading import Lock
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from deepracer_av.msg import RoadLaneInfo

img_mutex = Lock()
lanes_mutex = Lock()
image_detected = False
lanes_detected = False


# plots lines on the image with some color
def draw_lines(image, lines, color):
    lane_image = numpy.zeros_like(image)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            cv2.line(lane_image, (int(x1), int(y1)), (int(x2),int(y2)), color, 5)

    return cv2.addWeighted(image, 0.8, lane_image, 1.0, 1.0)


source_image = []
def source_img_callback(data):
    global source_image
    global image_detected
    img_mutex.acquire()
    source_image = bridge.imgmsg_to_cv2(data, "bgr8")
    image_detected = True
    img_mutex.release()
    return

two_lanes = []
def detected_lanes_callback(data):
    global two_lanes
    global lanes_detected
    lanes_mutex.acquire()
    right_lane = data.line_right_border
    left_lane = data.line_left_border = left_lane
    two_lanes = numpy.array([left_lane, right_lane])
    lanes_detected = True
    lanes_mutex.release()
    return 

def visualize_lanes():
    while(not rospy.is_shutdown()):
        if image_detected and lanes_detected:
            image_detected = False
            lanes_detected = False

            img_mutex.acquire()
            lanes_mutex.acquire()

            image_combined = draw_lines(source_image, two_lanes, (0,255,0))
            cv2.imshow('laneviz', image_combined)

            img_mutex.release()
            lanes_mutex.release()

def rospy_has_topic(topic)
    topics = rospy.get_published_topics()[0]
    for t in topics:
        if(topic in t[0]):
            return True
        else:
            return False


if __name__ == '__main__':
    try:
        
        if(rospy_has_topic('/video_mjpeg') and rospy_has_topic('/road_lanes')):
            image_sub = rospy.Subscriber("video_mjpeg", Image, source_img_callback)
            lanes_sub = rospy.Subscriber("road_lanes", RoadLaneInfo, detected_lanes_callback)
            bridge = CvBridge()
            rospy.init_node('lane_visualizer', anonymous=False)
            visualize_lanes()
        
        else:
            rospy.loginfo("The topics /video_mjpeg and/or /road_lanes are not available. Please launch the corresponding nodes first.")

    except rospy.ROSInterruptException:
        pass    