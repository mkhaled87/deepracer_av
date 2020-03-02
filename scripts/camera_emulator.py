#!/usr/bin/env python

import os
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

def get_media_file():
    return "media" + os.path.sep + "mp4" + os.path.sep + "lanefollow_no_stoplines.mp4"

def feed_generator():
    video_file = get_media_file()
    rospy.loginfo("Loading the media file " + video_file + " ...")
    video = cv2.VideoCapture(video_file)
    fps = video.get(cv2.CAP_PROP_FPS)
    frame_idx = 0
    rospy.loginfo("Starting the emulated camera stream and we publish to /video_mjpeg.")
    rate = rospy.Rate(fps)
    while(not rospy.is_shutdown()):
        _, image_rgb = video.read()
        frame_idx = frame_idx + 1 
        if frame_idx == video.get(cv2.CAP_PROP_FRAME_COUNT):
            rospy.loginfo("Finished playing the media file one time.")
            frame_idx = 0 #Or whatever as long as it is the same as next line
            video.set(cv2.CAP_PROP_POS_FRAMES, 0)

        image_message = bridge.cv2_to_imgmsg(image_rgb, "bgr8")
        rate.sleep()
        imagefeed_pub.publish(image_message)

def rospy_has_topic(topic):
    topics = rospy.get_published_topics()
    for t in topics:
        if(topic in t):
            return True

    return False

if __name__ == '__main__':
    try:
        if(rospy_has_topic('/video_mjpeg')):
            rospy.loginfo("Warning: The published /video_mjpeg is found. It looks like you are runnin on the AWS deepracer. This will conflect with the camera stream (unless disabled manually).")

        imagefeed_pub = rospy.Publisher('video_mjpeg', Image, queue_size=10)
        bridge = CvBridge()
        rospy.init_node('camera_emulator', anonymous=False)
        feed_generator()
        
    except rospy.ROSInterruptException:
        pass    