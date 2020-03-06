#!/usr/bin/env python

import os
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

video_files = [
    "lanefollow_no_stoplines.mp4",
    "lanefollow_nostop.mp4",
    "lanefollow_stoplines.mp4",
    "lanefollow_zigzag.mp4"
]
videos = []
current_video_file_idx = -1


def get_media_file(v_file):
    media_path = os.path.dirname(os.path.realpath(__file__)) + os.path.sep
    media_path += ".." + os.path.sep + "media"
    media_path += os.path.sep + "mp4" + os.path.sep
    return media_path + v_file


def load_videos():
    global videos
    for v_file in video_files:
        video_file = get_media_file(v_file)
        rospy.loginfo("Loading the media file " + video_file + " ...")
        video = cv2.VideoCapture(video_file)

        if not video.isOpened():
            rospy.loginfo("Failed to open the media file.")
            return False

        videos.append(video)

    return True


def get_next_video():
    global current_video_file_idx
    num_video_files = len(video_files)
    current_video_file_idx += 1

    if current_video_file_idx == num_video_files:
        current_video_file_idx = 0

    return videos[current_video_file_idx]


def feed_generator():
    if not load_videos():
        return

    video = get_next_video()
    fps = video.get(cv2.CAP_PROP_FPS)
    frame_idx = 0
    rospy.loginfo("Starting the emulated camera. We publish to /video_mjpeg.")
    rate = rospy.Rate(fps)
    while(not rospy.is_shutdown()):
        _, image_rgb = video.read()
        frame_idx = frame_idx + 1
        if frame_idx == video.get(cv2.CAP_PROP_FRAME_COUNT):
            rospy.loginfo("Finished playing a media file one time.")
            frame_idx = 0
            video = get_next_video()
            fps = video.get(cv2.CAP_PROP_FPS)
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
            rospy.loginfo(
                "Warning: The published /video_mjpeg is found. " +
                "It looks like you are runnin on the AWS deepracer. " +
                "This will conflect with the camera stream " +
                "(unless disabled manually).")

        imagefeed_pub = rospy.Publisher('video_mjpeg', Image, queue_size=10)
        bridge = CvBridge()
        rospy.init_node('camera_emulator', anonymous=False)
        feed_generator()

    except rospy.ROSInterruptException:
        pass
