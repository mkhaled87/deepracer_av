import os
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

def get_media_file():
    return "lanefollow_no_stoplines.mov"


def wait_for_subscribers()
    while imagefeed_pub.get_num_connections() < 1:
        # wait for a connection to publisher
        rospy.loginfo("Waiting for connection to publisher...")
        time.sleep(1)

    rospy.loginfo("Connected to publisher.")
    return

def feed_generator():
    rospy.loginfo("Loading the media file.")
    video_file = "media" + os.path.sep + get_media_file()
    video = cv2.VideoCapture(video_file)
    fps = video.get(cv2.CAP_PROP_FPS)
    frame_idx = 0
    rospy.loginfo("Starting the emulated camera stream and we publish to /video_mjpeg.")
    rate = rospy.Rate(fps)
    while(True):
        _, image_rgb = video.read()
        frame_idx = frame_idx + 1 
        if frame_idx == video.get(cv2.CAP_PROP_FRAME_COUNT):
            frame_idx = 0 #Or whatever as long as it is the same as next line
            video.set(cv2.CAP_PROP_POS_FRAMES, 0)

        image_message = bridge.cv2_to_imgmsg(image_rgb, "bgr8")
        rate.sleep()
        imagefeed_pub.publish(image_message)


if __name__ == '__main__':
    try:
        imagefeed_pub = rospy.Publisher('video_mjpeg', Image, queue_size=10)
        bridge = CvBridge()
        rospy.init_node('camera_emulator', anonymous=False)
        feed_generator()
        
    except rospy.ROSInterruptException:
        pass    