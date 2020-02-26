#!/usr/bin/env python

import time
import rospy
from ctrl_pkg.msg import ServoCtrlMsg

# this function applies the input to the 
# AWS DeepRacer 
# angle should be within -0.9 to 0.9
# throttle should be within -0.9 to 0.9
def apply_actions(pub, angle, throttle):
    msg = ServoCtrlMsg()
    msg.angle    = float(angle)
    msg.throttle = float(throttle)
    pub.publish(msg)
    rospy.loginfo('applied angle: %f ; throttle: %f' , angle, throttle)

def action_proxy():
    rospy.init_node('avdr_actionproxy', anonymous=False)
    publisher = rospy.Publisher('manual_drive', ServoCtrlMsg, queue_size=10)
    rospy.loginfo('waiting for a second to give th epublisher time to initialize.')
    rospy.sleep(1)

    apply_actions(publisher, 1.0, 1.0)
    rospy.spin()

if __name__ == '__main__':
    print('started')
    try:
        action_proxy()
    except rospy.ROSInterruptException:
        pass
