#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo

def callback(msg):
    pub.publish(msg)
rospy.init_node('aruco_sub')
pub = rospy.Publisher('/camera_info', CameraInfo, queue_size=1)
sub = rospy.Subscriber('/raspicam_node/camera_info', CameraInfo, callback, queue_size=1)
rospy.spin()