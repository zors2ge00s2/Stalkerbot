#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo

'''
Republishes the data from /raspicam_node/camera_info to /camera_info,
A node which aruco detect listens to
'''
def callback(msg):
    pub.publish(msg)
rospy.init_node('camera_publisher')
pub = rospy.Publisher('/camera_info', CameraInfo, queue_size=1)
sub = rospy.Subscriber('/raspicam_node/camera_info', CameraInfo, callback, queue_size=1)
rospy.spin()
