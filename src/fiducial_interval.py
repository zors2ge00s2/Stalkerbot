#!/usr/bin/env python

import rospy
from fiducial_msgs.msg import FiducialTransform
import time

current_time = time.time()
time_elapsed = 0
'''
Report the duration between current time and when last fiducial marker was recognized.
Subscribes to /stalkerbot/fiducial/transform, publishes to /stalkerbot/fiducial/interval
'''
while True:
    time_elapsed = time.time() - current_time
    time.sleep(0.2)


def fiducial_cb(msg):
    current_time = time.time()


rospy.init_node('fiducial_interval')
sub = rospy.Subscriber('/stalkerbot/fiducial/transform', FiducialTransform, fiducial_cb, queue_size=1)
fiducial_publisher = rospy.Publisher('/stalkerbot/fiducial/interval', rospy.Duration, queue_size=1)
rospy.spin()