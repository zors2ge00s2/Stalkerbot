#!/usr/bin/env python

import rospy
import os
import yaml
from std_msgs.msg import Time
from fiducial_msgs.msg import FiducialTransform

'''
Report the duration between current time and when last fiducial marker was recognized.
Subscribes to /stalkerbot/fiducial/transform, publishes to /stalkerbot/fiducial/interval
'''

rospy.init_node('fiducial_interval')
last_marker_detection_time = None

'''Update detection time upon marker detection'''
def fiducial_cb(msg):
    global last_marker_detection_time
    last_marker_detection_time = rospy.Time.now()

sub = rospy.Subscriber('/stalkerbot/fiducial/transform', FiducialTransform, fiducial_cb, queue_size=1)
interval_publisher = rospy.Publisher('/stalkerbot/fiducial/interval', Time, queue_size=1)

'''Extract rate from config file'''
rate = None
with open(os.path.dirname(__file__) + '/../config.yaml','r') as file:
    config = yaml.load(file, Loader=yaml.FullLoader)
    rate = rospy.Rate(config['core']['frequency']['interval'])

while not rospy.is_shutdown():
    current_time = rospy.Time.now()
    if last_marker_detection_time is not None:
        time_elapsed = current_time - last_marker_detection_time
        interval_publisher.publish(time_elapsed)
    rate.sleep()
    