#!/usr/bin/env python

import rospy
import yaml
import os.path
from fiducial_msgs.msg import FiducialTransformArray
from fiducial_msgs.msg import FiducialTransform

'''
Filters out undesired fiducial markers specified in the configuration file.
Subscribes to /fiducial_transforms, Publishes to /stalkerbot/fiducial/transform
'''

target_fiducials = None

with open(os.path.dirname(__file__) + '/../config.yaml','r') as file:
    config = yaml.load(file, Loader=yaml.FullLoader)
    target_fiducials = config['core']['target_fiducials']


def fiducials_cb(msg):
    for fiducial in msg.transforms:
        if fiducial.fiducial_id in target_fiducials:
            fiducial_publisher.publish(fiducial)

rospy.init_node('fiducial_filter')
sub = rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, fiducials_cb, queue_size=1)
fiducial_publisher = rospy.Publisher('/stalkerbot/fiducial/transform', FiducialTransform, queue_size=1)
rospy.spin()