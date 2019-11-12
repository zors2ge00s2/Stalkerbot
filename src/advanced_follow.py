#!/usr/bin/env python

import rospy
import yaml
import os
from geometry_msgs.msg import Twist
from std_msgs.msg import Time
from stalkerbot.msg import location_info
from fiducial_msgs.msg import FiducialTransformArray

class Follow():

    '''Sets values for linear and rotational velocity'''
    def location_cb(self, msg):
        self.location = msg

        linear_vel = msg.z_translation * 0.15
        rotational_vel = msg.x_translation/msg.z_translation * self.COEFFICIENT_ROTATIONAL_VELOCITY
        if linear_vel > self.MAXIMUM_LINEAR_VELOCITY:
            linear_vel = self.MAXIMUM_LINEAR_VELOCITY
        self.twist.linear.x = linear_vel
        self.twist.angular.z = rotational_vel

    def interval_cb(self, msg):
        self.interval = msg

    def __init__(self):

        '''Class variables'''
        self.location = None
        self.interval = None
        self.twist = Twist()

        cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        location_subscriber = rospy.Subscriber('/stalkerbot/location/marker/rad', location_info, self.location_cb, queue_size=1)
        interval_subscriber = rospy.Subscriber('/stalkerbot/fiducial/interval', Time, self.interval_cb, queue_size=1)

        '''Read Constants from config'''
        self.DESIRED_DISTANCE = 0
        self.FREQUENCY = 0
        self.MAXIMUM_LINEAR_VELOCITY = 0
        self.COEFFICIENT_ROTATIONAL_VELOCITY = 0
        with open(os.path.dirname(__file__) + '/../config.yaml','r') as file:
            config = yaml.load(file, Loader=yaml.FullLoader)
            self.DESIRED_DISTANCE = config['core']['desired_distance']
            self.FREQUENCY = config['core']['frequency']['cmd_vel']
            self.MAXIMUM_LINEAR_VELOCITY = config['core']['velocity']['linear']['maximum']
            self.COEFFICIENT_ROTATIONAL_VELOCITY = config['core']['velocity']['rotational']['coefficient']

        self.rate = rospy.Rate(self.FREQUENCY)
        while not rospy.is_shutdown():
            if self.location is not None and self.location.z_translation > self.DESIRED_DISTANCE:
                cmd_vel.publish(self.twist)
            else:
                cmd_vel.publish(Twist())
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('advanced_follow')
    try:
        follow = Follow()
    except rospy.ROSInterruptException:  pass