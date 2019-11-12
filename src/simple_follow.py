#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from stalkerbot.msg import location_info
from fiducial_msgs.msg import FiducialTransformArray

def location_cb(msg):
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0

    linear_vel = msg.z_translation * 0.15
    rotational_vel = msg.x_translation/msg.z_translation
    if linear_vel > 0.2:
        linear_vel = 0.2
        vel_msg.linear.x = linear_vel
        vel_msg.angular.z = rotational_vel
    print('---')
    print('current linear velocity: %.2f' % vel_msg.linear.x)
    print('current angular velocity: %.2f' % vel_msg.angular.z)
    cmd_vel.publish(vel_msg)


rospy.init_node('follow')
cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
sub = rospy.Subscriber('/stalkerbot/location/marker/rad', location_info, location_cb, queue_size = 1)
rospy.spin()