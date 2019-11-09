#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import math
from stalkerbot.msg import location_info
from fiducial_msgs.msg import FiducialTransformArray

fiducial_id = 101
def location_cb(msg):
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    if len(msg.transforms) > 0 :
        for tf in msg.transforms :
            #Limits the output of recognition to chosen fiducial ID
            if tf.fiducial_id == fiducial_id:
                    linear_vel = tf.transform.translation.z * 0.2 
                    rotational_vel = tf.transform.translation.x/tf.transform.translation.z
                    print "x: ", linear_vel
                    print "z: ", rotational_vel
                    if linear_vel > 0.2:
                        linear_vel = 0.2
                    vel_msg.linear.x = linear_vel
                    vel_msg.angular.z = rotational_vel
    cmd_vel.publish(vel_msg)



cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
rospy.init_node('fiducial_print')
sub = rospy.Subscriber('location', location_info, location_cb, queue_size = 1)
# sub = rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, fiducial_cb, queue_size = 1)
rospy.spin()