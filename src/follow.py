#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from stalkerbot.msg import location_info
from fiducial_msgs.msg import FiducialTransformArray

fiducial_id = 101
def location_cb(msg):
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    #Limits the output of recognition to chosen fiducial ID
    if msg.fiducial_id == fiducial_id:
        linear_vel = msg.z_translation * 0.15
        rotational_vel = msg.x_translation/msg.z_translation
        print "x: ", linear_vel
        print "z: ", rotational_vel
        if linear_vel > 0.2:
            linear_vel = 0.2
            vel_msg.linear.x = linear_vel
            vel_msg.angular.z = rotational_vel
    cmd_vel.publish(vel_msg)



rospy.init_node('fiducial_print')
cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
sub = rospy.Subscriber('location', location_info, location_cb, queue_size = 1)
# sub = rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, fiducial_cb, queue_size = 1)
rospy.spin()