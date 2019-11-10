#!/usr/bin/env python
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from stalkerbot.msg import location_info
from fiducial_msgs.msg import FiducialTransform

'''
Takes data from the marker and publishes the location and orientation of the target.
Subscribes to /stalkerbot/fiducial/transform, publishes to /stalkerbot/location/marker
'''

def fiducial_cb(msg):
    info = location_info()
    info.fiducial_id = msg.fiducial_id
    info.x_translation = msg.transform.translation.x
    info.y_translation = msg.transform.translation.y
    info.z_translation = msg.transform.translation.z

    orientation_q = msg.transform.rotation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    info.x_orientation = roll
    info.y_orientation = pitch
    info.z_orientation = yaw
    location_pub.publish(info)

location_pub = rospy.Publisher('/stalkerbot/location/marker', location_info, queue_size=1)
rospy.init_node('location_pub')
sub = rospy.Subscriber('/stalkerbot/fiducial/transform', FiducialTransform, fiducial_cb, queue_size = 1)
rospy.spin()