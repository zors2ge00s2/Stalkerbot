#!/usr/bin/env python
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from stalkerbot.msg import location_info
from fiducial_msgs.msg import FiducialTransformArray
fiducial_id = 100
def fiducial_cb(msg):
    for tf in msg.transforms:
        info = location_info()
        info.fiducial_id = tf.fiducial_id
        info.x_translation = tf.transform.translation.x
        info.y_translation = tf.transform.translation.y
        info.z_translation = tf.transform.translation.z

        orientation_q = tf.transform.rotation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        info.x_orientation = roll
        info.y_orientation = pitch
        info.z_orientation = yaw
        location_pub.publish(info)

location_pub = rospy.Publisher('location', location_info, queue_size=1)
rospy.init_node('data_process')
sub = rospy.Subscriber('fiducial_transforms', FiducialTransformArray, fiducial_cb, queue_size = 1)
rospy.spin()