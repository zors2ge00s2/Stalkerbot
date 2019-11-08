#!/usr/bin/env python
import rospy
import math
from stalkerbot.msg import location_info
from fiducial_msgs.msg import FiducialTransformArray
fiducial_id = 101
def fiducial_cb(msg):
    for tf in msg.transforms:
        info = location_info()
        info.x_translation = tf.transform.translation.x
        info.y_translation = tf.transform.translation.y
        info.z_translation = tf.transform.translation.z

        # if tf.fiducial_id == fiducial_id:
        #     t = TransformStamped()
        #     t.child_frame_id = "fiducial"
        #     t.header.frame_id = "burger"
        #     t.header.stamp = msg.header.stamp
        #     t.transform.translation.x = tf.transform.translation.x
        #     t.transform.translation.y = tf.transform.translation.y
        #     t.transform.translation.z = tf.transform.translation.z
        #     t.transform.rotation.x = tf.transform.rotation.x
        #     t.transform.rotation.y = tf.transform.rotation.y
        #     t.transform.rotation.z = tf.transform.rotation.z
        #     t.transform.rotation.w = tf.trasform.rotation.w


cmd_vel = rospy.Publisher('location', location_info, queue_size=1)
rospy.init_node('data_process')
sub = rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, fiducial_cb, queue_size = 1)
rospy.spin()