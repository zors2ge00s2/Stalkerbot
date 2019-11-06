#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import math
import tf2_ros
from fiducial_msgs.msg import FiducialTransformArray
br = tf2_ros.TransformBroadcaster()
tfBuffer = tf2_ros.Buffer(rospy.Time(30))
fiducial_id = 101
def fiducial_cb(msg):
    for tf in msg.transforms:
        if tf.fiducial_id == fiducial_id:
            t = TransformStamped()
            t.child_frame_id = "fiducial"
            t.header.frame_id = "burger"
            t.header.stamp = msg.header.stamp
            t.transform.translation.x = tf.transform.translation.x
            t.transform.translation.y = tf.transform.translation.y
            t.transform.translation.z = tf.transform.translation.z
            t.transform.rotation.x = tf.transform.rotation.x
            t.transform.rotation.y = tf.transform.rotation.y
            t.transform.rotation.z = tf.transform.rotation.z
            t.transform.rotation.w = tf.trasform.rotation.w
            br.sendTransform(t)
            tfBuffer.set_transform(t, "follow") 

# def cb(msg):
#     vel_msg = Twist()
#     vel_msg.linear.x = 0
#     vel_msg.angular.z = 0
#     if len(msg.transforms) > 0 :
#         for tf in msg.transforms :
#             if tf.fiducial_id == fiducial_id:
#                     try:
#                         trans = tfBuffer.lookup_transform(turtle_name, "burger", rospy.Time())
#                     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#                         rate.sleep()
#                         continue
#                     x = math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2) -1
#                     z = math.atan2(trans.transform.translation.y, trans.transform.translation.x) -1
#                     if x > 0.2:
#                         x = 0.2
#                     if z > 1:
#                         z = 1
#                     vel_msg.linear.x = x
#                     vel_msg.linear.z = z
#     cmd_vel.publish(vel_msg)



cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
tfBuffer = tf2_ros.Buffer()
rospy.init_node('fiducial_print')
sub = rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, fiducial_cb, queue_size = 1)
rospy.spin()