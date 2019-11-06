#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import math
import tf2_ros
from fiducial_msgs.msg import FiducialTransformArray

fiducial_id = 100
def fiducial_cb(msg):
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    if len(msg.transforms) > 0 :
        for tf in msg.transforms :
            '''
            Limits the output of recognition to fiducial ID 101
            '''
            if tf.fiducial_id == fiducial_id:
                    x = math.sqrt(tf.transform.translation.x ** 2 + tf.transform.translation.y ** 2)
                    z = math.atan2(tf.transform.translation.y, tf.transform.translation.x)
                    if x > 0.2:
                        x = 0.2
                    if z > 1:
                        z = 1
                    if z < -1:
                        z = -1
                    print x
                    print z
                    vel_msg.linear.x = x
                    vel_msg.angular.z = z
    cmd_vel.publish(vel_msg)



cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
tfBuffer = tf2_ros.Buffer()
rospy.init_node('fiducial_print')
sub = rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, fiducial_cb, queue_size = 1)
rospy.spin()