#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from stalkerbot.msg import filtered_transform
from fiducial_msgs.msg import FiducialTransform
import tf_conversions
import tf2_ros

def cb(msg):
    tf = msg.transform
    
    trans = tfBuffer.lookup_transform('base_link', 'map', rospy.Time())

    global client
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # goal.target_pose.pose.position.x = tf.transform.translation.x
    # goal.target_pose.pose.position.y = tf.transform.translation.y
    # goal.target_pose.pose.position.z = tf.transform.translation.z
    goal.target_pose.pose.position.x = trans.transform.translation.x + tf.transform.translation.x
    goal.target_pose.pose.position.y = trans.transform.translation.y + tf.transform.translation.y
    goal.target_pose.pose.position.z = trans.transform.translation.z + tf.transform.translation.z

    # goal.target_pose.pose.orientation.x = tf.transform.rotation.x
    # goal.target_pose.pose.orientation.y = tf.transform.rotation.y
    # goal.target_pose.pose.orientation.z = tf.transform.rotation.z
    # goal.target_pose.pose.orientation.w = tf.transform.rotation.w
    goal.target_pose.pose.orientation.w = 1
    client.send_goal(goal)
    print "moving to goal"
    print "x: ", goal.target_pose.pose.position.x, "y: ", goal.target_pose.pose.position.y, "z: ", goal.target_pose.pose.position.z

rospy.init_node('movebase_client')
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
sub = rospy.Subscriber('stalkerbot/fiducial/transform', filtered_transform, cb)
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
client.wait_for_server()
rospy.spin()
