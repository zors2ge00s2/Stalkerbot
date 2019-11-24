#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from stalkerbot.msg import filtered_transform
from fiducial_msgs.msg import FiducialTransform
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf_conversions
import tf2_ros
import math 

def cb(msg):
    tf = msg.transform
    
    trans = tfBuffer.lookup_transform('base_scan', 'target', rospy.Time())

    global client
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    # x = trans.transform.translation.x
    # if x > 0:
    #     x +0.25
    # if x < 0:
    #     x - 0.25
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = trans.transform.translation.x 
    goal.target_pose.pose.position.y = trans.transform.translation.y 
    goal.target_pose.pose.position.z = trans.transform.translation.z 
    # orientation_q = trans.transform.rotation
    # z = orientation_q.w
    # w = orientation_q.z * -1
    # orientation_q.z = z
    # orientation_q.w = w

    #orientation still wrong 
    # goal.target_pose.pose.orientation.x = orientation_q.x
    # goal.target_pose.pose.orientation.y = orientation_q.y
    # goal.target_pose.pose.orientation.z = orientation_q.z
    # goal.target_pose.pose.orientation.w = orientation_q.w
    #print "x: ", q[0], "y: ", q[1], "z: ", q[2], "w: ", q[3]

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
