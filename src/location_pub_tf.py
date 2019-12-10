#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import os
import tf2_ros
import yaml
import math
from stalkerbot.msg import filtered_transform


'''
The node broadcasts coordinates of the target with respect to the robot to tf
'''

class Location_publisher():

    def _fiducial_cb(self, msg):
        '''
        broadcasts coordinates of the target with respect to the robot to tf
        '''
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.seq = self._seq
        t.header.frame_id = self._FRAME_CAMERA
        t.child_frame_id = self._FRAME_TARGET

        # h = math.sqrt(msg.transform.transform.translation.y ** 2 + msg.transform.transform.translation.z ** 2)
        # theta_1 = math.radians(self._CAMERA_INCLINE)
        # theta_2 = math.atan(msg.transform.transform.translation.y / msg.transform.transform.translation.z)
        # t.transform.translation.x = h * (math.sin(math.radians(90) - theta_1 - theta_2))
        # t.transform.translation.y = msg.transform.transform.translation.x
        # t.transform.translation.z = h * (math.sin(theta_1 + theta_2))

        t.transform.translation.x = msg.transform.transform.translation.z 
        t.transform.translation.y = msg.transform.transform.translation.x
        t.transform.translation.z = msg.transform.transform.translation.y

        t.transform.rotation.x = msg.transform.transform.rotation.x
        t.transform.rotation.y = msg.transform.transform.rotation.y
        t.transform.rotation.z = msg.transform.transform.rotation.z
        t.transform.rotation.w = msg.transform.transform.rotation.w
        print "sending transform ", "x: ", t.transform.translation.x, "y: ", t.transform.translation.y, "z: ", t.transform.translation.z 
        # Publish the transform
        br.sendTransform(t)
        self._seq = self._seq + 1
        
    def __init__(self):

        '''class variable'''
        self._seq = 1

        '''class constants'''
        self._FRAME_CAMERA = ''
        self._FRAME_TARGET = ''
        self._FRAME_ROBOT = ''
        self._CAMERA_INCLINE = 35

        '''load yaml content'''
        with open(os.path.dirname(__file__) + '/../config.yaml','r') as file:
            config = yaml.load(file, Loader=yaml.FullLoader)
            self._FRAME_CAMERA = config['tf']['frame_name']['camera']
            self._FRAME_TARGET = config['tf']['frame_name']['target']
            self._FRAME_ROBOT = config['tf']['frame_name']['robot']

        '''decares tf transform between camera and base of the robot.
        Please refer to the urdf file of turtlebot for more information'''
        br_static = tf2_ros.StaticTransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.seq = 0
        t.header.frame_id = self._FRAME_ROBOT
        t.child_frame_id = self._FRAME_CAMERA

        t.transform.translation.x = 0.05
        t.transform.translation.y = 0
        t.transform.translation.z = 0.1
        
        '''
        Camera angle tilt 35 degrees
        '''
        t.transform.rotation.x = 0
        t.transform.rotation.y = 0.301
        t.transform.rotation.z = 0
        t.transform.rotation.w = 0.954
        br_static.sendTransform(t)

        sub = rospy.Subscriber('/stalkerbot/fiducial/transform', filtered_transform, self._fiducial_cb, queue_size = 1)
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('location_pub_tf')
    try:
        location_publisher = Location_publisher()
    except rospy.ROSInterruptException: 
        pass