#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import math
import os
import tf2_ros
import yaml
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from stalkerbot.msg import location_info
from stalkerbot.msg import filtered_transform


'''
The node broadcasts coordinates of the target with respect to the robot to tf

It also takes data from the marker and publishes the location and orientation of the target.
Subscribes to /stalkerbot/fiducial/transform, publishes to /stalkerbot/location/marker/rad
Also publishes to /stalkerbot/location/marker/deg for debugging purpose


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
        t.header.frame_id = self._FRAME_ROBOT
        t.child_frame_id = self._FRAME_TARGET
        t.transform.translation.x = msg.transform.transform.translation.x
        t.transform.translation.y = msg.transform.transform.translation.y
        t.transform.translation.z = msg.transform.transform.translation.z
        t.transform.rotation.x = msg.transform.transform.rotation.x
        t.transform.rotation.y = msg.transform.transform.rotation.y
        t.transform.rotation.z = msg.transform.transform.rotation.z
        t.transform.rotation.w = msg.transform.transform.rotation.w
        
        # Publish the trandsform.
        br.sendTransform(t)
        self._seq = self._seq + 1
        '''
        publishes the location and orientation of the target
        '''
        info_rad = location_info()
        info_rad.is_warm = msg.is_warm
        info_rad.x_translation = msg.transform.transform.translation.x
        info_rad.y_translation = msg.transform.transform.translation.y
        info_rad.z_translation = msg.transform.transform.translation.z


        '''quaternion to degrees'''
        orientation_q = msg.transform.transform.rotation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        info_rad.x_orientation = roll * self._ORIENTATION_X_COEFFICIENT
        info_rad.y_orientation = pitch * self._ORIENTATION_Y_COEFFICIENT
        info_rad.z_orientation = yaw * self._ORIENTATION_Z_COEFFICIENT

        info_deg = to_deg(info_rad)
        self._location_pub_rad.publish(info_rad)
        self._location_pub_deg.publish(info_deg)

    def __init__(self):

        '''class variable'''
        self._seq = 1

        '''class constants'''
        self._ORIENTATION_X_COEFFICIENT = 0
        self._ORIENTATION_Y_COEFFICIENT = 0
        self._ORIENTATION_Z_COEFFICIENT = 0
        self._FRAME_ROBOT = ''
        self._FRAME_TARGET = ''

        '''load yaml content'''
        with open(os.path.dirname(__file__) + '/../config.yaml','r') as file:
            config = yaml.load(file, Loader=yaml.FullLoader)
            self._ORIENTATION_X_COEFFICIENT = config['robot']['camera']['pose']['orientation']['x']['coefficient']
            self._ORIENTATION_Y_COEFFICIENT = config['robot']['camera']['pose']['orientation']['y']['coefficient']
            self._ORIENTATION_Z_COEFFICIENT = config['robot']['camera']['pose']['orientation']['z']['coefficient']
            self._FRAME_ROBOT = config['tf']['frame_name']['robot']
            self._FRAME_TARGET = config['tf']['frame_name']['target']

        self._location_pub_rad = rospy.Publisher('/stalkerbot/location/marker/rad', location_info, queue_size=1)
        self._location_pub_deg = rospy.Publisher('/stalkerbot/location/marker/deg', location_info, queue_size=1)
        rospy.init_node('location_pub')
        sub = rospy.Subscriber('/stalkerbot/fiducial/transform', filtered_transform, self._fiducial_cb, queue_size = 1)
        rospy.spin()

'''Convert type location_info from radians to degrees'''
def to_deg(info_rad):
    info_deg = location_info()
    info_deg.is_warm = info_rad.is_warm
    info_deg.x_translation = info_rad.x_translation
    info_deg.y_translation = info_rad.y_translation
    info_deg.z_translation = info_rad.z_translation
    info_deg.x_orientation = math.degrees(info_rad.x_orientation)
    info_deg.y_orientation = math.degrees(info_rad.y_orientation)
    info_deg.z_orientation = math.degrees(info_rad.z_orientation)

    return info_deg

if __name__ == '__main__':
    rospy.init_node('location_pub')
    try:
        location_publisher = Location_publisher()
    except rospy.ROSInterruptException:  pass