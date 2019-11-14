#!/usr/bin/env python
import rospy
import math
import os
import yaml
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from stalkerbot.msg import location_info
from fiducial_msgs.msg import FiducialTransform

'''
Takes data from the marker and publishes the location and orientation of the target.
Subscribes to /stalkerbot/fiducial/transform, publishes to /stalkerbot/location/marker/rad
Also publishes to /stalkerbot/location/marker/deg for debugging purpose
'''

class Location_publisher():

    def fiducial_cb(self, msg):
        info_rad = location_info()
        info_rad.fiducial_id = msg.fiducial_id
        info_rad.x_translation = msg.transform.translation.x
        info_rad.y_translation = msg.transform.translation.y
        info_rad.z_translation = msg.transform.translation.z

        orientation_q = msg.transform.rotation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        info_rad.x_orientation = roll * self.orientation_x_coefficient
        info_rad.y_orientation = pitch * self.orientation_y_coefficient
        info_rad.z_orientation = yaw * self.orientation_z_coefficient

        info_deg = to_deg(info_rad)
        self.location_pub_rad.publish(info_rad)
        self.location_pub_deg.publish(info_deg)

    def __init__(self):
        self.orientation_x_coefficient = 0
        self.orientation_y_coefficient = 0
        self.orientation_z_coefficient = 0

        with open(os.path.dirname(__file__) + '/../config.yaml','r') as file:
            config = yaml.load(file, Loader=yaml.FullLoader)
            self.orientation_x_coefficient = config['robot']['camera']['pose']['orientation']['x']['coefficient']
            self.orientation_y_coefficient = config['robot']['camera']['pose']['orientation']['y']['coefficient']
            self.orientation_z_coefficient = config['robot']['camera']['pose']['orientation']['z']['coefficient']

        self.location_pub_rad = rospy.Publisher('/stalkerbot/location/marker/rad', location_info, queue_size=1)
        self.location_pub_deg = rospy.Publisher('/stalkerbot/location/marker/deg', location_info, queue_size=1)
        rospy.init_node('location_pub')
        sub = rospy.Subscriber('/stalkerbot/fiducial/transform', FiducialTransform, self.fiducial_cb, queue_size = 1)
        rospy.spin()

'''Convert type location_info from radians to degrees'''
def to_deg(info_rad):
    info_deg = location_info()
    info_deg.fiducial_id = info_rad.fiducial_id
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