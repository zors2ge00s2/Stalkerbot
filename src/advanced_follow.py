#!/usr/bin/env python

import rospy
import yaml
import os
from geometry_msgs.msg import Twist
from std_msgs.msg import Time
from stalkerbot.msg import location_info
from fiducial_msgs.msg import FiducialTransformArray

class Follow():

    '''Sets values for linear and rotational velocity'''
    def _location_cb(self, msg):
        self._location = msg

        linear_vel = msg.z_translation * 0.12
        rotational_vel = msg.x_translation/msg.z_translation * self._COEFFICIENT_ROTATIONAL_VELOCITY
        if linear_vel > self._MAXIMUM_LINEAR_VELOCITY:
            linear_vel = self._MAXIMUM_LINEAR_VELOCITY
        self._twist.linear.x = linear_vel
        self._twist.angular.z = rotational_vel
        self._warm = msg.is_warm

    '''Update interval upon callback'''
    def _interval_cb(self, msg):
        self._interval = msg

    def __init__(self):

        '''Class variables'''
        self._warm = True
        self._location = None
        self._interval = None
        self._twist = Twist()

        cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        location_subscriber = rospy.Subscriber('/stalkerbot/location/marker/rad', location_info, self._location_cb, queue_size=1)
        interval_subscriber = rospy.Subscriber('/stalkerbot/fiducial/interval', Time, self._interval_cb, queue_size=1)

        '''Class constants,
        read constants from config'''
        self._DESIRED_DISTANCE = 0
        self._FREQUENCY = 0
        self._MAXIMUM_LINEAR_VELOCITY = 0
        self._COEFFICIENT_LINEAR_VELOCITY = 0
        self._COEFFICIENT_ROTATIONAL_VELOCITY = 0
        self._COEFFICIENT_ROTATIONAL_VELOCITY_BUFFER = 0
        self._DETECTION_BUFFER_SEC = 0
        self._MOVEMENT_BUFFER_SEC = 0
        self._MOVEMENT_BUFFER_NANOSEC = 0
        with open(os.path.dirname(__file__) + '/../config.yaml','r') as file:
            config = yaml.load(file, Loader=yaml.FullLoader)
            self._DESIRED_DISTANCE = config['core']['distance']['desired']
            self._FREQUENCY = config['core']['frequency']['cmd_vel']
            self._MAXIMUM_LINEAR_VELOCITY = config['core']['velocity']['linear']['maximum']
            self._COEFFICIENT_LINEAR_VELOCITY = config['core']['velocity']['linear']['coefficient']
            self._COEFFICIENT_ROTATIONAL_VELOCITY = config['core']['velocity']['rotational']['coefficient']
            self._COEFFICIENT_ROTATIONAL_VELOCITY_BUFFER = config['core']['velocity']['rotational']['buffer_coefficient']
            self._DETECTION_BUFFER_SEC = config['core']['buffer']['detection']['sec']
            self._MOVEMENT_BUFFER_SEC = config['core']['buffer']['movement']['sec']
            self._MOVEMENT_BUFFER_NANOSEC = config['core']['buffer']['movement']['nsec']

        self._rate = rospy.Rate(self._FREQUENCY)
        while not rospy.is_shutdown():
            '''if the marker is 'warm' and distance is bigger than desired distance
            AND interval is initiated and does not exceed detection buffer (not detecting marker for too long)'''
            if self._warm \
                and self._location is not None and self._location.z_translation > self._DESIRED_DISTANCE \
                and self._interval is not None and self._interval.data.secs < self._DETECTION_BUFFER_SEC:

                if self._interval.data.secs > self._MOVEMENT_BUFFER_SEC \
                    or (self._interval.data.secs == self._MOVEMENT_BUFFER_SEC and self._interval.data.nsecs < self._MOVEMENT_BUFFER_NANOSEC):
                    self._twist.angular.z *= self._COEFFICIENT_ROTATIONAL_VELOCITY_BUFFER

                cmd_vel.publish(self._twist)

            elif not self._warm:
                # TODO: An action which steers the robot to the correct position and will be interrupted immediately upon detection of another warm marker
                continue

            else:
                cmd_vel.publish(Twist())
            self._rate.sleep()

if __name__ == '__main__':
    rospy.init_node('advanced_follow')
    try:
        follow = Follow()
    except rospy.ROSInterruptException:  pass