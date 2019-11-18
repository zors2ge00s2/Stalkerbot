#!/usr/bin/env python

import rospy
import math
import os
import yaml
import tf2_ros
import copy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Time, Float32

from collections import deque
from statistics import mean

class Detect():

    def _vel_cb(self, msg):
        self._twist = msg

    def _update_transform(self):
        tf = self._tfBuffer.lookup_transform(self._FRAME_TARGET, self._FRAME_ROBOT, rospy.Time())
        if self._current_location is not None:
            self._last_location = self._current_location
        self._current_location = tf

        if self._last_location is not None:
            dist = self._distance()
            self._results.append(dist)
        
    def _distance(self):
        '''compute 3D distance between two points'''
        x1, x2 = self._last_location.transform.translation.x, self._current_location.transform.translation.x
        y1, y2 = self._last_location.transform.translation.y, self._current_location.transform.translation.y
        z1, z2 = self._last_location.transform.translation.z, self._current_location.transform.translation.z
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
        # Uncomment to debug
        # print distance / self._DETECTION_DISTANCE_TRIGGER * 100
        return distance

    def _get_moving_average(self):
        temp = copy.copy(self._results)
        temp.remove(max(temp))
        temp.remove(max(temp))
        temp.remove(min(temp))
        temp.remove(min(temp))
        _mean = mean(temp)
        # Uncomment to debug
        return _mean

    def __init__(self):
        '''class variables'''
        self._interval = None
        self._last_location = None
        self._current_location = None
        self._linger = 0
        self._twist = Twist()

        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)
        motion_publisher = rospy.Publisher('/stalkerbot/motion', Bool, queue_size = 1)
        vel_subscriber = rospy.Subscriber('cmd_vel', Twist, self._vel_cb, queue_size=1)

        '''class constants'''
        self._DETECTION_DISTANCE_TRIGGER_BASE = 0
        self._MAXIMUM_LINEAR_VELOCITY = 0
        self._FREQUENCY = 0
        self._LINGER_MAXIMUM = 0
        self._FRAME_ROBOT = ''
        self._FRAME_TARGET = ''

        with open(os.path.dirname(__file__) + '/../config.yaml','r') as file:
            config = yaml.load(file, Loader=yaml.FullLoader)
            self._DETECTION_DISTANCE_TRIGGER_BASE = config['core']['distance']['detection']['trigger_base']
            self._FREQUENCY = config['core']['frequency']['motion_detect']
            self._LINGER_MAXIMUM = config['core']['frequency']['motion_detect_linger']
            self._MAXIMUM_LINEAR_VELOCITY = config['core']['velocity']['linear']['maximum']
            self._FRAME_ROBOT = config['tf']['frame_name']['robot']
            self._FRAME_TARGET = config['tf']['frame_name']['target']

        '''create a deque'''
        self._results = deque([], maxlen=10)
        self._rate = rospy.Rate(self._FREQUENCY)
        while not rospy.is_shutdown():

            try:
                self._update_transform()
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
                motion_publisher.publish(False)
                self._rate.sleep()
                continue

            if len(self._results) < 10:
                motion_publisher.publish(False)
                self._rate.sleep()
                continue
            
            detected = False
            _moving_average = self._get_moving_average()
            _threshold = self._DETECTION_DISTANCE_TRIGGER_BASE * (1 + 4 * abs(self._twist.linear.x) / self._MAXIMUM_LINEAR_VELOCITY)

            ratio = _moving_average / _threshold * 100
            print ratio

            if _moving_average > _threshold:
                detected = True

            if detected == True:
                self._linger = self._LINGER_MAXIMUM
                motion_publisher.publish(True)
            elif detected == False and self._linger == 0:
                motion_publisher.publish(False)
            else:
                self._linger = self._linger - 1
                motion_publisher.publish(True)
            self._rate.sleep()

if __name__ == '__main__':
    rospy.init_node('motion_detect')
    try:
        detect = Detect()
    except rospy.ROSInterruptException:  pass
