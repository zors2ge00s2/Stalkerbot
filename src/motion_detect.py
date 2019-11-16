#!/usr/bin/env python

import rospy
import math
import os
import yaml
from std_msgs.msg import Bool, Time
from stalkerbot.msg import location_info

class Detect():
    '''Class keeps the last two frame of locations of the target. The callback updates the locations accordingly'''
    def _location_cb(self, msg):
        if self._current_location is not None:
            self._last_location = self._current_location
        self._current_location = msg
        
    '''Update interval upon callback'''
    def _interval_cb(self, msg):
        self._interval = msg

    def _distance(self):
        x1, x2 = self._last_location.x_translation, self._current_location.x_translation
        y1, y2 = self._last_location.y_translation, self._current_location.y_translation
        z1, z2 = self._last_location.z_translation, self._current_location.z_translation
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
        # Uncomment to debug
        # print distance / self._DETECTION_DISTANCE_TRIGGER * 100
        return distance

    def __init__(self):
        self._interval = None
        self._last_location = None
        self._current_location = None
        self._linger = 0

        motion_publisher = rospy.Publisher('/stalkerbot/motion', Bool, queue_size = 1)
        location_subscriber = rospy.Subscriber('/stalkerbot/location/marker/rad', location_info, self._location_cb, queue_size=1)
        interval_subscriber = rospy.Subscriber('/stalkerbot/fiducial/interval', Time, self._interval_cb, queue_size=1)

        self._DETECTION_DISTANCE_TRIGGER = 0
        self._FREQUENCY = 0
        self._LINGER_MAXIMUM = 0

        with open(os.path.dirname(__file__) + '/../config.yaml','r') as file:
            config = yaml.load(file, Loader=yaml.FullLoader)
            self._DETECTION_DISTANCE_TRIGGER = config['core']['distance']['detection']['trigger']
            self._FREQUENCY = config['core']['frequency']['motion_detect']
            self._LINGER_MAXIMUM = config['core']['frequency']['motion_detect_linger']
        
        self._rate = rospy.Rate(self._FREQUENCY)
        while not rospy.is_shutdown():
            
            if self._last_location is None:
                motion_publisher.publish(False)
                continue
            
            detected = False
            if self._distance() > self._DETECTION_DISTANCE_TRIGGER:
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
