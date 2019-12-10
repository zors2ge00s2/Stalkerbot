#!/usr/bin/env python

import rospy
import os
import yaml
from std_msgs.msg import Time, Bool

'''
Report the duration between current time and when last detection of motion was recognized.
Subscribes to /stalkerbot/motion/transform, publishes to /stalkerbot/motion/interval
'''

class Detection_Interval():

    '''Update detection time upon motion detection'''
    def detection_cb(self, msg):
        if msg.data:
            self._last_detection_time = rospy.Time.now()

    def __init__(self):
        self._last_detection_time = None

        motion_sub = rospy.Subscriber('/stalkerbot/motion', Bool, self.detection_cb, queue_size=1)
        interval_publisher = rospy.Publisher('/stalkerbot/motion/interval', Time, queue_size=1)

        '''Extract rate from config file'''
        rate = None
        with open(os.path.dirname(__file__) + '/../config.yaml','r') as file:
            config = yaml.load(file, Loader=yaml.FullLoader)
            rate = rospy.Rate(config['core']['frequency']['interval']['detection'])

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            if self._last_detection_time is not None:
                time_elapsed = current_time - self._last_detection_time
                interval_publisher.publish(time_elapsed)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('detection_interval')
    try:
        detection_interval = Detection_Interval()
    except rospy.ROSInterruptException:  pass

