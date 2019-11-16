#!/usr/bin/env python

import rospy
import os
import yaml
import actionlib
import stalkerbot.msg

class Search(name):
    _feedback = stalkerbot.msg.SearchFeedback
    _result = stalkerbot.msg.SearchResult
    _preempt_rate = rospy.Rate()

    with open(os.path.dirname(__file__) + '/../config.yaml','r') as file:
    config = yaml.load(file, Loader=yaml.FullLoader)
    _preempt_rate = config['core']['frequency']['search_preempt']

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,stalkerbot.msg.SearchAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        '''TODO:
        Find the target coordinate from fiducial
        Use move_base from AMCL
        Set up preempt condition'''

if __name__ == '__main__':
    rospy.init_node('search_server')
    search = Search(rospy.get_name())


