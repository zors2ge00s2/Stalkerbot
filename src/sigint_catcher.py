#!/usr/bin/env python
import rospy
import signal
import sys
from geometry_msgs.msg import Twist

'''stops the robot when the program is remotely interrupted'''
def signal_handler(sig, frame):
        print('Stopping Turtlebot3...')
        twist = Twist()
        cmd_vel.publish(twist)
        print('Exit program...')
        sys.exit(0)

rospy.init_node('sigint_catcher')
cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
signal.signal(signal.SIGINT, signal_handler)
signal.pause()

