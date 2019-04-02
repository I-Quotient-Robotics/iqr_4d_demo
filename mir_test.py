# -*- coding: utf-8 -*-
#!/usr/bin/env python

import sys
import signal

import rospy

from std_msgs.msg import String
from mir_api import mir_move_goal

running = False

PARK_A_GUID = '3c96dbb8-5219-11e9-82ef-94c691a737d7'
PARK_B_GUID = '6eadab0f-5219-11e9-82ef-94c691a737d7'
PARK_C_GUID = 'b44fd151-5219-11e9-82ef-94c691a737d7'
PARK_STOP_GUID = '3d1bfd29-5243-11e9-b6a0-94c691a737d7'

def signal_handler(signum, frame):
    global running
    running = False

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    rospy.init_node("mir_test")
    rospy.sleep(3)

    # Send to Park A
    print 'move to Park A'
    if mir_move_goal(PARK_A_GUID):
        pass
    else:
        print 'go Park A error'
        exit(-1)
    rospy.sleep(1)

    # Send to Park B
    print 'move to Park B'
    if mir_move_goal(PARK_B_GUID):
        pass
    else:
        print 'go Park B error'
        exit(-1)
    rospy.sleep(1)

    # Send to Park C
    print 'move to Park C'
    if mir_move_goal(PARK_C_GUID):
        pass
    else:
        print 'go Park C error'
        exit(-1)
    rospy.sleep(1)

    print('All finished')

