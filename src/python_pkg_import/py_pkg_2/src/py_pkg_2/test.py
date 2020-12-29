#!/usr/bin/env python

import rospy

from py_pkg_1 import common

if __name__=='__main__':
    rospy.init_node('test_node')
    common.say_hello_world()
