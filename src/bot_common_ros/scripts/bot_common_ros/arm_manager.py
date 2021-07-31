#! /usr/bin/env python

import os, sys
import numpy as np
from enum import Enum, unique
import rospy, actionlib
from bot_common_ros.msg import *





"""
config file:
arm_1:



"""




"""
mgr.drivers = {id1: driver_class, id2: driver_class2}


depending on what arms are connected, mgr instantiates the corresponding class objects for that driver / arm

"""


class ArmManager(object):

    def __init__(self):
        
        ping_topic = rospy.get_param("~ping_topic")
        ping = rospy.get_param(ping_topic)


        # define interface through subscribers
        # 
        self.sub = rospy.Subscriber("~output", Float32, self.cb)


        #
        self.srv_setups = []
        self.srv_cmds = []
        self.srv_fbs = []


    def setup(self):
        