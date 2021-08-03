#!/usr/bin/env python

import time
import pdb
import rospy
import json
import numpy as np
import matplotlib.pyplot as plt

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Wrench
from or_test.srv import *
import time
from copy import deepcopy

from id_ur16 import *


class JointMonitor(object):
    def __init__(self):
        self.js_msg = None
        self.force_cmd = None

    def jointstateCallback(self, jt_data):
        self.js_msg = jt_data

    def commandForceCallback(self, data):
        self.force_cmd = data


    def checkID(self,data):
        rospy.wait_for_service('/chisel/inverse_dynamics')

        try:
            serviceFun = rospy.ServiceProxy('/chisel/inverse_dynamics', inverse_dynamics)
            req = inverse_dynamicsRequest()        
            req.js = self.js_msg
            w = Wrench()
            w.force.x,w.force.y,w.force.z = self.force_cmd.data[:3]
            w.torque.x,w.torque.y,w.torque.z = self.force_cmd.data[3:]
            req.eef_force = w
            res = serviceFun(req)
            print res.je

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def checkPP(self,data):
        rospy.wait_for_service('/chisel/protective_prediction')

        try:
            serviceFun = rospy.ServiceProxy('/chisel/protective_prediction', protective_prediction)
            req = protective_predictionRequest()        
            req.js = self.js_msg
            w = Wrench()
            w.force.x,w.force.y,w.force.z = self.force_cmd.data[:3]
            w.torque.x,w.torque.y,w.torque.z = self.force_cmd.data[3:]
            req.eef_force = w
            res = serviceFun(req)
            print res.is_valid
            print res.je_diff

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def saveData(self, data):
        timestr = time.strftime("%Y%m%d-%H%M%S")
        with open(timestr + ".json", "w") as outfile:
            json.dump(self.collected, outfile)


    def show(self):
        self.env.SetViewer('qtcoin')


if __name__ == "__main__":
    jm = JointMonitor()

    rospy.init_node('inverse_dynamics_test')


    rospy.Subscriber("/joint_states", JointState,
                     jm.jointstateCallback)

    rospy.Subscriber("/chisel/force_cmd_ft", Float32MultiArray,
                     jm.commandForceCallback)


    rospy.Subscriber("/save_data", Bool,
                     jm.saveData)


    rospy.Subscriber("/checkPP", Bool,
                     jm.checkPP)


    rospy.Subscriber("/checkID", Bool,
                     jm.checkID)

    rospy.spin()
