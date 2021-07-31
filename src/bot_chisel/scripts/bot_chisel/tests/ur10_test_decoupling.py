#!/usr/bin/env python
# Copyright offworld.ai 2018

import time, os, sys, pdb
import rospy
from bot_common_ros.ur_control import *
from bot_common_ros.ur_utils import *
from bot_common_ros.ur_control import TimeOut
from bot_common_ros.ur_utils import SensorFeedbackControl, SensorFeedbackControl2
from bot_common_ros.arm_client import ArmClient
import math3d as m3d

import numpy as np

VEL                 = 0.5
ACC                 = 2.0 * VEL

class UR10TestDecoupling():
    def __init__(self):
        rospy.init_node('ur10_test', anonymous=False)
        self.direction_to_chisel    = 1.0
        self.is_sim = False
        self.run_arm_api = True
        self.URmonitor_percent = 0.0
        self.client_topic = "force_decouple_test/arm"
        self.server_input_param_path = "ur10_driver/arm_1/params"
        self.ur10_bringup()

    def start_testing(self):
        self.arm_client.reset_FT300()
        self.sensorFeedbackControl  = SensorFeedbackControl(self.arm_client)
        self.mw.moveToBBox([0.5, 0.9, 0.5], ACC, VEL)
        try:
            while not rospy.is_shutdown():
                self.forceAwayFromRock(0.0425, 0.05, self.direction_to_chisel)
        except KeyboardInterrupt:
            print("Shutting down.")
    
    def forceAwayFromRock(self, minDist, maxDist, directionToChisel):

        if self.direction_to_chisel > 0.0:
            wiggleBool = -1.0
        else:
            wiggleBool = +1.0
        wiggleTorque = wiggleBool * 75.0
        wiggleTime = 0.5
        wiggleT0 = time.time()
        dist = 100.0
        distancedOut = False
        timer = TimeOut(10.0)

        #print 'Initial dist: ', dist
        try:
            while not distancedOut and not rospy.is_shutdown():
                if timer.check():
                    pass
                    #break
                distancedOut = False
                pose = self.arm_client.get_pose()

                self.sensorFeedbackControl.update()
                torqueList, limits = self.sensorFeedbackControl.PID.getForceModeInput()

                yLimit = limits[4]
                yTorque = torqueList[4]
                
                print "get_force :: ", self.arm_client.get_force()

                max_torque_allowable = 25.0
                if np.abs(self.arm_client.get_force()[4]) > max_torque_allowable:
                    #self.arm_client.end_force_mode()
                    torque = [0.0, 0.0, 0.0, 0.0, -1.0 * yTorque,0]
                    limits = [0.0, 0.0, 0.0, 0.0, yLimit, 0]
                    print ("Cmd wrench: {}".format(torque))
                    print ("Cmd limits: {}".format(limits))
                    self.arm_client.force_mode(pose_base=pose, torque=torque, limits=limits, selection_vector=np.ones(6), setRemote=True)
                else:
                    torque = [0.0, 0.0, 0.0, 0.0, yTorque,0]
                    limits = [0.0, 0.0, 0.0, 0.0, yLimit, 0]
                    print ("Cmd wrench: {}".format(torque))
                    print ("Cmd limits: {}".format(limits))
                    #torque = torqueList
                    self.arm_client.force_mode(pose_base=pose, torque=torque, limits=limits, selection_vector=np.ones(6), setRemote=True)

                if (time.time() - wiggleT0) > wiggleTime:
                    wiggleBool *= -1.0 # toggle back and forth to create the wiggle
                    wiggleTorque = wiggleBool * 75.0
                    wiggleT0 = time.time()
                    
                time.sleep(0.05)
        except:
            pass

    def test(self):
        pass

    def ur10_min_max_angles(self, tcp):
        maxx = [ 10,  5, 30]
        minn = [ -10, -5, -30]

        #self.arm_client.set_min_max_angles(maxx, minn)
        # [roll, pitch, yaw] limits w.r.t. base frame
        max_out = np.array(maxx, dtype='float') * math.pi / 180.0 
        min_out = np.array(minn, dtype='float') * math.pi / 180.0
        return min_out, max_out

    def ur10_bringup(self):
        # ---------------------------------------------------------
        self.n0_extrema = np.array([ -0.6, 0.7, -0.35])
        self.nf_extrema = np.array([ 0.45, 1.2, 0.35])

        """ nominal max size"""
        corner = self.n0_extrema
        size = self.nf_extrema - self.n0_extrema
        # ---------------------------------------------------------
        boundingBox = np.concatenate([corner, size])
        params = {}
        params["bounding_box"] = boundingBox
        params['ur_monitor_expansion'] = self.URmonitor_percent
        #
        client_topic = self.client_topic
        if not self.run_arm_api:
            # test client
            self.arm_client = ArmClient(client_topic, sim=self.is_sim)
        else:
            # test API
            self.arm_client = ArmClient(client_topic, sim=self.is_sim, run_as_api=True, server_input_param_path=self.server_input_param_path, server_dry_run=False)
        #
        self.mw = URMiddleWare(self.arm_client, params)
        # calculate orientation from plane normal
        orient = getHorizontalOrient()
        self.mw.set_nominal_orient(orient) # default
        self.mw.set_min_max_angles_fxn(self.ur10_min_max_angles)


if __name__ == "__main__":
    test_app = UR10TestDecoupling()
    pdb.set_trace()
    test_app.start_testing()