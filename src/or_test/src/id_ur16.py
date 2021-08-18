#!/usr/bin/env python
from openravepy._openravepy_ import *
# from openravepy._openravepy_0_9 import *
import time
import argparse
import pdb
import rospy
import json
import numpy as np
import matplotlib.pyplot as plt
from openravepy import misc
from openravepy import *
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import WrenchStamped
from or_test.srv import *
import time
from copy import deepcopy



class IDMonitor(object):
    def __init__(self):

        self.env = Environment()

        self.dof_value = [0]*6
        self.dof_torque = [0]*6
        self.collected = {'pos': [], 'eff': [], 'for': []}
        self.force = [0]*3
        self.torque = [0]*3
        self.force_cmd = [0]*3
        self.torque_cmd = [0]*3
        self.jt_msg = [0]*6
        self.tm_msg = [0]*6
        self.threshold = np.array([70,100,100,40,40,40])
        self.worst_offset = np.array([0,0, -150, 0 , 0 ,0])

        plugin = RaveCreateModule(self.env, 'urdf')
        self.name = plugin.SendCommand(
            'load /home/rocky/catkin_ws/src/bot_digger_description/urdf/ur16_openrave.urdf')

        with self.env:
            # set a physics engine
            self.physics = RaveCreatePhysicsEngine(self.env, 'ode')
            self.physics.SetGravity(numpy.array((0, 0, -9.81)))
            self.env.SetPhysicsEngine(self.physics)

            # need to set base as static or it flies off
            base_link = self.env.GetKinBody('or_ur').GetLink('chisel_base')
            base_link.SetStatic(True)

            self.eef_link_str = 'chisel_tip_link'

            self.eef_link = self.env.GetKinBody(
                'or_ur').GetLink(self.eef_link_str)

            # record the joint names since we need the order
            body = self.env.GetKinBody(self.name)
            self.jt_names = [str(jt.GetName()) for jt in body.GetJoints()]

            # default joint order?
            #'chisel_elbow_joint'
            #'chisel_shoulder_lift_joint'
            #'chisel_shoulder_pan_joint'
            #'chisel_wrist_1_joint'
            #'chisel_wrist_2_joint'
            #'chisel_wrist_3_joint'

            self.env.StopSimulation()

    def setWorstOffset(self, offset):
        self.worst_offset = offset

    def getEFFForce(self,force):
        body = self.env.GetKinBody(self.name)
        link = body.GetLink('chisel_tip_link')
        T = link.GetTransform()
        adj = np.zeros((6,6))
        adj[:3,:3] = T[:3,:3]
        adj[3:,3:] = T[:3,:3]
        return adj.dot(force)



    def calInverseDynamics(self, jt_data , eef_force):
        body = self.env.GetKinBody(self.name)
        self.setJointState(jt_data)
        eef_force_body = self.getEFFForce(np.array(eef_force))
        link_id = body.GetLink(self.eef_link_str).GetIndex()
        torque = np.array(body.ComputeInverseDynamics(dofaccelerations = [0.]*6,externalforcetorque={link_id:eef_force_body})) 
        return torque

    def calInverseDynamicsService(self, req):
        eef_force = wrench2List(req.eef_force)
        torque = self.calInverseDynamics(req.js ,eef_force) 
        res = inverse_dynamicsResponse()
        res.je = self.wrapJointEffortMsg(torque)
        return res

    def calProtectivePrediction(self, jt_data , eef_force, eef_target):
        torque_expected = self.calInverseDynamics(jt_data,eef_force)

        eef_worst = deepcopy(eef_force)


        for idx in range(len(eef_worst)):
            eef_worst[idx] += self.worst_offset[idx]
        ## -------------------------------------
        # eef_worst[2] -= 100
        # eef_worst[0] += 70
        ## -------------------------------------

        torque_worst = self.calInverseDynamics(jt_data,eef_worst)

        # print(eef_force)
        # print(eef_worst)

        torque_diff = np.abs(torque_expected-torque_worst) 

        if np.any(torque_diff > self.threshold):
            return False,torque_diff

        return True,torque_diff

    def calProtectivePredictionService(self, req):
        eef_force = wrench2List(req.eef_force)
        is_valid,torque_diff = self.calProtectivePrediction(req.js, eef_force, req.eef_target)
        res = protective_predictionResponse()
        res.je_diff = self.wrapJointEffortMsg(torque_diff) 
        res.is_valid.data = is_valid
        return res

    def wrapJointEffortMsg(self, data):
        joints = JointState()
        joints.name = ['chisel_elbow_joint',
            'chisel_shoulder_lift_joint',
            'chisel_shoulder_pan_joint',
            'chisel_wrist_1_joint',
            'chisel_wrist_2_joint',
            'chisel_wrist_3_joint']
        joints.effort = data 
        return joints

    def setJointState(self, jt_data):
        # Match joint name order
        for name in jt_data.name:
            try:
                idx = self.jt_names.index(name)
                self.dof_value[idx] = jt_data.position[jt_data.name.index(
                    name)]
            except ValueError:
                continue
        body = self.env.GetKinBody(self.name)
        body.SetDOFValues(self.dof_value)



def wrench2List(data):
    return [data.force.x, data.force.y, data.force.z, data.torque.x, data.torque.y , data.torque.z]


if __name__ == "__main__":
    

    # Initialize parser
    parser = argparse.ArgumentParser()
    
    # Adding optional argument
    parser.add_argument("-b", "--Visual", help = "Show visual")
    
    # Read arguments from command line
    args = parser.parse_args()


    rospy.init_node('inverse_dynamics_monitor')

    idm = IDMonitor()
        # if args.Visual:
    
    idm.env.SetViewer('qtcoin')
    misc.DrawAxes(idm.env, matrixFromAxisAngle([0, 0, 0]))

    id_service = rospy.Service('/chisel/inverse_dynamics', inverse_dynamics, idm.calInverseDynamicsService)
    pp_service = rospy.Service('/chisel/protective_prediction', protective_prediction, idm.calProtectivePredictionService)

    rospy.spin()
