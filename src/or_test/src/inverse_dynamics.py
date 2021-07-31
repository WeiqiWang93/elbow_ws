#!/usr/bin/env python
from openravepy._openravepy_ import *
# from openravepy._openravepy_0_9 import *
import time
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
# from bot_chisel.msg import ChiselStatus
import time
# openravepy.loadstable()


CONTROL_RATE = 1000.


class JointMonitor(object):
    def __init__(self):
        self.env = Environment()
        # self.HAS_STARTED = False

        self.dof_value = [0]*6
        self.dof_torque = [0]*6
        self.collected = {'pos': [], 'eff': [], 'for': []}
        self.force = [0]*3
        self.torque = [0]*3
        self.force_cmd = [0]*3
        self.torque_cmd = [0]*3
        self.jt_msg = [0]*6
        self.tm_msg = [0]*6

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

            eef_link = self.env.GetKinBody(
                'or_ur').GetLink('chisel_chisel_link')
            # eef_link.SetStatic(True)

            # record the joint names since we need the order
            body = self.env.GetKinBody(self.name)
            self.jt_names = [str(jt.GetName()) for jt in body.GetJoints()]

            # default joint order?
            #'chisel_elbow_joint'
            #'chisel_shoulder_lift_joint'
            #'chisel_shoulder_pan_joint'
            #'chisel_wrist_1_joint'
            #'chisel_wrist_2_joint'
            #'chisel_wrist_3_joint's

            self.env.StopSimulation()

    def jointstateCallback(self, jt_data):
        # Match joint name order
        # jt_data = JointState()
        for name in jt_data.name:
            try:
                idx = self.jt_names.index(name)
                self.dof_value[idx] = jt_data.position[jt_data.name.index(
                    name)]
            except ValueError:
                continue

        # might as well call it here
        self.collectData()

    def jointeffortCallback(self, jt_data):
        # Match joint name
        # jt_data = Float32MultiArray()
        default_name = ['chisel_shoulder_pan_joint', 'chisel_shoulder_lift_joint',
                        'chisel_elbow_joint', 'chisel_wrist_1_joint', 'chisel_wrist_2_joint', 'chisel_wrist_3_joint']
        for idx in range(6):
            self.dof_torque[self.jt_names.index(default_name[idx])
                            ] = jt_data.data[idx]

    def externalForceCallback(self, data):
        # data = WrenchStamped()
        self.force = [data.wrench.force.x,data.wrench.force.y,data.wrench.force.z]
        self.torque = [data.wrench.torque.x,data.wrench.torque.y,data.wrench.torque.z]


    def commandForceCallback(self, data):
        # data = WrenchStamped()
        self.force_cmd = data.data[:3]
        self.torque_cmd = data.data[3:]


    def jointTorqueCallback(self, data):
        # data = WrenchStamped()
        self.jt_msg = data.data

    def targetMomentCallback(self, data):
        self.tm_msg = data.data

    def collectData(self):
        self.env.StopSimulation()
        body = self.env.GetKinBody(self.name)
        # print(self.dof_torque)
        body.SetDOFValues(self.dof_value)


        # calculate joint torque from measurement
        # link_id = body.GetLink('chisel_tool0').GetIndex()

        link_id = body.GetLink('chisel_chisel_link').GetIndex()
        



        # link = body.GetLinks()[7]
        # self.collected['pos'].append(self.dof_value)
        # self.collected['eff'].append(self.dof_torque)
        # self.collected['for'].append(self.physics.GetLinkForceTorque(link))
        torque = np.array(body.ComputeInverseDynamics(dofaccelerations = [0]*6,externalforcetorque={link_id:self.force+self.torque}))

        # torque = body.ComputeInverseDynamics(dofaccelerations = [0]*6,externalforcetorque={link_id:[0]*6})
        # print(link.GetTransform())
        # print(self.printInOrder(torque))
        torque_cmd = np.array(body.ComputeInverseDynamics(dofaccelerations = [0]*6,externalforcetorque={link_id:np.array(self.force_cmd+self.torque_cmd)*-1.0}))

        # print(self.printInOrder(torque))

        print(np.abs(torque-torque_cmd))


    def isChiseling(self):
        if np.abs(self.force_cmd[2]) > 50:
            return True

    def printInOrder(self, data):
        default_name = ['chisel_shoulder_pan_joint', 'chisel_shoulder_lift_joint',
                        'chisel_elbow_joint', 'chisel_wrist_1_joint', 'chisel_wrist_2_joint', 'chisel_wrist_3_joint']
        res = [0]*6
        for idx in range(6):
            res[default_name.index(self.jt_names[idx])] = data[idx]
        return res

    def saveData(self, data):
        timestr = time.strftime("%Y%m%d-%H%M%S")
        with open(timestr + ".csv", "w") as outfile:
            json.dump(self.collected, outfile)


    def show(self):
        self.env.SetViewer('qtcoin')


if __name__ == "__main__":
    jm = JointMonitor()
    jm.show()

    rospy.init_node('joint_money')
    # set up ros last

    rospy.Subscriber("/joint_states", JointState,
                     jm.jointstateCallback)
    rospy.Subscriber("/chisel/wrench", WrenchStamped,
                     jm.externalForceCallback)
    rospy.Subscriber("/chisel/force_cmd_ft", Float32MultiArray,
                     jm.commandForceCallback)

    rospy.Subscriber("/chisel/joint_torque", Float32MultiArray,
                     jm.jointTorqueCallback)

    rospy.Subscriber("/chisel/target_moment", Float32MultiArray,
                     jm.jointTorqueCallback)

    misc.DrawAxes(jm.env, matrixFromAxisAngle([0, 0, 0]))

    rospy.spin()

