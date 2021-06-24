#!/usr/bin/env python
from openravepy._openravepy_ import *
# from openravepy._openravepy_0_9 import *
import time
import pdb
import rospy
import numpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

# openravepy.loadstable()


CONTROL_RATE = 1000

class JointMonitor(object):
    def __init__(self):
        self.env = Environment()
        # self.HAS_STARTED = False

        self.dof_value = [0]*6
        self.dof_torque = [0]*6

        plugin = RaveCreateModule(self.env, 'urdf')
        self.name = plugin.SendCommand(
            'load /home/rocky/catkin_ws/src/bot_digger_description/urdf/ur16_openrave.urdf')

        with self.env:
            # set a physics engine
            self.physics = RaveCreatePhysicsEngine(self.env, 'ode')
            self.env.SetPhysicsEngine(self.physics)
            self.physics.SetGravity(numpy.array((0, 0, -9.8)))

            # need to set base as static or it flies off
            base_link = self.env.GetKinBody('or_ur').GetLink('chisel_base')
            base_link.SetStatic(True)

            # record the joint names since we need the order
            body = self.env.GetKinBody(self.name)
            self.jt_names = [jt.GetName() for jt in body.GetLinks()]

            self.env.StopSimulation()

        # set up ros last
        rospy.init_node("or_test_dynamics")
        rospy.Subscriber("/chisel/joint_state", JointState,
                         self.jointstateCallback)
        rospy.Subscriber("/chisel/joint_torques", JointState,
                         self.jointstateCallback)

    def jointstateCallback(self, jt_data):

        # in case we have to do real time
        # if not self.HAS_STARTED:
        # self.HAS_STARTED = True
        # self.env.StartSimulation(timestep=0.001)

        # Match joint name order
        # jt_data = JointState()
        for name in jt_data.name:
            try:
                idx = self.jt_names.index(name)
                self.dof_value[idx] = jt_data.position[jt_data.name.index(
                    name)]
            except ValueError:
                continue

        body = self.env.GetKinBody(self.name)
        body.SetDOFValues(self.dof_value)
        body.SetDOFTorques(self.dof_torque)
        self.env.StepSimulation(1/CONTROL_RATE)

        link = body.GetLink('chisel_forearm_link')
        print(self.physics.GetLinkForceTorque(link))

    def jointeffortCallback(self, jt_data):
        # Match joint name
        # jt_data = Float32MultiArray()
        default_name = ['chisel_shoulder_pan_joint', 'chisel_shoulder_lift_joint',
                        'chisel_elbow_joint', 'chisel_wrist_1_joint', 'chisel_wrist_2_joint', 'chisel_wrist_3_joint']
        for idx in range(6):
            self.dof_torque[self.jt_names.index[default_name[idx]]
                            ] = jt_data[idx]

    def show(self):
        self.env.SetViewer('qtcoin')


if __name__ == "__main__":
    jm = JointMonitor()
    jm.show()
    rospy.spin()


# env = Environment()
# plugin = RaveCreateModule(env, 'urdf')

# env.SetViewer('qtcoin')

# name = plugin.SendCommand('load /home/rocky/catkin_ws/src/bot_digger_description/urdf/ur16_openrave.urdf')


# jt = None
# link = None

# # while True:
# #     phy.AddJointTorque(jt, [20.0])
# #     phy.SimulateStep(5.0)
# #     print 'here'
# #     print phy.GetLinkForceTorque(link)
# #     time.sleep(1.0)

# with env:
#     # set a physics engine

#     physics = RaveCreatePhysicsEngine(env,'ode')
#     env.SetPhysicsEngine(physics)
#     physics.SetGravity(numpy.array((0,0,-9.8)))

#     # robot = env.GetRobots()[0]
#     # robot.GetLinks()[0].SetStatic(True)

#     body = env.GetKinBody(name)

#     link = body.GetLink('chisel_forearm_link')
#     jt = body.GetJoint('chisel_elbow_joint')

#     # need to set base as static or it flies off
#     base_link =env.GetKinBody('or_ur').GetLink('chisel_base')
#     base_link.SetStatic(True)
#     env.StopSimulation()
#     # env.StartSimulation(timestep=0.001)

#     for s in range(1000):
#         env.StepSimulation(0.001)

#     print physics.GetLinkForceTorque(link)

# for itry in range(5):
#     tor = 0.0
#     for i in range(100):
#         # have to lock environment when calling robot methods
#         with env:
#             tor = 0.2
#             physics.AddJointTorque(jt, [tor])
#             print physics.GetLinkForceTorque(link)
#         time.sleep(0.1)

# # reset the physics engine
# env.SetPhysicsEngine(None)
