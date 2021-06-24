#!/usr/bin/env python
from openravepy._openravepy_ import *
# from openravepy._openravepy_0_9 import *
import time
import pdb
import rospy
import numpy
from sensor_msgs.msg import JointState

# openravepy.loadstable()



class JointMonitor(object):
    def __init__(self):
        rospy.init_node("~")
        rospy.Subscriber("/chisel/joint_state", JointState , self.jointstateCallback)

        self.env = Environment()
        # self.HAS_STARTED = False


        plugin = RaveCreateModule(env, 'urdf')

        name = plugin.SendCommand('load /home/rocky/catkin_ws/src/bot_digger_description/urdf/ur16_openrave.urdf')

        with env:
            # set a physics engine

            physics = RaveCreatePhysicsEngine(env,'ode')
            env.SetPhysicsEngine(physics)
            physics.SetGravity(numpy.array((0,0,-9.8)))

            # robot = env.GetRobots()[0]
            # robot.GetLinks()[0].SetStatic(True)


            self.env.StopSimulation()
            

    def jointstateCallback(self, data):

        # if not self.HAS_STARTED:
            # self.HAS_STARTED = True
            # self.env.StartSimulation(timestep=0.001)
        self.env.StepSimulation(1/CONTROL_RATE)

        body = self.env.GetKinBody(name)

        link = body.GetLink('chisel_forearm_link')
        jt = body.GetJoint('chisel_elbow_joint')


env = Environment()
plugin = RaveCreateModule(env, 'urdf')

env.SetViewer('qtcoin')

name = plugin.SendCommand('load /home/rocky/catkin_ws/src/bot_digger_description/urdf/ur16_openrave.urdf')



jt = None
link = None

# while True:
#     phy.AddJointTorque(jt, [20.0])
#     phy.SimulateStep(5.0)
#     print 'here'
#     print phy.GetLinkForceTorque(link)
#     time.sleep(1.0)

with env:
    # set a physics engine
    
    physics = RaveCreatePhysicsEngine(env,'ode')
    env.SetPhysicsEngine(physics)
    physics.SetGravity(numpy.array((0,0,-9.8)))
    
    # robot = env.GetRobots()[0]
    # robot.GetLinks()[0].SetStatic(True)
    
    body = env.GetKinBody(name)
    
    link = body.GetLink('chisel_forearm_link')
    jt = body.GetJoint('chisel_elbow_joint')

    # need to set base as static or it flies off
    base_link =env.GetKinBody('or_ur').GetLink('chisel_base')
    base_link.SetStatic(True)
    env.StopSimulation()
    env.StartSimulation(timestep=0.001)

    # for  s in range(1000):
        # env.StepSimulation(0.001)

    print physics.GetLinkForceTorque(link)

for itry in range(5):
    tor = 0.0
    for i in range(100):
        # have to lock environment when calling robot methods
        with env:
            tor = 0.2
            physics.AddJointTorque(jt, [tor])        
            print physics.GetLinkForceTorque(link)
        time.sleep(0.01)

# reset the physics engine
env.SetPhysicsEngine(None)
