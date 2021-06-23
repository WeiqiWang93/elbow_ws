#!/usr/bin/env python
from openravepy import *
import time
import pdb
import rospy


env = Environment()
plugin = RaveCreateModule(env, 'urdf')

name = plugin.SendCommand('load /home/rocky/catkin_ws/src/bot_digger_description/urdf/sia20.urdf')



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
    link = body.GetLink('saw_link_t')
    jt = body.GetJoint('saw_joint_l')
    env.StopSimulation()
    env.StartSimulation(timestep=0.001)

    print physics.GetLinkForceTorque(link)

for itry in range(5):
    tor = 0.0
    for i in range(100):
        # have to lock environment when calling robot methods
        with env:
            tor += 2.0
            physics.AddJointTorque(jt, [tor])        
            print physics.GetLinkForceTorque(link)
        time.sleep(0.01)

# reset the physics engine
env.SetPhysicsEngine(None)
