import time, argparse
import sys
import copy
import math
import numpy as np
import pdb
import socket
import os
import rospy

import math3d as m3d
#
import rospy
from visualization_msgs.msg import Marker
import actionlib

# actionlib server stuff
from bot_common_ros.msg import ARMAction as Action, ARMGoal as Goal, ARMFeedback as Feedback, ARMResult as Result
from bot_common_ros.msg import ActionType, ProcessStatus, ArmStatus
#

done = False
status = None

def reset():
    global status, done
    done = False
    status = None

def update_status(msg):
    global status
    status = msg.result.arm_status.status

def wait_until_status(status_done):
    while not status == status_done:
        time.sleep(0.5)
    reset()

def test():
    __version__ = "1_0"
    rospy.init_node('TestRoboChiselActionServer_v'+__version__, anonymous=False)
    
    # set up action client
    client = actionlib.SimpleActionClient('/chisel/RoboChisel', Action)
    client.wait_for_server()
    s1 = rospy.Subscriber("/chisel/RoboChisel/result", Result, update_status)
    print "Connected to the RoboChisel server"
    #
    goal = Goal()
    result = None

    """ Go through FSM """
    # run op_init
    # INPUT GEOMETRY
    y_value = 1.0
    ll = np.array([ -0.60, y_value, -0.10])
    uu = np.array([ 0.10, y_value, 0.55])


    # Don't Touch
    scale = uu - ll
    center = ll + 0.5 * scale
    bbox = Marker( type = Marker.CUBE )
    bbox.pose.position.x = center[0]
    bbox.pose.position.y = center[1]
    bbox.pose.position.z = center[2]
    #
    bbox.scale.x = scale[0]
    bbox.scale.y = scale[1]
    bbox.scale.z = scale[2]
    # End Don't Touch


    # bbox.pose.position.x= -0.047139890492
    # bbox.pose.position.y= 1.27668404579
    # bbox.pose.position.z= 0.105292662978

    # #scale: 
    # bbox.scale.x= 0.52610963583
    # bbox.scale.y= 0.0689867734909
    # bbox.scale.z= 0.294296234846



    def run():
        global result
        client.send_goal(goal)
        client.wait_for_result()
        result = client.get_result()

    def init():
        bbox = rospy.wait_for_message('/bot_digger_chisel/operating_box', Marker, timeout=60.0)
        goal.action_type.type = ActionType.INIT
        goal.slots_z0 = 0.02
        goal.vertical_pitch = 0.05
        goal.quadrant = bbox
        run()

    def reset():
        bbox = rospy.wait_for_message('/bot_digger_chisel/operating_box', Marker, timeout=60.0)
        goal.action_type.type = ActionType.RESET
        goal.quadrant = bbox
        run()

    def test_shave(start_index = 0):
        # run op_execute
        goal.options = "SHAVER"
        goal.action_type.type = ActionType.EXECUTE
        goal.start_index = start_index
        client.send_goal(goal)

    def test_cleanup(start_index = 0):
        # run op_execute
        goal.options = "CLEANUP"
        goal.action_type.type = ActionType.EXECUTE
        goal.start_index = start_index
        client.send_goal(goal)

    def pause():
        # run op-pause
        goal.action_type.type = ActionType.PAUSE
        run()

    def aco():
        goal.action_type.type = ActionType.CONTINUE
        client.send_goal(goal)

    def shutdown():
        goal.action_type.type = ActionType.SHUTDOWN
        run()

    init()

    while True:
        pdb.set_trace()

if __name__ == '__main__':
    test()