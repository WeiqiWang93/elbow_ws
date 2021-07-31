#!/usr/bin/env python
import time
import sys
import copy
import math
import numpy as np
import pdb
import socket

import roslib
import rospy
import actionlib
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

def genPoses(initPose, maxPos, maxAngle, numSweep, backOff, sweepAxes):
    #generate poses in back-and-forth pattern

    #convert to numpy arrays
    initPose = np.array(initPose, ndmin=2) #2-dims - 1x6
    maxPos = np.array(maxPos) #2-dims - 1x3
    numSweep = np.array(numSweep) #2-dims - 1x3
    sweepAxes = np.array(sweepAxes)

    dPos = maxPos / (numSweep - 1) #1x3
    dAng = maxAngle / (numSweep - 1) #scalar

    sweepIdx = np.where(sweepAxes==1)[0]
    normalIdx = np.where(sweepAxes==0)[0]

    numPoses = 3*(np.prod(numSweep[sweepIdx])-1) #start at start, end at end
    poses = np.zeros([numPoses, 6]) #back up - move - approach
    bDir = 1
    bFlag = False
    pose = np.zeros([1,6])
    count = 1 #required because start at start
    for i in range(0, numPoses, 3):
        #just because, forward sweep 1st direction, back-and-forth sweep 2nd direction

        # add pose to back away from wall
        pose[0,normalIdx] -= backOff
        poses[i,:] = pose
        # next location
        if bFlag:
            pose[0,sweepIdx[1]] += dPos[sweepIdx[1]]
            pose[0,sweepIdx[1]+3] += dAng[sweepIdx[1]]
            bFlag = False
        else:
            pose[0,sweepIdx[0]] += bDir * dPos[sweepIdx[0]]
            pose[0,sweepIdx[0]+3] += bDir * dAng[sweepIdx[0]]
        poses[i+1,:] = pose
        # approach wall
        pose[0,normalIdx] += backOff
        poses[i+2,:] = pose

        count += 1
        if count >= numSweep[sweepIdx[1]]: #modulo operator --> gives remainder of division
            bDir = -1 * bDir #invert direction
            bFlag = True
            count = 0 #shouldn't be 1, isn't a typo

    out = initPose + poses
    #pdb.set_trace()
    return out.tolist() #broadcasting


def main():
    try:
        # -------- input for grid movement -----------
        # note: chisel default orientation is pointed straight up
        # global position - "bottom corner"
        x0 = -759/1000.0
        y0 = -450/1000.0
        z0 = -309/1000.0
        # rotation about global axes - in radians
        wx0 = math.pi/2.0
        wy0 = 0.0
        wz0 = 0.0
        #
        sweepAxes = [1, 0, 1] #sweep across y & z directions
        #
        maxAngle = 0 #45.0/360.0 * math.pi #in radians
        maxPos = [0.1, 0, 0.1] #final position - thickness of box in each dir
        numSweep = [2, 0, 2] #how many positions
        backOff = -0.05 #how much to back off the wall



        # ----------------------------------------------
        # ------------- rest is automated --------------
        # checks
        bCheck = False
        msg=''
        if sum(sweepAxes) != 2:
            bCheck = True
            msg += 'sweepAxes: Specify two axes to sweep\n'
        if np.array(numSweep)[np.array(sweepAxes)==0] != 0:
            bCheck = True
            msg += 'numSweep: Can\'t sweep in wrong direction'

        if bCheck:
            moveit_commander.roscpp_shutdown()
            print('Setup error. Exiting without execution')
            print('msg: \n'+msg)
            return

        # test 4 corners of box for physical reachability


        # --------------all good --> execute! -----------
        #-------------- generate poses -----------
        initPose = [x0, y0, z0, wx0, wy0, wz0]
        poseList = genPoses(initPose, maxPos, maxAngle, numSweep, backOff, sweepAxes)

        #init
        #moveit_commander.roscpp_initialize(sys.argv)
        #rospy.init_node("grid_move", anonymous=True, disable_signals=True)
        #robot = moveit_commander.RobotCommander()
        #scene = moveit_commander.PlanningSceneInterface()
        #group = moveit_commander.MoveGroupCommander("chisel")
        #group.set_pose_reference_frame("base_link")
        #group.set_planning_time(5) #seconds

        #init socket
        HOST="192.168.1.100"
        PORT=30002
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((HOST,PORT))



        #sleep -- required to work
        #rospy.sleep(1)
        #remove objects
        #scene.remove_world_object("ground")
        #scene.remove_world_object("rockWall")
        #rospy.sleep(1)

        #add ground to scene
        #ground_pose = geometry_msgs.msg.PoseStamped()
        #ground_pose.header.frame_id = "base_link" #robot.get_planning_frame()
        #ground_pose.pose.position.x = 0
        #ground_pose.pose.position.y = 0
        #ground_pose.pose.position.z = -.75
        #ground_pose.pose.orientation.w = 1.0
        #scene.add_box('ground',ground_pose, (10,10,1))
        #add rock wall to scene
        #wall_pose = geometry_msgs.msg.PoseStamped()
        #wall_pose.header.frame_id = "base_link" #robot.get_planning_frame()
        #wall_pose.pose.position.x = 0
        #wall_pose.pose.position.y = 1.1
        #wall_pose.pose.position.z = 0.75
        #wall_pose.pose.orientation.w = 1.0
        #scene.add_box('rockWall',wall_pose, (2,0.2,2))
        #sleep for it to take affect
        #rospy.sleep(1)

        #group.set_pose_target(initPose)
        #plan = group.plan()
        #print(group.get_current_pose())
        #print(plan)
        #check valid

        #move to start
        #group.go(wait=True)
        #rospy.sleep(1)

        #print(group.get_current_pose())
        #print('pose ref frame: ',group.get_pose_reference_frame())
        p1 = initPose
        p1[1] -= 0 #5/1000.0
        moveApproach = "movel(p[{0},{1},{2},{3},{4},{5}])\n".format(p1[0],p1[1],p1[2],p1[3],p1[4],p1[5])
        s.send(moveApproach)
        time.sleep(3.0)
        forceMode = \
        "force_mode(p[{0},{1},{2},{3},{4},{5}],[0,0,1,0,0,0],[0,0,20.0,0,0,0],2,[.1,.1,0.15,.31,.31,.31])\nsleep(1.0)\nend_force_mode()\n".format(p1[0],p1[1],p1[2],p1[3],p1[4],p1[5])
        s.send(forceMode)
        # wait = "sleep(2.0)\n"
        # s.send(wait)
        # time.sleep(2.0)
        # s.send("end_force_mode()\n")
        # forceMove = \
          # "movel(p[{0},{1},{2},{3},{4},{5}])\n".format(p1[0],p1[1],p1[2]+.1,p1[3],p1[4],p1[5])
        # s.send(forceMove)
        # time.sleep(1.0)

        # s.send("end_force_mode()\n")

        totalPose = len(poseList)

        rs = raw_input("Enter yes to execute loop\n")
        if rs != 'yes':
            return
        # main loop
        for i in range(0, totalPose, 1):
            #extract pose
            #pose = poseList[i] # 6-element list
            #set up new pose
            #group.set_pose_target(pose)
            #internal planning
            #plan = group.plan()
            #check for validity

            #execute movement on robot
            #group.go(wait=True)
            #rospy.sleep(0.05)

            #TO DO:
            p1 = poseList[i]
            moveApproach = "movel(p[{0},{1},{2},{3},{4},{5}])\n".format(p1[0],p1[1],p1[2],p1[3],p1[4],p1[5])
            s.send(moveApproach)
            time.sleep(2.0)

            #turn drill on

            #exert constant force on block
            forceMode = \
            "force_mode(get_actual_tcp_pose(),[0,0,1,0,0,0],[0,0,10,0,0,0],2,[.01,.01,0.1,.01,.01,.01])\n"
            s.send(forceMode)
            time.sleep(1.0)

            s.send("end_force_mode()\n")




            #turn drill off

        print('Loop Execution Complete')

        #keep node running completion
        #rospy.spin()
        #shut down controller
        s.close()
        #moveit_commander.roscpp_shutdown()
    except KeyboardInterrupt:
        print('\nShutting Down - Now')
        #rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
