#!/usr/bin/env python
import time
import sys
import copy
import roslib
import rospy
import actionlib
#from control_msgs.msg import *
#from trajectory_msgs.msg import *
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
Q1 = [2.2,0,-1.57,0,0,0]
Q2 = [1.5,0,-1.57,0,0,0]
Q3 = [1.5,-0.2,-1.57,0,0,0]

client = None

def dontuse():
    group.set_pose_reference_frame("tip_link")
    #move in rotation axes (z-dir)
    print('before execution')
    print(group.get_current_pose())
    print('pose ref frame: ',group.get_pose_reference_frame())
    group.set_pose_target([0.0, 0.0, 0.05, 0, 0, 0])
    group.set_pose_reference_frame("world")
    # -pi/2, -pi, 0
    plan = group.plan()

    group.go(wait=True)
    rospy.sleep(0.05)
    
    print('after execution')
    print(group.get_current_pose())

def main():
    global client
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group = moveit_commander.MoveGroupCommander("chisel")
        #sleep for w/e fucken reason -- required to work
        rospy.sleep(1)
        #remove objects
        scene.remove_world_object("ground")
        scene.remove_world_object("gbox")
        rospy.sleep(1)

        #add ground to scene
        ground_pose = geometry_msgs.msg.PoseStamped()
        ground_pose.header.frame_id = "world"
        #ground_pose.header.stamp = 
        #scene.add_plane('ground', ground_pose)
        #add ground box to scene

        ground_pose.pose.position.x = -5
        ground_pose.pose.position.y = -5
        ground_pose.pose.position.z = -1
        ground_pose.pose.orientation.w = 1
        #gpose = 
        scene.add_box('gbox',ground_pose, (10,10,1))

        #group = moveit_commander.MoveGroupCommander("endeffector")
        #pose goal
        rospy.sleep(1)
        print('Object Names: ', scene.get_objects())

        pose_target = geometry_msgs.msg.Pose()
        #pose_target.orientation.y = 1.0
        pose_target.orientation.w = 1.0
        pose_target.position.x = .0
        pose_target.position.y = 1
        pose_target.position.z = .1



        x = 0.1
        y = 1
        z = 0.4
        wx = -3.14/2
        wy = 0
        wz = 0


        group.set_pose_target([x,y,z,wx,wy,wz])
        plan1 = group.plan()
        #print(plan1)
        print(robot.get_group_names())
        group.go(wait=True)



        


        #shut down controller
        moveit_commander.roscpp_shutdown()



    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
