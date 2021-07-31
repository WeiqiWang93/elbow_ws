#!/usr/bin/env python
# Copyright offworld.ai 2018

import time, argparse
import sys
import copy
import math
import numpy as np
import pdb
import os
import threading
import math3d as m3d
import urx, URBasic
import logging
import rospy
from rospy_message_converter import message_converter
import actionlib
#
from control_msgs.msg import *
from trajectory_msgs.msg import *
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import WrenchStamped, TransformStamped
from sensor_msgs.msg import JointState 
from bot_common_ros.srv import ArmCmdSrv, ArmSetupSrv

#
import tf as ros_tf
import tf.msg # can be accessed with ros_tf.msg


#
import URBasic
import urx
from bot_common_ros.ur_control import URMiddleWare, getHorizontalOrient, force_stop2, getFloatArrMsg, checkBox
from bot_common_ros.ur_utils import rosParamToClassObject, movel, movels, movej, movejs, force_mode, get_force, get_speed, ROStoM3d, ROStoM3d_orient, \
                     get_pose, get_joints, fillTF

VEL                 = 0.5
ACC                 = 2.0 * VEL
CONSOLE_LOG         = True

class URGazeboDriver(object):
    """A Class that implements a driver functionality to interact with the ur bots in gazebo
    """

    def __init__(self, run_as_api=False):

        self.__version__        = "1_0"
        self.run_as_api = run_as_api
        self.logger             = logging.getLogger()
        self.logger.disabled    = not CONSOLE_LOG
        self.ARM_RUNNING        = True
        self.arm_params = {}
        self.input_param_path        = None
        self.arm_name           = "arm_{}"
        self.nominal_tcp_to_tcp = m3d.Transform()
        self.use_data_publisher = True
        
        # ROS action clients
        self.ur_gz_client = None
        # Threading
        self.data_thread = None
        

    def initialize_arm(self, arm_param_path):
        """ bring up the UR Gazebo. Currently does not actually brings up the robot as it is already done via launch file.
        Simply used to sets up the communication interface between the robot in gazebo and outside"""

        # Read values from ROS param server to get arm properties. Here we are only concerned about 'name' as this becomes the prefix for ROS srvs
        # hosted by this node. Is this the right place to read ROS params? 
       
        self.arm_params = rospy.get_param(arm_param_path)    
       
        if "name" in self.arm_params:
            self.arm_name = self.arm_params["name"][1:]
        else:
            self.arm_name = "arm"

        self.joint_names = self.arm_name["joint_names"]
       
        # ROS publishers
        self.pub_joints = rospy.Publisher('/' +self.arm_name+'/joint_states', JointState, queue_size=5)
        rospy.Subscriber('joint_states', JointState, self.joint_states_callback)

        # Action Clients
        self.ur_gz_client = actionlib.SimpleActionClient('/arm_controller_'+self.arm_name +'/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo('{}: Waiting for server...'.format(self.arm_name))
        self.ur_gz_client.wait_for_server()
        rospy.loginfo('{}: Connected to server.'.format(self.arm_name))

            # start secondary threads           
        self.data_thread = threading.Thread(name=self.arm_name+'_data_thread', target=self.data_publisher)      
        self.data_thread.setDaemon(True) #will make the thread close when this program ends
        self.data_thread.start()

        return True
   
    
    def movejs(self, joints_list=[]):
        
        kwargs2 = {}         
        traj = kwargs['joints_list']
        vel = kwargs['velocity']
        
        # Code to move the arm in gazebo.
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = self.joint_names
        g.trajectory.points = [JointTrajectoryPoint(positions=traj[i], velocities=[vel]*6, time_from_start=rospy.Duration(i*2.0)) for i in range(1, len(traj))]
        self.ur_gz_client.send_goal(g)

        try:
            self.ur_gz_client.wait_for_result()
        except KeyboardInterrupt:
            self.ur_gz_client.cancel_goal()
            raise
                
        return True


    #callback function: when a joint_states message arrives, save the values
    def joint_states_callback(self, msg):
        
        msg_pub = JointState()
        msg_pub.header = msg.header
        msg_pub.header.stamp = rospy.Time.now()
        var = list(msg.name)
        start_index = 0
        if self.arm_name == "saw":
            start_index = 6
        joints_indices = [list(msg.name).index(jname) for jname in self.joint_names]
        msg_pub.name = self.joint_names
        msg_pub.position = [list(msg.position)[j] for j in joints_indices]
        msg_pub.velocity = [list(msg.velocity)[j] for j in joints_indices]
        
        self.pub_joints.publish(msg_pub)
       
    
    def data_publisher(self):
        """ publishes tf and joints for this UR10. Follows same standard as ur_modern_driver
        get_pose
        get_speed
        get_joints
        get_force
        gate_status
        """
        pub_tf = rospy.Publisher('/tf', ros_tf.msg.tfMessage, queue_size=1)
        pub_tcp_speed = rospy.Publisher('/' +self.arm_name+'/tcp_speed', Float32MultiArray, queue_size=5)
        pub_tcp_force = rospy.Publisher('/' +self.arm_name+'/wrench', WrenchStamped, queue_size=5)
        pub_is_gate_open = rospy.Publisher('/' +self.arm_name+'/is_gate_open', Bool, queue_size=5)
        # pub_io_states = rospy.Publisher('/io_states', JointState, queue_size=5)

        rospy.loginfo("Data Publisher ready and starting.")

        # static transforms TODO: put nominal-tcp-to-tcp here (but then have to update it if tf changes from user)
        listener = ros_tf.TransformListener()
        while not rospy.is_shutdown():
            if self.ARM_RUNNING and self.use_data_publisher:
                # TCP pose in base frame
                t = TransformStamped()
                t.header.frame_id = self.arm_name+"_base"
                try:
                    (trans,rot) = listener.lookupTransform('/' +self.arm_name+'_base', '/' +self.arm_name+'_tip_link', rospy.Time(0))
                except (ros_tf.LookupException, ros_tf.ConnectivityException, ros_tf.ExtrapolationException):
                    continue
                t.header.stamp = rospy.Time.now()
                t.child_frame_id = self.arm_name+"_TCP"
                rot_m3d_orient = ROStoM3d_orient(rot)
                fillTF(trans, rot_m3d_orient.quaternion, t)
                tfm = ros_tf.msg.tfMessage([t])
                pub_tf.publish(tfm)

            

if __name__ == "__main__":
    """ start the class and spin rospy
    """
    rospy.init_node("ur_gazebo_driver", anonymous=True)
    run_test = rospy.get_param("~run_test", False)

    try:
        ur_gazebo_driver = URGazeboDriver()
        
        time.sleep(1.0)

        rospy.loginfo("ur_gazebo_driver: Start spinning forever now.")
        rospy.spin()
        
    finally:
        if not rospy.is_shutdown(): rospy.signal_shutdown("End of Program")