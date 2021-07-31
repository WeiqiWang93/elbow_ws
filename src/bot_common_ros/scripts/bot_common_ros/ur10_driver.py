#!/usr/bin/env python
# Copyright offworld.ai 2018

import time, argparse
import sys
import copy
import math
import numpy as np
import pdb
import os
import enum
import threading
import math3d as m3d
import urx, URBasic
import logging
import rospy
from rospy_message_converter import message_converter
import rosparam
from rospkg import RosPack

#
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray, Bool, Float32
from geometry_msgs.msg import WrenchStamped 
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState 
from bot_common_ros.srv import ArmCmdSrv
from bot_common_ros.srv import ArmSetupSrv
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import FollowJointTrajectoryGoal
from control_msgs.msg import FollowJointTrajectoryAction
import actionlib
from copy import deepcopy

#
import tf as ros_tf
import tf2_ros
import tf.msg # can be accessed with ros_tf.msg
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from tf.transformations import euler_matrix

#
import URBasic
import urx
from bot_common_ros.ur_control import URMiddleWare
from bot_common_ros.ur_control import getHorizontalOrient
from bot_common_ros.ur_control import force_stop2
from bot_common_ros.ur_control import getFloatArrMsg
from bot_common_ros.ur_control import checkBox
from bot_common_ros.ur_control import MotionPlanner
from bot_common_ros.ur_utils import rosParamToClassObject
from bot_common_ros.ur_utils import movel
from bot_common_ros.ur_utils import movels
from bot_common_ros.ur_utils import movej
from bot_common_ros.ur_utils import movejs
from bot_common_ros.ur_utils import force_mode
from bot_common_ros.ur_utils import get_force
from bot_common_ros.ur_utils import get_speed
from bot_common_ros.ur_utils import ROStoM3d
from bot_common_ros.ur_utils import m3d_to_ros 
from bot_common_ros.ur_utils import get_pose
from bot_common_ros.ur_utils import get_joints 
from bot_common_ros.ur_utils import fillTF
from bot_common_ros.ur_utils import rotate6V
from bot_common_ros.ur_utils import geometry_msg_to_m3d
from bot_common_ros.ur_utils import pose_dict_to_msg
from bot_common_ros.ur_utils import list_to_pose_msg
from bot_common_ros.arm_server import ArmServer
from bot_overseer_api import OverseerAPI

VEL                 = 0.5
ACC                 = 2.0 * VEL
CONSOLE_LOG         = True


class ARMState(enum.Enum):
    """Defines possible state the plugin is in during scanning
    """
    IDLE = 'IDLE'
    INITIALIZED = 'INITIALIZED'
    INCOMPLETE = 'INCOMPLETE'
    COMPLETE = 'COMPLETE'


class URGazeboDriver(object):
    """A Class that implements a driver functionality to interact with the ur bots in gazebo
    """

    def __init__(self, arm_name, joint_names):
        
        self.arm_name = arm_name
        self.joint_names = joint_names
        self.joint_state = []
        self.tf_listener = ros_tf.TransformListener()

        self.js_sub = rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)

        # Action Clients
        self.ur_gz_client = actionlib.SimpleActionClient('/arm_controller_'+self.arm_name +'/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo('{}: Waiting for server...'.format(self.arm_name))
        self.ur_gz_client.wait_for_server()
        rospy.loginfo('{}: Connected to server.'.format(self.arm_name))
        
       
    
    def movejs(self, **kwargs):
        
        vel = kwargs['velocity']

        if len(kwargs['joints_list']) == 1:
            traj = [self.joint_state] + kwargs['joints_list']
        else:
            traj = kwargs['joints_list']
    
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = self.joint_names
        g.trajectory.points = [JointTrajectoryPoint(positions=traj[i], velocities=[vel]*6, time_from_start=rospy.Duration(i*0.75)) for i in range(1, len(traj))]
        
        self.ur_gz_client.send_goal(g)
        try:
            self.ur_gz_client.wait_for_result()
        except KeyboardInterrupt:
            self.ur_gz_client.cancel_goal()
            raise
                
        return True


    
    def joint_states_callback(self, msg):
        """ callback function: when a joint_states message arrives, save the values"""
        
        msg_pub = JointState()
        msg_pub.header = msg.header
        msg_pub.header.stamp = rospy.Time.now()
        var = list(msg.name)
        
        joints_indices = [list(msg.name).index(jname) for jname in self.joint_names]
        msg_pub.name = self.joint_names
        self.joint_state = [list(msg.position)[j] for j in joints_indices]


    def get_pose(self):

        try:
            (trans,rot) = self.tf_listener.lookupTransform('/' +self.arm_name+'_base', '/' +self.arm_name+'_tip_link', rospy.Time(0))
            return ROStoM3d(trans, rot)
        except (ros_tf.LookupException, ros_tf.ConnectivityException, ros_tf.ExtrapolationException):
            return m3d.Transform()
        
        

class UR10Driver(object):
    """A generic class to interface with UR10

    This provides apis to initialize communication with an URx arm
    and initialize control parameters.

    This only allows connection to 1 arm. The node must be created (with input arguments) once per arm.

    Future work: turn into nodelet which is managed by arm_manager
    TODO: consider turning arm_client+application into a nodelet, under the same manager as ur10_driver,
    this reduces comm time, and also allows 1 action-server to be used, instead of 2 (even though right now,
    it's actually a service, not a full action-server)
    https://github.com/neka-nat/rospy_message_converter 

    TODO: incorporate ur_modern_driver by using its /urscript topic: https://github.com/ros-industrial/ur_modern_driver/issues/162#issuecomment-369843037
    """

    def __init__(self):
       
        self.use_rope_rob       = True
        self.use_urx            = True
        self.rob                = None
        self.ropeRob            = None
        self.e_series = False

        self.goal_joints = None
        self.goal_pose = None
        self.state = ARMState.IDLE.name
        self.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        self.safety_gate = threading.Event()
        self.sim = False
        self.in_force_mode = False
        self.force_cmd_ft = [0.0]*6
        self.force_cmd_limits = [0.0]*6
        self.target_moment = [0.0]*6

        rospack = RosPack() 
        self.extrinsics_file = os.path.join(rospack.get_path('bot_digger_description'), 'extrinsics.yaml')
        self.tf_listener = ros_tf.TransformListener()
        self.lock = threading.Lock()

        # Force command ROS Srv
        self.force_cmd_srv = rospy.Service('/ur10/force_cmd', ArmCmdSrv, self.force_cmd_srv_cb)


    def initialize_robot(self, **kwargs):
        """ Initialize UR10
            Input args: TCP, Tool Mass, Tool CG, robot_ip, use_force_sensor
        """
        arm_name = kwargs['arm_name']

        joint_names = [arm_name + "_" + jn for jn in self.joint_names]
    
        self.motion_planner = MotionPlanner(arm_name, arm_name + "_base", joint_names)
        
        tcp = m3d.Transform()
        mt_to_base = m3d.Transform()
        # tcp.pos.x, tcp.pos.y, tcp.pos.z = kwargs['tcp'][0], kwargs['tcp'][1], kwargs['tcp'][2]
        # tcp.set_orient = m3d.Orientation(m3d.quaternion.UnitQuaternion(1, 0, 0, 1))
        tcp.pos = rosparam.get_param('/extrinsics/' + kwargs['tcp_uri'] + '/TCP/pos')
        # tcp.pos = kwargs['tcp_position']
        rpy = rosparam.get_param('/extrinsics/' + kwargs['tcp_uri'] + '/TCP/rpy')
        tcp.orient = euler_matrix(rpy[0], rpy[1], rpy[2])[0:3, 0:3]

        # Read from Arm params
        mt_to_base.pos = kwargs['mt_to_base_position']
        mt_to_base.orient = euler_matrix(kwargs['mt_to_base_orient'][0], kwargs['mt_to_base_orient'][1], kwargs['mt_to_base_orient'][2])[0:3, 0:3]

        mass = kwargs['mass']
        cog = kwargs['cog']
        self.robot_ip = kwargs['robot_ip']
        
        use_force_sensor = kwargs['use_force_sensor']

        tcp_kwargs = rosparam.get_param('/extrinsics/' + kwargs['tcp_uri'])
        if tcp_kwargs.has_key('e_series'):
            self.e_series = tcp_kwargs['e_series']

        if None in [tcp, mass, cog, self.robot_ip, mt_to_base, use_force_sensor]:
            rospy.loginfo("Invalid arguments")
        
        self.MOUNT_TO_BASE = mt_to_base
        pose = Pose()
        pose.orientation.w = 1.0
        self.NOMINAL_TCP_TO_SENSOR = geometry_msg_to_m3d(pose)
        self.arm_name = arm_name
        self.use_force_sensor = use_force_sensor

        self.is_remote_control = True
        
        self.pub_static_tf2(tcp.inverse.pos, m3d_to_ros(tcp.inverse).orientation, arm_name + "_TCP", arm_name + "_NOMINAL_TCP")
        
        if kwargs.has_key('sim'):
            if kwargs['sim']:
                self.sim = True
                self.sim_driver = URGazeboDriver(arm_name, joint_names)
                self.use_force_sensor = False

                if self.state == 'IDLE':
                    self.is_arm_active = True
                    self.ros_pub_thread = threading.Thread(name='ur10_monitor_thread', target=self.ros_publisher)
                    self.ros_pub_thread.setDaemon(True) #will make the thread close when this program ends
                    self.ros_pub_thread.start()

                self.state = ARMState.INITIALIZED.name
                return 

        # ur-interface pkg
        if self.ropeRob is None:
            robotModel = URBasic.robotModel.RobotModel()
            if self.e_series:
                self.ropeRob = URBasic.urScriptExt.UrScriptExt(host=self.robot_ip, robotModel=robotModel, hasForceTorque=False)
            else:
                self.ropeRob = URBasic.urScriptExt.UrScriptExt(host=self.robot_ip, robotModel=robotModel, hasForceTorque=use_force_sensor)

        # # python-urx pkg
        if self.rob is None:

            self.rob = urx.Robot(self.robot_ip, use_rt=True)
            # turn on UR10
            self.power_on_robot()

        pose_vector = tcp
        self.nominal_tcp_to_tcp = m3d.Transform(pose_vector).inverse
        #
        self.rob.set_tcp(tcp)
        self.rob.set_payload(mass, cog)

        self.ropeRob.set_tcp(tcp)
        self.ropeRob.set_payload(mass, cog)
    
        if self.state == ARMState.IDLE.name:
            self.is_arm_active = True
            self.safety_gate.set() # Gate open
            
            self.monitor_thread = threading.Thread(name='ur10_monitor_thread', target=self.arm_monitor)
            self.monitor_thread.setDaemon(True) #will make the thread close when this program ends
            self.monitor_thread.start()

            self.ros_pub_thread = threading.Thread(name='ur10_monitor_thread', target=self.ros_publisher)
            self.ros_pub_thread.setDaemon(True) #will make the thread close when this program ends
            self.ros_pub_thread.start()

        self.state = ARMState.INITIALIZED.name

    def reinitialize(self):
        """ This method executes the following:
            1) Checks the is_remote_control flag
            2) Re-initializes the objects of classes from ur-interface and python-urx modules
            3) Kills the arm monitor and ros publsisher threads and restarts them.
            4) Updates the is_remote_control flag

            This method is executed whenever the operator switch between remote control and local control modes
        """ 
        
        self.state = ARMState.INCOMPLETE.name

        if self.state == 'IDLE':
            rospy.logwarn('Arm not initialized. Cannot reinitialize')
            self.state = ARMState.COMPLETE.name
            return False
        
        if self.is_remote_control or not self.e_series:
            self.state = ARMState.COMPLETE.name
            return True

        rospy.logwarn('Waiting for switch to remote control mode on Polyscope.....')
        self.lock.acquire()

        self.ropeRob.robotConnector.RealTimeClient.Disconnect()
        robotModel = self.ropeRob.robotConnector.RobotModel
        self.ropeRob.robotConnector.RealTimeClient = URBasic.realTimeClient.RealTimeClient(robotModel)

        while not self.ropeRob.robotConnector.DashboardClient.remote_control_mode():
            time.sleep(1.0)
        
        self.rob = None
        self.rob = urx.Robot(self.robot_ip, use_rt=True)
        
        robotModel = URBasic.robotModel.RobotModel()
        robotModel.ipAddress = self.robot_ip
       
        self.is_arm_active = False
        self.monitor_thread.join()
        self.ros_pub_thread.join()

        self.ropeRob.robotConnector.close()
        self.ropeRob = URBasic.urScriptExt.UrScriptExt(host=self.robot_ip, robotModel=robotModel, hasForceTorque=False)
        
        self.lock.release()

        self.is_remote_control = True
        self.is_arm_active = True
        self.monitor_thread = threading.Thread(name='ur10_monitor_thread', target=self.arm_monitor)
        self.monitor_thread.setDaemon(True) #will make the thread close when this program ends
        self.monitor_thread.start()

        self.ros_pub_thread = threading.Thread(name='ur10_monitor_thread', target=self.ros_publisher)
        self.ros_pub_thread.setDaemon(True) #will make the thread close when this program ends
        self.ros_pub_thread.start()

        while not self.ropeRob.robotConnector.RTDE.isRunning():
            time.sleep(0.1)

        rospy.logwarn('UR Driver reset')

        self.state = ARMState.COMPLETE.name
        return True


    def update_tcp_position(self, arm_uri):
        """ Method to read the correct TCP values from the control box and update the extrinsics file
            accordingly.
            Input:
                arm_uri: string value indicating the path to the position extrinsics parameters.
        """

        (trans,rot) = self.tf_listener.lookupTransform('/' +self.arm_name+'_base', '/' +self.arm_name+'_tool0', rospy.Time(0))
        tf_base_to_tool0 = ROStoM3d(trans, rot)

        tf = tf_base_to_tool0.inverse * self.robot_pose
        
        rosparam.load_file(self.extrinsics_file, default_namespace='/extrinsics')
        pos = rosparam.get_param('/extrinsics/' + arm_uri + '/TCP/pos')
        
        pos[0] = float(tf.pos.x)
        pos[1] = float(tf.pos.y)
        pos[2] = float(tf.pos.z)

        rosparam.set_param_raw('/extrinsics/' + arm_uri + '/TCP/pos', pos)
        
        rosparam.dump_params(self.extrinsics_file, param='/extrinsics/')

        return True
    

    def update_tcp_orientation(self, arm_uri, rpy=[], update_roll=False, update_pitch=False, update_yaw=False):
        """ Method to  compute  correct TCP rotation wrt the tool0 frame of the arm given that the
            TCP's (or tip_link) Z-axis is physically aligined with the YZ plane of the given reference frame.
            
            Input:
                arm_uri: string value indicating the path to the position extrinsics parameters.
                rpy: list containing rotations about base X, Y and Z axes to transfrom base frame to the 
                reference frame. 
        """
        
        while not rospy.is_shutdown():
            
            rospy.sleep(0.1)
            try:
                (trans, rot) = self.tf_listener.lookupTransform('/' +self.arm_name+'_base', '/' +self.arm_name+'_tool0', rospy.Time(0))
                tf_base_to_tool0 = ROStoM3d(trans, rot)
                break
            except:
                rospy.logwarn("Looking for transfom from arm base to tool0")

        tf_base_to_tcp_correct = m3d.Transform()
        tf_base_to_tcp_correct.orient.rotate_xt(rpy[0])
        tf_base_to_tcp_correct.orient.rotate_yt(rpy[1])
        tf_base_to_tcp_correct.orient.rotate_zt(rpy[2])

        tf_tool0_to_tcp_correct = tf_base_to_tool0.inverse * tf_base_to_tcp_correct
        
        quat = m3d_to_ros(tf_tool0_to_tcp_correct).orientation
        euler = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        
        rosparam.load_file(self.extrinsics_file, default_namespace='/extrinsics')
        rpy = rosparam.get_param('/extrinsics/' + arm_uri + '/TCP/rpy')
        
        if update_roll:
            rpy[0] = euler[0]

        if update_pitch:
            rpy[1] = euler[1]

        if update_yaw:
            rpy[2] = euler[2]
        
        rosparam.set_param_raw('/extrinsics/' + arm_uri + '/TCP/rpy', rpy)
        
        rosparam.dump_params(self.extrinsics_file, param='/extrinsics/')

        return True

       

    
    def power_on_robot(self):
        """
        Robot Modes		
        Mode		Description
        -1		ROBOT_MODE_NO_CONTROLLER
        0		ROBOT_MODE_DISCONNECTED
        1		ROBOT_MODE_CONFIRM_SAFETY
        2		ROBOT_MODE_BOOTING
        3		ROBOT_MODE_POWER_OFF
        4		ROBOT_MODE_POWER_ON
        5		ROBOT_MODE_IDLE
        6		ROBOT_MODE_BACKDRIVE
        7		ROBOT_MODE_RUNNING
        8		ROBOT_MODE_UPDATING_FIRMWARE
        """
        self.ropeRob.robotConnector.DashboardClient.ur_brake_release() # combines ur_power_on and ur_brake_release
        while not self.ropeRob.robotConnector.RobotModel.dataDir['robot_mode'] == 7:
            time.sleep(0.1)
        self.ARM_RUNNING = True

    def power_off_robot(self):
        """
        Robot Modes		
        Mode		Description
        -1		ROBOT_MODE_NO_CONTROLLER
        0		ROBOT_MODE_DISCONNECTED
        1		ROBOT_MODE_CONFIRM_SAFETY
        2		ROBOT_MODE_BOOTING
        3		ROBOT_MODE_POWER_OFF
        4		ROBOT_MODE_POWER_ON
        5		ROBOT_MODE_IDLE
        6		ROBOT_MODE_BACKDRIVE
        7		ROBOT_MODE_RUNNING
        8		ROBOT_MODE_UPDATING_FIRMWARE
        """
        self.ARM_RUNNING = False
        self.ropeRob.robotConnector.DashboardClient.ur_power_off()
        while not self.ropeRob.robotConnector.RobotModel.dataDir['robot_mode'] == 3:
            time.sleep(0.1)

    def close_connection(self):
        """ close down robot connection 
        """
        if self.rob is not None:
            self.rob.close()
        if self.ropeRob is not None:
            self.ropeRob.close()
        self.rob = None
        self.ropeRob = None
        self.is_arm_active = False       

    def stop(self, timeout=0.5):
        """ Bring arm to a quick smooth stop
        """
        self.state = ARMState.INCOMPLETE.name

        if self.rob is not None and self.ropeRob is not None:
            # force mode stop
            force_stop2(self.rob, self.ropeRob, timeout=timeout)
            self.ropeRob.end_force_mode()
            # position mode stop
            self.ropeRob.stopl(a=timeout)
            self.ropeRob.stop_joint_torques()

        self.state = ARMState.COMPLETE.name

    def pause(self):

        dbc = self.ropeRob.robotConnector.DashboardClient
        dbc.ur_pause()
        

    def resume(self):
        self.ropeRob.robotConnector.DashboardClient.ur_play()

    def getValidKwargs(self, func, argsDict):
        """ makes sure only the valid kwargs are kept for func """
        validArgs = func.func_code.co_varnames[:func.func_code.co_argcount]
        kwargsLen = len(func.func_defaults) # number of keyword arguments
        validKwargs = validArgs[-kwargsLen:] # because kwargs are last
        return dict((key, value) for key, value in argsDict.iteritems() 
                    if key in validKwargs)
    
    ############################### commands #################################
    def add_to_kwargs(self, kwargs, fct):
        """ add internal args to kwargs """
        kwargs['robot'] = self.ropeRob
        kwargs['URXrob'] = self.rob
        kwargs['tf'] = self.MOUNT_TO_BASE
        kwargs = self.getValidKwargs(fct, kwargs)
        return kwargs
    #
    def movel(self, **kwargs):
        
        fct = movel
        kwargs = self.add_to_kwargs(kwargs, fct)
        self.safety_gate.wait()
        fct(**kwargs)
    #
    def movels(self, **kwargs):

        if self.e_series:
            self.reinitialize()

        self.state = ARMState.INCOMPLETE.name
        kwargs2 = {}         
        kwargs2['poseList_base'] = []
        kwargs['acc'] = kwargs['acceleration']
        kwargs2['vel'] = kwargs['velocity']
        kwargs2['threshold'] = kwargs['threshold']
        kwargs2['wait'] = True
       
        for p in kwargs['poses']:
            pose = list_to_pose_msg(p)
            kwargs2['poseList_base'].append(geometry_msg_to_m3d(pose))
            
        fct = movels
        kwargs2 = self.add_to_kwargs(kwargs2, fct)
        self.safety_gate.wait()
        fct(**kwargs2)

        self.state = ARMState.COMPLETE.name
    #
    def movejs(self, **kwargs):

        if self.e_series:
            self.reinitialize()

        self.state = ARMState.INCOMPLETE.name

        if self.sim:
           
            self.sim_driver.movejs(**kwargs)
            self.state = ARMState.COMPLETE.name
            return 

        kwargs2 = {}         
        kwargs2['jointList'] = kwargs['joints_list']
        kwargs['acc'] = kwargs['acceleration']
        kwargs2['vel'] = kwargs['velocity']
        kwargs2['threshold'] = kwargs['threshold']
        kwargs2['wait'] = True

        if kwargs['joints_list']: 
            self.goal_joints = kwargs['joints_list'][-1]
            print self.goal_joints

        fct = movejs
        kwargs2 = self.add_to_kwargs(kwargs2, fct)
        self.safety_gate.wait()
        try:
            fct(**kwargs2)
        except:
            print "Robot exception"
            
        self.state = ARMState.COMPLETE.name

        # check if movejs is successful
        # if self.get_joint_goal_dist() <= kwargs['threshold']:
        #     self.state = "INCOMPLETE"
        #     print "INCOMPLETE"
        # else:
        #     print "COMPLETE"
        #     self.state = "COMPLETE"
    
    
    
    def force_cmd_srv_cb(self, request):
        """ Force command service callback. Service template is of type ArmCmdSrv uses following attributes
            geometry_msgs/Pose pose_base
            std_msgs/Float32MultiArray torque
            std_msgs/Float32MultiArray limits
            std_msgs/Float32MultiArray selection_vector
            bool setRemote
        """
        fct = force_mode
        kwargs2 = {}
        
        kwargs2['pose_base'] = geometry_msg_to_m3d(request.msg.pose_base)
        kwargs2['selection_vector'] = request.msg.selection_vector.data
        kwargs2['torque'] = request.msg.torque.data
        kwargs2['limits'] = request.msg.limits.data

        self.in_force_mode = True
        self.force_cmd_ft = request.msg.torque.data
        self.force_cmd_limits = request.msg.limits.data

        kwargs2 = self.add_to_kwargs(kwargs2, fct)
        self.safety_gate.wait()
        fct(**kwargs2)

        return True
        
    
    def force_mode(self, **kwargs):
        
        fct = force_mode
        kwargs2 = {}
        # pose = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose', kwargs['frame_transform'])
        pose = list_to_pose_msg(kwargs['frame_transform'])
        kwargs2['pose_base'] = geometry_msg_to_m3d(pose)
        kwargs2['selection_vector'] = kwargs['selection_vector']
        kwargs2['torque'] = kwargs['wrench']
        kwargs2['limits'] = kwargs['limits']

        self.in_force_mode = True
        self.force_cmd_ft = kwargs['wrench']
        self.force_cmd_limits = kwargs['limits']

        kwargs2 = self.add_to_kwargs(kwargs2, fct)
        self.safety_gate.wait()
        fct(**kwargs2)
    #
    def force_stop2(self, **kwargs):
        force_stop2(self.rob, self.ropeRob, timeout=0.5) # hard-set timeout
    #
    def stopj(self, **kwargs):
        self.ropeRob.stopj(a=0.5)
    #
    def end_force_mode(self, **kwargs):
        self.ropeRob.end_force_mode()
        self.in_force_mode = False
    #
    def reset_ft_sensor(self):
        """ Resets the force sensor readings to zero.
        """

        if not self.e_series:
            rospy.loginfo("Reseting FT300 sensor")
            prg = '''def prog():
        if(socket_open("127.0.0.1",63350,"acc")): 
            socket_send_string("SET ZRO","acc")
            sleep(0.1)
            socket_close("acc")
        end
    end
    '''
            self.ropeRob.robotConnector.RealTimeClient.SendProgram(prg)
        else:
            msg = 'FT sensor reset to zero'
            prg = 'zero_ftsensor()\ntextmsg({msg}\n'
            program_string = prg.format(**locals())
        
            self.ropeRob.robotConnector.RealTimeClient.Send(program_string)
            

    def set_speed(self, **kwargs):
        return

   
    def set_standard_digital_out(self, **kwargs):
        self.ropeRob.set_standard_digital_out(kwargs['id1'], kwargs['val'])


    ############################### getters #################################

    def get_pose(self):

        if self.sim:
            pose = m3d_to_ros(self.sim_driver.get_pose())
            pose_dict = message_converter.convert_ros_message_to_dictionary(pose)
            return pose_dict

        if self.rob and self.MOUNT_TO_BASE:
            
            pose = m3d_to_ros(get_pose(self.ropeRob, tf=self.MOUNT_TO_BASE))
            pose_dict = message_converter.convert_ros_message_to_dictionary(pose)
            return pose_dict
        else:
            return {}
    
    #
    def get_speed(self):
        return get_speed(self.ropeRob, tf=self.MOUNT_TO_BASE)
    #
    def get_joints(self):
        
        if self.sim:
            return self.sim_driver.joint_state

        return get_joints(self.ropeRob)

    def get_force(self):
        """ Method to get force values from Robotiq force sensor in the case of CB series
            and internal force sensor in the case of e-series arms.
            Force readings from e-series arm's internal force sensor is in mount frame (base frame in polyscope)
        """        
        if self.e_series:

            arr = np.array(self.ropeRob.robotConnector.RobotModel.dataDir['actual_TCP_force'])
            if arr is None:
                return None
            force = arr[0:3]
            torque = arr[3:6]
          
            return np.concatenate([force, torque])
        else:
            return get_force(self.ropeRob, tf=self.NOMINAL_TCP_TO_SENSOR)

    def get_robot_state(self):
        # if returns 0 we are in idle state, return 1 while robot is moving
        return (self.ropeRob.robotConnector.RobotModel.RuntimeState() and not self.ropeRob.robotConnector.RobotModel.StopRunningFlag())

    def get_arm_state(self):

        if not self.ropeRob:
            return ''

        mode_key = self.ropeRob.robotConnector.RobotModel.dataDir['robot_mode']
        mode = {}

        mode['-1']	=	'ROBOT_MODE_NO_CONTROLLER'
        mode['0']	=	'ROBOT_MODE_DISCONNECTED'
        mode['1']	=	'ROBOT_MODE_CONFIRM_SAFETY'
        mode['2']	=	'ROBOT_MODE_BOOTING'
        mode['3']	=	'ROBOT_MODE_POWER_OFF'
        mode['4']	=	'ROBOT_MODE_POWER_ON'
        mode['5']	=	'ROBOT_MODE_IDLE'
        mode['6']	=	'ROBOT_MODE_BACKDRIVE'
        mode['7']	=	'ROBOT_MODE_RUNNING'
        mode['8']	=	'ROBOT_MODE_UPDATING_FIRMWARE'

        return mode[str(mode_key)]
    
    def get_state(self):
        """ Plugin process status getter
        """
        return self.state
        
    def get_safety_status(self):
        rc = self.ropeRob.robotConnector.RobotModel
        return rc.SafetyStatus()
    
    def get_safety_status_tm(self):

        if not self.ropeRob:
            return ''
        result = self.get_safety_status()
        
        ss_flags = [result.NormalMode, result.ReducedMode, result.ProtectiveStopped, result.RecoveryMode, result.SafeguardStopped, result.SystemEmergencyStopped,\
                    result.RobotEmergencyStopped,\
                    result.EmergencyStopped,\
                    result.Violation,\
                    result.Fault,\
                    result.StoppedDueToSafety]

        ss_names = ['NormalMode', 'ReducedMode', 'ProtectiveStopped', 'RecoveryMode', 'SafeguardStopped', 'SystemEmergencyStopped',\
                    'RobotEmergencyStopped',\
                    'EmergencyStopped',\
                    'Violation',\
                    'Fault',\
                    'StoppedDueToSafety']
    
        return ss_names[ss_flags.index(True)]


    def get_joint_goal_dist(self):
        """ Distance between goal in joint space (via movejs or movejs)
            and current joints
        """
        self.lock.acquire()
        try:
            if self.goal_joints:
                a = np.array(self.get_joints())
                b = np.array(self.goal_joints)
                self.lock.release()
                return np.linalg.norm(a-b)
        except:
            raise
        
        self.lock.release()
        
    
            

    def get_pose_goal_dist(self):
        """ Distance between goal in pose space (via movels or movel)
            and current pose
        """
        if self.goal_pose != None:
            curr_pose = get_pose(self.rob, tf=self.MOUNT_TO_BASE)
            dist = self.goal_pose.pos.dist(curr_pose.pos)
            
            return dist
    
    def get_force_cmd_ft(self):

        return self.force_cmd_ft

    def get_force_cmd_limits(self):

        return self.force_cmd_limits

    def get_target_moment(self):

        return self.target_moment

    def arm_monitor(self):
        
        self.reset_ft_sensor()

        counter = 0
        
        while self.is_arm_active:

            # force cmd tm
            if not self.in_force_mode:
                self.force_cmd_ft = [0.0]*6
                self.force_cmd_limits = [0.0]*6

            # get target moment
            self.target_moment = list(self.ropeRob.get_target_moment())
            # Monitor safety
            safetyMode = self.get_safety_status()

            if not safetyMode.NormalMode :
                self.safety_cb()

            # Check at lower frequency
            counter += 1

            if counter >= 10:
                counter = 0
            
                if self.is_remote_control and self.e_series:
                    if not self.ropeRob.robotConnector.DashboardClient.remote_control_mode():
                        self.is_remote_control = False

            time.sleep(0.1)

            

    def safety_cb(self):
        
        msg_safety_event = Bool()
        msg_safety_event.data = True
        
        dbc = self.ropeRob.robotConnector.DashboardClient
        self.safety_gate.clear() # closes the gate
        print('Safety Event: Locked' if not self.safety_gate.isSet() else 'Safety Event: Unlocked')
        dbc.ur_close_popup() #in case a 'force_mode: orientation deviation' occurs
        dbc.ur_close_safety_popup()
        dbc.ur_unlock_protective_stop()
        timeSleep = 0.1
        time.sleep(timeSleep)
        self.safety_gate.set() # opens the gate        
        print('Safety Event: Locked' if not self.safety_gate.isSet() else 'Safety Event: Unlocked')


    def ros_publisher(self):

        pub_wrench = rospy.Publisher('/' + self.arm_name +'/wrench', WrenchStamped, queue_size=5)
        pub_force = rospy.Publisher('/' + self.arm_name +'/force_magnitude', Float32, queue_size=1)
        pub_joints = rospy.Publisher('/' + self.arm_name +'/joint_states', JointState, queue_size=5)
        pub_tf = rospy.Publisher('/tf', ros_tf.msg.tfMessage, queue_size=1)
        pub_force_cmd_ft = rospy.Publisher('/' + self.arm_name +'/force_cmd_ft', Float32MultiArray, queue_size=1)
        pub_force_cmd_limits = rospy.Publisher('/' + self.arm_name +'/force_cmd_limits', Float32MultiArray, queue_size=1)
        pub_target_moments = rospy.Publisher('/' + self.arm_name +'/target_moment', Float32MultiArray, queue_size=1)
        pub_joint_torques = rospy.Publisher('/' + self.arm_name +'/joint_torques', Float32MultiArray, queue_size=1)

        self.ropeRob.stop_joint_torques()
        rospy.sleep(1.0)
        # self.ropeRob.start_joint_torques()
        # self.ropeRob.start_jt_thread()

        # Nominal-TCP to TCP
        # TODO: change this to static transform outside of while loop

        while self.is_arm_active:

            time.sleep(0.1)
            
            if not self.sim:
                self.robot_pose = get_pose(self.ropeRob, tf=self.MOUNT_TO_BASE) # self.get_pose()
            else:
                self.robot_pose = self.sim_driver.get_pose()

            # TCP wrench in base frame
            if self.use_force_sensor:
                FT_vec = self.get_force()
                if FT_vec is not None:
                    msg = WrenchStamped()
                    force_msg = Float32()
                    ff = msg.wrench.force
                    tt = msg.wrench.torque
                    ff.x, ff.y, ff.z = FT_vec[0:3]
                    tt.x, tt.y, tt.z = FT_vec[3:6]
                    msg.header.stamp = rospy.Time.now()
                   
                    wrench = np.array([ff.x, ff.y, ff.z, tt.x, tt.y, tt.z])
                    wrench_tcp = wrench
                    
                    if self.e_series:
                        # Transform Force from mount frame to tool frame
                        tf_mount = m3d.Transform(self.ropeRob.get_actual_tcp_pose())
                        wrench_tcp = rotate6V(wrench, tf_mount, to_TF=True)
                        [ff.x, ff.y, ff.z, tt.x, tt.y, tt.z] = wrench_tcp
                
                    msg.header.frame_id = self.arm_name+"_TCP"
                    force_msg.data = np.linalg.norm(wrench_tcp)

                    pub_force.publish(force_msg)
                    pub_wrench.publish(msg)
            
            # Joint state
            js_msg = JointState()
            js_msg.header.frame_id = self.arm_name + "_base"
            jnames = self.joint_names
            js_msg.name = [self.arm_name + "_" + jn for jn in jnames]
            js_msg.position = self.get_joints()

            pub_joints.publish(js_msg)

            # TF 
            t = TransformStamped()
            t.header.frame_id = self.arm_name+"_base"

            t.header.stamp = rospy.Time.now()
            t.child_frame_id = self.arm_name+"_TCP" # # BASE TO NOMINAL_TCP, should be NOMINAL_TCP
            fillTF(self.robot_pose.pos, self.robot_pose.orient.get_quaternion(), t)
            tfm = ros_tf.msg.tfMessage([t])
            pub_tf.publish(tfm)

            # Force command
            msg = Float32MultiArray()
            msg.data = self.force_cmd_ft
            pub_force_cmd_ft.publish(msg)
            
            # Force command limits
            msg = Float32MultiArray()
            msg.data  = self.force_cmd_limits
            pub_force_cmd_limits.publish(msg)

            # target moment
            msg = Float32MultiArray()
            msg.data = self.target_moment
            pub_target_moments.publish(msg)

            # joint torques
            msg = Float32MultiArray()
            msg.data = self.ropeRob.get_joint_torques()
            pub_joint_torques.publish(msg)


    def pub_static_tf2(self, trans, quat, parent, child):

        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = TransformStamped()
  
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = parent
        static_transformStamped.child_frame_id = child
  
        static_transformStamped.transform.translation.x = trans[0]
        static_transformStamped.transform.translation.y = trans[1]
        static_transformStamped.transform.translation.z = trans[2]

        static_transformStamped.transform.rotation = quat
 
        broadcaster.sendTransform(static_transformStamped)

    def smart_move(self, pose_goal={}, joints_goal=[], acceleration=0.2, velocity=0.1, threshold=0.3):
       
        self.state = ARMState.INCOMPLETE.name
       
        if pose_goal:
            if type(pose_goal) == list:
                goal = list_to_pose_msg(pose_goal)
            elif type(pose_goal) == dict:
                goal = message_converter.convert_dictionary_to_ros_message(pose_goal)
            self.goal_pose = geometry_msg_to_m3d(goal)
        else:
            goal = joints_goal
            self.goal_joints = deepcopy(joints_goal)
            print self.goal_joints
        
        jlist, _ = self.motion_planner.plan(start_js=self.get_joints(), goal=goal)
        if jlist:
            self.movejs(joints_list=jlist, acceleration=0.9, velocity=0.5, threshold=threshold)
        else:
            # Motion planning failed
            pass
        
        self.reset_ft_sensor()
        self.state = ARMState.COMPLETE.name
            

if __name__ == "__main__":
    """ start the class and spin rospy
    """

    rospy.init_node("test_ur10_driver")
    arm_name = "chisel"
   
    mass = 12.6
    cog = [-0.0, 0.0, 0.24]
    robot_ip = "192.168.50.201"
    use_force_sensor = True 

    ur10 = UR10Driver()
   
    rospy.on_shutdown(ur10.close_connection)

    mt_to_base = [0, 0, 0]
    mt_to_base_orient = [-3.141592653589793, -1.5707963267948966, 0.0]
    tcp_uri = 'bot_digger_narrow_rack/ur16'
    ur10.initialize_robot(arm_name=arm_name, tcp_uri=tcp_uri, \
                            mass=mass, cog=cog, robot_ip=robot_ip, \
                            mt_to_base_position=mt_to_base, mt_to_base_orient=mt_to_base_orient, \
                            use_force_sensor=use_force_sensor)
   
    
    ur10.update_tcp_orientation('bot_digger_narrow_rack/ur10/chisel', rpy=[-1.57, -1.57, 0.0], update_yaw=True)
    
    ur10.update_tcp_position('bot_digger_narrow_rack/ur10/chisel')
    # overseer_api = OverseerAPI()

    # overseer_api.run_private_op('UR10.INIT_ARM', arm_name=arm_name, tcp_position=tcp_pos, tcp_orient=tcp_orient, \
    #                             mass=mass, cog=cog, robot_ip=robot_ip, \
    #                             mt_to_base_position=mt_to_base, mt_to_base_orient=mt_to_base_orient, \
    #                             use_force_sensor=use_force_sensor)
    rospy.spin()
    
    