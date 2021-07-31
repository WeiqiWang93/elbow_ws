#!/usr/bin/env python

import time, argparse
import sys
import copy
import math
import numpy as np
import pdb
import os
import threading
import math3d as m3d
import logging
import rospy
from rospy_message_converter import message_converter

#
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import WrenchStamped, TransformStamped
from sensor_msgs.msg import JointState 
from bot_common_ros.srv import ArmCmdSrv, ArmSetupSrv
from trajectory_msgs.msg import JointTrajectoryPoint


#
import tf as ros_tf
import tf.msg # can be accessed with ros_tf.msg

# Motoman HSE 
from motoman_hse_driver import *
from motoman_ros_driver import *
from motoman_gazebo_driver import *
from bot_common_ros.ur_utils import list_to_tf, tf_to_list, tf_to_arr
from bot_common_ros.ur_control import URMiddleWare, getHorizontalOrient, force_stop2, getFloatArrMsg, checkBox, MotionPlanner
from bot_overseer_api import OverseerAPI

class SIA20Plugin(object):
  # server_input_param_path --> arm_params_driver ie /sia20_driver/arm_1_saw/params
  # client_input_param_path --> arm_params_tool ie /robo_saw_goelz/arm

    def __init__(self):
        self.motoman_hse = None 
        self.motoman_ros = None
        self.motoman_gazebo = None
        self.OA = None

        self.is_sim = False
        self.state = 'IDLE'
        self.joint_names = ['joint_s', 'joint_l', 'joint_e', 'joint_u', 'joint_r', 'joint_b', 'joint_t']
        self.arm_params_path = "/sia20_driver/arm_1_saw/params"
        self.tool_params_path = "/robo_saw_goelz/arm"


    def initialize_robot(self, **kwargs):
        """ Initialize SIA20
            Input args: TCP, Tool Mass, Tool CG, robot_ip, use_force_sensor, sim
            needs to have everything from ArmClient.setup()
        """
        default_kwargs = {'name' : 'saw', 'robot_ip' : '192.168.50.40', 'sim' : True, 'overseer' : True,
            'joint_r' : 0, 'joint_b' : 0, 'joint_t' : 0}
        default_kwargs.update(kwargs)
        kwargs = default_kwargs

        self.arm_name = kwargs['name']  # saw
        self.robot_ip = kwargs['robot_ip']
        self.is_sim = kwargs['sim']
        self.using_overseer = kwargs['overseer']

        if 'arm_params_path' in kwargs:
            self.arm_params_path = kwargs['arm_params_path']
        if 'tool_params_path' in kwargs:
            self.tool_params_path = kwargs['tool_params_path']

        if self.using_overseer:
            self.OA = OverseerAPI()

        self.frame_id_ = rospy.get_param("~frame_id", "saw_base")
        self.mp = MotionPlanner("saw", self.frame_id_)
        self.arm_params = rospy.get_param(self.arm_params_path) # read arm_params.yaml for arm info
        self.tool_params = rospy.get_param(self.tool_params_path) # read arm_params.yaml for tool info
        self.set_defaults(self.tool_params)

        self.connect_to_robot()
        
        # rotation matrix arm-mount to arm-base TF
        quat = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Quaternion', self.arm_params["MOUNT_TO_BASE"])
        self.MOUNT_TO_BASE = m3d.Transform()
        self.MOUNT_TO_BASE.orient = ROStoM3d_orient(quat)
        self.STEP_MOUNT_TO_BASE= m3d.Transform(self.MOUNT_TO_BASE)
        self.MOUNT_TO_BASE.pos.z -= 0.410 # 41 cm differenc between URDF and SIA20/FS100 definition

        # start threads
        self.safety_gate = threading.Event()
        self.safety_gate.set()

        self.monitor_thread = threading.Thread(name=self.arm_name+'_monitor_thread', target=self.arm_monitor)
        self.monitor_thread.setDaemon(True) #will make the thread close when this program ends
        self.monitor_thread.start()

        self.data_thread = threading.Thread(name=self.arm_name+'_data_thread', target=self.data_publisher)
        self.data_thread.setDaemon(True) #will make the thread close when this program ends
        self.data_thread.start()
    
        self.state = 'INITIALIZED'
        return self.state

    def get_state(self):
        """ Plugin process status getter
            States: 'IDLE', 'INITIALIZED', 'INCOMPLETE', 'COMPLETE'
        """
        print self.state
        return self.state
        

    # got rid of bringup
    # Connect to MotomanHSE and MotomanROS, and call power_on_robot
    def connect_to_robot(self):
        
        if self.is_sim:
            rospy.logwarn("Starting simulated SIA20 driver...")
            self.motoman_gazebo = MotomanGazeboDriver()
            self.motoman_gazebo.INIT(overseer=self.OA, mp=self.mp)
            self.ARM_RUNNING = True
            self.use_data_publisher = False
            rospy.logwarn("Completed launching simulated SIA20 driver!")
        
        else:
            self.motoman_hse = MotomanHSE()
            self.motoman_ros = MotomanROS() 
            self.power_on_robot()
            self.motoman_hse.set_shock_level(2)
            self.use_data_publisher = True

    # Uses HSE to enable servos
    def power_on_robot(self):
        self.motoman_hse.power_on()
        self.ARM_RUNNING = True

        # check connectivity 
        res1 = self.motoman_ros.enableMoveJ()
        res2 = self.motoman_ros.disableMoveJ()

        if res1 == False and res2 == False:
            rospy.logerr("Failed to connect to MotoROS server for motion commands. There may be another client already connected to the MotoROS server.")
        
        elif res1 == False and res2 == True:
            rospy.logwarn("Connection established to MotoROS server, but failed to enable servos. Check robot E-stop and/or teach lock before sending motion commands.")

        if self.motoman_hse.exit == True:
            rospy.logerr("FATAL ERROR: SIA20 driver failed to initialized due to crash in MotomanHSE connection. Please restart the driver.")
            self.power_off_robot()
        else:
            rospy.loginfo("SIA20 driver successfully connected to the robot!")

    def power_off_robot(self):
        self.motoman_hse.power_off()

    def on_shutdown(self):
        #
        rospy.loginfo("SIA20 Driver: Shutting Down")
        #
        self.GLOBAL_DONE = True
        self.use_monitor = False
            
        if self.motoman_hse is not None:
            self.STOP()
            self.close_robot()

    def close_robot(self):
        self.power_off_robot()
        time.sleep(1)
        self.motoman_hse = None
        self.motoman_ros = None

    def RESET(self):
        """ clear current tool parameters to be ready for next tool
        """
        super(SIA20Plugin, self).RESET()

        # monitoring thread
        self.sia20_params_set    = False

    def IDLE(self):
        """ IDLE means the arm servos are off but OBP / electronics is still on
        """
        self.power_off_robot()

    def STOP(self, timeout=0.5):
        """ Bring arm to a quick smooth stop
        """
        if self.motoman_hse:
            self.motoman_hse.stopl()
            # robot.stopl
            # force_stop2(self.rob, self.ropeRob, timeout=timeout)
            # self.ropeRob.end_force_mode()
        
        if self.motoman_ros:
            self.motoman_ros.stopj()

    def getValidKwargs(self, func, argsDict):
        """ makes sure only the valid kwargs are kept for func """
        validArgs = func.func_code.co_varnames[:func.func_code.co_argcount]
        kwargsLen = len(func.func_defaults) # number of keyword arguments
        validKwargs = validArgs[-kwargsLen:] # because kwargs are last
        return dict((key, value) for key, value in argsDict.iteritems() 
                    if key in validKwargs)


    ############################### commands w/o kwargs #################################
    
    ######### MotomanHSE Commands #########
    
    ### kwargs 
    def movel(self, **kwargs):
        if self.is_sim:
            fct = self.motoman_gazebo.movel
        
        else:
            fct = self.motoman_hse.movel
            kwargs = self.add_to_kwargs(kwargs, fct)

            # should handle tf conversion, required since moving in SIA20 base frame
            pose_mount = self.MOUNT_TO_BASE * kwargs["pose"]
            kwargs["pose"] = pose_mount
            
        fct(**kwargs)


    def movels(self, **kwargs):
        if self.is_sim:
            fct = fct = self.motoman_gazebo.movels

        else:
            fct = self.motoman_hse.movels
            kwargs = self.add_to_kwargs(kwargs, fct)

            # should handle tf conversion
            poseList_mount = self.MOUNT_TO_BASE * kwargs["poseList_base"]
            kwargs["poseList_base"] = poseList_mount

        fct(**kwargs)

    def stepl(self, **kwargs):
        if self.is_sim:
            fct = self.motoman_gazebo.stepl

        else:
            fct = self.motoman_hse.stepl
            kwargs = self.add_to_kwargs(kwargs, fct)
            
            goal = kwargs["pose"]
            kwargs["vel"] = max(kwargs["pose"].get_pos()) / 10.0

            # should handle tf conversion, required since moving in SIA20 base frame
            pose_mount = self.STEP_MOUNT_TO_BASE * kwargs["pose"] * self.STEP_MOUNT_TO_BASE.inverse
            kwargs["pose"] = pose_mount

            pdb.set_trace()

        fct(**kwargs)

    def movejs(self, **kwargs):
        if len(kwargs['jointList']) == 0:
            rospy.logwarn('movejs received an empty jointList, converting with jtp_list_to_list()')

        # convert the jointList input to the proper type (movejs only)
        self.jtp_list_to_list(kwargs)

        if len(kwargs['jointList']) == 0:
            rospy.logwarn('movejs still has empty jointList, return w/o moving')
            return

        if self.is_sim:
            fct = self.motoman_gazebo.movejs
        else:
            fct = self.motoman_hse.movejs
            kwargs = self.add_to_kwargs(kwargs, fct)
        
        fct(**kwargs)

    def stepj(self, **kwargs):

        if self.is_sim:
            fct = self.motoman_gazebo.stepj
        else:
            fct = self.motoman_hse.stepj
            kwargs = self.add_to_kwargs(kwargs, fct)

        if "velocity" in kwargs:
            kwargs["vel_rot"] = kwargs.pop("velocity")
            
        fct(**kwargs)

    # convert jtp_list to list
    def jtp_list_to_list(self, kwargs):

        # case 1: single list, instead of lists of lists
        if not isinstance(kwargs["jointList"][0], list):
            kwargs["jointList"] = [kwargs["jointList"]]
            rospy.logwarn("jtp_list_to_list case1")

        # case 2: jtp type
        if isinstance(kwargs["jointList"][0], JointTrajectoryPoint):
            rospy.logwarn("jtp_list_to_list case2")
            jointList = []
            for jt in kwargs["jointList"]:
                jointList.append(jt.positions)
            kwargs["jointList"] = jointList

        return

    def set_standard_digital_out(self):
        fct = self.motoman_hse.stepl
        kwargs = self.add_to_kwargs(kwargs, fct)
        fct(**kwargs)

    ### commands w/o args
    def stopl(self):
        self.motoman_hse.stopl()

    ### getters
    def get_pose(self):
        if self.is_sim:
            return self.motoman_gazebo.get_pose()
        
        else:
            # Returns a transform as required by SRM
            pose_mount = self.motoman_hse.get_pose()
            pose_base = self.MOUNT_TO_BASE.inverse * pose_mount
            return pose_base

    def get_torque(self):
        return self.motoman_hse.get_torque()
    
    # list[7,] of speed for x,y,z,rx,ry,rz,re
    # currently just supports translational speed
    def get_speed(self):
        
        _speed = self.motoman_hse.get_speed()
        _speed = [-_speed[2], _speed[0], -_speed[1]] # speed_mount to speed_base: [-z , +x , -y]
        return _speed
        
    ### status and safety
    # returns True when executing movel, returns False when done moving (ie not executing movel)
    def get_robot_state(self):
        # return not self.motoman_hse.motionDone
        return self.motoman_hse.is_robot_moving
        
    # True indicates error, triggers safety_cb
    def get_safety_status(self):
        if self.is_sim:
            return False
        else:
            return self.motoman_hse.get_status()
    
    # Safety_cb resets the error and enables servo
    def safety_cb(self):
        self.motoman_hse.get_alarm()
        self.motoman_hse.reset_error()
        self.motoman_hse.servo_on(True)
        self.motoman_ros.enableMoveJ()  # Enable/disable will reset the error
        self.motoman_ros.disableMoveJ()
        # time.sleep(0.5)
        return

    def set_speed(self, **kwargs):
        fct = self.motoman_hse.set_speed
        kwargs = self.add_to_kwargs(kwargs, fct)
        fct(**kwargs)
        return

    ######### MotomanROS Commands #########

    ### commands w/o args
    def stopj(self):
        self.motoman_ros.stopj()

    ### getters
    def get_joints(self):
        if self.is_sim:
            return self.motoman_gazebo.get_joints()
        else:
            return self.motoman_ros.get_joints()

    def get_joint_speed(self):
        return self.motoman_ros.get_joint_velocities()
    
    ######### MotoPlus Commands (force sensor) #########

    ### kwargs 
    def force_mode(self, **kwargs):
        fct = force_mode
        kwargs = self.add_to_kwargs(kwargs, fct)
        fct(**kwargs)

    def force_move(self):
        return

    ### commands w/o args
    def end_force_mode(self):
        return

    def reset_FT300(self):
        return

    def force_stop2(self, **kwargs):
        self.STOP()

        # force_stop2(self.rob, self.ropeRob, timeout=0.5) # hard-set timeout
        return
    
    ### getters
    def get_force(self):
        return

    ######### Misc commands ######### 
    def wait_for_bbox_recovery(self):
        """
        while not in box
            wait
        """
        pass
    
    # Implement abstract method to save nominal-TCP to TCP tf
    # ArmServer loads from param server and passes TCP, mass, CG as a lookup map params[]
    def set_defaults(self, params):
        """ Save nominal-tcp to TCP tf, and set
        """
        pose_vector = params["TCP"]
        self.nominal_tcp_to_tcp = m3d.Transform(pose_vector).inverse

        # load joint_names from arm_params
        self.joint_names = self.arm_params["joint_names"]
        prefix = self.arm_name.replace("/","") + "_"
        self.joint_names_prefixed = self.joint_names[:]
        self.joint_names_prefixed[:] = [prefix + x for x in self.joint_names_prefixed]

        # Send TCP to robot
        # self.ropeRob.set_tcp(params["TCP"])
        # self.ropeRob.set_payload(params["mass"], params["CG"])

    def add_to_kwargs(self, kwargs, fct):
        """ add internal args to kwargs """
        # kwargs['robot'] = self.ropeRob
        # kwargs['URXrob'] = self.rob
        kwargs['tf'] = self.MOUNT_TO_BASE
        kwargs = self.getValidKwargs(fct, kwargs)
        return kwargs

    def smart_move(self, pose_goal={}, joints_goal=[], acceleration=0.2, velocity=0.1, threshold=0.3):
        self.state = "INCOMPLETE"
        
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
        
        jlist, _ = self.mp.plan(start_js=self.get_joints(), goal=goal)
        if jlist:
            self.movejs(joints_list=jlist, acceleration=0.9, velocity=0.5, threshold=threshold)
        else:
            # Motion planning failed
            pass
            
        self.state = "COMPLETE"


    def arm_monitor(self):
        while not rospy.is_shutdown():           
            self.safety_gate.wait() # otherwise may override bbox gate
            safetyMode = self.get_safety_status()

            if safetyMode == True: # True indicates error has been detected
                self.safety_cb()

    def init_overseer_publishers(self):
        # Joint torque publishers for influxdb
        self.pub_torque_1 = rospy.Publisher(self.arm_name+"/joint_torque_1", Float32, queue_size=30, latch=True)
        self.pub_torque_2 = rospy.Publisher(self.arm_name+"/joint_torque_2", Float32, queue_size=5, latch=True)
        self.pub_torque_3 = rospy.Publisher(self.arm_name+"/joint_torque_3", Float32, queue_size=5, latch=True)
        self.pub_torque_4 = rospy.Publisher(self.arm_name+"/joint_torque_4", Float32, queue_size=5, latch=True)
        self.pub_torque_5 = rospy.Publisher(self.arm_name+"/joint_torque_5", Float32, queue_size=5, latch=True)
        self.pub_torque_6 = rospy.Publisher(self.arm_name+"/joint_torque_6", Float32, queue_size=5, latch=True)
        self.pub_torque_7 = rospy.Publisher(self.arm_name+"/joint_torque_7", Float32, queue_size=5, latch=True)

        # Joint pos publishers for influxdb
        self.pub_joint_1 = rospy.Publisher(self.arm_name+"/joint_1", Float32, queue_size=30, latch=True)
        self.pub_joint_2 = rospy.Publisher(self.arm_name+"/joint_2", Float32, queue_size=30, latch=True)
        self.pub_joint_3 = rospy.Publisher(self.arm_name+"/joint_3", Float32, queue_size=30, latch=True)
        self.pub_joint_4 = rospy.Publisher(self.arm_name+"/joint_4", Float32, queue_size=30, latch=True)
        self.pub_joint_5 = rospy.Publisher(self.arm_name+"/joint_5", Float32, queue_size=30, latch=True)
        self.pub_joint_6 = rospy.Publisher(self.arm_name+"/joint_6", Float32, queue_size=30, latch=True)
        self.pub_joint_7 = rospy.Publisher(self.arm_name+"/joint_7", Float32, queue_size=30, latch=True)

        # Arm pose publishers for influxdb
        self.pub_pose_x  = rospy.Publisher(self.arm_name+"/pose_x", Float32, queue_size=5, latch=True)
        self.pub_pose_y  = rospy.Publisher(self.arm_name+"/pose_y", Float32, queue_size=5, latch=True)
        self.pub_pose_z  = rospy.Publisher(self.arm_name+"/pose_z", Float32, queue_size=5, latch=True)
        self.pub_pose_rx = rospy.Publisher(self.arm_name+"/pose_rx", Float32, queue_size=5, latch=True)
        self.pub_pose_ry = rospy.Publisher(self.arm_name+"/pose_ry", Float32, queue_size=5, latch=True)
        self.pub_pose_rz = rospy.Publisher(self.arm_name+"/pose_rz", Float32, queue_size=5, latch=True)

        # TCP speed publishers for influxdb
        self.pub_vel_x  = rospy.Publisher(self.arm_name+"/vel_x", Float32, queue_size=50, latch=True)
        self.pub_vel_y  = rospy.Publisher(self.arm_name+"/vel_y", Float32, queue_size=50, latch=True)
        self.pub_vel_z  = rospy.Publisher(self.arm_name+"/vel_z", Float32, queue_size=50, latch=True)
        return

    def data_publisher(self):
        """ publishes tf and joints for this UR10. Follows same standard as ur_modern_driver
        get_pose
        get_speed
        get_joints
        get_force
        gate_status
        """
        pub_tf = rospy.Publisher('/tf', ros_tf.msg.tfMessage, queue_size=1)
        pub_tcp_speed = rospy.Publisher(self.arm_name+'/tcp_speed', Float32MultiArray, queue_size=5)
        self.pub_torques = rospy.Publisher(self.arm_name+'/joint_torques', JointState, queue_size=5)
        pub_tcp_force = rospy.Publisher(self.arm_name+'/wrench', WrenchStamped, queue_size=5)
        pub_is_gate_open = rospy.Publisher(self.arm_name+'/is_gate_open', Bool, queue_size=5)
        pub_is_robot_moving = rospy.Publisher(self.arm_name+'/is_robot_moving', Bool, queue_size=5)
        pub_robot_ready = rospy.Publisher(self.arm_name+'/is_robot_ready', Bool, queue_size=5)
        pub_get_pose = rospy.Publisher(self.arm_name+'/pose', Float32MultiArray, queue_size=5)
        
        # Initialize overseer publishers
        self.init_overseer_publishers()

        rospy.loginfo("Data Publisher ready and starting.")

        # static transforms TODO: put nominal-tcp-to-tcp here (but then have to update it if tf changes from user)

        start_time = time.time()
        one_sec_time = time.time()
        counter = 0

        while not rospy.is_shutdown():
            if self.ARM_RUNNING and self.use_data_publisher:

                time.sleep(0.010) # throttle to ~75 hz


                # TCP pose in base frame    1000 Hz (1 ms)
                t = TransformStamped()
                t.header.frame_id = self.arm_name+"_base"
                self.robot_pose = self.get_pose()
                t.header.stamp = rospy.Time.now()
                t.child_frame_id = self.arm_name+"_TCP" # # BASE TO NOMINAL_TCP, should be NOMINAL_TCP
                fillTF(self.robot_pose.pos, self.robot_pose.orient.get_quaternion(), t)
                tfm = ros_tf.msg.tfMessage([t])
                pub_tf.publish(tfm)

                # # Nominal-TCP to TCP      1300 Hz (0.7 ms)
                # # TODO: change this to static transform outside of while loop
                nominal_tcp_to_tcp = self.nominal_tcp_to_tcp
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = self.arm_name+"_TCP"
                t.child_frame_id = self.arm_name+"_NOMINAL_TCP"
                fillTF(nominal_tcp_to_tcp.pos, nominal_tcp_to_tcp.orient.get_quaternion(), t)
                tfm = ros_tf.msg.tfMessage([t])
                pub_tf.publish(tfm)
                
                # TCP speed in base frame     2000 Hz (0.5 ms)
                # get_speed() might be buggy getting into influx
                speed_base = self.get_speed()
                msg = getFloatArrMsg(speed_base)
                pub_tcp_speed.publish(msg)
                # publish speed to overseer
                self.pub_vel_x.publish(abs(msg.data[0]))
                self.pub_vel_y.publish(abs(msg.data[1]))
                self.pub_vel_z.publish(abs(msg.data[2]))

                # publish the entire pose (including rotation)    1000 Hz (1 ms)
                # TCP pose in base frame
                pos = self.robot_pose.get_pos()[0:3]
                rot = tf.transformations.euler_from_matrix(self.robot_pose.get_orient().get_matrix())
                pose_list = np.append(pos,rot).tolist()
                msg = getFloatArrMsg(pose_list)
                pub_get_pose.publish(msg)
                # publish pose to overseer
                self.pub_pose_x.publish(msg.data[0])
                self.pub_pose_y.publish(msg.data[1])
                self.pub_pose_z.publish(msg.data[2])
                self.pub_pose_rx.publish(msg.data[3])
                self.pub_pose_ry.publish(msg.data[4])
                self.pub_pose_rz.publish(msg.data[5])

                # # TCP joint-state     2500 Hz (0.4 ms)
                joints = self.get_joints()
                msg = JointState()
                msg.position[:] = joints[:] # copy into the msg
                msg.name = self.joint_names_prefixed

                # Get joint torques
                msg.effort[:] = self.get_torque()

                # publish the joint torques of SIA20 Arm into individual topics for influxdb
                self.pub_torque_1.publish(msg.effort[0])
                self.pub_torque_2.publish(msg.effort[1])
                self.pub_torque_3.publish(msg.effort[2])
                self.pub_torque_4.publish(msg.effort[3])
                self.pub_torque_5.publish(msg.effort[4])
                self.pub_torque_6.publish(msg.effort[5])
                self.pub_torque_7.publish(msg.effort[6])

                # publish joint positions of sia20 arm into individual topics for influxdb
                self.pub_joint_1.publish(msg.position[0])
                self.pub_joint_2.publish(msg.position[1])
                self.pub_joint_3.publish(msg.position[2])
                self.pub_joint_4.publish(msg.position[3])
                self.pub_joint_5.publish(msg.position[4])
                self.pub_joint_6.publish(msg.position[5])
                self.pub_joint_7.publish(msg.position[6])

                # Timestamp pub_joints msg      2500 Hz (0.4 ms)
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = self.arm_name

                # publish
                self.pub_torques.publish(msg)

                #gate
                msg = Bool(self.safety_gate.is_set()) # true if open
                pub_is_gate_open.publish(msg)
                msg = Bool(self.get_robot_state())
                pub_is_robot_moving.publish(msg)

                try: 
                    
                    msg = Bool(self.get_robot_mode() == 7)
                    pub_robot_ready.publish(msg)
                except:
                    pass

    ############################### mission op wrappers #################################

    # input: op_stepj(joint="joint_s", ang_velocity=10.0, duration=1.0)
    # units are deg/s, s
    def op_stepj(self, **kwargs):
        print "op_stepj called."

        # default velocity values for joints
        new_kwargs = {'joint': 'joint_s', 'ang_velocity' : 5.0, 'joint_s' : 0, 'joint_l' : 0, 'joint_e' : 0, 'joint_u' : 0,
                    'joint_r' : 0, 'joint_b' : 0, 'joint_t' : 0, 'duration' : 1.0}
        new_kwargs.update(kwargs)

        # Set the joint value of the joint to be stepped
        new_kwargs[new_kwargs["joint"]] = np.deg2rad(float(new_kwargs["duration"]) * new_kwargs["ang_velocity"])

        # filter kwargs by joint names
        joint_kwargs = { k: new_kwargs[k] for k in self.joint_names }

        # Order the kwargs based on the joint_names list
        index_map = {v: i for i, v in enumerate(self.joint_names)}
        sorted_kwargs = sorted(joint_kwargs.items(), key=lambda pair: index_map[pair[0]])  # Ordered list of tuples
        velocities = np.asarray(list(map(lambda x: x[1], sorted_kwargs))) # extract just the joint values
        
        joint_distances = velocities[:]
        
        new_kwargs["joints"] = joint_distances # np array
        new_kwargs["wait"] = True
        new_kwargs["radius"] = 0.03
        new_kwargs["threshold"] = 0.01
  
        # Call movejs
        self.stepj(**new_kwargs)
        return True

    # Sample input: op_movej(shoulder_pan_joint=0.2, shoulder_lift_joint=1.3, elbow_joint=2, wrist_1_joint=3, wrist_2_joint=4, wrist_3_joint=5, vel=0.5, wait=True)
    def op_movejs(self, **kwargs):
        print "OP movejs called."
        # filter kwargs by joint names
        joint_kwargs = { k: kwargs[k] for k in self.joint_names }

        # Order the kwargs based on the joint_names list
        index_map = {v: i for i, v in enumerate(self.joint_names)}
        sorted_kwargs = sorted(joint_kwargs.items(), key=lambda pair: index_map[pair[0]])  # Ordered list of tuples
        jointList = [list(map(lambda x: x[1], sorted_kwargs))] # extract just the joint values
        new_kwargs = {}
        
        # Update kwargs, remove the joint names
        for name in self.joint_names:
            del kwargs[name]
        new_kwargs["jointList"] = jointList
        new_kwargs["acc"] = kwargs["acceleration"]
        new_kwargs["vel"] = kwargs["velocity"]
        new_kwargs["wait"] = True
        new_kwargs["radius"] = 0.03
        new_kwargs["threshold"] = 0.01

        # Call movejs
        self.movejs(**new_kwargs)

        return True

    # Sample input: op_movel(x=0, y=1, z=2, roll=0, yaw=0, pitch=0, vel=0.5, wait=True)
    # (jointList=[jtp.positions], poseList_base=[tf], vel = vel_plunge, wait=False)
    def op_movel(self, **kwargs):

        # filter kwargs by cartesian_names
        cartesian_names = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
        cartesian_kwargs = { k: kwargs[k] for k in cartesian_names }

        # Order the kwargs based on the cartesian_names list
        index_map = {v: i for i, v in enumerate(cartesian_names)}
        sorted_kwargs = sorted(cartesian_kwargs.items(), key=lambda pair: index_map[pair[0]])  # Ordered list of tuples
        poseList = list(map(lambda x: x[1], sorted_kwargs)) # extract just the list

        # Convert from list to transform
        poseList = list_to_tf(poseList)
        kwargs["pose"] = poseList

        # Call movel
        self.movel(**kwargs)

        # TODO: list_to_tf function is broken for the Orientation portion

        return True

    # Sample input: x=0, y=0, z=-0.010, roll=5.0, pitch=0, yaw=0, duration=1.0
    # inputs velocity, angular velocity, duration
    # units are m/s, deg/s, s
    def op_stepl(self, **kwargs):

        print  "op_stepl called."

        # default values for kwargs
        new_kwargs = {'x': 0, 'y': 0, 'z': 0, 'roll': 0, 'pitch': 0, 'yaw': 0, 'duration': 0.3}
        new_kwargs.update(kwargs)

        # Calculate poseList_base based on velocity & duration for each coordinate
        velocities = np.array([new_kwargs["x"], new_kwargs["y"], new_kwargs["z"], 
                        new_kwargs["roll"], new_kwargs["pitch"], new_kwargs["yaw"]])
        step_distances = velocities[:] * kwargs["duration"]

        # Convert step_distances to a tf
        step_distances[3:] = np.deg2rad(step_distances[3:]) # convert deg to rad
        step_distances = list_to_tf(step_distances)

        # Add poseList and vel to new_kwargs
        new_kwargs["vel"] = max(velocities[0:2])
        new_kwargs["vel_rot"] = np.deg2rad(max(velocities[3:]))
        new_kwargs["poseList_base"] = [step_distances]
        new_kwargs["pose"] = step_distances
        new_kwargs["wait"] = True
        new_kwargs["duration"] = kwargs["duration"]

        # Temp: remove frame, tool
        if "frame" in new_kwargs: del new_kwargs["frame"]
        if "tool" in new_kwargs: del new_kwargs["tool"]

        # Call stepl
        self.stepl(**new_kwargs)

        return True        


if __name__ == "__main__":
    """ start the class and spin rospy
    """
    print "Starting Driver node"
    rospy.init_node("sia20_driver", anonymous=True)
    run_test = rospy.get_param("~run_test", False)
    run_api  = rospy.get_param("~run_api", False)
    sim      = rospy.get_param("~sim", False)
    
    try:
        if not run_api:
            sia20_driver = SIA20Plugin()
            sia20_driver.INIT()
            time.sleep(1.0)

        if run_test:
            test(run_api, sim)

        if not run_api:
            rospy.loginfo("sia20_driver: Start spinning forever now.")
            rospy.spin()
        
    finally:
        if not rospy.is_shutdown(): rospy.signal_shutdown("End of Program")