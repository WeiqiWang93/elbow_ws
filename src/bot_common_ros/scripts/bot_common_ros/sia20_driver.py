#!/usr/bin/env python

import time
import argparse
import sys
import copy
import math
import pdb
import os
import threading
import logging
from copy import deepcopy
from enum import Enum


import numpy as np
import math3d as m3d
import rospy
from rospy_message_converter import message_converter
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from std_msgs.msg import Float32 
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState 
from bot_common_ros.srv import ArmCmdSrv
from bot_common_ros.srv import ArmSetupSrv
from trajectory_msgs.msg import JointTrajectoryPoint
import tf as ros_tf
import tf.msg # can be accessed with ros_tf.msg

from datetime import date
# Motoman HSE 
from motoman_hse_driver import *
from motoman_ros_driver import *
from motoman_hse_constants import *
from motoman_gazebo_driver import *


from bot_common_ros.ur_utils import list_to_tf
from bot_common_ros.ur_utils import tf_to_list
from bot_common_ros.ur_utils import tf_to_arr
from bot_common_ros.ur_utils import linear_interp_tf
from bot_common_ros.ur_utils import list_to_pose_msg
from bot_common_ros.ur_utils import geometry_msg_to_m3d
from bot_common_ros.ur_control import URMiddleWare
from bot_common_ros.ur_control import getHorizontalOrient
from bot_common_ros.ur_control import force_stop2
from bot_common_ros.ur_control import getFloatArrMsg
from bot_common_ros.ur_control import checkBox
from bot_common_ros.ur_control import MotionPlanner
from bot_common_ros.ur_control import m3d_to_ros
from bot_common_ros.ur_control import ros_to_m3d
from bot_overseer_api import OverseerAPI


class SIA20DriverState(Enum):
    """ Defines the possible SIA20 Driver connection status
    """

    IDLE = 0
    INITIALIZED = 1
    INCOMPLETE = 2
    COMPLETE = 3
    FAILED = 4
    JOB_COMPLETE = 5

class SIA20Driver(object):
  # server_input_param_path --> arm_params_driver ie /sia20_driver/arm_1_saw/params
  # client_input_param_path --> arm_params_tool ie /robo_saw_goelz/arm

    def __init__(self, debug_mode=False):
        self.debug_mode = debug_mode

        self.motoman_hse = None 
        self.motoman_ros = None
        self.motoman_gazebo = None
        self.OA = None
        self.using_overseer = True
        self.powered_off = True
        self.init_complete = False

        self.is_sim = False
        self.state = SIA20DriverState.IDLE
        self.joint_names = ['joint_s', 'joint_l', 'joint_e', 'joint_u', 'joint_r', 'joint_b', 'joint_t']
        self.arm_params_path = "/sia20_driver/arm_1_saw/params"
        self.tool_params_path = "/robo_saw_goelz/arm"

        # init
        self.arm_name = "saw"
        self.pose_base = m3d.Transform()
        self.joint_torques = [0] * 7
        self.torque_percent = np.zeros(7)
        self.arm_type = "sia20"
        self.last_jointList, self.last_poseList_base, self.last_vel, self.last_wait = None, None, None, None

        self.joints = [0]*7
        self.tcp_speed = [0]*3
        self.robot_state = False
        self.mp_success = True
        self.last_mp = [[]]
        self.last_mp_joints_goal = []
        self.last_mp_pose_goal = []
        self.move_to_orthogonal_jlist = [[]]

        self.overseer_api = OverseerAPI()
        self.job_name = ""
        self.job_line_num = 0
        self.job_step_num = 0
        self.hse_speed_override = 100

        # path to catkin_ws/data
        self.path_to_jobs = '../../../../catkin_ws/data/fs100_jobs/'

        # ROS publishers
        self.pub_tf = rospy.Publisher('/tf', ros_tf.msg.tfMessage, queue_size=1)
        self.pose_list_publisher = rospy.Publisher("/robo_saw/roboSaw/spline_poses", PoseArray, queue_size=30, latch=True)

        # job line num dict
        self.job_line_dict = {}

    def initialize_robot(self, **kwargs):
        """ Initialize SIA20
            Input args: TCP, Tool Mass, Tool CG, robot_ip, use_force_sensor, sim
            needs to have everything from ArmClient.setup()
        """
        default_kwargs = {'name' : 'saw', 'robot_ip' : '192.168.50.40', 'sim' : False, 'overseer' : True,
            'joint_r' : 0, 'joint_b' : 0, 'joint_t' : 0}
        default_kwargs.update(kwargs)
        kwargs = default_kwargs

        self.powered_off = False

        self.arm_name = kwargs['name']  # saw
        self.robot_ip = kwargs['robot_ip']
        self.using_overseer = kwargs['overseer']
        
        prefix = self.arm_name.replace("/","") + "_"
        self.joint_names_prefixed = self.joint_names[:]
        self.joint_names_prefixed[:] = [prefix + x for x in self.joint_names_prefixed]

        self.ns_prefix = "/saw"
        self.pub_get_pose = rospy.Publisher(self.ns_prefix+'/pose', Float32MultiArray, queue_size=5, latch=True)
        # if rospy.get_namespace().strip("/") != "saw":
        #     self.ns_prefix = "saw"  # namespace isn't saw, so enforce prefix
        # else:
        #     self.ns_prefix = ""

        if 'arm_params_path' in kwargs:
            self.arm_params_path = kwargs['arm_params_path']
        if 'tool_params_path' in kwargs:
            self.tool_params_path = kwargs['tool_params_path']
        if self.using_overseer:
            self.OA = OverseerAPI()

        # Initialize motion planner if possible
        self.frame_id_ = rospy.get_param("~frame_id", "saw_base")
        try:
            self.motion_planner = MotionPlanner("saw", "saw_base", nominal_tcp_frame="saw_tool0")
        except:
            rospy.logwarn("SIA20 Motion Planning was not launched, skipping MP initialization.")
            self.motion_planner = None

        self.arm_params = rospy.get_param(self.arm_params_path) # read arm_params.yaml
        self.arm_type = self.arm_params["arm_type"]

        self.tool_params = rospy.get_param(self.tool_params_path)
        self.set_defaults(self.tool_params)
        if self.state == SIA20DriverState.IDLE:
            rospy.loginfo("Initializing SIA20 driver")
            self.is_sim = kwargs['sim']
            connection = self.connect_to_robot()
            rospy.loginfo("SIA20 driver connection successful.")

            if connection == False:
                self.state = SIA20DriverState.IDLE
                return False

            self.listener = ros_tf.TransformListener()
            time.sleep(0.25)
            
            # rotation matrix arm-mount to arm-base TF
            quat = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Quaternion', self.arm_params["MOUNT_TO_BASE"])
            self.MOUNT_TO_BASE = m3d.Transform()
            self.MOUNT_TO_BASE.orient = ROStoM3d_orient(quat)
            self.STEP_MOUNT_TO_BASE= m3d.Transform(self.MOUNT_TO_BASE)
            self.BASE_TO_MOUNT = self.STEP_MOUNT_TO_BASE.inverse
            self.MOUNT_TO_BASE.pos.z -= 0.410 # 41 cm differenc between URDF and SIA20/FS100 definition
            self.INVERSE_MOUNT_TO_BASE = self.MOUNT_TO_BASE.inverse

            self.MOUNT_TO_BASE_NEW = m3d.Transform()
            self.MOUNT_TO_BASE_NEW.pos.x -= 0.410 # -41 cm difference in x

        
            # start threads
            self.safety_gate = threading.Event()
            self.safety_gate.set()

            if not self.using_overseer:
                self.data_thread = threading.Thread(name=self.arm_name+'_data_thread', target=self.data_publisher)
                self.data_thread.setDaemon(True) #will make the thread close when this program ends
                self.data_thread.start()

        else:
            self.STOP()
            rospy.loginfo("SIA20DriverState is not IDLE, skipping connect_to_robot()")
            



        self.state = SIA20DriverState.COMPLETE
        rospy.loginfo("SIA20 init function complete, returning now")
        return self.state

    def get_state(self):
        """ Plugin process status getter
            States: 'IDLE', 'INITIALIZED', 'INCOMPLETE', 'COMPLETE'
        """
        return self.state.name
    
    def get_arm_type(self):
        return self.arm_type

    # got rid of bringup
    # Connect to MotomanHSE and MotomanROS, and call power_on_robot
    def connect_to_robot(self):
        """ Return self.init_complete (True for success, False for failed)
        """
        
        if self.is_sim:
            rospy.logwarn("Starting simulated SIA20 driver...")
            self.motoman_gazebo = MotomanGazeboDriver()
            self.motoman_gazebo.INIT(overseer=self.OA, mp=self.motion_planner)
            self.ARM_RUNNING = True
            self.use_data_publisher = False
            rospy.logwarn("Completed launching simulated SIA20 driver!")
            self.init_complete = True
        
        else:
            if self.motoman_hse == None:
                self.motoman_hse = MotomanHSE(debug_mode=self.debug_mode)
                time.sleep(0.1)
            else:
                rospy.loginfo("MotomanHSE already active, did not re-instantiate")

            self.motoman_ros = MotomanROS() 

            if self.motoman_hse.exit:
                self.init_complete = False
                return self.init_complete

            if self.power_on_robot() == True:
                self.init_complete = True
            else:
                self.init_complete = False
                return self.init_complete

            # TODO: investigate set_shock_level implementation
            # self.motoman_hse.set_shock_level(2)

            if self.using_overseer:
                self.use_data_publisher = False
            else:
                self.use_data_publisher = True
            
            return self.init_complete

    # Uses HSE to enable servos
    def power_on_robot(self):
        """ Returns False if failed to connect (ie robot is not powered on), returns True if initialized successfully
        """
        self.motoman_hse.power_on()
        self.ARM_RUNNING = True

        if self.get_torque() == [0]*7 and self.get_joints() == [0]*7:
            rospy.logerr("Failed to connect to robot. Please check Ethernet connection and FS100 power.")
            return False

        # check connectivity 
        res1 = self.motoman_ros.enableMoveJ()
        res2 = self.motoman_ros.disableMoveJ()

        if res1 == False and res2 == False:
            rospy.logwarn("Failed to connect to MotoROS server for motion commands. There may be another client already connected to the MotoROS server.")
        
        elif res1 == False and res2 == True:
            rospy.logwarn("Connection established to MotoROS server, but failed to enable servos. Check robot E-stop and/or teach lock before sending motion commands.")

        if self.motoman_hse.exit == True:
            rospy.logerr("FATAL ERROR: SIA20 driver failed to initialized. There may be already another connection/client active. Close all connections and restart the driver.")
            self.power_off()
        else:
            rospy.loginfo("SIA20 driver successfully connected to the robot!")

        # Give 0.5 seconds for tm to update
        time.sleep(0.5)

        return True

    # Power off servos & update robot state, but keeps MotoROS driver telemetry running
    def power_off(self):
        if not self.powered_off:
            self.motoman_hse.power_off()
            self.powered_off = True
            self.state = SIA20DriverState.IDLE
        else:
            rospy.logwarn("SIA20 Driver is already powered off.")

        return

    def on_shutdown(self):
        #
        rospy.loginfo("SIA20 Driver: Shutting Down")
        self.shutdown_robot()

    # Full shutdown of robot; driver telemetry disconnects
    def shutdown_robot(self):
        self.power_off()
        self.motoman_hse = None
        self.motoman_ros = None

    def RESET(self):
        """ clear current tool parameters to be ready for next tool
        """
        super(SIA20Driver, self).RESET()

        # monitoring thread
        self.sia20_params_set    = False

    def IDLE(self):
        """ IDLE means the arm servos are off but OBP / electronics is still on
        """
        self.power_off()

    def STOP(self, timeout=0.5):
        """ Bring arm to a quick smooth stop
        """
        self.state = SIA20DriverState.INCOMPLETE
        if self.powered_off:
            rospy.logwarn("SIA20 driver is off, ignored STOP command.")
            self.state = SIA20DriverState.COMPLETE
            return

        if self.is_sim:
            self.motoman_gazebo.STOP()

        if self.motoman_hse:
            self.motoman_hse.stop()

        self.state = SIA20DriverState.COMPLETE
        return

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
        return True

    def test_op(self):
        fp = os.path.join(self.path_to_jobs, "MOVELJ5UP.JBI")
        self.motoman_hse.delete_file(filename="MOVELS0401.JBI")
        time.sleep(1)
        self.motoman_hse.send_file(filename=fp)
        return

    # Function used by movels for JBI files generation
    # Input param: kwargs
    # Output: return filename if generate successfully,
    #         return None if error happens
    # the file will be saved under ../../../../data/fs100_jobs

    # kwargs['vel_rot'] --> ' VJ='+str(kwargs['vel_rot'])
    def generate_jbi(self, **kwargs):
        self.state = SIA20DriverState.INCOMPLETE

        #kwargs['vel'] = int(round(kwargs['vel'] * 1000.0)) # convert m to mm
        
        if( kwargs == None or kwargs['job_name'] == None or kwargs['jbi_traj'] == None ):
            return None
        else:
            if not os.path.exists(self.path_to_jobs):
                os.makedirs(self.path_to_jobs)
            job_name = kwargs['job_name']
            pathNum = len(kwargs['jbi_traj'])
            self.job_line_dict[job_name] = pathNum
            content = '/JOB\r\n'
            content += '//NAME ' + job_name + '\r\n'
            content += '//POS\r\n'
            content += '///NPOS '+str(pathNum)+',0,0,0,0,0\r\n'
            content += '///TOOL 0\r\n'
            content += '///POSTYPE PULSE\r\n'
            content += '///PULSE\r\n'

            # all C-Variables
            # [0,1,3,4,5,6,2]
            for index in range(pathNum):
                entry = kwargs['jbi_traj'][index]
                joint = entry['joints']
                content += 'C' + str(index).zfill(5)
                content += '='
                thirdJoint = None
                for jointIdx in range(len(joint)):
                    jointPulse = int(round(self.motoman_hse.pulse_per_deg[jointIdx] * math.degrees(joint[jointIdx])))
                    if(jointIdx != 2):
                        content += str(jointPulse) + ','
                    else:
                        thirdJoint = jointPulse
                content += str(thirdJoint) + '\r\n'
            content += '//INST\r\n'
            content += '///DATE '+ date.today().strftime("%Y/%m/%d %H:%M") + "\r\n"
            content += '///ATTR SC,RW\r\n'
            content += '///GROUP1 RB1\r\n'
            content += 'NOP\r\n'
            # Corresponding vel values
            for index in range(pathNum):
                entry = kwargs['jbi_traj'][index]
                motionType = None
                if( entry['motion_type'] == "movel" ):
                    motionType = "MOVL"
                elif( entry['motion_type'] == "movej"):
                    motionType = "MOVJ"
                velType = None
                if(entry['vel_type'] == 'vel'):
                    velType = "V"
                    speed = entry['speed'] * 1000.0 # convert m/s to mm/s
                elif(entry['vel_type'] == 'vel_rot'):
                    velType = "VR"
                    speed = np.round(np.rad2deg(entry['speed']), 1) # convert rad/s to deg/s
                
                content += motionType + ' C' + str(index).zfill(5) + ' ' + velType +'='+str(speed) + '\r\n'
            content += 'END\r\n'
            filename = os.path.join(self.path_to_jobs, kwargs['job_name'])
            filename += ".JBI"
            # if file exists already, delete first
            if(os.path.exists(filename)):
                os.remove(filename)
            fp = open(filename,'w')
            fp.write(content)
            fp.close()
            self.state = SIA20DriverState.JOB_COMPLETE
            return filename

    def delete_file(self, filename):
        """
        Deletes file on the teach pendant
        filename: Filename including the extension
        """
        self.state = SIA20DriverState.INCOMPLETE
        self.motoman_hse.delete_file(filename=filename)
        self.state = SIA20DriverState.JOB_COMPLETE

    def send_file(self, filename):
        """
        Sends a file from the OBP to the teach pendant
        Note: File must not currently be present on the teach pendant        
        filename: Name of the file including the extension
        """
        self.state = SIA20DriverState.INCOMPLETE
        filepath = os.path.join(self.path_to_jobs, filename)
        self.motoman_hse.send_file(filename=filepath)
        self.state = SIA20DriverState.JOB_COMPLETE


    def select_job(self, **kwargs):
        """ Select job and line_num for SIA20
        Args:
                job_name        string of job
                line_num        line num of job
        """
        if "job_name" not in kwargs:
            rospy.logwarn("Select job failed, no job name provided.")
            return False

        if "line_num" not in kwargs:
            kwargs["line_num"] = 0

        rospy.loginfo("Select job {} and line_num {}".format(kwargs["job_name"], kwargs["line_num"]))
        self.motoman_hse.select_job(job_name = kwargs["job_name"], line_num = kwargs["line_num"])
        rospy.sleep(0.1)

        counter = 1
        while not rospy.is_shutdown() and self.job_name != kwargs['job_name']:
            # rospy.loginfo("Current job name = {}, Selected job name = {}".format(self.job_name, kwargs['job_name']))
            self.motoman_hse.select_job(job_name=kwargs["job_name"], line_num=kwargs["line_num"])
            rospy.sleep(0.1)
            counter += 1

        if counter > 1:
            rospy.loginfo("Select_job called {} times".format(counter))

        self.state = SIA20DriverState.JOB_COMPLETE
        return True

    def update_hse_job_info(self):
        """ Calls HSE function to update job info and update member variables (db-defined 5 Hz)
        This function should only appear in the telemetry db once to prevent unnecessary 
        calls to HSE server. This function causes data transfer between HSE driver & server.

        Returns:
            True  - self.joint_torques updated successfully
            False - Exception (because driver not initialized)
        """
        try:
            self.motoman_hse.update_job_info()
            self.job_name = self.motoman_hse.get_job_name()
            self.job_line_num = self.motoman_hse.get_job_line_num()
            self.job_step_num = self.motoman_hse.get_job_step_num()
            self.hse_speed_override = self.motoman_hse.get_speed_override()
            return True
        except:
            return False

    def get_job_name(self):
        return self.job_name

    def get_job_line_num(self):
        return self.job_line_num

    def get_job_step_num(self):
        return self.job_step_num

    def get_speed_override(self):
        return self.hse_speed_override

    def play_job(self, **kwargs):
        """
        kwargs:
        wait
        job_name
        line_num
        """
        # self.state = SIA20DriverState.INCOMPLETE
        # self.select_job(**kwargs)
        # self.play_job()     
        rospy.loginfo("Play Job")
        self.motoman_hse.play_job()
        rospy.sleep(0.5)

        # Check that the job is playing and play again if needed
        while not rospy.is_shutdown() and self.job_line_num == 0:
            rospy.loginfo("Playing Job again...")
            self.motoman_hse.play_job()
            rospy.sleep(0.5)
        
        if kwargs['wait'] == True:
            # based on job name wait until job line num is max
            try:
                rospy.loginfo("play_job waiting for motion to complete")
                while not rospy.is_shutdown() and self.job_line_num != (self.job_line_dict[self.job_name]+1):
                    rospy.loginfo("job line num = {}".format(self.job_line_num))
                    rospy.loginfo("break job line num = {}".format(self.job_line_dict[self.job_name]+1))
                    # rospy.loginfo("STATE = {}".format(self.state))
                    rospy.sleep(1.0)

                rospy.loginfo("play_job is done waiting")
            except:
                rospy.loginfo("play_job except, passing")
                pass

        self.state = SIA20DriverState.JOB_COMPLETE
        return True


    def movels(self, **kwargs):
        """ Execute a linear trajectory with SIA20
            Args:
                poseList_base   list list of poses (m3d.Transform)
                jointList       list list of joint-states (radians)
                use_job         bool flag
                job_name        string of job
                line_num        line num of job
        """
        self.state = SIA20DriverState.INCOMPLETE

        if "use_job" not in kwargs:
            kwargs["use_job"] = False

        # Case 0: poseList_base provided as pose
        if isinstance(kwargs["poseList_base"], Pose):
            kwargs["poseList_base"] = [kwargs["poseList_base"]]

        # Case 1: poseList_base provided as [x,y,z,roll,pitch,yaw], use list_to_tf to convert
        if isinstance(kwargs["poseList_base"][0], list):
            poseList = [list_to_tf(p) for p in kwargs["poseList_base"][:]]
            kwargs["poseList_base"] = poseList
        
        # Case 2: poseList_base provided as [geometry_msgs.Pose]
        if isinstance(kwargs["poseList_base"][0], Pose):
            # rospy.logwarn("Case 2: poseList_base provided as [geometry_msgs.Pose]")
            poseList = [geometry_msg_to_m3d(p) for p in kwargs["poseList_base"][:]]
            kwargs["poseList_base"] = poseList

        if "jointList" in kwargs:
            # convert the jointList input to the proper type (movejs only)
            # self.jtp_list_to_list(kwargs)
            self.convert_kwargs(kwargs)

        if self.is_sim:
            fct = fct = self.motoman_gazebo.movels

        elif kwargs["use_job"] == True:
            self.select_job(**kwargs)
            self.play_job(**kwargs)

        else:
            fct = self.motoman_hse.movels
            kwargs = self.add_to_kwargs(kwargs, fct)

            # should handle tf conversion
            poseList_mount = self.MOUNT_TO_BASE * kwargs["poseList_base"]
            kwargs["poseList_base"] = poseList_mount
            fct(**kwargs)

            if "wait" in kwargs:
                if kwargs["wait"] == True:
                    self.get_last_pt()
        
        self.state = SIA20DriverState.COMPLETE
        
        return True

    def stepl(self, **kwargs):
        if isinstance(kwargs["poseList_base"][0], list):
            poseList = [list_to_tf(p) for p in kwargs["poseList_base"][:]]
            kwargs["poseList_base"] = poseList

        if "pose" in kwargs:
            if isinstance(kwargs["pose"], list):
                kwargs["pose"] = list_to_tf(kwargs["pose"])

        if self.is_sim:
            fct = self.motoman_gazebo.stepl

        else:
            fct = self.motoman_hse.stepl
            kwargs = self.add_to_kwargs(kwargs, fct)

            # should handle tf conversion, required since moving in SIA20 base frame
            pose_mount = self.STEP_MOUNT_TO_BASE * kwargs["pose"] * self.BASE_TO_MOUNT
            kwargs["pose"] = pose_mount

        fct(**kwargs)
        return True

    def movejs(self, **kwargs):
        self.state = SIA20DriverState.INCOMPLETE

        # convert the jointList input to the proper type (movejs only)
        # self.jtp_list_to_list(kwargs)
        self.convert_kwargs(kwargs)

        if len(kwargs['jointList']) == 0:
            rospy.logwarn('movejs still has empty jointList, return w/o moving')
            self.state = SIA20DriverState.COMPLETE
            return False

        if self.is_sim:
            fct = self.motoman_gazebo.movejs
        else:
            fct = self.motoman_hse.movejs
            kwargs = self.add_to_kwargs(kwargs, fct)
        
        fct(**kwargs)

        if "wait" in kwargs:
            if kwargs["wait"] == True:
                self.get_last_pt()
        
        self.state = SIA20DriverState.COMPLETE
        return True

    def stepj(self, **kwargs):

        if self.is_sim:
            fct = self.motoman_gazebo.stepj
        else:
            fct = self.motoman_hse.stepj
            kwargs = self.add_to_kwargs(kwargs, fct)

        if "velocity" in kwargs:
            kwargs["vel_rot"] = kwargs.pop("velocity")
            
        fct(**kwargs)
        return True

    def convert_kwargs(self, kwargs):

        if "jointList" in kwargs:
            # case 0: put kwargs["jointList"] into a list
            if not isinstance(kwargs["jointList"], list):
                kwargs["jointList"] = [kwargs["jointList"]]

            # case 1: jtp type
            if isinstance(kwargs["jointList"][0], JointTrajectoryPoint):
                rospy.logwarn("jtp_list_to_list converting JointTrajectoryPoint to list")
                jointList = []
                for jt in kwargs["jointList"]:
                    jointList.append(list(jt.positions))
                kwargs["jointList"] = jointList

            # case 2: single list, instead of lists of lists
            if not isinstance(kwargs["jointList"][0], list):
                kwargs["jointList"] = [kwargs["jointList"]]
                rospy.logwarn("jtp_list_to_list converting list[] into list of list [[]]")
        
        if "poseList_base" in kwargs:
            # Case 0: poseList_base provided as pose, re-wrap into list
            if isinstance(kwargs["poseList_base"], Pose):
                kwargs["poseList_base"] = [kwargs["poseList_base"]]

            # Case 1: poseList_base provided as [x,y,z,roll,pitch,yaw], use list_to_tf to convert
            if isinstance(kwargs["poseList_base"][0], list):
                poseList = [list_to_tf(p) for p in kwargs["poseList_base"][:]]
                kwargs["poseList_base"] = poseList
            
            # Case 2: poseList_base provided as [geometry_msgs.Pose]
            if isinstance(kwargs["poseList_base"][0], Pose):
                # rospy.logwarn("Case 2: poseList_base provided as [geometry_msgs.Pose]")
                poseList = [geometry_msg_to_m3d(p) for p in kwargs["poseList_base"][:]]
                kwargs["poseList_base"] = poseList

            # Add "pose_goal" to kwargs
            kwargs["pose_goal"] = kwargs["poseList_base"][0]

    def jtp_list_to_list(self, kwargs):

        # case 0: put kwargs["jointList"] into a list
        if not isinstance(kwargs["jointList"], list):
            kwargs["jointList"] = [kwargs["jointList"]]

        # case 1: jtp type
        if isinstance(kwargs["jointList"][0], JointTrajectoryPoint):
            rospy.logwarn("jtp_list_to_list converting JointTrajectoryPoint to list")
            jointList = []
            for jt in kwargs["jointList"]:
                jointList.append(list(jt.positions))
            kwargs["jointList"] = jointList

        # case 2: single list, instead of lists of lists
        if not isinstance(kwargs["jointList"][0], list):
            kwargs["jointList"] = [kwargs["jointList"]]
            rospy.logwarn("jtp_list_to_list converting list[] into list of list [[]]")

        return

    def set_standard_digital_out(self):
        fct = self.motoman_hse.stepl
        kwargs = self.add_to_kwargs(kwargs, fct)
        fct(**kwargs)

    def update_hse_pose(self, source="tip_link"):
        """Update the m3d.Transform pose from saw_base to saw_TCP (self.pose_base)

        Get the pose from saw_base to saw_TCP. Default arg source can be set
        to "tip_link" or "FS100" to specify the source of the pose calculation.
        This function should only appear in the telemetry db once to prevent unnecessary 
        calls to HSE server. This function causes data transfer between HSE driver & server.

        Args:
            source = "tip_link": Calculated from saw_goelz.urdf.xacro (Calibration in tip_fixed_joint defines tool0 to tip_link)
            source = "FS100": Returns value from FS100 HSE server (based on teach pendant calibration)  

        Returns:
            True  - self.pose_base updated successfully
            False - Exception (because driver not initialized)
        """
        if self.state == SIA20DriverState.IDLE:
            self.pose_base = m3d.Transform()
            return True
        else:  
            try:
                if self.is_sim:
                    self.pose_base = self.motoman_gazebo.get_pose()
                    self.pose_base_fs100 = self.motoman_gazebo.get_pose()
                
                else:
                    # Get value of pose_base_fs100 from FS100 HSE Server and publish to /tf as saw_TCP
                    self.pose_base_fs100 = self.INVERSE_MOUNT_TO_BASE * self.motoman_hse.get_pose()
                    self.publish_tf(self.pose_base_fs100, parent_frame="saw_base", child_frame="saw_TCP")

                    # Get value of pose_base from tf lookup between saw_base and saw_tip_link (assumes calibrated TCP)
                    topic_i = self.arm_name + "_base"
                    topic_f = self.arm_name + "_tip_link"
                    self.pose_base = self.get_m3d_pose(topic_i, topic_f)

                    # Publish self.pose_base rostopic /saw/pose
                    self.pub_get_pose.publish(getFloatArrMsg(tf_to_list(self.pose_base)))

                return True
                
            except:
                self.pose_base = m3d.Transform()
                return False

    def publish_tf(self, transform, parent_frame="saw_base", child_frame="saw_TCP"):
        """ Publish ROS tf from parent_frame to child_frame
        This will be published at the rate of update_hse_pose()
        Args:
            transform: m3d.Transform() from parent_frame to child_frame
            parent_frame: string for parent_frame
            child_frame: string for child_frame
        """
        if self.state != SIA20DriverState.IDLE:
            try:
                t = TransformStamped()
                t.header.frame_id = parent_frame
                t.header.stamp = rospy.Time.now()
                t.child_frame_id = child_frame
                fillTF(transform.pos, transform.orient.get_quaternion(), t)
                tfm = ros_tf.msg.tfMessage([t])
                self.pub_tf.publish(tfm)
            except:
                rospy.logerr("sia20_driver publish_tf exception")
        else:
            return

    def get_pose(self, source = "tip_link"):
        """Getter to return self.pose_base when prompted by overseer telemetry
        
        Applications should call this function to receive most recently updated self.pose_base
        This function can be called multiple times in the telemetry db.
        """

        return self.pose_base

    def get_pose_list_degrees(self):
        """Getter to return pose_list when prompted by overseer tm (this function is for HMI)
        xyz in meters, rpy in degrees
        """
        if self.state == SIA20DriverState.IDLE:
            return [0]*6
        else:
            self.get_pose()
            pose_list = tf_to_list(self.pose_base)
            pose_list[3] = np.rad2deg(pose_list[3])
            pose_list[4] = np.rad2deg(pose_list[4])
            pose_list[5] = np.rad2deg(pose_list[5])
            return pose_list

    def get_pose_list(self):
        """Getter to return pose_list when prompted by overseer tm
        xyz in meters, rpy in radians
        """
        if self.state == SIA20DriverState.IDLE:
            return [0]*6
        else:
            self.get_pose()
            pose_list = tf_to_list(self.pose_base)
            return pose_list

    def get_m3d_pose(self, topic1, topic2):
        """Helper function for self.update_hse_pose() to calculate tf
        """
        (trans, rot) = self.listener.lookupTransform(topic1, topic2, rospy.Time(0)) # from 2 to 1 aka frame 2 w.r.t. frame 1
        pose = ROStoM3d(trans, rot)
        return pose

    def update_hse_joint_torques(self):
        """ Calls HSE function to get joint torque values and update self.joint_torques
        This function should only appear in the telemetry db once to prevent unnecessary 
        calls to HSE server. This function causes data transfer between HSE driver & server.

        Returns:
            True  - self.joint_torques updated successfully
            False - Exception (because driver not initialized)
        """
        try:
            self.joint_torques = self.motoman_hse.get_torque()
            return True
        except:
            self.joint_torques = [0] * 7
            return False

    def get_torque(self):
        """ Getter to return joint torque values (N) when prompted by overseer tm
        """
        return self.joint_torques
        
    def get_torque_percentage(self):
        """ Getter to return normalized joint torque values (%) when prompted by overseer tm
        """
        self.torque_percent = np.zeros(7)

        try:
            for i in range(0,7):
                self.torque_percent[i] = self.joint_torques[i] / SIA20_RATED_JOINT_SERVO_TORQUES[i]
                self.torque_percent[i] = round(self.torque_percent[i], 5)
        except:
            pass

        return self.torque_percent.tolist()

    def get_torque_max(self):
        return np.max(self.torque_percent)

    def get_tcp_speed(self):
        """ Getter to return self.tcp_speed (list) when prompted by overseer tm
        """
        self.tcp_speed = self.get_speed()
        return self.tcp_speed

    def get_tcp_speed_magnitude(self):
        """ Getter to return tcp speed magnitude
        """
        return np.linalg.norm(self.tcp_speed[0:3])


    def get_speed(self):
        """ Calls HSE function to update self.tcp_speed

        self.motoman_hse.get_speed() does not require data transfer to HSE server.
        The calculation is internal based last num_samples of pose data.
        """
        try:
            if self.is_sim:
                self.tcp_speed = self.motoman_gazebo.get_speed()
            else:
                speed_mount = self.motoman_hse.get_speed()

                # TODO: these lines below are causing huge CPU load
                if not isinstance(speed_mount, m3d.Transform):
                    speed_mount = list_to_tf(speed_mount)

                speed_base = self.BASE_TO_MOUNT * speed_mount # convert speed_mount to speed_base
                self.tcp_speed = np.abs(tf_to_arr(speed_base)).tolist()

        except:
            self.tcp_speed = [0] * 6
            
        return self.tcp_speed

    def get_rx_period(self):
        try:
            if self.is_sim:
                self.rx_period = self.motoman_gazebo.get_rx_period()
            else:
                self.rx_period = self.motoman_hse.get_rx_period()

        except:
            self.rx_period = 0

        return self.rx_period

    ### status and safety
    # returns True when executing movel, returns False when done moving (ie not executing movel)
    def get_robot_state(self):
        # return not self.motoman_hse.motionDone
        self.robot_state = self.is_robot_moving()
        return self.robot_state

    def is_robot_moving(self):
        try:
            if self.is_sim:
                return self.motoman_gazebo.is_robot_moving
            else:
                return self.motoman_hse.is_robot_moving
        except:
            return False

    def get_safety_status(self):
        """ Sends robot status data request to FS100 HSE Server.

        This function should only appear in the telemetry db once to prevent unnecessary 
        calls to HSE server. This function causes data transfer between HSE driver & server.

        Returns:
            True (error)
            False (okay)
        """
        try:
            if self.is_sim:
                return False
            else:
                return self.motoman_hse.get_status()
        except:
            return False

    # Called in is_stalled function
    def get_motion_alarm(self):
        """ Function called by robosaw control loop in the is_stalled() function.

        self.motoman_hse.get_motion_alarm() does not require data transfer to HSE server.

        Returns:
            True (motion alarm)
            False (okay)
        """
        try:
            if self.is_sim:
                return False
            else:
                ret = self.motoman_hse.get_motion_alarm()
                if ret == True:
                    rospy.logwarn("get_motion_alarm TRUE")
                    return True
                else:
                    return False
        except:
            return False

    def set_speed(self, **kwargs):
        """ Send a percentage (1-255%) to set_speed of initial arm motion command
        """

        if self.is_sim:
            pass
        else:
            fct = self.motoman_hse.set_speed
            kwargs = self.add_to_kwargs(kwargs, fct)
            fct(**kwargs)
        return
    
    def get_last_pt(self):
        """ Returns a dict for info about the last successfully executed move to a waypoint
        Called by stall_recovery() in robosaw
        self.motoman_hse.get_last_pt() does not require data transfer to HSE server.

        Returns dict with keys:
            duration
            jointList
            wait
            vel
            pose
            poseList_base
        """

        kwargs = {}

        if self.is_sim:
            kwargs["duration"] = 2.0
            kwargs["jointList"] = [self.motoman_gazebo.get_last_pt()]
            kwargs["wait"] = False
        
        else:
            kwargs["vel"] = 0.10 # TODO: Dummy value, replace with the value in HSE driver
            joints, pose = self.motoman_hse.get_last_pt()
            
            if pose is not None:
                # pose_mount = self.motoman_hse.list_to_transform(pose) # convert to SRM frame
                kwargs["pose"] = self.BASE_TO_MOUNT * pose
                kwargs["poseList_base"] =  [kwargs["pose"]]
                # rospy.loginfo("get_last_pt updated pose {}".format(tf_to_list(kwargs["pose"])))

            if joints is not None:
                kwargs["jointList"] = [joints]
                # rospy.loginfo("get_last_pt updated joints {}".format(joints))

            kwargs["wait"] = False

        # Map the kwargs to variables for overseer telemetry
        if "jointList" in kwargs:
            jointList = kwargs["jointList"]
            if isinstance(jointList[0], np.ndarray):
                jointList[0] = jointList[0].tolist()
            self.last_jointList = jointList
        if "poseList_base" in kwargs:
            poseList = [tf_to_list(p) for p in kwargs["poseList_base"][:]]
            self.last_poseList_base = poseList
        if "vel" in kwargs:
            self.last_vel = float(kwargs["vel"])
        if "wait" in kwargs:
            self.last_wait = kwargs["wait"]

        return kwargs

    # List of kwargs retrieved from get_last_pt()
    # Order is [jointList[], poseLise_base[], vel, wait]
    def get_last_pt_list(self):
        """ Returns a list for info about the last successfully executed move to a waypoint
        Used by overseer telemetry as overseer tm cannot return a dict

        Returns list with order:
            [jointList, poseList_base, vel, wait]
        """
        if self.last_wait is not None:
            self.last_wait = float(self.last_wait)

        return [self.last_jointList, self.last_poseList_base, self.last_vel, self.last_wait]

    ######### MotomanROS Commands #########

    ### deprecated stopj
    def stopj(self):
        self.motoman_ros.stopj()

    ### getters
    def get_joints(self):
        """ Returns a list joint_states as retrieved from MotomanROS
        This function causes data transfer between MotomanROS driver & server.

        Returns joints list with order:
            [S, L, E, U, R, B, T]
        """
        try:
            if self.is_sim:
                self.joints = self.motoman_gazebo.get_joints()
            else:
                self.joints = self.motoman_ros.get_joints() # only used when not in overseer mode
        except:
            self.joints = [0.0] * 7
        
        return self.joints

    # Need to call get_joints() since data_publisher is not active
    # TODO: should overseer call update_tm?
    def get_joints_degrees(self):
        return np.rad2deg(np.asarray(self.get_joints())).tolist()

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
        pose_tf = list_to_tf(pose_vector)
        self.nominal_tcp_to_tcp = m3d.Transform(pose_tf).inverse

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

    def smart_move(self, **kwargs):
        self.state = SIA20DriverState.INCOMPLETE
        self.mp_success = False
        retval = True

        # convert the jointList input to the proper type (movejs only)
        # self.jtp_list_to_list(kwargs)
        self.convert_kwargs(kwargs)

        # Iterate thru jointList goals
        if "jointList" in kwargs:

            for j in kwargs["jointList"]:
                kwargs["joints_goal"] = j
                retval = self.smart_motion(**kwargs)

                if retval == False:
                    if self.state == SIA20DriverState.FAILED:
                        rospy.logwarn("Smart move FAILED because MP was not started")
                        retval = False
                    else:
                        break
        
        # Iterate thru poseLise_base goals
        elif "poseList_base" in kwargs:
            for p in kwargs["poseList_base"]:
                kwargs["pose_goal"] = p
                retval = self.smart_motion(**kwargs)

                if retval == False:
                    if self.state == SIA20DriverState.FAILED:
                        rospy.logwarn("Smart move FAILED because MP was not started")
                        retval = False
                    else:
                        break

        # Pass thru to smart_motion
        else:
            retval = self.smart_motion(**kwargs)

        if retval == False:
            rospy.logerr("SIA20 smart_move failed!")
            retval = True # return True or else recovery mission won't trigger
        else:
            retval = True
        
        time.sleep(0.5)
        self.state = SIA20DriverState.COMPLETE

        return retval

    def smart_motion(self, **kwargs):
        # Attempt to start motion_planner, if unable to do so return a failed smart move.
        if self.motion_planner == None:
            try:
                self.motion_planner = MotionPlanner("saw", self.frame_id_)
            except:
                rospy.logwarn("SIA20 Motion Planning was not launched, unable to plan smart move.")
                self.motion_planner = None
                self.mp_success = False
                self.state = SIA20DriverState.FAILED
                return False

        # fill out telemetry
        # if "pose_goal" in kwargs:          
        #     self.last_mp_pose_goal = kwargs["pose_goal"]
        # else:
        #     self.last_mp_pose_goal = []
            
        if "joints_goal" in kwargs:
            self.last_mp_joints_goal = kwargs["joints_goal"]
        else:
            self.last_mp_joints_goal = []

        # Handle case of open-loop motion (from saw_smart_move_recovery mission using edit_args)
        if "open_loop" not in kwargs:
            kwargs["open_loop"] = False

        if kwargs["open_loop"]:
            rospy.logwarn("Attempting open_loop move")
            self.set_speed(vel=100)
            kwargs["wait"] = True
            kwargs["vel_rot"] = np.deg2rad(3.0) # assign 3.0 deg/s as low open-loop velocity
            self.movejs(**kwargs)
            self.mp_success = True
            self.state = SIA20DriverState.COMPLETE
            return self.mp_success

        if "pose_goal" in kwargs and "joints_goal" not in kwargs:
            
            pose_goal = kwargs["pose_goal"]
            if type(pose_goal) == list:
                if type(pose_goal[0]) == Pose:
                    goal = pose_goal[0]                
                # goal = list_to_pose_msg(pose_goal)
            elif type(pose_goal) == dict:
                goal = message_converter.convert_dictionary_to_ros_message(pose_goal)
            elif type(pose_goal) == m3d.Transform:
                goal = m3d_to_ros(pose_goal)

        else:
            joints_goal = kwargs["joints_goal"]
            if type(joints_goal[0]) == list:
                joints_goal = joints_goal[0]

            if isinstance(joints_goal, np.ndarray):
                joints_goal = joints_goal.tolist()

            goal = joints_goal
            self.goal_joints = deepcopy(joints_goal)
        
        jlist, _ = self.motion_planner.plan(start_js=self.get_joints(), goal=goal)    

        # Successful motion plan
        if jlist:
            vel = float(np.deg2rad(5.0))
            if "vel_rot" in kwargs:
                vel = max(vel, kwargs["vel_rot"]) # max of (5 deg/s, vel_rot)
                
            self.set_speed(vel=100)
            self.movejs(jointList=jlist, vel_rot=vel, duration=1.0, wait=True) # threshold
            self.mp_success = True
            self.last_mp = jlist

        # Failed motion plan
        else:
            rospy.logwarn("Motion plan failed.")
            plan_interpolated_pose = False

            if plan_interpolated_pose == True:
                if "pose_goal" not in kwargs:
                    rospy.logwarn("No pose_goal so cannot attempt interpolated pose, motion planning failed!")
                    self.mp_success = False
                    rospy.sleep(0.5)

                else:
                    rospy.logwarn("Attempting plan again with interpolated pose.")
                    self.mp_success = self.plan_interpolated_pose(**kwargs)

                # If mp_success still False, query user for open-loop motion before aborting
                if self.mp_success == False:
                    self.last_mp = [[]]
                    rospy.logwarn("Smart move failed, use saw_smart_move_recovery")
            
            else:
                pass

        return self.mp_success

    def compare_joints(self, goal_joints = None, threshold = None, wait = False, timeout=120.0):
        """ Compares current joint-state to goal_joints to determine arm is currently at goal_joints
            Args:
                goal_joints (list): list of target joint-state
                threshold (radians): The sum of the difference in each joint must be lower than threshold
                wait (bool): Blocks return until the goal_joints is reached or timeout. Only should use when running private op.
                timeout (float): Function returns after timeout
            Return:
                True if current joint-state is within threshold of goal_joints, False if not
        """
        if threshold == None:
            threshold = np.deg2rad(1.0)
        
        try:
            if wait == False:
                return np.sum(np.abs(np.array(self.joints - np.array(goal_joints))) < np.deg2rad(threshold))
            else:
                t_start = rospy.get_time()
                retval = False

                # Block move_to_pre_plunge() from continuing until the motion is completed or timeout
                while rospy.get_time() - t_start < timeout:
                    if np.sum(np.abs(np.array(self.joints - np.array(goal_joints))) < np.deg2rad(threshold)):
                        retval = True
                        break
                    rospy.sleep(0.5)
            
                return retval

        except:
            rospy.logerr("Exception in compare_joints due to format error with goal_joints {}".format(goal_joints))
            return False

    def move_to_pre_plunge(self, **kwargs):
        """ Overseer async op called to move arm fron pre_plunge_seed to pre-plunge for specific slot.
            Plans motions from known states (pre_plunge_seed, unstow, zeros) to slot pre-plunge. If successful
            plan is found, the arm will first execute activities to move from pre_plunge_seed to known position, 
            then follow the planned joint traj to reach slot pre-plunge.

            Args:
                jointList: list list of slot's pre-plunge jointList
            Returns:
                True/False for successful motion plan. Note for async ops the return value is insignificant, 
                look at the SAW.ARM.MP_SUCCESS telemetry to determine if motion was successful.
            
        """
        self.convert_kwargs(kwargs)
        goal = kwargs["jointList"][0]
        self.state = SIA20DriverState.INCOMPLETE
        vel = float(np.deg2rad(5.0))

        # Define hard-coded joint states
        self.pp_seed_js = [-1.401734709739685, -0.22796660661697388, 0.9733960628509521, -1.893000602722168, 1.1397932767868042, 0.6715342402458191, 0.7807060480117798]
        self.unstow_js = [-0.3290060382890889, 0.07041149845741845, 0.003250958398522741, -2.2234799156877165, 0.8850857549141891, 0.9706233049751605, 1.4712016609490295]
        self.right_edge_js = [-0.5880883192110806, 0.6985693117897026, 0.5757256120953244, -2.1119834035538605, 1.0424390799513086, 1.0462394793966319, 1.8963687173471786]
        self.zeros_js = [0, 0, 0, 0, 0, 0, 0]

        # First, attempt a motion plan from the current joints
        rospy.loginfo("attempt plan from current joints")
        jlist, _ = self.motion_planner.plan(start_js=self.joints, goal=goal)
        if jlist:
            rospy.loginfo("Found MP from current joints. Execute mp.")
            self.set_speed(vel=100)
            self.movejs(jointList=jlist, vel_rot=vel, duration=1.0, wait=True) # threshold
            self.mp_success = True
            self.last_mp = jlist
            self.state = SIA20DriverState.COMPLETE
            return True

        # If not at pre-plunge seed, attempt smart_move
        if self.compare_joints(goal_joints=self.pp_seed_js, wait=False) == False:

            # move from pp_seed_js to unstow
            rospy.loginfo("not at pre-plunge so attempt MP to pre-plunge")
            self.overseer_api.run_private_op("REMOVE_ACTIVITY", activity_name="robosaw_smart_move_to_pre_plunge")
            self.overseer_api.run_private_op("LOAD_ACTIVITY", activity_name="robosaw_smart_move_to_pre_plunge")
            self.overseer_api.run_private_op("START_ACTIVITY", activity_name="robosaw_smart_move_to_pre_plunge")
            if (self.compare_joints(goal_joints=self.unstow_js, wait=True)) == False:
                rospy.logerr("Failed to smart_move to pre_plunge_seed, move_to_pre_plunge failed.")
                return False


        # Start plan from current joints (should be pp_seed_js)
        rospy.loginfo("attempt plan from pp_seed_js")
        jlist, _ = self.motion_planner.plan(start_js=self.pp_seed_js, goal=goal)
        if jlist:
            rospy.loginfo("Found MP from pre_plunge_seed. Execute mp.")
            self.set_speed(vel=100)
            self.movejs(jointList=jlist, vel_rot=vel, duration=1.0, wait=True) # threshold
            self.mp_success = True
            self.last_mp = jlist
            self.state = SIA20DriverState.COMPLETE
            return True

        # Then plan from unstow
        rospy.loginfo("attempt plan from unstow")
        jlist, _ = self.motion_planner.plan(start_js=self.unstow_js, goal=goal)
        if jlist:
            rospy.loginfo("Found MP from unstow. First move to unstow.")

            # move from pp_seed_js to unstow
            self.overseer_api.run_private_op("REMOVE_ACTIVITY", activity_name="robosaw_pre_plunge_to_unstow")
            self.overseer_api.run_private_op("LOAD_ACTIVITY", activity_name="robosaw_pre_plunge_to_unstow")
            self.overseer_api.run_private_op("START_ACTIVITY", activity_name="robosaw_pre_plunge_to_unstow")
            if (self.compare_joints(goal_joints=self.unstow_js, wait=True)) == False:
                rospy.logerr("Failed to move from pre_plunge to unstow, move_to_pre_plunge failed.")
                return False
            
            rospy.loginfo("Now execute MP from unstow to pre plunge")
            self.set_speed(vel=100)
            self.movejs(jointList=jlist, vel_rot=vel, duration=1.0, wait=True) # threshold
            self.mp_success = True
            self.last_mp = jlist
            self.state = SIA20DriverState.COMPLETE
            return True

        # Then plan zeros
        rospy.loginfo("attempt plan from zeros")
        jlist, _ = self.motion_planner.plan(start_js=self.zeros_js, goal=goal)
        if jlist:
            rospy.loginfo("Found MP from zeros. First move to zeros.")

            # move from pp_seed_js to unstow
            self.overseer_api.run_private_op("REMOVE_ACTIVITY", activity_name="robosaw_pre_plunge_to_zeros")
            self.overseer_api.run_private_op("LOAD_ACTIVITY", activity_name="robosaw_pre_plunge_to_zeros")
            self.overseer_api.run_private_op("START_ACTIVITY", activity_name="robosaw_pre_plunge_to_zeros")
            if (self.compare_joints(goal_joints=self.zeros_js, wait=True)) == False:
                rospy.logerr("Failed to move from pre_plunge to zeros, move_to_pre_plunge failed.")
                return False
            
            rospy.loginfo("Now execute MP from zeros to pre plunge")
            self.set_speed(vel=100)
            self.movejs(jointList=jlist, vel_rot=vel, duration=1.0, wait=True) # threshold
            self.mp_success = True
            self.last_mp = jlist
            self.state = SIA20DriverState.COMPLETE
            return True

        # If this part is reached, then the MP failed
        self.mp_success = False
        self.last_mp = []
        self.state = SIA20DriverState.COMPLETE
        return False

    def move_to_orthogonal(self, translation = None, execute_motion = True):
        """ Use motion planner to plan to a orthogonal pose. This is used for TCP calibration.
        Args:
            translation: List (x,y,z) of translation of saw_TCP. If set to None, use the current SIA20 pose translation.
            execute_motion: boolean flag, if True then immediately execute the motion as a smart_move
        Returns:
            True if the motion plan is found
        """
        self.state = SIA20DriverState.INCOMPLETE
        if translation is None:
            self.goal_pose = self.get_pose()
        else:
            self.goal_pose = m3d.Transform()
            self.goal_pose.pos.x = translation[0]
            self.goal_pose.pos.y = translation[1]
            self.goal_pose.pos.z = translation[2]

        self.goal_pose.orient = getHorizontalOrient()
        self.goal_pose_list = tf_to_list(self.goal_pose)
        self.publish_pose_list([self.goal_pose])
        self.geom_pose = m3d_to_ros(self.goal_pose)

        move_vel = np.deg2rad(5.0)
        retval = self.smart_move(poseList_base=[self.goal_pose_list], vel_rot=move_vel, wait=True)

        if retval == True:
            self.move_to_orthogonal_jlist = self.last_mp
            rospy.loginfo("move_to_orthogonal success with jlist {}".format(self.move_to_orthogonal_jlist))
        else:
            self.move_to_orthogonal_jlist = [[]]
            rospy.logwarn("move_to_orthogonal failed")

        self.state = SIA20DriverState.COMPLETE
        return retval

    def return_from_orthogonal(self):
        """ Returns robosaw from orthogonal bias js back to original js. 
        Only call this from robosaw_bias_and_return_to_orthogonal activity.
        Args/Returns: None
        """
        self.state = SIA20DriverState.INCOMPLETE

        if self.move_to_orthogonal_jlist == [[]]:
            rospy.logwarn("Empty move_to_orthogonal_jlist, cannot return from orthogonal")
        
        else:
            # Reverse the jlist
            self.move_to_orthogonal_jlist.reverse()

            # Execute the motion
            rospy.loginfo("Return from orthogonal with jlist {}".format(self.move_to_orthogonal_jlist))
            vel = float(np.deg2rad(5.0))
            self.set_speed(vel=100)
            self.movejs(jointList=self.move_to_orthogonal_jlist, vel_rot=vel, duration=1.0, wait=True) # threshold

            # Reset to empty jlist so motion cannot be done again by accident
            self.move_to_orthogonal_jlist = [[]]

        self.state = SIA20DriverState.COMPLETE

    def publish_pose_list(self, pose_list):
        msg = PoseArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id_
        for m3d_pose in pose_list:
            ros_pose = m3d_to_ros(m3d_pose)
            msg.poses.append(ros_pose)
        self.pose_list_publisher.publish(msg)

    def plan_interpolated_pose(self, **kwargs):
        # if no pose goal, this method fails
        if not "pose_goal" in kwargs:
            rospy.logwarn("No pose_goal, method 2 fails. User needs to select a different strategy")
            return False

        # TODO: work on this linear interp
        joints_goal = kwargs["joints_goal"]
        pose_goal = kwargs["pose_goal"]
        # Instead of linear interpolation can also be a random sampling in (x, y, z, r, p, y)
        # with ranges defined between current and goal pose
        interp_pose = linear_interp_tf(self.get_pose(), pose_goal, pose_spacing=0.02) # consider reversing interp_pose, so more cartesian space is covered
        # in_collision, colliding_links, poses_m3d, jt = self.motion_planner.check_collision_list("saw", interp_pose, excluded_links=[], seed_js=joints_goal)
        in_collision, colliding_links, poses_m3d, jt = self.motion_planner.check_collision_list_slot("saw", interp_pose, [""], \
            seed_js=joints_goal, dist_from_seed=[0.7]*7)
        success_js = None
        success_pose = None
        for idx, jtp in enumerate(jt):
            if in_collision[idx]:
                # jointstate is in collision or IK not found
                continue
            try:
                jlist_interp, _ = self.motion_planner.plan(start_js=self.get_joints(), goal=list(jtp))
                if jlist_interp:
                    self.movejs(jointList=jlist_interp, vel_rot=np.deg2rad(5.0), duration=1.0)
                    success_js = list(jtp)
                    success_pose = poses_m3d[idx]
                    break
                else:
                    rospy.logwarn("could not plan to intermediate jointstate = {}".format(jtp))
                    continue
            except:
                rospy.logerr("error in motion planner plan function")
        
        if success_js:
            # we were able to plan to a jointstate within 0.7 radians of the goal jointstate
            # first try to use motion plan again from this jointstate
            jlist, _ = self.motion_planner.plan(start_js=self.get_joints(), goal=self.goal_joints)
            jlist = None # explicity use movelj for now
            if jlist:
                self.movejs(jointList=jlist, vel_rot=np.deg2rad(5.0), duration=1.0)
                self.last_mp = jlist
                retval = True
            else:
                rospy.loginfo("attempting to generate movelj trajectory")
                linear_move_pose_list = linear_interp_tf(self.get_pose(), pose_goal, pose_spacing=0.03)
                in_collision, colliding_links, poses_m3d, jt = self.motion_planner.check_collision_list_slot("saw", linear_move_pose_list, [""], \
                    seed_js=joints_goal, dist_from_seed=[0.7]*7)
                
                # If there are collisions, reject and return False
                if True in in_collision:
                    rospy.logerr("could not find collision free joint trajectory for movelj")
                    retval = False
                    return retval

                # Otherwise, create the list and perform movelj
                joint_traj = []
                for jtp in jt:
                    joint_traj.append(list(jtp))
                joint_traj.append(self.goal_joints)
                pose_traj = linear_move_pose_list + [pose_goal]

                try:
                    self.movels(jointList=joint_traj, poseList_base=pose_traj, vel=0.010, wait=True) # vel_rot=np.deg2rad(30.0)
                    retval = True
                except:
                    rospy.logerr("movels failed")
                    retval = False
        else:
            rospy.logerr("could not find a plan to any intermediate jointstate")
            retval = False
        
        return retval
            
    def get_mp_success(self):
        return self.mp_success

    def get_last_mp(self):
        """ Returns last attempted motion plan (list of joint-states)
        If last attempted motion plan failed, then this returns an empty list [[]]
        """
        return self.last_mp

    def get_last_mp_joints_goal(self):
        """ Returns list of of last attempted motion plan joints goal (regardless of success/failure)
        """
        return self.last_mp_joints_goal

    def get_last_mp_pose_goal(self):
        """ Returns list of of last attempted motion plan pose goal (regardless of success/failure)
        """
        return self.last_mp_pose_goal

    def get_init_complete(self):
        """ Returns boolean to show if sia20_driver has been initialized successfully"""
        return self.init_complete

    def init_overseer_publishers(self):
        # Joint torque publishers for influxdb
        self.pub_torque_1 = rospy.Publisher(self.ns_prefix+"/joint_torque_1", Float32, queue_size=30, latch=True)
        self.pub_torque_2 = rospy.Publisher(self.ns_prefix+"/joint_torque_2", Float32, queue_size=5, latch=True)
        self.pub_torque_3 = rospy.Publisher(self.ns_prefix+"/joint_torque_3", Float32, queue_size=5, latch=True)
        self.pub_torque_4 = rospy.Publisher(self.ns_prefix+"/joint_torque_4", Float32, queue_size=5, latch=True)
        self.pub_torque_5 = rospy.Publisher(self.ns_prefix+"/joint_torque_5", Float32, queue_size=5, latch=True)
        self.pub_torque_6 = rospy.Publisher(self.ns_prefix+"/joint_torque_6", Float32, queue_size=5, latch=True)
        self.pub_torque_7 = rospy.Publisher(self.ns_prefix+"/joint_torque_7", Float32, queue_size=5, latch=True)

        # Joint pos publishers for influxdb
        self.pub_joint_1 = rospy.Publisher(self.ns_prefix+"/joint_1", Float32, queue_size=30, latch=True)
        self.pub_joint_2 = rospy.Publisher(self.ns_prefix+"/joint_2", Float32, queue_size=30, latch=True)
        self.pub_joint_3 = rospy.Publisher(self.ns_prefix+"/joint_3", Float32, queue_size=30, latch=True)
        self.pub_joint_4 = rospy.Publisher(self.ns_prefix+"/joint_4", Float32, queue_size=30, latch=True)
        self.pub_joint_5 = rospy.Publisher(self.ns_prefix+"/joint_5", Float32, queue_size=30, latch=True)
        self.pub_joint_6 = rospy.Publisher(self.ns_prefix+"/joint_6", Float32, queue_size=30, latch=True)
        self.pub_joint_7 = rospy.Publisher(self.ns_prefix+"/joint_7", Float32, queue_size=30, latch=True)

        # Arm pose publishers for influxdb
        self.pub_pose_x  = rospy.Publisher(self.ns_prefix+"/pose_x", Float32, queue_size=5, latch=True)
        self.pub_pose_y  = rospy.Publisher(self.ns_prefix+"/pose_y", Float32, queue_size=5, latch=True)
        self.pub_pose_z  = rospy.Publisher(self.ns_prefix+"/pose_z", Float32, queue_size=5, latch=True)
        self.pub_pose_rx = rospy.Publisher(self.ns_prefix+"/pose_rx", Float32, queue_size=5, latch=True)
        self.pub_pose_ry = rospy.Publisher(self.ns_prefix+"/pose_ry", Float32, queue_size=5, latch=True)
        self.pub_pose_rz = rospy.Publisher(self.ns_prefix+"/pose_rz", Float32, queue_size=5, latch=True)

        # TCP speed publishers for influxdb
        self.pub_vel_x  = rospy.Publisher(self.ns_prefix+"/vel_x", Float32, queue_size=50, latch=True)
        self.pub_vel_y  = rospy.Publisher(self.ns_prefix+"/vel_y", Float32, queue_size=50, latch=True)
        self.pub_vel_z  = rospy.Publisher(self.ns_prefix+"/vel_z", Float32, queue_size=50, latch=True)
        return
    
    def update_tm(self):
        self.get_joints() # returns self.joints
        self.get_pose()  # returns self.pose_base
        self.get_speed() # returns self.tcp_speed
        self.get_torque() # returns self.joint_torques
        self.get_torque_percentage() # returns self.torque_percent
        self.get_robot_state() # return self.robot_state

    def data_publisher(self):
        """ publishes tf and joints for this UR10. Follows same standard as ur_modern_driver
        get_pose
        get_speed
        get_joints
        get_force
        gate_status
        """
        pub_tf = rospy.Publisher('/tf', ros_tf.msg.tfMessage, queue_size=1)
        pub_tcp_speed = rospy.Publisher(self.ns_prefix+'/tcp_speed', Float32MultiArray, queue_size=5, latch=True)
        pub_torques = rospy.Publisher(self.ns_prefix+'/joint_torques', JointState, queue_size=5, latch=True)
        pub_tcp_force = rospy.Publisher(self.ns_prefix+'/wrench', WrenchStamped, queue_size=5, latch=True)
        pub_is_gate_open = rospy.Publisher(self.ns_prefix+'/is_gate_open', Bool, queue_size=5, latch=True)
        pub_is_robot_moving = rospy.Publisher(self.ns_prefix+'/is_robot_moving', Bool, queue_size=5, latch=True)
        pub_robot_ready = rospy.Publisher(self.ns_prefix+'/is_robot_ready', Bool, queue_size=5, latch=True)
        pub_get_pose = rospy.Publisher(self.ns_prefix+'/pose', Float32MultiArray, queue_size=5, latch=True)
        
        # Initialize overseer publishers
        self.init_overseer_publishers()

        rospy.loginfo("Data Publisher ready and starting.")

        # static transforms TODO: put nominal-tcp-to-tcp here (but then have to update it if tf changes from user)

        start_time = time.time()
        one_sec_time = time.time()
        counter = 0
        self.update_tm()

        while not rospy.is_shutdown():
            if self.ARM_RUNNING and self.use_data_publisher:
                # Every 1 second, calculate hz
                if self.debug_mode:
                    counter += 1
                    if time.time() - one_sec_time > 1.0:
                        # rospy.loginfo("{} SIA20 Hz".format(counter))
                        counter = 0
                        one_sec_time = time.time()

                self.update_tm()

                time.sleep(0.010) # throttle to ~75 hz

                # TCP pose in base frame    1000 Hz (1 ms)
                t = TransformStamped()
                t.header.frame_id = self.arm_name+"_base"
                self.robot_pose = self.pose_base
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
                
                # # TCP speed in base frame     2000 Hz (0.5 ms)
                speed_base = self.tcp_speed
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

                # TCP joint-state     2500 Hz (0.4 ms)
                joints = self.joints
                msg = JointState()
                msg.position[:] = joints[:] # copy into the msg
                msg.name = self.joint_names_prefixed

                # Get joint torques
                msg.effort[:] = self.joint_torques

                # publish the joint torques (Newtons) of SIA20 Arm into individual topics for influxdb
                self.pub_torque_1.publish(msg.effort[0])
                self.pub_torque_2.publish(msg.effort[1])
                self.pub_torque_3.publish(msg.effort[2])
                self.pub_torque_4.publish(msg.effort[3])
                self.pub_torque_5.publish(msg.effort[4])
                self.pub_torque_6.publish(msg.effort[5])
                self.pub_torque_7.publish(msg.effort[6])

                # # publish joint positions of sia20 arm into individual topics for influxdb
                self.pub_joint_1.publish(msg.position[0])
                self.pub_joint_2.publish(msg.position[1])
                self.pub_joint_3.publish(msg.position[2])
                self.pub_joint_4.publish(msg.position[3])
                self.pub_joint_5.publish(msg.position[4])
                self.pub_joint_6.publish(msg.position[5])
                self.pub_joint_7.publish(msg.position[6])

                # now modify pub_torques publisher to have percentages for torques
                # self.torque_percent = np.zeros(7)
                # for i in range(0,7):
                #     self.torque_percent[i] = msg.effort[i] / SIA20_RATED_JOINT_SERVO_TORQUES[i]
                #     self.torque_percent[i] = round(self.torque_percent[i], 5)
                    
                msg.effort = self.torque_percent.tolist()

                # Timestamp pub_joints msg      2500 Hz (0.4 ms)
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = self.arm_name

                # publish % torque onto saw/joint_torques/effort
                pub_torques.publish(msg)

                #gate
                msg = Bool(self.safety_gate.is_set()) # true if open
                pub_is_gate_open.publish(msg)
                msg = Bool(self.robot_state)
                pub_is_robot_moving.publish(msg)

                try: 
                    
                    msg = Bool(self.get_robot_mode() == 7)
                    pub_robot_ready.publish(msg)
                except:
                    pass

    ############################### mission op wrappers #################################

    # input: op_stepj(joint="joint_s", ang_velocity=10.0, duration=1.0)
    # units are deg/s, s
    # output: joints[] in rads for stepj
    def op_stepj(self, **kwargs):

        if self.state == SIA20DriverState.IDLE:
            rospy.logwarn("SIA20 not yet initialized, run init_sia20 mission, stepj command ignored")
            return False

        # default velocity values for joints
        new_kwargs = {'joint': 'joint_s', 'ang_velocity' : 5.0, 'joint_s' : 0, 'joint_l' : 0, 'joint_e' : 0, 'joint_u' : 0,
                    'joint_r' : 0, 'joint_b' : 0, 'joint_t' : 0, 'duration' : 1.0}
        new_kwargs.update(kwargs)

        # Set the joint value of the joint to be stepped
        new_kwargs[new_kwargs["joint"]] = float(np.deg2rad(float(new_kwargs["duration"])) * new_kwargs["ang_velocity"])

        # filter kwargs by joint names
        joint_kwargs = { k: new_kwargs[k] for k in self.joint_names }

        # Order the kwargs based on the joint_names list
        index_map = {v: i for i, v in enumerate(self.joint_names)}
        sorted_kwargs = sorted(joint_kwargs.items(), key=lambda pair: index_map[pair[0]])  # Ordered list of tuples
        velocities = np.asarray(list(map(lambda x: x[1], sorted_kwargs))) # extract just the joint values
        
        joint_distances = velocities[:]

        # calculate vel_rot based on the joint step and duration
        new_kwargs["vel_rot"] = np.divide(np.max(np.abs(joint_distances)), new_kwargs["duration"])
        new_kwargs["joints"] = joint_distances # np array
        new_kwargs["wait"] = False
        new_kwargs["radius"] = 0.03
        new_kwargs["threshold"] = 0.01
        new_kwargs["timeout"] = 3.0 * kwargs["duration"] # timeout is 3x of duration
  
        # Call movejs
        self.stepj(**new_kwargs)
        return True

    # Sample input: op_movej(shoulder_pan_joint=0.2, shoulder_lift_joint=1.3, elbow_joint=2, wrist_1_joint=3, wrist_2_joint=4, wrist_3_joint=5, vel=0.5, wait=True)
    def op_movejs(self, **kwargs):

        if self.state == SIA20DriverState.IDLE:
            rospy.logwarn("SIA20 not yet initialized, run init_sia20 mission, movej command ignored")
            return False

        # filter kwargs by joint names
        joint_kwargs = { k: kwargs[k] for k in self.joint_names }

        # Order the kwargs based on the joint_names list
        index_map = {v: i for i, v in enumerate(self.joint_names)}
        sorted_kwargs = sorted(joint_kwargs.items(), key=lambda pair: index_map[pair[0]])  # Ordered list of tuples
        jointList = [list(map(lambda x: x[1], sorted_kwargs))] # extract just the joint values
        new_kwargs = {}

        # convert jointList from deg to rad
        jointList = np.deg2rad(np.asarray(jointList)).tolist()
        
        # Update kwargs, remove the joint names
        for name in self.joint_names:
            del kwargs[name]
        new_kwargs["jointList"] = jointList
        new_kwargs["acc"] = kwargs["acceleration"]
        new_kwargs["vel_rot"] = np.deg2rad(10.0) # placeholder constant velocity
        new_kwargs["wait"] = False
        new_kwargs["radius"] = 0.03
        new_kwargs["threshold"] = 0.01

        # Call movejs
        self.movejs(**new_kwargs)
        return True

    # Sample input: op_movel(x=0, y=1, z=2, roll=0, yaw=0, pitch=0, vel=0.5, wait=True)
    # (jointList=[jtp.positions], poseList_base=[tf], vel = vel_plunge, wait=False)
    def op_movel(self, **kwargs):

        if self.state == SIA20DriverState.IDLE:
            rospy.logwarn("SIA20 not yet initialized, run init_sia20 mission, movel command ignored")
            return False

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

        kwargs["wait"] = False

        # Call movel
        self.movel(**kwargs)

        return True

    # Sample input: x=0, y=0, z=-0.010, roll=5.0, pitch=0, yaw=0, duration=1.0
    # inputs velocity, angular velocity, duration
    # units are m/s, deg/s, s
    def op_stepl(self, **kwargs):
        '''
        HMI Forward/Away -- +/- X
        HMI Up/Down --- +/- Z
        HMI Left/right --- +/- Y
        HMI Roll
        HMI Pitch
        HMI Yaw Right/Left
        '''

        if self.state == SIA20DriverState.IDLE:
            rospy.logwarn("SIA20 not yet initialized, run init_sia20 mission, stepl command ignored")
            return False

        # default values for kwargs
        new_kwargs = {'x': 0, 'y': 0, 'z': 0, 'roll': 0, 'pitch': 0, 'yaw': 0, 'duration': 0.3}
        new_kwargs.update(kwargs)

        # Calculate poseList_base based on velocity & duration for each coordinate
        velocities = np.array([new_kwargs["x"], new_kwargs["y"], new_kwargs["z"], 
                        new_kwargs["roll"], new_kwargs["pitch"], new_kwargs["yaw"]])
        step_translations = velocities[0:3] * kwargs["duration"]
        step_rotations = np.deg2rad(velocities[3:6]) * kwargs["duration"]

        # Convert step_distances to a tf
        step_distances = np.append(step_translations, step_rotations)
        step_distances = list_to_tf(step_distances)

        # Add poseList and vel to new_kwargs
        new_kwargs["vel"] = float(np.max(np.abs(velocities[0:3])))
        new_kwargs["vel_rot"] = float(np.deg2rad(max(velocities[3:])))
        new_kwargs["poseList_base"] = [step_distances]
        new_kwargs["pose"] = step_distances
        new_kwargs["wait"] = False
        new_kwargs["duration"] = kwargs["duration"]
        new_kwargs["timeout"] = 3.0 * kwargs["duration"] # timeout is 3x of duration

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
            sia20_driver = SIA20Driver()
            sia20_driver.initialize_robot()
            time.sleep(1.0)

        if run_test:
            test(run_api, sim)

        if not run_api:
            rospy.loginfo("sia20_driver: Start spinning forever now.")
            rospy.spin()
        
    finally:
        if not rospy.is_shutdown(): rospy.signal_shutdown("End of Program")