import math3d as m3d
import numpy as np
import math, time, os, sys, signal, threading, copy
import pdb

import rospy, roslib, cv2
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool, Empty
from std_msgs.msg import Float32
import image_geometry
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import PointCloud2
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
import sensor_msgs.point_cloud2 as pc2
import message_filters
import threading, socket
import trajectory_msgs.msg
import moveit_msgs.msg
from rospy_message_converter import message_converter

#
import tf as ros_tf
import tf.msg # can be accessed with ros_tf.msg
from geometry_msgs.msg import WrenchStamped, TransformStamped
from sensor_msgs.msg import JointState 

from bot_common_ros.ur_utils import *
from bot_common_ros.ur_utils import rosParamToClassObject, m3d_to_ros, ROStoM3d, movel, movej, movejs, force_mode, get_force, get_speed, ROStoM3d_orient, \
                     get_pose, get_joints, movels, list_to_tf

from bot_common_ros.ur_control import force_stop2

# 
from bot_common_ros.msg import ArmCommand, ArmSetup
from bot_common_ros.srv import ArmCmdSrv, ArmSetupSrv
from ur10_driver import UR10Driver
from sia20_driver import SIA20Driver

class ArmClient(object):
    """ Client to interface with arm_manager
    in future may inherit from URMiddleWare
    """
    def __init__(self, client_input_param_path=None, sim=None, run_as_api=None, server_input_param_path=None, server_dry_run=None):

        # Overseer calls constructor with empty args
        if all(v is None for v in [client_input_param_path, sim, run_as_api, server_input_param_path, server_dry_run]):
            rospy.loginfo("empty args")

        # launch file calls constructor with args
        else:
            rospy.loginfo("there are arguments")
            self.setup(client_input_param_path, sim, run_as_api, server_input_param_path, server_dry_run)

    # setup function must be called before using ArmClient. if the constructor is given args then this is called automatically.
    def setup(self, client_input_param_path="/ur10_driver/defaults_topic", sim=False, run_as_api=False, server_input_param_path="/ur10_driver/arm_1_saw/params", server_dry_run=False):
        print("ArmClient setup called")
        self.client_input_param_path = client_input_param_path
        self.run_as_api = run_as_api
        self.is_sim = sim

        self.robot_driver = None

        self.arm_params = {}
        self.arm_params = rospy.get_param(self.client_input_param_path)
        arm_id = self.arm_params["id"]
        if "name" in self.arm_params:
            self.arm_name = self.arm_params["name"]
        else:
            self.arm_name = 'arm_{}'.format(arm_id)

        self.listener = ros_tf.TransformListener()
        time.sleep(0.25)

        # load arm_type from arm_params
        self.arm_type = self.arm_params["arm_type"]
        rospy.loginfo("ArmClient loaded arm_type {}".format(self.arm_type))

        # Define the joint_names
        self.joint_names = rospy.get_param(server_input_param_path + "/joint_names")
        rospy.wait_for_service(self.arm_name+'/cmd')
        rospy.wait_for_service(self.arm_name+'/setup')
        self.srv_cmd   = rospy.ServiceProxy(self.arm_name+'/cmd', ArmCmdSrv)
        self.setup_cmd = rospy.ServiceProxy(self.arm_name+'/setup', ArmSetupSrv)

        #subscribers
        self.tcp_speed = np.zeros(6)
        self.joints = np.zeros(6)
        self.tcp_force = np.zeros(6)
        self.flag_is_gate_open = True
        self.flag_is_robot_moving = False
        self.flag_is_robot_ready = False
        self.colliding_pose = None
        self.flag_safety_event_occured = False
        s1 = rospy.Subscriber(self.arm_name+'/tcp_speed', Float32MultiArray, self.save_tcp_speed)
        s2 = rospy.Subscriber(self.arm_name+'/joint_states', JointState, self.save_joints)
        s3 = rospy.Subscriber(self.arm_name+'/wrench', WrenchStamped, self.save_tcp_force)
        s4 = rospy.Subscriber(self.arm_name+'/is_gate_open', Bool, self.save_is_gate_open)
        s5 = rospy.Subscriber(self.arm_name+'/is_robot_moving', Bool, self.save_is_robot_moving)
        s6 = rospy.Subscriber(self.arm_name+'/safety_event_occured', Bool, self.save_safety_event_occured)
        s7 = rospy.Subscriber(self.arm_name+'/colliding_pose', geometry_msgs.msg.Pose, self.save_colliding_pose)
        s8 = rospy.Subscriber(self.arm_name+'/is_robot_ready', Bool, self.save_is_robot_ready)
        

        # defaults

        self.defaults = {
                "acc": 0.5,
                "vel": 0.25,
                "wait": True,
                "timeout": 5,
                "threshold": 0.0,
                "radius": 0.025,
                "setRemote": True,
                "id1": 0,
                "val": 0
        }

        # send input topic
        rospy.sleep(0.5)
        self.set_defaults_topic(self.client_input_param_path)
        print "Finished setup."
        return True


    def fill_defaults(self, kwargs):
        """ Fill the default values before encoding
        """
        out = self.defaults.copy()
        out.update(kwargs)
        # print("Make sure out is as desired. out: {}".format(out))
        # pdb.set_trace()
        return out

    def construct_cmd_req(self, cmd_type, **kwargs):
        """ Take the kwargs and map it to ArmCmd request
        """
        kwargs = self.fill_defaults(kwargs)
        kwargs['type'] = cmd_type
        if not cmd_type in ['force_stop2', 'force_move', 'end_force_mode', 'stopj', 'reset_FT300', 'set_standard_digital_out', 'set_speed', 'stepl', 'stepj']:
            valid, missing_kwargs = self.check_valid_req(eval(cmd_type), kwargs)
            if not valid:
                rospy.loginfo("Tried calling {} with invalid kwargs. Returning".format(cmd_type))
                rospy.loginfo("Missing kwargs: {}".format(missing_kwargs))
                return

        # convert m3d poses to ROS poses
        pose_m3d          = kwargs.pop("pose", m3d.Transform())
        pose_base_m3d     = kwargs.pop("pose_base", m3d.Transform())
        pose_list_m3d     = kwargs.pop("pose_list", [])
        poseList_base_m3d = kwargs.pop("poseList_base", [])
        #
        pose          = m3d_to_ros(pose_m3d)
        pose_base     = m3d_to_ros(pose_base_m3d)
        pose_list     = [m3d_to_ros(pp) for pp in pose_list_m3d]
        poseList_base = [m3d_to_ros(pp) for pp in poseList_base_m3d]
        # convert lists to float32multiarrayjoints
        joints = kwargs.pop("joints", [])
        torque = kwargs.pop("torque", [])
        limits = kwargs.pop("limits", [])
        selection_vector = kwargs.pop("selection_vector", [])

        jointList = kwargs.pop("jointList", [[]])

        # Checks if jointList is list of lists; if not, then convert [] to [[]]
        if not any(isinstance(el, list) for el in jointList):
            jointList = [jointList]

        

        cmd_req = message_converter.convert_dictionary_to_ros_message('bot_common_ros/ArmCommand', kwargs) #TODO: incorporate req/res fork of rospy_message_converter
        cmd_req.pose = pose
        cmd_req.pose_base = pose_base
        cmd_req.pose_list = pose_list
        cmd_req.poseList_base = poseList_base
        #
        cmd_req.joints = Float32MultiArray(data=joints)
        cmd_req.torque = Float32MultiArray(data=torque)
        cmd_req.limits = Float32MultiArray(data=limits)
        cmd_req.selection_vector = Float32MultiArray(data=selection_vector)
        # convert 2d float32multiarray
        cmd_req.jointList = [ Float32MultiArray(data=jl) for jl in jointList ]

        ### Call it! ###
        self.srv_cmd(cmd_req)

    def check_valid_req(self, func, kwargs):
        """ must check validity here (required kwargs) because ROS msg automatically default fills out
        RECALL: robot (or urxRobot) and tf are filled by the server.
        """
        validArgs = func.func_code.co_varnames[:func.func_code.co_argcount]
        kwargsLen = len(func.func_defaults) # number of keyword arguments
        validKwargs = validArgs[-kwargsLen:]
        set_req = set(validKwargs)
        set_have = set(kwargs)
        if 'robot' in set_req: set_req.remove('robot')
        if 'URXrob' in set_req: set_req.remove('URXrob')
        if 'tf' in set_req: set_req.remove('tf')
        missing_kwargs = set_req - set_have
        valid = len(missing_kwargs) == 0
        return valid, missing_kwargs

    ############################### API commands #################################
    def movel(self, **kwargs):
        if self.run_as_api:
            self.robot_driver.movel(**kwargs)
        else:
            self.construct_cmd_req('movel', **kwargs)
    def movels(self, **kwargs):
        #if self.is_sim: return
        if self.run_as_api:
            self.robot_driver.movels(**kwargs)
        else:
            self.construct_cmd_req('movels', **kwargs)
    def stepl(self, **kwargs):
        #if self.is_sim: return
        # if self.run_as_api:
        #     self.robot_driver.movejs(**kwargs)
        # else:
        #     self.construct_cmd_req('stepl', **kwargs)
        self.construct_cmd_req('stepl', **kwargs)
    def movejs(self, **kwargs):
        if self.is_sim: 
            kwargs["jointList"].insert(0, self.get_joints().tolist())
            self.construct_cmd_req('movejs', **kwargs)
        elif self.run_as_api:
            self.robot_driver.movejs(**kwargs)
        else:
            self.construct_cmd_req('movejs', **kwargs)
    def stepj(self, **kwargs):
        #if self.is_sim: return
        if self.run_as_api:
            self.robot_driver.movejs(**kwargs)
        else:
            self.construct_cmd_req('stepj', **kwargs)
    def stop(self, **kwargs):
        if self.run_as_api:
            self.robot_driver.force_stop2(**kwargs)
        else:
            self.construct_cmd_req('force_stop2', **kwargs)
    def stopj(self, **kwargs):
        if self.run_as_api:
            self.robot_driver.stopj(**kwargs)
        else:
            self.construct_cmd_req('stopj', **kwargs)
    def force_mode(self, **kwargs):
        if self.run_as_api:
            self.robot_driver.force_mode(**kwargs)
        else:
            self.construct_cmd_req('force_mode', **kwargs)
    def force_move(self, **kwargs):
        if self.run_as_api:
            self.robot_driver.force_move(**kwargs)
        else:
            self.construct_cmd_req('force_move', **kwargs)
    def end_force_mode(self, **kwargs):
        if self.run_as_api:
            self.robot_driver.end_force_mode(**kwargs)
        else:
            self.construct_cmd_req('end_force_mode', **kwargs)
    def reset_FT300_2(self):
        if self.run_as_api:
            self.robot_driver.reset_FT300()
        else:
            self.construct_cmd_req('reset_FT300')
    def set_standard_digital_out(self, **kwargs):
        if self.run_as_api:
            self.robot_driver.set_standard_digital_out(**kwargs)
        else:
            self.construct_cmd_req('set_standard_digital_out', **kwargs)
    def set_speed(self, **kwargs):
        if self.run_as_api:
            self.robot_driver.set_speed(**kwargs)
        else:
            self.construct_cmd_req('set_speed', **kwargs)


    ############################### setup commands #################################
    def set_bounding_box(self, bbox_marker):
        if self.run_as_api:
            self.robot_driver.set_bbox_from_ros(bbox_marker)
        else:
            msg = ArmSetup(type = 'set_bounding_box', bbox=bbox_marker)
            self.setup_cmd(msg)
    #
    def set_defaults_topic(self, topic_name):
        if self.run_as_api:
            self.robot_driver.set_arm_defaults_from_param_server(topic_name)
        else:
            msg = ArmSetup(type = 'set_defaults_topic', topic_name=topic_name)
            self.setup_cmd(msg)
    #
    def use_bounding_box(self, flag):
        if self.run_as_api:
            self.robot_driver.use_bounding_box = flag
        else:
            msg = ArmSetup(type = 'use_bounding_box', flag=flag)
            self.setup_cmd(msg)
    #
    def use_monitor(self, flag):
        if self.run_as_api:
            self.robot_driver.use_monitor = flag
        else:
            msg = ArmSetup(type = 'use_monitor', flag=flag)
            self.setup_cmd(msg)
    #
    def use_data_publisher(self, flag):
        if self.run_as_api:
            self.robot_driver.use_data_publisher = flag
        else:
            msg = ArmSetup(type = 'use_data_publisher', flag=flag)
            self.setup_cmd(msg)
    #
    def reset_FT300(self):
        if self.run_as_api:
            self.robot_driver.reset_FT300()
        else:
            msg = ArmSetup(type = 'reset_FT300')
            self.setup_cmd(msg)

    #
    def get_m3d_pose(self, topic1, topic2):
        (trans, rot) = self.listener.lookupTransform(topic1, topic2, rospy.Time(0)) # from 2 to 1 aka frame 2 w.r.t. frame 1
        pose = ROStoM3d(trans, rot)
        return pose

    ######################## subscribers ########################

    def save_tcp_speed(self, msg):
        self.tcp_speed = np.array(msg.data[:])

    def save_joints(self, msg):
        self.joints = np.array(msg.position[:])

    def save_tcp_force(self, msg):
        """ Return force in the TCP frame
        """
        ff = msg.wrench.force; tt = msg.wrench.torque
        out = np.array([ff.x, ff.y, ff.z, tt.x, tt.y, tt.z])
        # convert to correct frame
        frame_i = msg.header.frame_id
        frame_t = self.arm_name + "_TCP"
        tf = self.get_m3d_pose(frame_i, frame_t)
        # just do orientation
        out_tcp = rotate6V(out, tf, to_TF=True)
        self.tcp_force =  out_tcp

    def save_is_gate_open(self, msg):
        self.flag_is_gate_open = msg.data
    
    def save_is_robot_moving(self, msg):
        self.flag_is_robot_moving = msg.data
    
    def save_is_robot_ready(self, msg):
        self.flag_is_robot_ready = msg.data
    
    def save_safety_event_occured(self, msg):
        print("Safety Event Occured Flag = {}".format(msg.data))
        self.flag_safety_event_occured = msg.data
    
    def save_colliding_pose(self, msg):
        self.colliding_pose = msg

    ############################### data #################################

    def get_pose(self):
        """ use tf listener """
        topic_i = self.arm_name + "_base"
        topic_f = self.arm_name + "_tip_link"
        return self.get_m3d_pose(topic_i, topic_f)

    def get_tcp_speed(self):
        return self.tcp_speed

    def get_joints(self):
        return self.joints

    def get_tcp_force(self):
        return self.tcp_force

    def get_force(self):
        return self.get_tcp_force()
    
    def is_robot_moving(self):
        return self.flag_is_robot_moving

    def is_gate_open(self):
        return self.flag_is_gate_open

    def safety_event_wait(self):
        done = self.is_gate_open()
        while not done and not rospy.is_shutdown():
            rospy.sleep(0.1)
            done = self.is_gate_open()

    def wait(self): self.safety_event_wait()

    ############################### mission op wrappers #################################
    # input: op_movej(joint_s=0, joint_l=1, joint_e=2, joint_u=0, joint_r=0, joint_b=0, joint_t=0, duration=500)
    # units are deg/s, ms
    def op_stepj(self, **kwargs):

        # default step values for joints
        new_kwargs = {'joint_s' : 0, 'joint_l' : 0, 'joint_e' : 0, 'joint_u' : 0,
                    'joint_r' : 0, 'joint_b' : 0, 'joint_t' : 0}
        new_kwargs.update(kwargs)

        print "op_stepj called."
        # filter kwargs by joint names
        joint_kwargs = { k: new_kwargs[k] for k in self.joint_names }

        # Order the kwargs based on the joint_names list
        index_map = {v: i for i, v in enumerate(self.joint_names)}
        sorted_kwargs = sorted(joint_kwargs.items(), key=lambda pair: index_map[pair[0]])  # Ordered list of tuples
        joints = list(map(lambda x: x[1], sorted_kwargs)) # extract just the joint values

        # Update kwargs, remove the joint names
        for name in self.joint_names:
            del new_kwargs[name]
        
        if new_kwargs["ang_velocity"]:
            new_kwargs["vel_rot"] = kwargs["ang_velocity"]
            del new_kwargs["ang_velocity"]

        new_kwargs["joints"] = joints
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
    def op_movels(self, **kwargs):

        # filter kwargs by cartesian_names
        cartesian_names = ['x', 'y', 'z', 'roll', 'yaw', 'pitch']
        cartesian_kwargs = { k: kwargs[k] for k in cartesian_names }

        # Order the kwargs based on the cartesian_names list
        index_map = {v: i for i, v in enumerate(cartesian_names)}
        sorted_kwargs = sorted(cartesian_kwargs.items(), key=lambda pair: index_map[pair[0]])  # Ordered list of tuples
        poseList = [list(map(lambda x: x[1], sorted_kwargs))] # extract just the list

        # Convert from list to transform
        poseList = list_to_tf(poseList)

        # Update kwargs, remove the cartesian names
        for name in cartesian_names:
            del kwargs[name]
        kwargs["poseList_base"] = poseList

        # Call movel
        self.movels(kwargs)

        return True


    # inputs are in velocity and duration
    # simply multiply to get the distance
    # use the greatest velocity as the "master" velocity   
    # {"kwargs":{"frame":"tool","duration":0.3,"x":0,"y":0.001063330078125,"z":0.074,"roll":0,"pitch":0,"yaw":0,"private_op":true,"operation_name":"SAW.ARM.STEPL"}
    # input: op_movel(x=0, y=1, z=2, roll=0, yaw=0, duration=500)
    # units are m/s, deg/s, ms
    def op_stepl(self, **kwargs):
        print  "op_stepl called. kwargs: {}".format(kwargs)

        cartesian_names = ['x', 'y', 'z', 'roll', 'yaw', 'pitch']
        new_kwargs = {'x' : 0, 'y' : 0, 'z' : 0,
                    'roll' : 0, 'yaw' : 0, 'pitch' : 0}
        new_kwargs.update(kwargs)

        # Calculate poseList_base based on velocity & duration for each coordinate
        velocities = np.array([new_kwargs["x"], new_kwargs["y"], new_kwargs["z"], 
                        new_kwargs["roll"], new_kwargs["yaw"], new_kwargs["pitch"]])
        poseList = velocities[:] * duration/1000.0
        poseList = list_to_tf(poseList.to_list())

        # Add poseList and vel to new_kwargs
        new_kwargs["vel"] = max(velocities[0:2])
        new_kwargs["vel_rot"] = max(velocities[3:])
        new_kwargs["poseList_base"] = [poseList]
        new_kwargs["pose"] = poseList
        new_kwargs["wait"] = True

        # Update new_kwargs, remove the cartesian names
        for name in cartesian_names:
            del new_kwargs[name]
        
        # Temp: remove frame, tool, duration
        if "frame" in new_kwargs: del new_kwargs["frame"]
        if "tool" in new_kwargs: del new_kwargs["tool"]
        if "duration" in new_kwargs: del new_kwargs["duration"]

        # Call movels
        self.stepl(**new_kwargs)

        return True
