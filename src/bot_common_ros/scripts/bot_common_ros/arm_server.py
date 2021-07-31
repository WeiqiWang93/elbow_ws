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
import logging
from abc import ABCMeta, abstractmethod
import rospy
from rospy_message_converter import message_converter

#
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray, Bool, Float32
from geometry_msgs.msg import WrenchStamped, TransformStamped, Pose
from sensor_msgs.msg import JointState 
from bot_common_ros.srv import ArmCmdSrv, ArmSetupSrv

#
import tf as ros_tf
import tf.msg # can be accessed with ros_tf.msg


#
from bot_common_ros.ur_control import URMiddleWare, getHorizontalOrient, getFloatArrMsg, checkBox
from bot_common_ros.ur_utils import rosParamToClassObject, ROStoM3d, ROStoM3d_orient, \
                     get_pose, get_joints, fillTF

CONSOLE_LOG         = True

class ArmServer(object):
    # https://stackoverflow.com/questions/44576167/force-child-class-to-override-parents-methods
    __metaclass__ = ABCMeta
    """A generic class to interface with an Arm

    This only allows connection to 1 arm. The node must be created (with input arguments) once per arm.

    """

    def __init__(self, run_as_api=False):
        self.run_as_api = run_as_api
        self.logger             = logging.getLogger()
        self.logger.disabled    = not CONSOLE_LOG
        self.ARM_RUNNING        = False
        self.arm_name           = "arm_{}"
        self.__version__        = "1_0"
        self.input_param_path        = None
        self.dry_run            = None
        self.nominal_tcp_to_tcp = m3d.Transform()
        self.default_params = {"TCP": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], # position and orientation of TCP w.r.t. Nominal-TCP
                                "mass": 0.0,
                                "CG": [0.0, 0.0, 0.0] # center of gravity in m of end-effector
                            }

        # default data values
        self.robot_pose = None
        self.bounding_box = None
        self.use_bounding_box     = False
        self.use_monitor        = True
        self.use_data_publisher = True
        self.INIT_COMPLETE      = False
        self.arm_params = {}



    def INIT(self):
        """ INIT function for setup
        By end of this robot should be ready to receive a command
        """
        if not self.run_as_api:
            # private params
            try:
                self.input_param_path = rospy.get_param("~input_param_path")
                self.dry_run     = rospy.get_param("~dry_run")
            except:
                ss = '{}: Must provide topics ~input_param_path and ~dry_run.'.format(self.arm_name)
                rospy.logerr(ss)
                rospy.signal_shutdown(ss) 
                return
        ### option to overwrite default param values
        topic = rospy.get_param("~defaults_topic", "")
        if len(topic) > 0:
            self.default_params = rospy.get_param(topic)
        rospy.on_shutdown(self.on_shutdown)

        # start arm connection
        rospy.loginfo("Begin arm_bringup()")
        self.arm_bringup()
        rospy.loginfo("Finished arm_bringup()")

        # services
        if not self.run_as_api:
            self.srv_setup = rospy.Service(self.arm_name+'/setup', ArmSetupSrv, self._arm_setup_server)
            self.srv_cmd   = rospy.Service(self.arm_name+'/cmd', ArmCmdSrv, self._arm_cmd_server)
            self.pub_safety_event_occured = rospy.Publisher(self.arm_name+'/safety_event_occured', Bool, queue_size=5)
            self.pub_colliding_pose = rospy.Publisher(self.arm_name+'/colliding_pose', Pose, queue_size=5)

        self.RESET()
        self.INIT_COMPLETE = True

        rospy.loginfo('{}: init complete.'.format(self.arm_name))

    def set_inputs(self, input_param_path, dry_run):
        """ api to set variables usually set by rospy.get_param
        """
        self.input_param_path = input_param_path
        self.dry_run     = dry_run

    def arm_bringup(self):
        """ bring up the arm """
        if self.input_param_path is None or self.dry_run is None:
            ss = '{}: Must set inputs input_param_path and dry_run before arm_bringup.'.format(self.arm_name)
            rospy.logerr(ss)
            rospy.signal_shutdown(ss) 
        #

        # overwrite default zero values with input from param server
        self.arm_params = rospy.get_param(self.input_param_path) # read arm_params.yaml
        # rospy.loginfo("arm_bringup() input_param_path: {}".format(self.input_param_path))
        assert( "id" in self.arm_params )
        #rosParamToClassObject(self, rosParams)
        arm_id = self.arm_params["id"]
        if "name" in self.arm_params:
            self.arm_name = self.arm_params["name"]
        else:
            self.arm_name = 'arm_{}'.format(arm_id)

        assert( isinstance(self.arm_params, dict) )

        # load joint_names from arm_params
        self.joint_names = self.arm_params["joint_names"]
        prefix = self.arm_name.replace("/","") + "_"
        self.joint_names_prefixed = self.joint_names[:]
        self.joint_names_prefixed[:] = [prefix + x for x in self.joint_names_prefixed]
        # rospy.loginfo("loaded joint_names {}".format(self.joint_names))
        # rospy.loginfo("created joint_names_prefixed {}".format(self.joint_names_prefixed))

        # load arm_type from arm_params
        self.arm_type = self.arm_params["arm_type"]
        rospy.loginfo("loaded arm_type {}".format(self.arm_type))

        # rotation matrix arm-mount to arm-base TF
        quat = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Quaternion', self.arm_params["MOUNT_TO_BASE"])
        self.MOUNT_TO_BASE = m3d.Transform()
        self.MOUNT_TO_BASE.orient = ROStoM3d_orient(quat)

        # Apply 41 cm offset for SIA20 origin
        if self.arm_type == "sia20":
            self.MOUNT_TO_BASE.pos.z -= 0.410

        # moved 0.410 offset to sia20_driver
        # self.MOUNT_TO_BASE.pos.z -= 0.410

        # rotation matrix TCP to non-rotated TCP
        quat = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Quaternion', self.arm_params["NOMINAL_TCP_TO_SENSOR"])
        self.NOMINAL_TCP_TO_SENSOR = ROStoM3d_orient(quat)

        #
        if not self.dry_run:
            # start safety event -- default is open
            self.safety_gate = threading.Event()
            self.safety_gate.set()

            # connect to physical robot through TCP/IP
            self.connect_to_robot()

            # set default defaults
            self.set_defaults(self.default_params)

            # start secondary threads
            self.monitor_thread = threading.Thread(name=self.arm_name+'_monitor_thread', target=self.arm_monitor)
            self.data_thread = threading.Thread(name=self.arm_name+'_data_thread', target=self.data_publisher)
            self.monitor_thread.setDaemon(True) #will make the thread close when this program ends
            self.data_thread.setDaemon(True) #will make the thread close when this program ends
            self.monitor_thread.start()
            self.data_thread.start()

        return True

    @abstractmethod
    def on_shutdown(self):
        """ shutdown hook to safely bring robot to stop in case of unexpected code failure """
        raise NotImplementedError
    #
    @abstractmethod
    def connect_to_robot(self):
        """ connect to physical robot """
        raise NotImplementedError
    #
    @abstractmethod
    def set_defaults(self, params):
        """ Save nominal-tcp to TCP tf, and set
        """
        raise NotImplementedError
    #
    @abstractmethod
    def STOP(self, timeout=0.5):
        """ Bring arm to a quick smooth stop
        """
        raise NotImplementedError
    
    def set_arm_defaults_from_param_server(self, input_param_path):
        """ take values from param-server and forward them to the robot
        """
        params = rospy.get_param(input_param_path)
        self.set_defaults(params)

    def set_bbox_from_ros(self, user_box):
        """ Fxn to automatically set ur10 params from the ROS server as a function of the user-set bounding box
        """
        assert( user_box.type == Marker.CUBE )
        #
        self.bbox_x0         = user_box.pose.position.x - user_box.scale.x * 0.5
        self.bbox_y0         = user_box.pose.position.y - user_box.scale.y * 0.5
        self.bbox_z0         = user_box.pose.position.z - user_box.scale.z * 0.5
        self.bbox_xf = self.bbox_x0 + user_box.scale.x
        self.bbox_yf = self.bbox_y0 + user_box.scale.y
        self.bbox_zf = self.bbox_z0 + user_box.scale.z
        corner = np.array([     self.bbox_x0, self.bbox_y0, self.bbox_z0])
        top_corner = np.array([ self.bbox_xf, self.bbox_yf, self.bbox_zf])  # 40cm seems to be max, with 20deg
        assert(not np.any( corner > top_corner )) # make sure final is greater than initial
        size = top_corner - corner
        self.bounding_box = np.concatenate([corner, size])

    def RESET(self):
        """ clear current tool parameters to be ready for next tool
        """
        # monitoring thread
        self.use_bounding_box     = False
        self.use_monitor        = True
        self.use_data_publisher = True
        self.INIT_COMPLETE      = False

    ############################### servers #################################

    def _arm_setup_server(self, setup_req):
        """
        Marker set_bounding_box
        str set_defaults_topic
        bool turn on/off bbox check
        bool turn on/off monitor (default off)
        bool turn on/off data publisher (default on)
        """
        ###print setup_req
        fct_name = setup_req.msg.type
        # rospy.loginfo("Setup Srv: received command: {}. Flag: {}".format(fct_name, setup_req.msg.flag))
        
        if fct_name == "set_bounding_box":
            user_box = setup_req.msg.bbox
            self.set_bbox_from_ros(user_box)
        elif fct_name == "set_defaults_topic":
            topic_name = setup_req.msg.topic_name
            self.set_arm_defaults_from_param_server(topic_name)
        elif fct_name == "use_bounding_box":
            self.use_bounding_box = setup_req.msg.flag
        elif fct_name == "use_monitor":
            self.use_monitor = setup_req.msg.flag
        elif fct_name == "use_data_publisher":
            self.use_data_publisher = setup_req.msg.flag
        elif fct_name == "reset_FT300":
            self.reset_FT300()
        
        rospy.loginfo("Setup Srv: finished command: {}. Flag: {}".format(fct_name, setup_req.msg.flag))
        return True
    
    def _arm_cmd_server(self, cmd_req):
        """ Read the common arm message and parse it for specific commands
        valid commands:
        uint8 MOVEL = 1
        uint8 MOVELS = 2
        uint8 MOVEJ = 3
        uint8 MOVEJS = 4
        uint8 STOP = 5
        uint8 FORCE_MODE = 6
        uint8 END_FORCE_MODE = 7
        uint8 SHUTDOWN = 8
        uint8 SET_SPEED = 9
        """
        ###print cmd_req
        kwargs = message_converter.convert_ros_message_to_dictionary(cmd_req.msg)
        
        # convert pose and pose_list from ROS to m3d
        kwargs['pose']              =  ROStoM3d(cmd_req.msg.pose.position,      cmd_req.msg.pose.orientation)
        kwargs['pose_base']         =  ROStoM3d(cmd_req.msg.pose_base.position, cmd_req.msg.pose_base.orientation)
        kwargs['pose_list']         = [ROStoM3d(pose.position,           pose.orientation) for pose in cmd_req.msg.pose_list]
        kwargs['poseList_base']     = [ROStoM3d(pose.position,           pose.orientation) for pose in cmd_req.msg.poseList_base]
        # convert float32multiarray
        kwargs['joints'] = kwargs['joints']['data']
        kwargs['torque'] = kwargs['torque']['data']
        kwargs['limits'] = kwargs['limits']['data']
        kwargs['selection_vector'] = kwargs['selection_vector']['data']
        # convert 2d float32multiarray
        kwargs['jointList'] = [ jl['data'] for jl in kwargs['jointList'] ]

        # change ROS default value to kwarg default
        if np.isclose(kwargs['threshold'], 0.0): kwargs['threshold'] = None

        # get fct
        fct = eval('self.'+str(kwargs['type']))

        # call it
        if self.dry_run:
            print("Kwargs: {}".format(kwargs))
        else:
            self.safety_gate.wait() # make sure gate is open
            try:
                fct(**kwargs)
            except:
                return False

        return True


    ############################### commands #################################

    @abstractmethod
    def movel(self, **kwargs):
        raise NotImplementedError
    #
    @abstractmethod
    def movels(self, **kwargs): 
        raise NotImplementedError
    #
    @abstractmethod
    def movejs(self, **kwargs):
        raise NotImplementedError
    #
    @abstractmethod
    def stepl(self, **kwargs):
        raise NotImplementedError
    #
    @abstractmethod
    def stepj(self, **kwargs):
        raise NotImplementedError
    #
    @abstractmethod
    def force_mode(self, **kwargs):
        raise NotImplementedError
    #
    @abstractmethod
    def force_move(self, **kwargs):
        raise NotImplementedError
    #
    @abstractmethod
    def force_stop2(self, **kwargs):
        raise NotImplementedError
    #
    @abstractmethod
    def stopj(self, **kwargs):
        raise NotImplementedError
    #
    @abstractmethod
    def end_force_mode(self, **kwargs):
        raise NotImplementedError
    #
    @abstractmethod
    def reset_FT300(self, wait=True): 
        raise NotImplementedError
    #
    @abstractmethod
    def set_standard_digital_out(self, **kwargs):
        raise NotImplementedError
    #
    @abstractmethod
    def set_speed(self, **kwargs):
        raise NotImplementedError

    ############################### getters #################################

    @abstractmethod
    def get_pose(self):
        raise NotImplementedError
    #
    @abstractmethod
    def get_speed(self):
        raise NotImplementedError
    #
    @abstractmethod
    def get_joints(self):
        raise NotImplementedError
    #
    @abstractmethod
    def get_force(self):
        raise NotImplementedError
    
    @abstractmethod
    def get_robot_state(self):
        raise NotImplementedError

    @abstractmethod
    def get_torque(self):
        raise NotImplementedError    

    ################################################################

    @abstractmethod
    def wait_for_bbox_recovery(self):
        """
        while not in box
            wait
        """
        raise NotImplementedError

    @abstractmethod
    def get_safety_status(self):
        raise NotImplementedError

    @abstractmethod
    def safety_cb(self):
        raise NotImplementedError

    def arm_monitor(self):
        np.set_printoptions(precision=3)
        self.reset_FT300()

        while not rospy.is_shutdown():
            if self.ARM_RUNNING and self.use_monitor:
                # bounding box
                if self.use_bounding_box:
                    if self.robot_pose is not None and self.bounding_box is not None:
                        b_inside = checkBox(self.robot_pose.pos.array, self.bounding_box)
                        if not b_inside[0]: #outside bounding box
                            print '\arm_monitor: Pos: ', self.robot_pose.pos
                            print 'arm_monitor: BBox: ', self.bounding_box
                            print("Left the bounding box. Locking main thread. Stopping robot.")
                            self.safety_gate.clear() # closes the gate
            
                            # TODO: sia20 automated for sia20
                            # out = raw_input("\n Arm outside bbox. Want to continue? (y/n) \n") 
                            out = ['y']
                            if out[0].lower() == 'y':
                                self.safety_gate.set()
                                continue
                            else:
                                self.STOP()
                                self.wait_for_bbox_recovery()
                        
                            
                self.safety_gate.wait() # otherwise may override bbox gate
                safetyMode = self.get_safety_status()

                # if UR safety alarm triggered, then reset
                if self.arm_type == "ur10":
                    if not safetyMode.NormalMode :
                        self.safety_cb()
                
                # TODO: reset motoman alarm 
                if self.arm_type == "sia20":
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
            ###print self.ARM_RUNNING,  self.use_data_publisher
            if self.ARM_RUNNING and self.use_data_publisher:
                
                # if one_sec_time - start_time < 10.0:
                    
                #     if time.time() - one_sec_time < 1.0:
                #         counter += 1
                #     else:
                #         rospy.loginfo("ArmServer {} Hz".format(counter))
                #         counter = 0
                #         one_sec_time = time.time()
                # TOTAL: 300 Hz (~33 ms)
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

                # # Get joint torque (SIA20 only)
                if self.arm_type == "sia20":

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

                # TCP wrench in base frame
                if self.arm_params['use_force_sensor']:
                    FT_vec = self.get_force()
                    if FT_vec is not None:
                        msg = WrenchStamped()
                        ff = msg.wrench.force
                        tt = msg.wrench.torque
                        ff.x, ff.y, ff.z = FT_vec[0:3]
                        tt.x, tt.y, tt.z = FT_vec[3:6]
                        msg.header.stamp = rospy.Time.now()
                        msg.header.frame_id = self.arm_name+"_NOMINAL_TCP"
                        pub_tcp_force.publish(msg)

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

                # TODO: add rate of publishing
