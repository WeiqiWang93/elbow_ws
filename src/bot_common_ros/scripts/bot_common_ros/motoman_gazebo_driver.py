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
from std_msgs.msg import Float32MultiArray, Bool, Empty, Float32
from geometry_msgs.msg import WrenchStamped, TransformStamped
from sensor_msgs.msg import JointState 
from bot_common_ros.srv import ArmCmdSrv, ArmSetupSrv
from actionlib_msgs.msg import GoalStatus
from random import randrange

#
import tf as ros_tf
import tf.msg # can be accessed with ros_tf.msg


#
import URBasic
import urx
from bot_common_ros.ur_control import URMiddleWare, getHorizontalOrient, force_stop2, getFloatArrMsg, checkBox
from bot_common_ros.ur_utils import rosParamToClassObject, ROStoM3d, ROStoM3d_orient, m3d_to_ros, fillTF, tf_to_list, list_to_tf, tf_to_arr
import Queue

VEL                 = 0.5
ACC                 = 2.0 * VEL
CONSOLE_LOG         = True

class MotomanGazeboDriver(object):
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
        self.dry_run            = None
        self.nominal_tcp_to_tcp = m3d.Transform()
        self.use_data_publisher = True
        
        # default data values
        self.robot_pose = None
        self.bounding_box = None
        

        # ROS service servers
        self.srv_setup = None
        self.srv_cmd   = None

        # ROS action clients
        self.motoman_gz_client = None


        # Threading
        self.data_thread = None
        self.safety_gate = None
        self.lock = threading.Lock()
        self.move_thread_handle = None
        self.goal_queue = Queue.Queue()

        # overseer
        self.OA = None

        self.is_robot_moving = False
        self.prev_success_goal = []


    def INIT(self, overseer=None, mp=None):
        """ INIT function for setup
        By end of this robot should be ready to receive a command
        """
        rospy.loginfo('{}: init started.'.format(self.arm_name))
        rospy.on_shutdown(self.on_shutdown)

        if overseer is not None:
            self.OA = overseer
            rospy.loginfo("Gazebo driver connecting to overseer.")

        if mp is not None:
            self.mp = mp

        # start ur gazebo connection
        self.motoman_gazebo_bringup()

        self.RESET()
        self.INIT_COMPLETE = True

        rospy.loginfo('{}: init complete.'.format(self.arm_name))



    def motoman_gazebo_bringup(self):
        """ bring up the UR Gazebo. Currently does not actually brings up the robot as it is already done via launch file.
        Simply used to sets up the communication interface between the robot in gazebo and outside"""

        # Read values from ROS param server to get arm properties. Here we are only concerned about 'name' as this becomes the prefix for ROS srvs
        # hosted by this node. Is this the right place to read ROS params? 
        '''
        self.input_param_path = rospy.get_param("~input_param_path")
        self.dry_run     = rospy.get_param("~dry_run")
        self.arm_params = rospy.get_param(self.input_param_path)
        # Input checks
        if self.input_param_path is None or self.dry_run is None:
            ss = '{}: Must set inputs input_param_path and dry_run before ur10_bringup.'.format(self.arm_name)
            rospy.logerr(ss)
            rospy.signal_shutdown(ss)     
        assert( isinstance(self.arm_params, dict) )
        assert( "id" in self.arm_params )

        arm_id = self.arm_params["id"]
        if "name" in self.arm_params:
            self.arm_name = self.arm_params["name"][1:]
        else:
            self.arm_name = 'arm_{}'.format(arm_id) # IS THIS ALTERNATIVE ALSO USED IN THE CLIENTS FOR THIS DRIVER?

        assert self.arm_name in ["saw", "chisel"], "Only 'saw' and 'chisel' arms are supported at the moment. "
        '''
        self.arm_name = "saw"

        try:
            self.joint_names = rospy.get_param(self.arm_name + "_joint_names")
        except:
            self.joint_names = ["saw_joint_s", "saw_joint_l", "saw_joint_e", "saw_joint_u", "saw_joint_r", "saw_joint_b", "saw_joint_t"]

        # Host ROS Service servers
        self.srv_setup = rospy.Service('/' + self.arm_name+'/setup', ArmSetupSrv, self._arm_setup_server)
        self.srv_cmd   = rospy.Service('/' + self.arm_name+'/cmd', ArmCmdSrv, self._arm_cmd_server)
        
        # ROS publishers
        self.pub_joints = rospy.Publisher('/' +self.arm_name+'/joint_states', JointState, queue_size=5)

        # Convert /joint_states to /saw/joint_states
        if self.OA is None:
            rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        else:
            rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
            self.OA.start_tm()

        # load params
        self.tool_params_path = "/robo_saw_goelz/tool"
        self.tool_params = rospy.get_param(self.tool_params_path)
        self.tcp_to_corner = m3d.Transform(self.tool_params["TCP_to_corner"])

        # Action Clients
        self.motoman_gz_client = actionlib.SimpleActionClient('/arm_controller_'+self.arm_name +'/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo('{}: Waiting for server...'.format(self.arm_name))
        self.motoman_gz_client.wait_for_server()
        rospy.loginfo('{}: Connected to server.'.format(self.arm_name))

        self.move_thread_handle = threading.Thread(target=self.move_thread)
        self.move_thread_handle.setDaemon(True)
        self.move_thread_handle.start()    

        if not self.dry_run:
            # start safety event -- default is open. It's a flag based on which other threads are paused from continuing.
            # The flag indicates that robot bringup was successful.
            self.safety_gate = threading.Event()
            self.safety_gate.set()
           
            # start secondary threads           
            self.data_thread = threading.Thread(name=self.arm_name+'_data_thread', target=self.data_publisher)      
            self.data_thread.setDaemon(True) #will make the thread close when this program ends
            self.data_thread.start()

        # Give tme for threads to start
        time.sleep(0.5)

        return True
   
    
    def on_shutdown(self):
        #
        rospy.loginfo("")
        rospy.loginfo("SIA20 Gazebo driver: Shutting Down")
        self.GLOBAL_DONE = True
        self.use_monitor = False

        if self.OA:
            self.OA.stop_tm()
 

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

    def RESET(self): # DO NOT UNDERSTAND THE PURPOSE OF THIS FUNCTION?
        """ clear current tool parameters to be ready for next tool
        """
        # monitoring thread
        self.useBoundingBox     = False # Does not have any relavance to this driver.
        self.INIT_COMPLETE      = False # This not useful.
        self.ur10_params_set    = False # Not used anywhere else. 

    def IDLE(self):
        """ IDLE means the arm is shut off but the elx is still on
        """
        pass

    def STOP(self, timeout=0.5):
        """ Bring arm to a quick smooth stop
        """
        with self.goal_queue.mutex:
            self.goal_queue.queue.clear()
        
        self.motoman_gz_client.cancel_all_goals()
        self.motoman_gz_client.wait_for_result()

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
        rospy.loginfo("Setup Srv: received command: {}. Flag: {}".format(fct_name, setup_req.msg.flag))
        
        if fct_name == "set_bounding_box":
            user_box = setup_req.msg.bbox
            self.set_bbox_from_ros(user_box)
        elif fct_name == "set_defaults_topic": # '_topic' SHOULD BE CHANGED TO SOMETHING LIKE '_param_path'. 
            topic_name = setup_req.msg.topic_name # This attribute is used to specify path in ROS param server for arm end-effector properties such as TCP, CG, etc.
        elif fct_name == "use_bounding_box":
            self.useBoundingBox = setup_req.msg.flag
        elif fct_name == "use_monitor":
            self.use_monitor = setup_req.msg.flag
        elif fct_name == "use_data_publisher":
            self.use_data_publisher = setup_req.msg.flag
        elif fct_name == "reset_FT300":
            pass
        
        rospy.loginfo("Setup Srv: finished command: {}. Flag: {}".format(fct_name, setup_req.msg.flag))
        return True
            

    def set_standard_digital_out(self, **kwargs):
        pass

    def move_thread(self):
        
        while not rospy.is_shutdown():
            goal = self.goal_queue.get(block=True)
            self.motoman_gz_client.send_goal(goal)
            self.is_robot_moving = True

            try:
                self.motoman_gz_client.wait_for_result()
                result = self.motoman_gz_client.get_state()

                # Evaluate the goal result
                if result == GoalStatus.SUCCEEDED:
                    self.prev_success_goal = goal.trajectory.points[0].positions
                    rospy.loginfo("Motoman Gazebo driver motion success!")
                else:
                    rospy.logwarn("Motoman Gazebo driver failed goal, GoalStatus: {}".format(result))

                # if queue is empty, robot isn't moving
                if self.goal_queue.empty():
                    self.is_robot_moving = False
                
            except KeyboardInterrupt:
                self.motoman_gz_client.cancel_goal()
                raise

    def get_speed(self):
        return [randrange(10), randrange(10), randrange(10)]

    def get_rx_period(self):
        return randrange(10)

    def get_last_pt(self):
        return self.prev_success_goal

    def movejs(self, **kwargs):

        jointList = kwargs["jointList"][:]

        # Add current joints to the trajectory
        jointList.insert(0, self.joints)

        # wait
        if "wait" in kwargs:
            wait = kwargs["wait"]
        else:
            wait = True

        # Duration
        if "duration" in kwargs:
            duration = kwargs["duration"]
        else:
            duration = 2.0

        if type(jointList) != list:
            rospy.logerr("motoman_gazebo_driver received improper jointList input")
            return False

        # Code to move the arm in gazebo. (one waypoint at a time)
        for j in jointList:
            g = FollowJointTrajectoryGoal()
            g.trajectory = JointTrajectory()
            g.trajectory.joint_names = self.joint_names
            traj = j
            length = len(jointList[0])
            g.trajectory.points = [JointTrajectoryPoint(positions=traj, velocities=[0]*length, accelerations=[0]*length, time_from_start=rospy.Duration(duration))]
            self.goal_queue.put(g)

        self.is_robot_moving = True

        if wait:
            while self.is_robot_moving and not rospy.is_shutdown():
                time.sleep(0.1)

        return True
    
    def stepj(self, **kwargs):

        joints_step = kwargs["joints"]   # np array
        joints_cur = np.asarray(self.joints)

        kwargs["jointList"] = [joints_step + joints_cur]
        self.movejs(**kwargs)

        return True

    def movel(self, **kwargs):
        pose = kwargs["pose"]

        # print "cur pose: {}".format(tf_to_arr(self.tcp_to_base))
        # print "goal pose: {}".format(tf_to_arr(pose))

        # if IK already provided (ie from rsas), call movejs
        if "jointList" in kwargs:
            self.movejs(**kwargs)
            return True

        if not isinstance(pose, m3d.Transform) and not isinstance(pose, geometry_msgs.msg.Pose):
            rospy.logerr("kwargs pose must be of type m3d.Transform or geometry_msgs.msg.Pose, incorrect type {} given.".format(type(pose)))
            return False

        # if no IK, then perform mp on each pose
        ik = self.mp.get_ik_joints(self.joints, pose)

        if len(ik) == 0:
            rospy.logerr("Could not find IK, movel failed.")
            return False
        else:
            kwargs["jointList"] = [list(ik)]
            self.movejs(**kwargs)
            return True

    def movels(self, **kwargs):
        # if IK already provided (ie from rsas), call movejs
        if "jointList" in kwargs:
            self.movejs(**kwargs)

        # if no IK, then perform mp on each pose
        else:
            for pose in kwargs["poseList_base"]:
                kwargs["pose"] = pose
                self.movel(**kwargs)

        return True

    def stepl(self, **kwargs):
        step_tf = kwargs["pose"]

        # Method 1: convert tf_to_arr, then add arrays, then convert back to tf
        cur = tf_to_arr(self.tcp_to_base)
        dist = tf_to_arr(kwargs["pose"])
        goal = cur + dist
        kwargs["pose"] = list_to_tf(goal)
        
        # # Method 2: Keep in tf form, add position vectors & multiply rotation matrices
        # pos = self.tcp_to_base.get_pos() + step_tf.get_pos()
        # rot = self.tcp_to_base.get_orient() * step_tf.get_orient()
        # goal_2 = m3d.Transform(rot, pos)   # compare kwargs["pose"] vs goal_2
        # goal_2_arr = tf_to_arr(goal_2)   # compare goal vs goal_2_arr

        # # Method 3: Use HSE driver list_to_transform method
        # self.current_pose = cur.tolist()
        # cur_3 = self.list_to_transform(self.current_pose[:])
        # trans_3 = cur_3.get_pos() + step_tf.get_pos()
        # rot_3 = cur_3.get_orient() * step_tf.get_orient()
        # goal_3 = m3d.Transform(rot_3, trans_3) # compare to goal_2
        # goal_3_arr = tf_to_arr(goal_3)  # compare to goal_2_arr

        # pdb.set_trace()
        return self.movel(**kwargs)

    # def list_to_transform(self, pose):
        
    #     _rx,_ry,_rz = pose[3:6]

    #     # verify rotations are in acceptable range, apply modulo if necessary
    #     if _rx < -180 or _rx > 180:
    #         _rx = _rx % 180
    #     if _ry < -90 or _ry > 90:
    #         rospy.logerr("ry must be -pi/2 < ry < pi/2")
    #         return
    #     if _rz < -180 or _rz > 180:
    #         _rz = _rz % 180
    #     pose[3:6] = _rx, _ry, _rz

    #     rx,ry,rz = pose[3:6]
    #     position = pose[0:3]
    #     # rx,ry,rz = [math.radians(x) for x in pose[3:6]] # converting to radians
    #     # position = [x/1000.0 for x in pose[0:3]] # convert mm to meters
    #     matrix = tf.transformations.euler_matrix(rx,ry,rz)[0:3, 0:3]
    #     rotation = m3d.Orientation(matrix)
    #     transform = m3d.Transform(rotation, position)
    #     return transform

    ################################################################

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
        """

        # call it
        if self.dry_run:
           pass
        else:
            self.safety_gate.wait() # make sure gate is open

        # Code to move the arm in gazebo.
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = self.joint_names
        traj = cmd_req.msg.jointList
        length = len(cmd_req.msg.jointList[0].data)
        g.trajectory.points = [JointTrajectoryPoint(positions=traj[i].data, velocities=[0]*length, accelerations=[0]*length, \
            time_from_start=rospy.Duration(i*2.0)) for i in range(0, len(traj))]

        self.motoman_gz_client.send_goal(g)

        try:
            self.motoman_gz_client.wait_for_result()
        except KeyboardInterrupt:
            self.motoman_gz_client.cancel_goal()
            raise
                
        return True

    #callback function: when a joint_states message arrives, save the values
    # remaps from /joint_states to saw/joint_states
    def joint_states_callback(self, msg):

        saw_joints_indices = [10, 8, 7, 12, 9, 6, 11]

        if self.OA is None:
            msg_pub = JointState()
            msg_pub.header.stamp = rospy.Time.now()
            
            msg_pub.header = msg.header

            start_index = 0
            if self.arm_name == "chisel":
                start_idx = 0
                end_idx = 6
                msg_pub.name = self.joint_names
                msg_pub.position = msg.position[start_idx:end_idx]
                msg_pub.velocity = msg.velocity[start_idx:end_idx]

            elif self.arm_name == "saw":
                start_idx = 6
                joints_indices = [list(msg.name).index(jname) for jname in self.joint_names]
                msg_pub.name = self.joint_names
                self.joints = [list(msg.position)[j] for j in joints_indices]
                msg_pub.position = self.joints
                msg_pub.velocity = [list(msg.velocity)[j] for j in joints_indices]

                self.pub_joints.publish(msg_pub)

        # set self.joints equal to subscribed rostopic
        else:
            if self.arm_name == "saw":
                start_idx = 6
                joints_indices = [list(msg.name).index(jname) for jname in self.joint_names]
                self.joints = [list(msg.position)[j] for j in joints_indices]


    def get_joints(self):
        return self.joints
    
    def get_pose(self):
        return self.tcp_to_base
    
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
        pub_get_pose = rospy.Publisher(self.arm_name+'/pose', Float32MultiArray, queue_size=5)

        rospy.loginfo("Data Publisher ready and starting.")

        # static transforms TODO: put nominal-tcp-to-tcp here (but then have to update it if tf changes from user)
        listener = ros_tf.TransformListener()
        while not rospy.is_shutdown():
            time.sleep(0.100) # reduce sim driver rate

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
                self.tcp_to_base = ROStoM3d(trans, rot)
                
                # pub tcp to corner
                t = TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = self.arm_name + "_TCP"
                t.child_frame_id = self.arm_name + "_TCP_to_corner"
                corner_tf = self.tcp_to_corner
                # print "tcp ", self.tcp_to_base
                # print "corner_tf ", corner_tf
                # corner_tf_ros = m3d_to_ros(corner_tf)
                fillTF(corner_tf.get_pos(), corner_tf.get_orient().quaternion, t)
                tfm = ros_tf.msg.tfMessage([t])
                pub_tf.publish(tfm)

                #gate
                msg = Bool(self.safety_gate.is_set()) # true if open
                pub_is_gate_open.publish(msg)

                # pose pub
                pos = self.tcp_to_base.get_pos().get_array()
                rot = self.tcp_to_base.get_orient().to_euler("xyz")
                msg = getFloatArrMsg(np.append(pos, rot).tolist())
                pub_get_pose.publish(msg)

if __name__ == "__main__":
    """ start the class and spin rospy
    """
    rospy.init_node("motoman_gazebo_driver", anonymous=True)
    run_test = rospy.get_param("~run_test", False)

    try:
        motoman_gazebo_driver = MotomanGazeboDriver()
        motoman_gazebo_driver.INIT()
        time.sleep(1.0)

        rospy.loginfo("motoman_gazebo_driver: Start spinning forever now.")
        rospy.spin()
        
    finally:
        if not rospy.is_shutdown(): rospy.signal_shutdown("End of Program")