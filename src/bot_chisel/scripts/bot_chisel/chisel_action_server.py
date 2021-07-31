#!/usr/bin/env python


# ----------------------------------------------------------------------------------------------------
import time, argparse
import sys
from threading import Thread
import copy
import math
import numpy as np
import pdb
import os
import math3d as m3d

import rospy, cv2, rosparam
import tf as ROS_TF

# ROS messages
import sensor_msgs
from sensor_msgs.msg import Image, PointCloud2 as pc2, PointField
from geometry_msgs.msg import PoseStamped, PointStamped, PoseArray, WrenchStamped, Wrench
from visualization_msgs.msg import Marker
import std_msgs
from std_msgs.msg import Float32MultiArray, Bool, Int32, String
from std_msgs.msg import Empty as Empty_msg
from std_srvs.srv import Empty as Empty_srv
import image_geometry

#
import yaml
from enum import Enum

from itertools import groupby


# actionlib server stuff
from bot_common_ros.msg import ARMAction as Action, ARMGoal as Goal, ARMFeedback as Feedback, ARMResult as Result
from bot_common_ros.msg import ActionType, ProcessStatus
from bot_common_ros.actions import FSMActionServer
from bot_common_ros.arm_client import ArmClient
from bot_common_ros.msg import ArmCommand
from bot_chisel.msg import ChiselStatus

from bot_common_ros.ur_control import *
from bot_common_ros.ur_control import TorreyUSBScale
from bot_common_ros.ur_control import URMiddleWare, getAngleLimit, TimeOut, Mapping, QueryNormal, WaitForActive, CheckDistance, getDist, CheckSphere, RayIntersect, RayIntersect_ARR, force_touch3
from bot_common_ros.ur_utils import TCP_BASE_TF, getHorizontalOrient, m3d_to_ros, linearControl, rotate3V, rotate6V, UR_FORCE_PID, rosParamToClassObject, closest_pt_to_array, SensorFeedbackControl, SensorFeedbackControl2, ROStoM3d, fill_pose_stamped
from bot_common_ros.ur_utils import *


from bot_chisel.chisel_force_control import ChiselForceControl
from bot_chisel.chisel_decouple_control import ChiselDecoupleControl

from bot_chisel.msg import CHISELAction, CHISELGoal, CHISELFeedback, CHISELResult
from bot_perception_ros.srv import updatePC
from bot_common_ros.srv import ArmCmdSrv
from dynamic_reconfigure.server import Server

from bot_chisel.cfg import ChiselControllerConfig
from rospy_message_converter import message_converter
import random

from bot_overseer_api import OverseerAPI

VEL = 0.3
ACC = 2 * VEL
POWER_OFF = 0
POWER_ON = 1




class CheckProgress():
    def __init__(self, timeout, min_dist):
        self.timeout = timeout
        self.min_dist = min_dist
        self.reset()

    def update(self, pose):
        self.poses.append(pose)
        self.times.append(time.time())
    
    def query(self, timeout):

        t_start_timeout_now = time.time() - timeout # start time of check progress if timeout where to happen now.
        arr = np.array(self.times)
        times = np.expand_dims(arr, axis=1)
        delta_times = times - t_start_timeout_now
        idx, _ = closest_pt_to_array(times, t_start_timeout_now)
        
        return self.poses[idx], delta_times[idx]

    def check(self, current_pose):
        pose, dt = self.query(self.timeout)
        if dt > 0.0:
            # means that the delta time wasn't actualy less than self.timeout
            # seconds ago
            return False, 0.0
        else:
            d_pos = pose.pos.array - current_pose.pos.array
            dist = np.linalg.norm(d_pos)
            was_no_progress = dist < self.min_dist
            return was_no_progress, dist

    def reset(self):
        self.poses = []
        self.times = []


class RoboChiselActionServer(FSMActionServer):
    def __init__(self, *args):
        self.__version__ = "1_0"
        rospy.init_node('RoboChiselActionServer_v'+self.__version__, anonymous=False)
        super(RoboChiselActionServer, self).__init__(*args)

        self.overseer_api = OverseerAPI()
        self.attack_pose_pub = rospy.Publisher("/chisel/attack_pose", PoseStamped, queue_size=1)
        self.pub_target_frame = rospy.Publisher("/chisel/target_frame", PoseStamped, queue_size=1)
        self.pub_stuck_pose = rospy.Publisher("/chisel/stuck_pose", PoseStamped, queue_size=1)
        # Performance metrics
        self.pub_pre_attack_duration = rospy.Publisher("/chisel/pre_attack_duration", Float32, queue_size=1)
        self.pub_attack_duration = rospy.Publisher("/chisel/attack_duration", Float32, queue_size=1)
        self.pub_extract_duration = rospy.Publisher("/chisel/extraction_duration", Float32, queue_size=1)
        self.pub_mass_per_attack = rospy.Publisher("/chisel/mass_per_attack", Float32, queue_size=1)
        self.pub_total_mass = rospy.Publisher("/chisel/total_mass", Float32, queue_size=1) 
        self.pub_attack_angle = rospy.Publisher("/chisel/attack_angle", Float32, queue_size=1)

        self.collision_monitor_sub = rospy.Subscriber("/pathserver_chisel/colliding_pairs", String, self.collision_monitor)
        self.contact_with_wall = False
        self.contact_with_track_mount = False
       
        self.GLOBAL_DONE = False
             
        self.using_overseer    = rospy.get_param("~use_overseer")
        self.dry_run      = True
        self.run_collision_check = False
              
        self.motion_primitive_timeout = 30.0
        
        # Force conitrol
        self.force_control = ChiselForceControl()
        self.decouple_control = ChiselDecoupleControl()

        self.wrench = rospy.Subscriber("/chisel/wrench", WrenchStamped, self.wrench_cb)

        self.motion_planner = MotionPlanner("chisel", "chisel_base")

        # These params are common for middleware usage and chisel controller
        self.direction_to_chisel = -1
       
        self.nominal_angle = 60.0 / 180.0 * np.pi # overwritten by YAML
      
        self.weight_scale = TorreyUSBScale("/dev/ttyUSB0")
        
        self.as_assert(self.nominal_angle > 0.0)  # supposed to be positive regardless of direction-to-chisel angle
        
        ###
        self.minimum_progress_timeout = 3.0
        self.minimum_progress_dist = 0.015
 
        #----------------------------------------------
        # YAML parameters which get overwritten by loading in the RoboChiselActionServer.yaml file        
        self.result = CHISELResult()

        self.INIT_COMPLETE = False
        self.restart_shave = False
        self.qn = QueryNormal(ns="/ctm")

        self.tf_listener = ROS_TF.TransformListener()
        self.upc = rospy.ServiceProxy('/ctm/wall/updatePC', bot_perception_ros.srv.updatePC)

        # Force cmd ROS service
        self.ur_force_cmd = rospy.ServiceProxy('/ur10/force_cmd', ArmCmdSrv)

        # Dynamic reconfigure
        self.dyno_srv = Server(ChiselControllerConfig, self.dynamic_configure_cb)
    

    def wrench_cb(self, msg):

        self.wrench_tcp = msg.wrench
        self.force_control.wrench_tcp = msg.wrench
        self.decouple_control.wrench_tcp = msg.wrench

    def get_tf_base_tcp(self):

        (trans, rot) = self.tf_listener.lookupTransform("chisel_base", "chisel_TCP", rospy.Time(0))
        return ROStoM3d(trans, rot)

    def param_init(self):
        # get parameters from param-server or yaml
        params_as = rospy.get_param("/chisel/action_server")
        params_mw = rospy.get_param("/chisel/middleware")
        # read values from param-server into already-existing attributes
        rospy.logwarn("WARNING: Resetting all (valid) default-values to the yaml values.")
        rosParamToClassObject(self, params_as, must_already_exist=True)
        rosParamToClassObject(self, params_mw, must_already_exist=True)

    def get_tool_euler_angles(self, orient):
        """ Method to return euler angles to rotate from target-frame to tcp frame
        """
        euler = ((self.target_frame.inverse).orient * orient).to_euler('YXZ')
        return euler

    def decouple_check(self):
        """ just make sure we're decoupled. This only triggers when it needs to, otherwise skips over
        """
        self.check_faults()
        self.decouple_from_rock(0.05)

    def decouple_from_rock(self, maxDist, final_pose=None):
        """ decouple from rock by neutralizing the torque vector and wiggling out
        """
        t_extract_start = time.time()
        self.decouple_control.init_PID_controllers()
        # TODO: parameterize
        wiggleBool = -1.0 * np.sign(self.direction_to_chisel)
        wiggleTorque = wiggleBool * self.decouple_wiggle_torque
        wiggleTime = 0.5
        time_before_stuck = self.decouple_stuck_timeout
        wiggle_torque_tool_y = self.max_torque_tool_y * 0.4 * self.direction_to_chisel
       
        #c
        stuck_timer = TimeOut(time_before_stuck)
        wiggle_timer = TimeOut(wiggleTime)
        #
        done = False
        decouple_in_progress = True
        
        pose = self.get_tf_base_tcp()
        initial_y = final_pose.position.y

        # decouple_angle = self.get_tool_euler_angles(geometry_msg_to_m3d(final_pose).orient)[0]
        decouple_angle = self.get_tool_euler_angles(pose.orient)[0]

        rospy.logwarn('decouple progress {}'.format(decouple_in_progress))
        rospy.logwarn('not done {}'.format(not done))
        rospy.logwarn('self.contact with wall {}'.format(self.contact_with_wall))
        time.sleep(1.0)
        
        while (decouple_in_progress and not done) and self.contact_with_wall:
            
            close_to_attack_pose = pose.pos.y - initial_y < self.decouple_dist_thresh
            decouple_in_progress = not close_to_attack_pose
            done = self.check_faults()  
            pose = self.get_tf_base_tcp()
              
            torque, limits = self.decouple_control.get_force_inputs(self.direction_to_chisel, \
                                                    decouple_angle, initial_y, self.target_frame)
          
            torque[3] += wiggleTorque
            
            limits[2] = 0.1
            limits[3] = 0.03
            limits[4] = 0.03
            
            done = self.check_faults()
            if np.abs(self.wrench_tcp.torque.y) > self.max_torque_tool_y:

                self.overseer_api.run_private_op('UR10.STOP', timeout=0.5)
                rospy.logwarn("Torque is greater than max-allowable. Use polyscope to decouple. Then click continue")
                self.update_status(ChiselStatus.STUCK)
                self.publish_stuck_pose(pose)
                done = self.check_faults()
            else:
                torque = torque.tolist()
                limits = limits.tolist()
                torque[3] =  torque[3]
                tf_force = pose
                
                self.force_mode(pose_base=tf_force, torque=torque, limits=limits, selection_vector=np.ones(6), setRemote=True)
                rospy.logwarn("Sending decouple force commands.{}, {}, {}".format(str(torque[0]), str(torque[1]), str(torque[2])))
            if wiggle_timer.check_and_reset():
                wiggleBool *= -1.0 # toggle back and forth to create the wiggle
                wiggleTorque = wiggleBool * self.decouple_wiggle_torque

            if stuck_timer.check_and_reset():
                rospy.loginfo("Auto-decouple timed-out. Use polyscope to decouple. Then click continue")
                self.publish_stuck_pose(pose)
                self.update_status(ChiselStatus.STUCK)
            
            if self.contact_with_track_mount:
                rospy.logerr('Contact between arm and tracks.')
                self.update_status(ChiselStatus.STUCK)
            
            rospy.logwarn('decouple progress {}'.format(decouple_in_progress))
            rospy.logwarn('not done {}'.format(not done))
            rospy.logwarn('self.contact with wall {}'.format(self.contact_with_wall))

            self.pub_attack_angle.publish(np.rad2deg(self.get_tool_euler_angles(pose.orient)[0]))
            time.sleep(0.15)

            
        self.pub_extract_duration.publish(time.time() -  t_extract_start)
        self.overseer_api.run_private_op('UR10.STOP', timeout=0.5)


    def publish_stuck_pose(self, tf):
        
        ps = PoseStamped()
        ps.header.frame_id = 'chisel_base'
        ps.pose = m3d_to_ros(tf)

        self.pub_stuck_pose.publish(ps)


    def dynamic_configure_cb(self, config, level):
        """ Calback for dynamic reconfigure.
        """
        self.decouple_force = config.decouple_force
        self.decouple_max_torque = config.decouple_max_torque
        self.decouple_wiggle_torque = config.decouple_wiggle_torque
        self.decouple_stuck_timeout = config.decouple_stuck_timeout
        self.arm_overstretch_thresh = config.arm_overstretch_thresh
        self.minimum_progress_timeout = config.minimum_progress_timeout
        self.minimum_progress_dist = config.minimum_progress_dist
        self.move_dist_thresh = config.move_dist_thresh
        self.penetration_tol = config.penetration_tol
        self.max_torque_tool_y = config.max_torque_tool_y

        return config  


    def check_x_value(self, nominal, pose):
        """ tolerance > 0.0 means it's on the 'inside' of the x-value w.r.t. direction_to_chisel
        """
        dist = pose.pos.x - nominal
        if self.direction_to_chisel > 0.0:
            is_past = dist > 0
        else:
            is_past = dist < 0
        if is_past:
            rospy.logwarn("Shave done: reached end of x-value.")
        return is_past


    def set_rotation_extrema(self, tf_contact):
        
        max_angles = self.get_tool_euler_angles(tf_contact.orient)
        min_angles = self.get_tool_euler_angles(tf_contact.orient)

        if self.direction_to_chisel == 1:    
            max_angles[0] = 0.5*(self.get_tool_euler_angles(tf_contact.orient)[0] + self.nominal_angle)
            min_angles[0] = self.nominal_angle
        else:
            max_angles[0] = 0.5*(self.get_tool_euler_angles(tf_contact.orient)[0] + self.nominal_angle)
            min_angles[0] = self.nominal_angle

        rospy.loginfo("Get extrema angles {} {}".format(min_angles[0], max_angles[0]))
        self.force_control.set_rotation_extrema(min_angles, max_angles)
        self.decouple_control.set_rotation_extrema(min_angles, max_angles)



    def force_touch3(self, threshold, contactThreshold, cmd_speed = 0.12):

        if contactThreshold > threshold:
            print("force_touch3: ContactThreshold must be less than threshold. Returning.")
            return

        def func():
            fz = self.wrench_tcp.force.z
            return fz
        contact_timeout = TimeOut(10.0)
        lc1 = linearControl(0.0, threshold, func, cmd_speed, cmd_speed) # 0.125 to 0.05
        contactPose = self.get_tf_base_tcp()
        contactFlag = False
        force_cmd = rospy.get_param('/chisel_ctrl/contact_force_cmd')
        
        while abs(func()) < threshold: 
            speed = lc1.getControl()
            # print 'force_touch3: speed', speed
            pose = self.get_tf_base_tcp()
            
            self.force_mode(pose_base=pose, torque=[0,0,force_cmd,0,0,0], limits=[0,0,speed,0,0,0], selection_vector=np.ones(6), setRemote=True)
            
            # print 'force_touch3: force', abs(func())
            if not contactFlag and abs(func()) > contactThreshold:
                contactPose = pose
                contactFlag = True
            if contact_timeout.check():
                rospy.loginfo("Took too long to make contact.")
                self.overseer_api.run_private_op('UR10.STOP', timeout=0.5)
                return None
            time.sleep(0.1)

        rospy.loginfo("Contact made at {}". format(contactPose.pos.y))
        return contactPose


    def motion_primitive_shave(self, exit_x_value):
        """ shave the rock, monitoring progress and resetting if necessary
        """
        done = False
        self.power_on()  # turn on drill

        # contact the rock
        tf_contact = self.force_touch3(80, 50)
        self.tf_contact = tf_contact
        self.pub_pre_attack_duration.publish(time.time() - self.time_start)

        if tf_contact is None:
            self.update_status(ChiselStatus.STUCK)
            return            
        
        self.set_rotation_extrema(tf_contact)
        self.attack_timeout = TimeOut(self.motion_primitive_timeout)
        self.rotation_timeout = TimeOut(5.0)
        self.check_progress = CheckProgress(self.minimum_progress_timeout, self.minimum_progress_dist)

        time_attack_start = time.time()
        done = False

        while not done:
            
            current_pose = self.get_tf_base_tcp()
            
            # Exit loop when shave completed or stalled
            if self.check_shave_done(exit_x_value) or self.check_stall():
               break
            
            attack_angle = self.get_attack_angle(tf_contact, current_pose)
            rospy.loginfo("Target angle for the attack. {}".format(attack_angle))
            rospy.loginfo("Direction for the attack. {}".format(self.direction_to_chisel))

            torque, limits = self.force_control.get_force_inputs(self.direction_to_chisel, attack_angle, \
                                                            tf_contact.pos.z, self.target_frame)
            print torque
            torque = torque.tolist()
            limits = limits.tolist()
            tf_force = current_pose

            # if self.direction_to_chisel > 0.0:
            #     tf_force.orient.rotate_zt(-3.14)
            self.force_mode(pose_base=tf_force, torque=torque, limits=limits, \
                                    selection_vector=np.ones(6), setRemote=True)
            # -------------------------------------------------------------------
            # Check for faults
            done = self.check_faults()
            time.sleep(0.1)
            self.pub_attack_angle.publish(self.get_tool_euler_angles(current_pose.orient)[0])
        
        self.pub_attack_duration.publish(time.time() - time_attack_start)
        self.overseer_api.run_private_op('UR10.STOP', timeout=0.5)

        
        # Extract from wall
        self.decouple_from_rock(0.07, self.attackPose)
             
        print('finish back-up. Turn drill off.')
        self.power_off()

        if self.get_state() == ChiselStatus.ACTIVE:
            self.update_status(ChiselStatus.DONE)
       


    def get_attack_angle(self, tf_contact, current_pose):

    # Adjust nominal angle based on penetration status
        if current_pose.pos.y - tf_contact.pos.y > self.penetration_tol:
            rospy.loginfo("Penetrated the wall at {}".format(current_pose.pos.y))
            return 0.8*self.nominal_angle
            
        else:
            nominal_angle_final = self.get_tool_euler_angles(tf_contact.orient)[0]
            return nominal_angle_final


    def check_shave_done(self, exit_x_value):

        done = False
        # check distance to end
        if self.check_x_value(exit_x_value, self.get_tf_base_tcp()):
            done = True
            rospy.loginfo("Shave done: Reached exit x.")

        # check for time out
        if self.attack_timeout.check():
            done = True
            rospy.loginfo("Shave done: timed out.")

        if done or self.check_faults():
            done = True
            rospy.loginfo("Shave done: Check Flags.")    
            
        return done


    def check_stall(self):

        current_pose = self.get_tf_base_tcp()
        self.check_progress.update(current_pose) 
        stalled = False

        if self.check_progress.check(current_pose)[0]:
            rospy.logwarn("Stalled: check_progress")
            stalled = True

        if np.abs(self.wrench_tcp.torque.y) > self.max_torque_tool_y:
            rospy.logwarn("Stalled: overtorqued: {}".format(self.wrench_tcp.torque.y))
            stalled = True
        
        tf_base_tool0 =  self.get_tf_base_tcp() * self.tf_tcp_to_tool0
        dist_base_tool0 = tf_base_tool0.pos.dist(m3d.Vector(0, 0, 0))
        if dist_base_tool0 > self.arm_overstretch_thresh:
            stalled = True
            rospy.logwarn("Stall: Arm Streching out")

        if self.rotation_timeout.check():
            if abs(self.get_tool_euler_angles(self.tf_contact.orient)[0]) < (abs(self.nominal_angle) - np.deg2rad(5.0)): # TODO: parameterize
                rospy.logwarn("Stall: Unable to rotate.")
                stalled = True

        if abs(self.get_tool_euler_angles(self.tf_contact.orient)[0]) > (abs(self.nominal_angle) + np.deg2rad(5.0)): # TODO: parameterize
                rospy.logwarn("Stall: Overrotation.")
                stalled = True

        if stalled:
            pass
            # self.update_status(ChiselStatus.STALL) 
            
        return stalled



    def move_to_attack_pose(self, attack_pose, attack_traj):

        pose_dict = message_converter.convert_ros_message_to_dictionary(attack_pose) 
        self.overseer_api('UR10.SMART_MOVE', pose_dict=pose_dict, velocity=0.4, acceleration=0.8)

        
        rospy.loginfo("Moved to attack pose")
        return True


    def action_single_step(self, goal):
        
        self.time_start = time.time()
        
        # get attack position
        attack_pose = goal.attack_pose
               
        weight_pre_attack = self.weight_scale.getTotalWeight()
        
        self.attackPose = m3d_to_ros(self.get_tf_base_tcp()) # attack_pose
        self.direction_to_chisel = goal.attack_params.direction_to_chisel
        self.nominal_angle = goal.attack_params.nominal_angle * -1 * self.direction_to_chisel
        exit_x_value = goal.attack_params.exit_x_value
        rospy.loginfo("Exit x value: {}".format(exit_x_value))
        
        self.target_frame = geometry_msg_to_m3d(goal.target_frame)
       
        ps = PoseStamped()
        ps.pose = goal.target_frame
        ps.header.frame_id = "chisel_base"
        self.pub_target_frame.publish(ps)
        # execute motion primitive
        self.motion_primitive_shave(exit_x_value)

        if self.restart_shave:
            self.motion_primitive_shave(exit_x_value)
        
        weight_post_attack = self.weight_scale.getTotalWeight()

        self.pub_mass_per_attack.publish(weight_post_attack - weight_pre_attack)

        return True

    
    def force_mode(self, pose_base=None, torque=[0]*6, limits=[0]*6, selection_vector=np.ones(6), setRemote=True):
        """ geometry_msgs/Pose pose_base
            std_msgs/Float32MultiArray torque
            std_msgs/Float32MultiArray limits
            std_msgs/Float32MultiArray selection_vector
            bool setRemote
        """

        if not pose_base:
            pose_base = self.get_tf_base_tcp()

        if self.use_ros_api:
            req = ArmCommand()
            req.pose_base = m3d_to_ros(pose_base)
            req.torque.data = torque
            req.limits.data = limits
            req.selection_vector.data = list(selection_vector)
            resp = self.ur_force_cmd(req)

            return

        # message_converter.convert_ros_message_to_dictionary(m3d_to_ros(pose_base))
        pose_base = pose_msg_to_list(m3d_to_ros(pose_base))
        
        # check for remote control mode
        self.overseer_api.run_private_op('UR10.REINIT')
        # Using below limits eliminates protective events.
        # limits = [0.1, 0.1, 0.1, 0.05, 0.05, 0.050]

        
        self.overseer_api.run_private_op('UR10.FORCE_MODE', frame_transform=pose_base, wrench=torque, \
                                                    selection_vector=selection_vector, limits=limits)


    def check_faults(self):
        """ Check operational flags
        """
        preempted = super(RoboChiselActionServer, self).check_flags()
        done = rospy.is_shutdown()
        
        arm_active = self.overseer_api.get_tm('CHISEL.ARM.STATE')=='ROBOT_MODE_RUNNING'

        if self.check_status(ChiselStatus.STUCK) or not arm_active:
            self.is_stuck = True 
            self.update_status(ChiselStatus.STUCK)
            done = True
            # self.lock_gate()
            # self.gate()
            # self.update_status(ChiselStatus.ACTIVE)
            # self.power_on()

        if self.GLOBAL_DONE: 
            done = True
        done = done or preempted

        # # wait for UR monitor to handle Polyscope errors
        # while self.overseer_api.get_tm('CHISEL.ARM.SAFETY_STATUS')!='NormalMode':
        #     pass

        return done

    def collision_monitor(self, msg):
        self.colliding_pairs = msg.data.split(",")

        if '<octomap>-chisel_chisel_link' in self.colliding_pairs:
            self.contact_with_wall = True
        else:
            self.contact_with_wall = False

        links = []
        for pair in self.colliding_pairs:
            links += pair.split('-')
            
        if 'camera_track_mount' in links:
            self.contact_with_track_mount = True
        else:
            self.contact_with_track_mount = False

            
    def SHAVER(self, goal):
        #
        self.as_assert(self.INIT_COMPLETE)
        print("Begin the chiseling shaving action.")

        # ---------------------------------------------------------------
        done = False
        
        pose = PoseStamped()
        pose.header.frame_id = "chisel_base"
        pose.pose = goal.attack_pose
        self.attack_pose_pub.publish(pose)

        # execute single action
        if not self.action_single_step(goal):
            print("Action single step failed...")
        
        

    def CLEANUP(self, goal):
        """ after shaving, clean up the remaining protusions using the sandstone-targeter CV
        """
        pass

    def ACTIVE(self, goal):
        """ Parse goal to determine action
        """

        self.SHAVER(goal)

        # Scan the wall
        # self.scan_wall(goal.attack_pose)
            
        self.power_off()

        self.result.chisel_pose = m3d_to_ros(self.get_tf_base_tcp())
        self.result.chisel_status.status = self.get_state()
       
    

    def power_off(self):
        if not self.dry_run:
            self.overseer_api.run_private_op('CHISEL.HAMMER.ON', state=0, type=1)
        time.sleep(0.1)

    def power_on(self):
        if not self.dry_run:
            self.overseer_api.run_private_op('CHISEL.HAMMER.ON', state=1, type=1)
        time.sleep(0.1)

    def emergency_stop(self):

        self.overseer_api.run_private_op('UR10.STOP', timeout=0.5)
        self.power_off()
    
    def PAUSED(self):
        
        self.overseer_api.run_private_op('UR10.STOP', timeout=0.5)
        self.power_off()
        #
        super(RoboChiselActionServer, self).PAUSED()

    def RESET(self, goal):

        self.update_status(ChiselStatus.DONE)
        
        time.sleep(1.5)
    
    def DONE(self, goal):
        self.WAIT()
        

    def WAIT(self):
        sleep_hz = 10.0 # hz
        ros_rate = rospy.Rate(sleep_hz)
        ros_rate.sleep()

    def IDLE(self, goal):
        """ FSM loop calls this. 
        """        
        self.WAIT()
    
    
    def INIT(self, goal):
        """INIT -- unstows arm for operation, reads operating box, sets middle-ware parameters
        """
        self.INIT_COMPLETE = False
        self.param_init()
        
        # Init controller gains

        self.force_control.init_PID_controllers()
        self.decouple_control.init_PID_controllers()

        self.dry_run             = rospy.get_param("~dry_run", True)
        self.run_collision_check = rospy.get_param("~run_collision_check", False)
        self.decouple_dist_thresh = rospy.get_param("/chisel_ctrl/decouple_dist_thresh")
        self.colliding_pairs = []

        self.use_ros_api = rospy.get_param("/chisel_ctrl/use_ros_api")

        (trans, rot) = self.tf_listener.lookupTransform("chisel_tip_link", "chisel_tool0", rospy.Time(0))
        self.tf_tcp_to_tool0 = ROStoM3d(trans, rot)

        self.is_stuck = False 

        if self.dry_run:
            rospy.loginfo("Initing dry-run")
        else:
            rospy.loginfo("Initing wet-run")

        time.sleep(0.5) #time for ROS nodes to become active

        rospy.on_shutdown(self.on_shutdown) # callback when rospy gets shut down

        # self.scan_wall(None)
        self.update_status(ChiselStatus.READY)
        self.INIT_COMPLETE = True


    def READY(self, goal):
        """ Unstow complete, waiting for user to press Op-Execute
        """
        self.WAIT()

    def STUCK(self, goal):
        """ Nothing to be done, waiting for user intervention
        """
        self.WAIT()
       
        # rospy.logerr("Arm is stuck")
    

    def STALLED(self, goal):
        """ Nothing to be done, waiting for user intervention
        """
        self.WAIT()

    def SLIPPED(self, goal):
        """ Nothing to be done, waiting for user intervention
        """
        self.WAIT()

    def SHUTDOWN(self, goal):
        """stow arm, return to Idle
        """
        self.overseer_api.run_private_op('UR10.STOP', timeout=0.5)
        self.power_off()
        
        self.update_status(ChiselStatus.IDLE)

    def _fill_process_status(self):
        """ Process status:
        uint8 UNINIT=0
        uint8 READY=1
        uint8 ACTIVE=2
        uint8 COMPLETE=3
        uint8 FAILED=4
        uint8 STANDBY=5
        uint8 status
        """
        status_msg = ChiselStatus
        mapping = { status_msg.IDLE:     ProcessStatus.UNINIT,
                    status_msg.INIT:     ProcessStatus.UNINIT,
                    status_msg.READY:    ProcessStatus.READY,
                    status_msg.ACTIVE:   ProcessStatus.ACTIVE,
                    status_msg.RESET:    ProcessStatus.ACTIVE,
                    status_msg.PAUSED:   ProcessStatus.STANDBY,
                    status_msg.STUCK:    ProcessStatus.COMPLETE,
                    status_msg.STALL:    ProcessStatus.COMPLETE,
                    status_msg.SHUTDOWN: ProcessStatus.ACTIVE,
                    status_msg.DONE: ProcessStatus.COMPLETE
        }
        out = ProcessStatus(status = mapping[self.get_state()])
        return out

    def _fill_feedback(self):
        """ Define a feedback message
        ProcessStatus process_status
        ChiselStatus arm_status
        float32 percentComplete
        float32 totalVolume
        """
        out = CHISELFeedback()
        # out.percent_complete = 0.0
        out.process_status = self._fill_process_status()
        self._feedback = out

    def _fill_result(self):
        """ Define the result
        ProcessStatus process_status
        ChiselStatus arm_status
        """
        self._result = self.result
        self._result.process_status = self._fill_process_status()

    def on_shutdown(self):
        #
        self.update_status(ChiselStatus.SHUTDOWN)
        print("\non_shutdown")
               
        self.GLOBAL_DONE = True
        self.power_off()
        #
        self.overseer_api.run_private_op('UR10.STOP', timeout=0.5)
        self.overseer_api.run_private_op('UR10.STOP', timeout=0.5)

        if not rospy.is_shutdown(): rospy.signal_shutdown("End of Program")

    def MAIN(self):
        """
        idle -> [ready, shutdown]
        ready -> active
        active -> [ready, stuck, stall, slip]
        stuck -> [ready]
        stall -> [ready]
        slip -> [ready]
        shutdown -> [idle]
        """
        # define valid operation transitions
        status_msg = ChiselStatus
        op_transitions = {status_msg.IDLE:     [status_msg.INIT],
                            status_msg.READY:  [status_msg.INIT, status_msg.SHUTDOWN, status_msg.ACTIVE],
                            status_msg.ACTIVE: [status_msg.STUCK, status_msg.SLIP,  status_msg.STALL, status_msg.INIT, status_msg.SHUTDOWN],
                            status_msg.RESET: [status_msg.READY, ],
                            status_msg.STUCK:  [status_msg.SHUTDOWN, status_msg.RESET],
                            status_msg.STALL:  [status_msg.SHUTDOWN, status_msg.RESET],
                            status_msg.SLIP:  [status_msg.SHUTDOWN, status_msg.RESET],
                            status_msg.PAUSED: [status_msg.INIT, status_msg.ACTIVE, status_msg.READY],
                            status_msg.DONE: [status_msg.ACTIVE, status_msg.SHUTDOWN, status_msg.INIT]
        }
        self.fsm_register_op_transitions(op_transitions)

        #
        op_wait_states = {status_msg.IDLE: [status_msg.READY],
                            status_msg.INIT: [status_msg.READY],
                            status_msg.READY: [status_msg.IDLE, status_msg.READY, status_msg.RESET],
                            status_msg.ACTIVE: [status_msg.SHUTDOWN, status_msg.READY, status_msg.STALL, status_msg.STUCK,  status_msg.PAUSED, status_msg.DONE],
                            status_msg.RESET: [status_msg.READY, status_msg.DONE],
                            status_msg.STUCK:  [status_msg.SHUTDOWN, status_msg.RESET],
                            status_msg.STALL:  [status_msg.SHUTDOWN, status_msg.RESET],
                            status_msg.SLIP:  [status_msg.SHUTDOWN, status_msg.RESET],
                            status_msg.PAUSED: [status_msg.INIT, status_msg.ACTIVE, status_msg.RESET],
                            status_msg.DONE: [status_msg.ACTIVE, status_msg.RESET, status_msg.SHUTDOWN, status_msg.INIT]
                            
        }
        self.fsm_register_op_waits(op_wait_states)
        
        # register status changes
        status_changes = {ActionType.INIT: status_msg.INIT,
                    ActionType.EXECUTE:    status_msg.ACTIVE,
                    ActionType.RESET:      status_msg.RESET,
                    ActionType.SHUTDOWN:   status_msg.SHUTDOWN
        }
        self.fsm_register_status_changes(status_changes)

        # register functions -- don't need to specify anything for pause & continue
        mapping = { status_msg.IDLE:     self.IDLE,
                    status_msg.INIT:     self.INIT,
                    status_msg.READY:    self.READY,
                    status_msg.ACTIVE:   self.ACTIVE,
                    status_msg.RESET:    self.RESET,
                    status_msg.STUCK:    self.STUCK,
                    status_msg.STALL:    self.STALLED,
                    status_msg.SLIP:     self.SLIPPED,
                    status_msg.SHUTDOWN: self.SHUTDOWN,
                    status_msg.DONE:     self.DONE
                    
        }
        self.fsm_register_state_functions(mapping)

        # run the infinite game-loop
        self.start() # starts the action-server
        rospy.loginfo("Starting infinite FSM loop now.")
        self.fsm_loop()

if __name__ == '__main__':
    
    robo_chisel = RoboChiselActionServer("RoboChisel", CHISELAction, ChiselStatus, ChiselStatus.IDLE)
    try:
        robo_chisel.MAIN()
    finally:
        if not rospy.is_shutdown(): print("End of main. Try-loop Shutdown."); rospy.signal_shutdown("End of Program")

