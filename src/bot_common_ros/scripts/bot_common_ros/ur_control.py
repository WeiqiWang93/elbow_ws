#library for Universal Robot <-- urx control
import urx, URBasic
import math3d as m3d
import numpy as np
import math, time, os, sys, signal, threading, copy, usb, serial
import pdb

import math3d as m3d
from math3d.interpolation import SO3Interpolation as Slerp
from scipy import interpolate
import matplotlib.pyplot as plt # plots

import tf as ROS_TF
import datetime
from rdp import rdp


import rospy, roslib, cv2
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool, Empty
from std_msgs.msg import Float32, String
import image_geometry
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import PointCloud2, JointState
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
import sensor_msgs.point_cloud2 as pc2
import message_filters
import threading, socket
import trajectory_msgs.msg
import moveit_msgs.msg
from bot_arm_planner.srv import GeneratePath, CheckCollision, CheckCollisionList, GetIK
import  bot_perception_ros.srv
import  bot_arm_planner.srv
from tf import TransformListener

from rospy_message_converter import message_converter
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from std_msgs.msg import String
from shape_msgs.msg import SolidPrimitive
#

from bot_common_ros.ur_utils import *
import bot_perception_ros
from bot_perception_ros import *
from copy import deepcopy


class URMiddleWare(object):
    """ Class to handle interfacing to the UR10 libraries and provide high-level functionality.
    This handles all the parameters needed to run the robot, which can be provided from any source, including yaml files.
    """
    def __init__(self, arm_client, params, listener=None):
        assert( isinstance(params, dict) )
        assert( "bounding_box" in params )
        assert( "ur_monitor_expansion" in params )
        self.arm_client = arm_client
        self.listener = listener
        self.motion_planner = MotionPlanner(params["group_name"], params["frame_id"])
        self.params = params
        self.set_bounding_box(self.params["bounding_box"])
        self.nominal_orient = None
        self.min_max_angles_fxn = None
        self.min_max_euler_angles_fxn = None
        self.minn = np.zeros(3)
        self.maxx = np.zeros(3)
        self.minn_euler = np.zeros(3)
        self.maxx_euler = np.zeros(3)
        self.pu = np.array([1.0, 0.0, 0.25]) # global-y doesn't matter (right now)
        self.pl = np.array([0.0, 0.0, 0.25]) # global-y doesn't matter (right now)
        self.gazebo = False

        # private members
        self._use_mp = True

    def set_use_mp(self, flag):
        if (isinstance(flag, bool)):
            self._use_mp = flag
        else:
            rospy.logerr("set_use_mp: flag was not bool")

    def using_mp(self):
        return self._use_mp
  
    def open_loop_move(self, use_mp, **kwargs):        
        self.set_use_mp(False)
        self.smart_move(**kwargs)
        self.set_use_mp(use_mp)
    
    def open_loop_move_new(self, **kwargs):
        seed_js = rospy.wait_for_message("/" + self.params["group_name"]+ "/joint_states", JointState, 60.0)
        # 
        # pdb.set_trace()
        kwargs['vel_rot'] = 0.08
        kwargs['vel'] = 0.015
        if "pose" in kwargs:
            target_type = "movel"
            wps = [kwargs["pose"]]
        elif "poseList_base" in kwargs:
            target_type = "movels"
            wps = [kwargs["poseList_base"]]
        elif "joints" in kwargs:
            target_type = "movej"
            wps = [kwargs["joints"]]
        elif "jointList" in kwargs:
            target_type = "movejs"
            wps = kwargs["jointList"]
        if target_type == 'movels':
            pdb.set_trace()
            ik = []
            for wp in wps:
                success, joints = self.motion_planner.get_ik_srv(self.params["group_name"], seed_js.position, wp[0])
                if success:
                    ik.append(list(joints))                    
                    self.arm_client.movejs(jointList=ik, vel=0.015, wait=True)
                else:
                    pdb.set_trace()
                    self.arm_client.movels(poseList_base = wps, acc = kwargs["acc"], vel_rot=kwargs["vel_rot"], wait=True)
        elif target_type == "movel":
            ik = []
            wp = wps[0]
            success, joints = self.motion_planner.get_ik_srv(self.params["group_name"], seed_js.position, wp[0])
            if success:
                ik.append(list(joints))                
                self.arm_client.movejs(jointList=ik, timeout = 5.0, wait=True)
            else:
                pdb.set_trace()
                self.arm_client.movel(pose = [wp], acc=kwargs["acc"], vel_rot=kwargs["vel_rot"], wait=True)
        elif target_type == "movejs":
            self.arm_client.movejs(jointList=wps, acc=kwargs["acc"], vel_rot=kwargs["vel_rot"], wait=True, radius=0.05, threshold=0.01)


    def smart_move(self, **kwargs):
        """ deduce type of move from the inputs provided
        total inputs from all types of moves:
        pose, joints, poseList_base, jointList, 
            acc, vel, radius, wait, timeout, threshold

        motion-planner input which uses 'wp' variable must be of type m3d.Transform
        """
        if "tol" in kwargs:
            tolerances = kwargs["tol"]
        else:
            tolerances = []
        if "pose" in kwargs:
            target_type = "movel"
            wps = [kwargs["pose"]]
        elif "poseList_base" in kwargs:
            target_type = "movels"
            wps = kwargs["poseList_base"]
        elif "joints" in kwargs:
            target_type = "movej"
            wps = [kwargs["joints"]]
        elif "jointList" in kwargs:
            target_type = "movejs"
            wps = kwargs["jointList"]
        else:
            rospy.logerr("invalid target_type")

        # if using motion-planning, obtain joint states
        if self._use_mp:
            for wp in wps:
                success = False
                start_js = self.arm_client.get_joints()
                if isinstance(wp, m3d.Transform):
                    final_pts, _ = self.motion_planner.plan(start_js=start_js, goal=wp, max_tolerances=tolerances)

                elif isinstance(wp, np.ndarray):
                    final_pts, _ = self.motion_planner.plan(start_js=start_js, goal=list(wp), max_tolerances=tolerances)
                else:
                    rospy.logerr("invalid input to motion planner.")

                # Added trace to overwrite final_pts if desired, overwrite with final_pts = [[]]

                # safety if MP fails
                if len(final_pts) == 0:
                    print("\n\n\n")
                    rospy.logerr("Motion Planning Failed. Throwing Trace")
                    k = raw_input("Do you want to continue with an Open-Loop Movement?")
                    if k == 'y':
                        kwargs["vel"] = kwargs["vel"]/2
                        fcn = eval("self.arm_client."+target_type)
                        fcn(**kwargs)
                    else:
                        return
                    
                    
        else:
            if self.gazebo:
                # TO-DO: Add compatability for movels 
                if target_type == 'movel':
                    ik = []
                    for wp in wps:
                        seed_js = rospy.wait_for_message("/saw/joint_states", JointState, 60.0)
                        js = self.motion_planner.get_ik_joints(seed_js.position, wp)
                        ik.append(list(js))
                        ik.append(list(js))
 
                    self.arm_client.movejs(jointList=ik, acc=kwargs["acc"], vel=kwargs["vel"], radius=0.05, wait=True, threshold=0.3)
            else:
                # self.open_loop_move_new(**kwargs)
                fcn = eval("self.arm_client."+target_type)
                fcn(**kwargs)

    def check_collision(self, group, pose_start, pose_end, excluded_links=[]):
        """ transform pose into /base_footprint frame, then call the MP check """
        ik_seed_js = self.arm_client.get_joints()
        in_collision, poses_ros = self.motion_planner.check_collision(group, ik_seed_js, pose_start, pose_end, excluded_links)
        out = [ROStoM3d(ps.pose.position, ps.pose.orientation) for ps in poses_ros]
        return in_collision, out
    
    def check_collision_list(self, group, input_poses, excluded_links=[], attached_markers = []):
        ik_seed_js = self.arm_client.get_joints()
        in_collision, poses_ros = self.motion_planner.check_collision_list(group, ik_seed_js, input_poses, excluded_links, attached_markers)
        out = [ROStoM3d(ps.pose.position, ps.pose.orientation) for ps in poses_ros]
        return in_collision, out

    def compare_to_joint_limit(self, pose_list):
        joint_limits = rospy.get_param("/robot_description_planning/joint_limits/")
        tool_joint = self.motion_planner.joint_names[-1]
        min_wrist3 = joint_limits[tool_joint]["min_position"]
        max_wrist3 = joint_limits[tool_joint]["max_position"]

        joints0 = self.arm_client.get_joints()
        success, joints = self.motion_planner.get_ik_srv(self.params["group_name"], joints0, pose_list[0])
        success2, joints2 = self.motion_planner.get_ik_srv(self.params["group_name"], joints0, pose_list[1])

        good = True
        tol = 20.0 / 57.0 # 20 degrees
        if success and success2:
            # check for logical IK values -- this check means that a large movement was made in wrists 1 & 2 joint space which is usually undesirable
            diff_mid_final   = np.linalg.norm(joints2[3:5] - joints[3:5])
            diff_mid_initial = np.linalg.norm(joints0[3:5] - joints[3:5])
            diff_initial_final = np.linalg.norm(joints0[3:5] - joints2[3:5])
            if diff_initial_final > 0.5 and (diff_mid_final > 0.7 * diff_initial_final or diff_mid_initial > 0.7 * diff_initial_final):
                rospy.logerr("CAUTION: IK MIGHT BE WRONG ROTATION")
                pdb.set_trace()
                # sys.exit()
                
            # check final position w.r.t. middle-position joints
            wrist3 = joints[5]
            if joints0[5] > joints2[5]:
                max_j = joints0[5]
                min_j = joints2[5]
            else:
                max_j = joints2[5]
                min_j = joints0[5]
            good = wrist3 > min_j and wrist3 < max_j
        else:
            success = False
        good = good or np.abs(max_j - min_j) < tol
        if not good:
            print(" not good, check it out to confirm correct behavior! ")
            pdb.set_trace()
        return success, good

    def gen_correct_rotation(self, tf, move_offset=0.0):
        pose_list = self.gen_3pt_spline(self.arm_client.get_pose(), tf, move_offset)
        # check joint values if target type is movel or movels
        success, good = self.compare_to_joint_limit(pose_list)
        if not success:
            rospy.logerr("IK was not successful. Do you want to continue without checking the middle-pose?")
            pdb.set_trace()

        if not good:
            print(" not good, check it out to confirm correct behavior! ")
            # pdb.set_trace()
            pose_list[0].orient.rotate_zt(np.pi)

        return pose_list
    
    def publish_pose_list(self, pose_list, pose_list_pub):
        msg = PoseArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "chisel_base"
        for m3d_pose in pose_list:
            ros_pose = m3d_to_ros(m3d_pose)
            msg.poses.append(ros_pose)
        pose_list_pub.publish(msg)
    
    def moveToBBox(self, alphaARR, acc, vel, use_3pt_spline=False):
        tf = m3d.Transform()
        tf.orient = self.get_nominal_orient()
        tf.pos = getAlphaOfBoundingBox(self.get_bounding_box(), alphaARR)
        if use_3pt_spline:
            pose_list = self.gen_correct_rotation(tf)
            self.smart_move(poseList_base=pose_list, tol = [0]*6, acc=acc, vel=vel, radius=0.03, threshold=0.01)
        else:
            self.smart_move(pose=tf, tol = [], acc=acc, vel=vel, wait=True)

    def get_bounding_box(self):
        return np.copy(self.bounding_box)

    def set_bounding_box(self, bbox):
        assert(len(bbox) == 6)
        self.bounding_box = np.array(bbox)
        # set in arm-client
        expanded_bbox = self.getExpandedBoundingBox(self.params['ur_monitor_expansion'])
        self.arm_client.set_bounding_box(bbox_to_ros(self.bounding_box))

    def get_nominal_orient(self):
        return self.nominal_orient.copy()

    def set_nominal_orient(self, orient):
        assert(isinstance(orient, m3d.Orientation))
        self.nominal_orient = orient

    def get_max_angles(self, pos_arr):
        if self.min_max_angles_fxn is None:
            return self.minn, self.maxx
        else:
            minAngle_base, maxAngle_base = self.min_max_angles_fxn(pos_arr)
            return minAngle_base, maxAngle_base

    def set_min_max_angles_fxn(self, fxn):
        self.min_max_angles_fxn = fxn

    def set_euler_angles(self, minn, maxx):
        self.minn_euler = np.copy(minn)
        self.maxx_euler = np.copy(maxx)

    def get_max_euler_angles(self, pos_arr):
        if self.min_max_euler_angles_fxn is None:
            return self.minn_euler, self.maxx_euler
        else:
            minn, maxx = self.min_max_euler_angles_fxn(pos_arr)
            return minn, maxx

    def set_min_max_euler_angles_fxn(self, fxn):
        self.min_max_euler_angles_fxn = fxn

    def getExpandedBoundingBox(self, percentage):
        """ percentage is how much ADDITIONAL size to add on
        forces a minimum expansion of 10cm in the x & z sides in-case bbox is too small to begin with.
        """
        boundingBox = self.get_bounding_box()
        expansion = boundingBox[3:6] * percentage # by % on either side
        expansion = np.clip(expansion, 0.1, None) # same thing as `expansion[expansion < 0.1] = 0.1`
        #
        boundingBox[0] -= expansion[0]
        boundingBox[1] -= expansion[1]
        boundingBox[2] -= expansion[2]
        boundingBox[3:6] += 2.*expansion[0:3]
        # rospy.loginfo("bounding box: {}".format(boundingBox))
        return boundingBox

    def angles_from_maxima(self, pose):
        ornt = pose.orient.copy()
        anglesOriginal = getAngleLimit(ornt * m3d.Vector(0, 0, -1))

    def limit_tool_eulers(self, pose):
        """ use euler angles to limit the input orient. Assumes min/max euler angles are in order "YXZ"
        """
        eulers = self.get_tool_euler_angles(pose.orient)
        minn, maxx = self.get_max_euler_angles(pose.pos.array)
        eulers_clipped = np.clip(eulers, minn, maxx)
        new_ornt_hf = m3d.Orientation.new_euler(eulers_clipped, "YXZ")
        new_ornt_bf = getHorizontalOrient() * new_ornt_hf
        new_pose = pose.copy()
        new_pose.orient = new_ornt_bf
        # pdb.set_trace()
        return new_pose
    


    def get_tool_euler_angles(self, orient):
        """ get euler angles from the orient to the horizontal orientation, following a "YXZ" (RH rule: Down-Right-Forward) euler transformation.
        YXZ means rotation about the tool-y, then rotation about the tool-x, then rotation about the tool-z

        assumes that pose is in the "tool_base" frame.
        """
        o_bh = getHorizontalOrient()
        o_hf = o_bh.inverse * orient

        eulers = o_hf.to_euler("YXZ")
        # pdb.set_trace()
        return eulers

    def checkAngleLimits(self, pose, maxAngle):
        # check limits - make sure angle isn't too extreme
        # maxAngle is a delta-angle in tool-frame

        minAngle_base, maxAngle_base = self.get_max_angles(pose.pos.array)
        ornt = pose.orient.copy()
        anglesOriginal = getAngleLimit(ornt * m3d.Vector(0, 0, -1))
        ornt.rotate_yt( maxAngle )
        angles = getAngleLimit(ornt * m3d.Vector(0, 0, -1))
        angles = angles.clip(minAngle_base, maxAngle_base)
        #convert back to tool
        anglesOriginal_tool = pose.orient.inverse * anglesOriginal
        angles_tool = pose.orient.inverse * angles
        deltaAngle = angles_tool - anglesOriginal_tool

        #pdb.set_trace() #check if it changes a copy, or the original
        return deltaAngle

    def gen_3pt_spline(self, init_pose, target_pose, offset, offset_dir=None):
        """ generate 3 point spline from init_pose --> midpoint w/ arc --> target_pose
        """
        def slerp(p0, p1, t):
            omega = np.arccos(np.dot(p0/np.linalg.norm(p0), p1/np.linalg.norm(p1)))
            so = np.sin(omega)
            return np.sin((1.0-t)*omega) / so * p0 + np.sin(t*omega)/so * p1

        tf = m3d.Transform()
        pts = calcMidPoint(init_pose, target_pose, offset=offset, offset_dir=offset_dir) # for now offset is zero because I force-backwards at end of motion primitive
        bbox = self.get_bounding_box()
        bbox[1] += 0.05 # increase y a little so that we don't overshoot (unlikely, but still)
        pts = pts.clip(bbox[0:3], bbox[0:3]+bbox[3:6])
        tf.pos = pts[1] #get mid point
        angs1 = getAngleLimit(init_pose.orient * m3d.Vector(0, 0, -1))
        angs2 = getAngleLimit(target_pose.orient * m3d.Vector(0, 0, -1))

        q1 = init_pose.orient.quaternion.array; q2 = target_pose.orient.quaternion.array
        avg_quat = slerp(q1, q2, 0.5)

        tf.orient = m3d.Orientation(m3d.UnitQuaternion(*avg_quat))
        pose_list = [tf, target_pose]
        return pose_list

    def calc_attack_pose(self, optimalPt, normal, initial_angle, direction_to_chisel, qn, check_in_bbox=True):
        if check_in_bbox:
            bbox = self.getExpandedBoundingBox(0.0) #same as chiselID
            if not checkBox(optimalPt, bbox):
                print("checkAttack: not in bbox.")
                return None

        # TODO: do this right        
        print("initial angle = {}".format(initial_angle)) 
        # calculate attack pose
        attackPose = m3d.Transform()
        attackPose.pos = optimalPt
        attackPose.orient = getHorizontalOrient()
        normalAngles = getAngleLimit(normal) #is w.r.t. horizontal +y-axis
        attackPose.orient.rotate_xb(normalAngles[0])
        attackPose.orient.rotate_zb(normalAngles[2])

        # delta_angles = self.checkAngleLimits(attackPose, direction_to_chisel * nominal_angle)
        attackPose.orient.rotate_yt( direction_to_chisel * initial_angle)
        # attackPose.orient.rotate_xt( delta_angles[0] )
        outPose = self.limit_tool_eulers(attackPose)
        pdb.set_trace()
        return outPose

    def check_mode_arr(self, pos_arr):
        return getChiselModeArr(pos_arr, self.pu, self.pl, self.get_bounding_box())

    def check_attack(self, attack_pos, normal, initial_angle, centroid, qn):
        """ given the attack_pos, pose-normal, and initial-angle, and centroid, find the attack-pose and direction-to-chisel
        """
        centroid   = np.array(centroid)
        attack_pos = np.array(attack_pos)
        ptToCentroid = centroid - attack_pos

        attackPose = m3d.Transform()
        attackPose.pos = attack_pos
        attackPose.orient = self.get_nominal_orient()

        # direction of chisel
        ptToCentroid_tool = attackPose.orient.inverse * ptToCentroid
        direction_to_chisel = 1.0 if ptToCentroid_tool[0] > 0.0 else -1.0 #single line if-else statement

        # check if direction_to_chisel is valid in mode of attack position
        direction_mode, _, _ = self.check_mode_arr(attack_pos)
        #
        if (attackPose.orient * m3d.Vector.e1).z > 0.0:
            direction_to_chisel_base = -1.0 * direction_to_chisel
        else:
            direction_to_chisel_base = direction_to_chisel
        #
        if direction_mode == 0: valid = True
        elif direction_mode == 1 and direction_to_chisel_base > 0.0: valid = True
        elif direction_mode == -1 and direction_to_chisel_base < 0.0: valid = True
        else: valid = False

        if valid == False: return valid, None, None

        attackPose = self.calc_attack_pose(attack_pos, normal, initial_angle, direction_to_chisel, qn)
        return valid, attackPose, direction_to_chisel

def publishFloat(pub, list1):
    """Publish float array onto ROS publisher
    """
    msgState = Float32MultiArray()
    msgState.data[:] = list1[:]
    pub.publish(msgState)

def getFloatArrMsg(list1):
    """Convert list-like to ROS Float32MultiArray
    """
    msg = Float32MultiArray()
    msg.data[:] = list1[:]
    return msg

def publishBool(pub, bool1):
    """Publish bool onto ROS publisher
    """
    b = Bool()
    b.data = bool1
    pub.publish(b)



class MappingFlag(object):
    """Interface to trigger offworld-ROS map fxns.
    """
    def __init__(self):
        """
        """
        self.pub = rospy.Publisher('wall/reconstructed/flagGo', Bool, queue_size=10)        
        self.pubReset = rospy.Publisher('wall/reconstructed/resetMap', Bool, queue_size=10)        
    def publish(self):
        """
        """
        b = Bool()
        b.data = True
        self.pub.publish(b)
    def reset(self):
        """
        """
        b = Bool()
        b.data = True
        self.pubReset.publish(b)

class PeakFinder(object):
    """Interface to trigger offworld-ROS peak finding CV.
    """
    def __init__(self, ns=""):
        """
        """
        rospy.wait_for_service(ns+'/peakFinder')
        self.srv = rospy.ServiceProxy(ns+'/peakFinder', bot_perception_ros.srv.peakFinder)

    def reset(self):
        """
        """
        self.peak = np.NaN * np.ones([1,3])

    def publish(self, arr):
        """
        """
        self.reset()
        resp = self.srv(getFloatArrMsg(arr))
        if resp.SUCCESS:
            arr = resp.output.data[:]
            self.peak = arr[0:3]

class CentroidFinder(object):
    """Interface to trigger offworld-ROS centroid finding CV.
    """
    def __init__(self, ns=""):
        """
        """
        rospy.wait_for_service(ns+'/centroidFinder')
        self.srv = rospy.ServiceProxy(ns+'/centroidFinder', bot_perception_ros.srv.centroidFinder)

    def reset(self):
        """
        """
        self.intersectPt = np.NaN * np.ones([1,3])
        self.centroid = np.NaN * np.ones([1,3])

    def publish(self, arr):
        """
        """
        self.reset()
        resp = self.srv(getFloatArrMsg(arr))
        if resp.SUCCESS:
            arr = resp.output.data[:]
            self.intersectPt = arr[0:3]
            self.centroid = arr[3:6]


class DymoUSBScale(object):
    def __init__(self):

        # Note: can be powered only through usb.
        # Note: to deactivate auto-shutoff, hold down the (kg <-> lb) button while pressing 'on'

        # Dymo brand S250 250 lb Portable Digital Shipping Scale
        VENDOR_ID = 0x0403 #0x0922
        PRODUCT_ID = 0x6001 #0x8009
        # find the USB devices
        gen = usb.core.find(find_all=True,
                                    idVendor=VENDOR_ID,
                                    idProduct=PRODUCT_ID)
        self.devices = [x for x in gen]
        self.endpoints = []
        for device in self.devices:
            # detach if needed
            try:
                device.detach_kernel_driver(0)
            except Exception, e:
                pass # already unregistered

            # use the first/default configuration
            device.set_configuration()
            # first endpoint
            self.endpoints.append( device[0][(0,0)][0] )

    def close(self):
        [ usb.util.dispose_resources(device) for device in self.devices]

    def getPacket(self, device, endpoint):
        # read a data packet
        attempts = 3
        data = None
        while data is None and attempts > 0:
            try:
                data = device.read(endpoint.bEndpointAddress,
                                endpoint.wMaxPacketSize)
            except usb.core.USBError as e:
                data = None
                if e.args == ('Operation timed out',):
                    attempts -= 1
                    continue
        return data

    def getWeight(self, device, endpoint):
        # for the Dymo brand S250 250 lb Portable Digital Shipping Scale
        # third data point in array data(2) gives metric or pounds
        # forth data point in array data(3) gives scaling factor: either 0.1 or 0.01
        DATA_MODE_GRAMS = 3
        DATA_MODE_POUNDS = 12

        data = self.getPacket(device, endpoint)

        if data[3] == 255:
            scaling_factor = 0.1
        elif data[3] == 254:
            scaling_factor = 0.01

        raw_weight = data[4] + data[5] * 256

        pounds = None
        kilograms = None
        if data[2] == DATA_MODE_POUNDS:
            pounds = raw_weight * scaling_factor
        elif data[2] == DATA_MODE_GRAMS:
            kilograms = raw_weight * scaling_factor

        if kilograms is None:
            print("DymoUSBScale: Warning: didn't read kilograms. Check that scales are on and set to 'kg'.")
        return kilograms or 0.0 # returns 0.0 if mass is None

    def getTotalWeight(self):
        # call getWeight for each scale and sum up the totals
        total = 0.0
        count = 0
        for device in self.devices:
            endpoint = self.endpoints[count]
            count += 1
            total += self.getWeight(device, endpoint)
        return total

class WeightReward(object):
    """ROS wrapper to publish USB weight as a reward stream.
    """
    def __init__(self):
        """
        """
        self.scale = DymoUSBScale()
        self.pubReward = rospy.Publisher("/scale/reward", Float32, queue_size=10)
        self.reset()

    def reset(self):
        """
        """
        self.old = self.scale.getTotalWeight()

    def getReward(self):
        """
        """
        current = self.scale.getTotalWeight()
        print("Total Weight: {}".format(current))
        reward = np.clip( (current - self.old) , 0.0, None) # clip to positive
        self.old = current
        
        msg = Float32(data = reward)
        self.pubReward.publish(msg)
    
    def close(self):
        """
        """
        self.scale.close()


class TorreyUSBScale(object):
    
    def __init__(self, serial_ports):

        self.scales = []
        try:
            for port in serial_ports:
                scale = serial.Serial(
                    port=port,
                    baudrate=9600,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS
                )
                self.scales.append(scale)
        except:
            pass
    
    def _getScaleReading(self, scale):
        scale.write('P')
        weight = ""
        while True:
            c = scale.read()
            weight += c
            if ord(c) == 13:
                break
        try:
            weight = float(weight[0:5])
        except:
            weight = 0.0
        return weight
    
    def getTotalWeight(self):
        total = 0.0
        for scale in self.scales:
            total += self._getScaleReading(scale)
        return total
    
    def close(self):
        for scale in self.scales:
            scale.close()


class RayIntersect(object):
    """Interface to trigger offworld-ROS ray intersection. Get the closest point in the reconstructed point-cloud to a ray defined by a point and a direction.
    """
    def __init__(self, ns=""):
        """
        """
        rospy.wait_for_service(ns+'/rayIntersect')
        self.srv = rospy.ServiceProxy(ns+'/rayIntersect', bot_perception_ros.srv.rayIntersect)

    def reset(self):
        """
        """
        self.intersectPt = np.NaN * np.ones([1,3])

    def publish(self, arr):
        """
        """
        self.reset()
        resp = self.srv(getFloatArrMsg(arr))
        if resp.SUCCESS:
            arr = resp.output.data[:]
            self.intersectPt = arr[0:3]

class RayIntersect_ARR(RayIntersect):
    def __init__(self, ns=""):
        rospy.wait_for_service(ns+'/rayIntersect_arr')
        self.srv = rospy.ServiceProxy(ns+'/rayIntersect_arr', bot_perception_ros.srv.rayIntersect)

    def reset(self):
        self.intersectPt = np.NaN * np.ones([1,3])

    def publish(self, arr):
        self.reset()
        resp = self.srv(getFloatArrMsg(arr))
        if resp.SUCCESS:
            arr = resp.output.data[:]
            self.intersectPt = []
            for idx in range(0, len(arr), 3):
                self.intersectPt.append = arr[idx: idx+3]
            self.intersectPt = np.array(self.intersectPt)

class MotionPlanner(object):
    """ Class that implements a client for the Motion Planning service.
        Also executes the plan on the robot.
        Set class member 'use_ur_driver' to true for executing pre-interpolated
        Trajectories on the robot. 
    """
    def __init__(self, group_name, frame_id="base", joint_names=[], nominal_tcp_frame=""):
        """ Valid group-names: 'chisel', 'saw'
        """
        self.group_name = group_name
        self.frame_id_ = frame_id

        self.tf = TransformListener()

        if len(nominal_tcp_frame):
            self.use_tool0_ik = True
            while not rospy.is_shutdown():
                try:
                    trans, rot = self.tf.lookupTransform(group_name+"_tip_link", nominal_tcp_frame, rospy.Time(0))
                    self.tf_tcp_to_tool0 = ROStoM3d(trans, rot)
                    self.tf_tool0_to_tcp = self.tf_tcp_to_tool0.inverse
                    break
                except:
                    rospy.logerr("Could not find transform from {} to {}".format(group_name+"_tip_link", nominal_tcp_frame))
                    pass
                rospy.sleep(0.01)
        else:
            self.use_tool0_ik = False
        
        # Diagnostics publishers
        self.pub_planning_data = rospy.Publisher("/planning_performance",
                                                 Float32MultiArray, 
                                                 queue_size=1, latch=True
                                                 )
        self.pub_goal = rospy.Publisher('/motion_planning/goal',
                                        geometry_msgs.msg.PoseStamped,
                                        queue_size=1)

        # Service clients for motion planning for saw and chisel
        self.generate_path_chisel = rospy.ServiceProxy(
                                    '/generate_path_chisel', 
                                    bot_arm_planner.srv.GeneratePath
                                    )
        self.generate_path_saw = rospy.ServiceProxy(
                                    '/generate_path_saw',  
                                    bot_arm_planner.srv.GeneratePath
                                    )
        self.mps = {"chisel": self.generate_path_chisel,
                    "saw": self.generate_path_saw}
        
        # Service clients for collision checking for saw and chisel
        self.check_collision_saw = rospy.ServiceProxy(
                                    '/check_collision_saw',
                                    CheckCollision
                                    )
        self.check_collision_chisel = rospy.ServiceProxy(
                                        '/check_collision_chisel', 
                                        CheckCollision
                                        )
        self.check_collision_list_saw = rospy.ServiceProxy(
                                        '/check_collision_list_saw', 
                                        CheckCollisionList
                                        )
        self.check_collision_list_chisel = rospy.ServiceProxy(
                                            '/check_collision_list_chisel',
                                            CheckCollisionList
                                            )
        self.services = {"chisel": self.check_collision_chisel, 
                         "saw": self.check_collision_saw,
                         } 
        self.services_ccl = {"chisel": self.check_collision_list_chisel,
                             "saw": self.check_collision_list_saw
                             }
        
        # Service client for geting IK
        self.get_ik = rospy.ServiceProxy('/' + '/get_ik_' +
                                         group_name, GetIK)

        
        ### self.logfile = open(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")+".txt","w")
        self.avg_planning_time = 0.0
        self.count = 0.0
        self.success_rate = 0.0
        self.success_count = 0.0
        self.total_time = 0.0
        self.goal_wall_offset = 0.02
        self.joint_prefix = self.group_name

        if joint_names:
            self.joint_names = joint_names
        else:
            self.joint_names = rospy.get_param("/pathserver_" + group_name +
                                           "/" + group_name + "_joint_names")

    def get_robot_state_msg(self, joints):
        """ Function to generate a RobotState msg to be input to the planning service
            This message contains the joints position representing the plan's start state.
            Input:
            joints: A list of float values (length of number of joints)
        """
        rstate = moveit_msgs.msg.RobotState()
        # Set a valid robot state
        rstate.joint_state.name = self.joint_names
        rstate.joint_state.position = joints
        return rstate
       
    def log_metrics(self):
        plan_time = time.time() - self.plan_start_time
        print "PLAN TIME = " + str(plan_time) + " secs"
        # Update data
        self.success_count += 1
        self.total_time += plan_time
        self.avg_planning_time = self.total_time/self.success_count
        self.success_rate = self.success_count/self.count
        # Publish to a topic
        plan_data = Float32MultiArray()
        plan_data.data = [self.count, self.avg_planning_time, self.success_rate]
        self.pub_planning_data.publish(plan_data)
    
    def format_pose(self, input):
        """ Method to parse the format of the input pose (pose of <group>_tip_link in
            base frame and returns a pose in geometry_msgs/Pose format.
            if propery use_tool0_ik is enabled then the input pose is tranformed to represent
            pose of the <group>_tool0 link.
         
            input - pose of type geometry_msgs/Pose or math3d.Transform
            return - pose of type geometry_msgs/Pose
        """
        # Parse format
        if isinstance(input, m3d.Transform):
            if self.use_tool0_ik:
                pose = m3d_to_ros(input * self.tf_tcp_to_tool0)
            else:
                pose = m3d_to_ros(input)

        elif isinstance(input,  geometry_msgs.msg.Pose):
            if self.use_tool0_ik:
                tf_base_tcp = geometry_msg_to_m3d(input)
                pose = m3d_to_ros(tf_base_tcp * self.tf_tcp_to_tool0)
            else:
                pose = input
        
        return pose


    def get_plan(self, joints, tf_goal=None, joints_goal=None, max_tolerances=[]):
        """ Method to call the Planning service.
            Input arguments:
            attack_pose - The goal pose in geometry_msgs/pose.
            joints - The start state in joints space.
        """
              
        try:
            self.count = self.count + 1
            plan_start_time = time.time()

            # Start-state in joints space
            rstate = self.get_robot_state_msg(joints)
            
            # goal-state in joints space
            joints_goal_msg = Float32MultiArray()

            # Goal in cartesian space
            goal_pose = geometry_msgs.msg.PoseStamped()
            goal_pose.header.frame_id = self.frame_id_

            pos_tol_msg = Float32MultiArray()
            angle_tol_msg = Float32MultiArray()

            # Planning result
            path = trajectory_msgs.msg.JointTrajectory()
            
            # Planning to a cartesian pose goal if joints goal is empty
            if joints_goal is None:


                goal_pose.pose = self.format_pose(tf_goal)

                # Visualize goal
                goal_pose.header.stamp = rospy.Time.now()
                self.pub_goal.publish(goal_pose)     

                # IK input needs to be in the world frame. 
                pose_world = PoseStamped()
                pose_world.header.stamp = rospy.Time.now()
                pose_world = self.tf.transformPose("base_footprint", goal_pose)
                group = String()
                group.data = self.group_name
                
                
             
                if  True or len(max_tolerances) == 0:
                    # Determine IK for goal pose
                    ik = self.get_ik(pose_world.pose, group, rstate)
                    if not ik.found_ik:
                        joints_goal_msg.data = []
                    else:
                        joints_goal_msg = ik.joints

                    # Calling path server with joints goal 
                    planner_resp = self.mps[self.group_name](rstate, goal_pose, joints_goal_msg,
                                                             pos_tol_msg, angle_tol_msg
                                                             )

                else:
                    # Call planner with empty joints goal, hence enforcing pose goal.
                    joints_goal_msg.data = []
                    pos_tol_msg.data = max_tolerances[0:3]           
                    angle_tol_msg.data = max_tolerances[3:]
                    # Iterative call planner until max tolerances are reached
                    
                    planner_resp = self.mps[self.group_name](rstate, goal_pose, joints_goal_msg,
                                                             pos_tol_msg, angle_tol_msg
                                                             )
                
            # Planning to a joints goal
            else:
                joints_goal_msg.data = joints_goal
                planner_resp = self.mps[self.group_name](rstate, goal_pose, joints_goal_msg,
                                                         pos_tol_msg, angle_tol_msg
                                                         )
            
            rospy.loginfo("Calling Planning Service...")
            self.count = self.count + 1

            # If planning is successfull log metrics and return traj
            if planner_resp.SUCCESS:

                plan_time = time.time() - plan_start_time
                rospy.loginfo("PLAN TIME = : {} secs".format(plan_time))
               
                # Update planning metrics
                self.success_count += 1
                self.total_time += plan_time
                self.avg_planning_time = self.total_time/self.success_count
                self.success_rate = self.success_count/self.count

                # Publish metrics
                plan_data = Float32MultiArray()
                plan_data.data = [self.count, self.avg_planning_time, self.success_rate]
                self.pub_planning_data.publish(plan_data)

                return (planner_resp.magic_points, planner_resp.poses)

            else:
                self.pub_goal.publish(goal_pose)
                path.points = []
                return (path, path)

        except rospy.ServiceException, e:
            rospy.logerr("Planning Service call failed: {}".format(e))

    def plan(self, start_js, goal, max_tolerances=[]):
        """ start_js - The start state in joints space.
            If goal is an m3d transform, tolerance is a list of
            magnitude values specifing min=-mag and max=mag values for
            attributes in the order: x, y, z, r, p, y in frame self.frame_id_

            If goal is a joint state (a list of float) tolerance is a list
            a list of magnitude values of the same length as goal indicating.
            -mag and mag value for each joint.
        """ 
        # VEL deg/s
        # ACC deg/s^2
        goalPose = None
        temp_joints = start_js
        temp_joints = (temp_joints)

        # Goal
        if isinstance(goal, list):
            result = self.get_plan(temp_joints, tf_goal=None, joints_goal=goal, max_tolerances=max_tolerances)
        else:
            result = self.get_plan(temp_joints, tf_goal=goal, joints_goal=None, max_tolerances=max_tolerances)
        if result is None:
            final_pts = []
            return final_pts, []

        (way_points, trans) = result

        # Post-process the trajectory to downsample way-points
        if len(way_points.points) == 0:
            final_pts = []
            pose_list = []
        else:
            # self.logfile.write(str(plan_time) + ",")
            li = [list(x.positions) for x in way_points.points]               
            li_sub = [l[2:5] for l in li]
            li_comp = rdp(li_sub, epsilon=0.002)
            indices = []               
            for j in range(len(li_comp)):
                indices = indices + [i for i in range(len(li_sub)) if li_sub[i] == li_comp[j]]
            print "Indices: "
            print indices

            final_pts = [li[p] for p in indices]
            final_pts = li
            pose_list = [trans.poses[p] for p in indices]

            if self.use_tool0_ik:
                pose_list = [m3d_to_ros(geometry_msg_to_m3d(pose) * self.tf_tool0_to_tcp) for pose in pose_list]

        return final_pts, pose_list


    def get_ik_srv(self, group, ik_seed_js, start_pose):
        
        first_pose = self.format_pose(start_pose)

        # Transform the poses to /world frame
        start_pose_world = PoseStamped()
        start_pose_world.header.stamp = rospy.Time.now()
        start_pose_world.header.frame_id = self.frame_id_
        start_pose_world.pose = first_pose
        start_pose_world = self.tf.transformPose("base_footprint", start_pose_world)

        gp = String()
        gp.data = group

        ik_seed_state = self.get_robot_state_msg(ik_seed_js)

        ik = self.get_ik(start_pose_world.pose, gp, ik_seed_state)
        success = ik.found_ik
        out = np.array(ik.joints.data)

        return success, out

    def check_collision(self, group, ik_seed_js, start_pose, goal_pose, excluded_links=[]):
        """ Checks collision of the given group defined in diggerbot.srdf
            Input:
            groups can be "saw", "chisel", "saw_guard"
            excluded_links: This arg takes a list of links that are to be
            excluded from collision checking against the octomap.

            Output:
            in_collision: a list of boolean in_collision flags
            poses: list of ROS pose-stamped messages of each pose tested
        """
        # TODO: transform to /base_footprint frame
        last_pose = geometry_msgs.msg.Pose()
        last_pose = self.format_pose(goal_pose)
        
        first_pose = self.format_pose(start_pose)
        # TODO: Check validity of excluded links.

        # Transform the poses to /world frame
        start_pose_world = PoseStamped()
        start_pose_world.header.stamp = rospy.Time.now()
        start_pose_world.header.frame_id = self.frame_id_
        start_pose_world.pose = first_pose
        start_pose_world = self.tf.transformPose("base_footprint", start_pose_world)

        # Transform the poses to /world frame
        last_pose_world = PoseStamped()
        last_pose_world.header.stamp = rospy.Time.now()
        last_pose_world.header.frame_id = self.frame_id_
        last_pose_world.pose = last_pose
        last_pose_world = self.tf.transformPose("base_footprint", last_pose_world)

        # The following function overrides the planning server's default planning group
        gp = String()
        gp.data = group

        # Obtain joint state of the start pose from Ik
        # With given seed state.
        ik_seed = self.get_robot_state_msg(ik_seed_js)
        ik = self.get_ik(start_pose_world.pose, gp, ik_seed)
        start_state = self.get_robot_state_msg(ik.joints.data)
        
        # Perform collision checking and obtain result
        resp = self.services[self.group_name](last_pose_world.pose, first_pose,
                                              gp, start_state, excluded_links)
        in_collision = resp.result
        poses = [self.tf.transformPose(self.frame_id_, PoseStamped(resp.pose_array.header, pose)) for pose in resp.pose_array.poses]

        if self.use_tool0_ik:
            poses = [m3d_to_ros(geometry_msg_to_m3d(pose.pose) * self.tf_tool0_to_tcp) for pose in poses]
            poses = [PoseStamped(resp.pose_array.header, pose) for pose in poses]

        return in_collision, poses

    def check_collision_list_slot(self, group, input_poses, excluded_links, seed_js, dist_from_seed=[], attached_markers=None, preempt=False):
        """ converts output of check collision list to poses"""
        in_collision, poses_ros, joints, colliding_links = self.check_collision_list(group, seed_js, input_poses, excluded_links,\
             attached_markers, preempt, dist_from_seed)
        
        out = [ROStoM3d(ps.pose.position, ps.pose.orientation) for ps in poses_ros]
        return in_collision, colliding_links, out, joints

    def check_collision_list(self, group, ik_seed_js, input_poses, excluded_links=[], attached_markers=None, preempt=False, dist_from_seed=[]):
        """ Checks collision of the given group defined in diggerbot.srdf
            groups can be "saw", "chisel", "saw_guard"
            excluded_links: This arg takes a list of links that are to be 
            excluded from collision checking against the octomap.
            input_poses: List of poses to be checked for collision.

        returns 
            in_collision: a list of boolean in_collision flags
            poses: list of ROS pose-stamped messages of each pose tested
        """
        input_poses_world = []
        dist_from_seed_msg = Float32MultiArray()
        dist_from_seed_msg.data = dist_from_seed

        for pose in input_poses:
            last_pose = geometry_msgs.msg.Pose()

            last_pose = self.format_pose(pose)
            # TODO: Check validity of excluded links.

            # Transform the poses to /world frame
            last_pose_world = PoseStamped()
            last_pose_world.header.stamp = rospy.Time.now()
            last_pose_world.header.frame_id = self.frame_id_
            last_pose_world.pose = last_pose
            last_pose_world = self.tf.transformPose("base_footprint", last_pose_world)
            input_poses_world.append(last_pose_world.pose)

        # The following function overrides the planning server's default planning group
        gp = String()
        gp.data = group
        if ik_seed_js is None:
            rstate = self.get_robot_state_msg(rospy.wait_for_message("/saw/joint_states", JointState, 60.0))
        else:
            rstate = self.get_robot_state_msg(ik_seed_js)
        # Attach collision objects
        # convert visualization_msgs.marker.cube to moveit_msgs/AttachedCollisionObject
        if attached_markers:
            attached_objects = self.viz_markers_to_aco(attached_markers)
            rstate.attached_collision_objects.append(attached_objects)
            
        ip = geometry_msgs.msg.PoseArray()
        ip.poses = input_poses_world
        # pdb.set_trace()
        try:
            resp = self.services_ccl[self.group_name](ip, gp, rstate, excluded_links, preempt, dist_from_seed_msg)
        except:
            rospy.wait_for_service("/check_collision_"+self.group_name)
            resp = self.services_ccl[self.group_name](ip, gp, rstate, excluded_links, preempt, dist_from_seed_msg)

        in_collision = resp.result
        poses = [self.tf.transformPose(self.frame_id_, PoseStamped(resp.pose_array.header, pose)) for pose in resp.pose_array.poses]

        if self.use_tool0_ik:
            poses = [m3d_to_ros(geometry_msg_to_m3d(pose.pose) * self.tf_tool0_to_tcp) for pose in poses]
            poses = [PoseStamped(resp.pose_array.header, pose) for pose in poses]

        joints = [jtp.positions for jtp in resp.joint_trajectory.points]
        
        return in_collision, poses, joints, resp.colliding_links

    def viz_markers_to_aco(self, ms):
        """ input: ms - List of markers to be fed inside a Attached collision object variable.
        """
        aco = moveit_msgs.msg.AttachedCollisionObject()
        aco.object.header = ms[0].header
        aco.object.operation = 0
        aco.object.id = "0"
        aco.link_name = "saw_base_link"
        aco.object.primitive_poses = [m.pose for m in ms]
        for m in ms:
            curr_primitive = SolidPrimitive(type = 1)
            curr_primitive.dimensions = [m.scale.x, m.scale.y, m.scale.z]
            aco.object.primitives.append(curr_primitive)
        return aco
        
    def get_ik_joints(self, ik_seed_js, start_pose):
        """ Gets the joint values based on Inverse Kinematics.
            Input: m3d pose or geometry_msgs.msg.Pose of the end-effector wrt base frame of the arm
            Output: array of joint values
        """
        if isinstance(start_pose, m3d.Transform):
            first_pose = m3d_to_ros(start_pose)
        else:
            first_pose = start_pose
        assert( isinstance(first_pose, geometry_msgs.msg.Pose) )

        # Transform the poses to /world frame
        start_pose_world = PoseStamped()
        start_pose_world.header.stamp = rospy.Time.now()
        start_pose_world.header.frame_id = self.frame_id_
        start_pose_world.pose = first_pose
        start_pose_world = self.tf.transformPose("base_footprint", start_pose_world)

        # The following function overrides the planning server's default planning group
        gp = String()
        gp.data = self.group_name
        ik_seed_state = self.get_robot_state_msg(ik_seed_js)

        ik = self.get_ik(start_pose_world.pose, gp, ik_seed_state)
        return ik.joints.data
    
    

    

def calibrateOrientation(pt1, pt2, rob=None):
    # calibrate by pointing 
    pt1 = np.array(pt1)
    pt2 = np.array(pt2)
    delta = pt2 - pt1
    angles = np.arctan(delta[1] / delta)
    angles = m3d.Vector(angles)

    mustClose = False
    if rob is None:
        rob = init_robot()
        mustClose = True
    pose = arm_client.get_pose()
    if mustClose: rob.close()

    angles[1] = 0 # ignore y-base
    angles_tool = rotate3V(rob, angles, to_TCP=True)
    xd, yd, zd = angles_tool
    print("Delta-x-tool: {}, Delta-y-tool {}".format(xd, yd))
    pdb.set_trace()
    return

def calibrateTranslation(pt1, pt2, rob=None):
    # calibrate by pointing 
    pt1 = np.array(pt1)
    pt2 = np.array(pt2)
    delta = pt2 - pt1

    mustClose = False
    if rob is None:
        rob = init_robot()
        mustClose = True
    pose = arm_client.get_pose()
    if mustClose: rob.close()
    
    delta_tool = rotate3V(rob, delta, to_TCP=True)
    print("Delta-tool {}".format(delta_tool))
    pdb.set_trace()
    return

class VolumeEstimate(object):
    """Interface to trigger offworld-ROS volumeEstimation CV. Get the 
    """
    def __init__(self, inputTopic, ns=""):
        self.volume = 0
        self.lock = False

        self.inputTopic = ns+inputTopic
        self.pub = rospy.Publisher("wall/volumeEstimation/double/input", PointCloud2, queue_size=10)
        self.pubBBox = rospy.Publisher("wall/volumeEstimation/boundingBox", Float32MultiArray, queue_size=5)
        self.sub = rospy.Subscriber("wall/volumeEstimation/output", Float32, self.cb)

    def cb(self, value):
        self.volume = value.data
        if self.volume < 0.0: # flag returned
            print "\nWARNING: VolumeEst. Something went wrong.\n   Review VolumeEst. output.\n"
        else: 
            print "\nINFO: VolumeEst. The volume of the object is : "+ str(self.volume)
        self.lock = False
    
    def pushBoundingBox(self, attackPos):
        width = 0.2 
        corner = np.array(attackPos) - width
        size   = 2*width*np.ones(3)
        bbox = np.concatenate([corner, size])
        publishFloat(self.pubBBox, bbox)
        time.sleep(0.1)

    def reset(self):
        self.pc_msgs = []

    def capture(self, pcMsg=None): # get message directly
        if pcMsg is None:
            pcMsg = rospy.wait_for_message(self.inputTopic, PointCloud2)
        self.pc_msgs.append(pcMsg)

    def publish(self):
        self.volume = 0
        self.lock = True
        self.pub.publish(self.pc_msgs[0])
        self.pub.publish(self.pc_msgs[1])
        while self.lock:
            time.sleep(0.05)
    

class QueryNormal(object):
    def __init__(self, ns=""):
        self.reset()
        self.sqDist = 0
        rospy.wait_for_service(ns+'/queryNormal')
        self.srv = rospy.ServiceProxy(ns+'/queryNormal', bot_perception_ros.srv.queryNormal)

    def reset(self):
        self.normal = np.nan * np.ones(3)
        self.contact = np.nan * np.ones(3)
        
    def publish(self, array):
        # if no data, hold last
        resp = self.srv(getFloatArrMsg(array))
        if resp.SUCCESS:
            arr = np.array(resp.output.data[:])
            self.normal = arr[0:3]
            self.contact = arr[3:6]
            self.sqDist = arr[6]

class checkForce(object):
    def __init__(self):
        # limits
        maxForce = 225.0 # Newtons
        maxTorque = 0.5 #is actually the rotation speed
        self.maxControl = np.array([maxForce]*3+[maxTorque]*3, dtype='float')
        deadForce = 5.0
        deadTorque = (2.5/180. * math.pi) * Kp_angle # is actually rotation speed, multiply by Kp_angle because the deadband operates on Kp_angle * delta_angle
        self.deadband = np.array([deadForce]*3+[deadTorque]*3, dtype='float')
        limitSpeed = 0.15 # in m/s
        limitAngSp = 0.1 #this number always gets overwritten. Doesn't matter.
        self.limitsDefault = np.array([limitSpeed]*3+[limitAngSp]*3, dtype='float')

def getAngleLimit(posVec):
    # finds the yaw and pitch of posVec w.r.t. horizontal=+y-axis
    # finds the roll w.r.t. horizontal=+x-axis
    X = m3d.Vector(0, posVec[1], posVec[2]) # projected onto Y-Z plane - pitch
    Y = m3d.Vector(posVec[0], 0, posVec[2]) # projected onto X-Z plane - roll
    Z = m3d.Vector(posVec[0], posVec[1], 0) # projected onto X-Y plane - yaw
    angleX = X.signed_angle(m3d.Vector(0, -1, 0), m3d.Vector(-1, 0, 0)) #angle with horizontal rock-normal
    angleY = Y.signed_angle(m3d.Vector(-1, 0, 0), m3d.Vector(-1, 0, 0)) #angle with horizontal rock-normal
    angleZ = Z.signed_angle(m3d.Vector(0, -1, 0), m3d.Vector(0, 0, -1)) #angle with horizontal rock-normal
    angles = np.array([angleX, 0, angleZ])
    return angles

def checkAngleLimits(pose, maxAngle):
    # check limits - make sure angle isn't too extreme
    # maxAngle is a delta-angle in tool-frame

    minAngle_base, maxAngle_base = getMaxAngles(pose)
    ornt = pose.orient.copy()
    anglesOriginal = getAngleLimit(ornt * m3d.Vector(0, 0, -1))
    ornt.rotate_yt( maxAngle )
    angles = getAngleLimit(ornt * m3d.Vector(0, 0, -1))
    angles = angles.clip(minAngle_base, maxAngle_base)
    #convert back to tool
    anglesOriginal_tool = pose.orient.inverse * anglesOriginal
    angles_tool = pose.orient.inverse * angles
    deltaAngle = angles_tool - anglesOriginal_tool

    #pdb.set_trace() #check if it changes a copy, or the original
    return deltaAngle[1] # tool-y direction

class DynamicNormal(object):
    def __init__(self, arm_client, useML=False, ns=""):
        # ---- PARAMS ----
        self.arm_client = arm_client
        self.Debug = True
        self.nominal_angle = 40. /180.*math.pi
        self.nominal_depth = 0.02 # 2 cm depth
        self.timeOut = 3.5
        self.distanceOut = 0.15
        self.timeToComplete = 4.0 # assume we want constant arc-speed
        self.error_target_limit = 1.0 # distance in meters
        self.useML = useML

        # setup
        self.normal = np.zeros(3)
        self.contact = np.zeros(3)
        self.sqDist = 0.0

        self.queryNormal = QueryNormal(ns=ns)
        self.subSetup = rospy.Subscriber("wall/dynamicNormal/output", PointCloud2, self.cb)
        self.pubSetup = rospy.Publisher("wall/dynamicNormal/input", Float32MultiArray, queue_size=1)
        self.pubDeltaAngle = rospy.Publisher("wall/dynamicNormal/deltaAngle", Float32MultiArray, queue_size=1)
        self.pubActive = rospy.Publisher("wall/dynamicNormal/active", Bool, queue_size=1)
        if self.useML:
            self.angle_waiter = WaitForTopic("/chiseler/action", Float32MultiArray)
            self.ml_client_waiter = WaitForActive("/chiseler")
            self.q_value_sub = rospy.Subscriber("/chiseler/maxQ", Float32, self.save_max_q)
            self.max_q_value = 0.0
            _, angle_offset, _ = getOffsets()
            if False and np.isclose(angle_offset, -270.0, 1e-3):
                self.b_ML_toggle = True # this means the camera is oriented downwards, meaning that camera left & right have switched directions w.r.t. the TCP frame
            else:
                self.b_ML_toggle = False

    def init(self):
        # gains
        K_line   = 1.5 * 0.5 * self.nominal_angle / (0.02 + 0.02) # guarantees 0deg AOA when resting on original surface --- was 1.5 * 0.5 * nominal_angle / (0.02+0.02)
        K_target = 10 / 0.01 * math.pi / 180. # 10 degrees per 1 cm ------- not used
        K_force  = 25. / 30  * math.pi / 180. # 25.0 degrees per 30 N*m torque
        Kp_angle = 0.7 / (45./180.*math.pi) # 0.5 rad/s per 45 degrees offset
        # PID objects
        PID_line   = UR_FORCE_PID(numParams = 1) # dist to closest pt on line
        PID_target = UR_FORCE_PID(numParams = 1) # dist to virtual target pt
        PID_force  = UR_FORCE_PID(numParams = 1) # protection against over-torqueing, operate on tool-y torque only
        PID_angle  = UR_FORCE_PID() # lower level angle --> force_mode input
        #
        PID_line.setKp([K_line])
        PID_line.maxControl = np.array([40./180.*math.pi])
        #
        PID_target.setKp([K_target])
        PID_target.maxControl = np.array([40./180.*math.pi])
        #
        PID_force.setKp([K_force])
        PID_force.maxControl = np.array([40./180.*math.pi])
        # angle control limits
        maxForce = 225.0 # Newtons
        maxTorque = 0.55 #is actually the rotation speed
        maxControl = np.array([maxForce]*3+[maxTorque]*3, dtype='float')
        deadForce = 5.0
        deadTorque = (2.5/180. * math.pi) * Kp_angle # is actually rotation speed, multiply by Kp_angle because the deadband operates on Kp_angle * delta_angle
        deadband = np.array([deadForce]*3+[deadTorque]*3, dtype='float')
        limitSpeed = 0.15 # in m/s
        limitAngSp = 0.1 #this number always gets overwritten. Doesn't matter.
        limitsDefault = np.array([limitSpeed]*3+[limitAngSp]*3, dtype='float')
        PID_angle.setKp( np.array([0,0,225.0]+[0.0, Kp_angle, 0.0]) )
        PID_angle.maxControl    = maxControl
        PID_angle.deadband      = deadband
        PID_angle.limitsDefault = limitsDefault
        self.PID_line = PID_line
        self.PID_target = PID_target
        self.PID_force = PID_force
        self.PID_angle = PID_angle

    def cb(self, cloud):
        generator = pc2.read_points(cloud, field_names = ("x", "y", "z", "normal_x", "normal_y", "normal_z"), skip_nans=True)
        if cloud.width == 0: # no points
            print('Empty Line to follow')
            self.cbEmpty = True
            self.cbComplete = True
            return
        # using each points normal, project it '$depth' meters in the opposite-of-normal direction to create the shell
        self.chiselPts = [] #np.zeros([cloud.width, 3])
        self.chiselPtsProjected = [] #np.zeros([cloud.width, 3])
        for i, p in enumerate(generator): #go through all points in the generator
            pt = np.array(p[0:3])
            normal = np.array(p[3:6])
            if np.isnan(normal).any():
                continue
            self.chiselPts.append(pt)
            proj = pt+ -1. * self.nominal_depth * normal
            self.chiselPtsProjected.append(proj)
        self.chiselPts = np.asarray(self.chiselPts)
        self.chiselPtsProjected = np.asarray(self.chiselPtsProjected)
        self.cbComplete = True


    def anglesInToolFrame(self, pose1, pose2):
        # angles of pose1 in the base frame, transformed into frame pose2
        angles = getAngleLimit(pose1.orient * m3d.Vector(0, 0, -1))
        angles_tool = pose2.orient.inverse * angles
        return angles_tool

    # force sensor error fxn
    def shrinkOperator(self, array, shrinkValue):
        sign = np.sign(array)
        array -= sign * shrinkValue # origin translation. Subtact if positive, add if negative (bring closer to zero)
        idx = (sign*array) < 0.0
        array[idx] = 0.0 
        return array
    def forceSensorError(self):
        shrinkValue = 5.0 # newton*m
        yTorq = self.arm_client.get_force()[4:5]
        self.shrinkOperator(yTorq, shrinkValue)
        return yTorq

    def main(self, rob, ropeRob, directionToChisel):
        Debug = self.Debug
        # params
        t0 = time.time() # this t0 used in the timeOut
        # ---
        # ---
        nominal_angle = self.nominal_angle # w.r.t. NORMAL
        nominal_depth = self.nominal_depth
        timeOut = self.timeOut
        distanceOut = self.distanceOut
        timeToComplete = self.timeToComplete
        error_target_limit = self.error_target_limit
        depth_limit = 0.08 #+ (0.02 + offset)# depth from initPose, which is taken from the position BEFORE force_touch3
        
        PID_line = self.PID_line
        PID_target = self.PID_target
        PID_force = self.PID_force
        PID_angle = self.PID_angle

        controls = []
        times = []

        # check sphere object
        checkSphere = CheckSphere(rob, ropeRob, distanceOut)

        if not Debug:
            # setup takes ~0.11 seconds, which is close enough amount of time we'd want to ensure coupling
            #force_mode_tool(ropeRob, [0,0,+225.0,0,0,0], [0,0,0.1,0,0,0], setRemote=True)
            #force_mode(ropeRob, arm_client.get_pose(), [0,0,+225.0,0,0,0], [0,0,0.2,0,0,0], setRemote=True)
            initPose = force_touch3(rob, ropeRob, checkSphere,  120, 50)
            time.sleep(0.1)

        # set up numpy vectors
        y_tool = np.array([0, 1, 0])
        x_tool = np.array([1, 0, 0])
        y_base = rotate3V(rob, y_tool, to_TCP=False) #convert to np array
        x_base = rotate3V(rob, x_tool, to_TCP=False) #convert to np array

        # slice the point cloud with plane in line with chisel z-x plane
        normal = y_base
        slicePlaneNormal = x_base * directionToChisel
        ptOnPlane = initPose.pos.array
        data = np.concatenate([normal, ptOnPlane, slicePlaneNormal])
        #pdb.set_trace()

        self.cbComplete = False
        self.cbEmpty = False
        publishFloat(self.pubSetup, data)

        # get result
        while not self.cbComplete:
            time.sleep(0.01)
        if self.cbEmpty:
            return (True, None, None)

        chiselPts = self.chiselPts
        chiselPtsProjected = self.chiselPtsProjected

        # reverse order if necessary
        if directionToChisel < 0.0:
            chiselPtsProjected = np.flipud(chiselPtsProjected)

        # set up spline
        if chiselPtsProjected.shape[0] > 3:
            tck, u = interpolate.splprep(chiselPtsProjected.T) # tck = (t,c,k) = knots, coefficients, degree
        else:
            k = chiselPtsProjected.shape[0] - 1
            if k < 1:
                print("DynamicNormal:main. Not enough points in spline.")
                return (True, None, None)
            tck, u = interpolate.splprep(chiselPtsProjected.T, k=k) # tck = (t,c,k) = knots, coefficients, degree
        # simple way to calculate arc-length
        unew = np.arange(0, 1.01, 0.01) # 100 values
        dists = np.zeros_like(unew)

        out = np.array(interpolate.splev( unew, tck)).T # gets into correct form --> out[index, axis]
        #for i, u in enumerate(unew):
        #    dists[i] = interpolate.splint(0.0, u, tck) #idk what this actually integrates
        dists = unew # for now, assume the distance is fairly linear between control points (decent assumption due to PointCloud data)
        distMax = 1.0

        if Debug: #plotting
            distToRobot = np.linalg.norm(out - initPose.pos.array, axis=1) #make sure distance is increasing as fxn of index --> correct direction
            fig = plt.figure()
            plt.plot(distToRobot)
            plt.show() 

            fig = plt.figure()
            ax = plt.axes(projection = '3d')
            x, y, z = out.T
            x1, y1, z1 = chiselPts.T
            ax.plot3D(x, y, z, 'g')
            ax.scatter3D(x1, y1, z1, 'b')
            ax.set_xlim(-1.0, 0.0)
            ax.set_ylim(1.0, 2.0)
            plt.show()

        print "DynamicNormal: Seconds to prepare: ", time.time() - t0
        if Debug:
            pdb.set_trace()
        publishBool(self.pubActive, True)
        #t0 = time.time()
        distancedOut = checkSphere.check() # just in case force_touch3 slipped
        checkSphere.reset()
        normalAngle_tool_OLD = None
        timedOut = False
        firstLoop = True
        while not timedOut and not distancedOut:
            tLoop = time.time()
            robotPose = arm_client.get_pose()
            robotPos = robotPose.pos.array
            vecToAdd_tcp = m3d.Vector(0.0, 0.0, -0.03)
            vecToAdd_base = robotPose.orient * vecToAdd_tcp
            y_tool_WRT_base = rotate3V(rob, y_tool, to_TCP=False) #convert to np array

            # calculate control due to distance FROM virtual spline
            self.queryNormal.publish(robotPos+vecToAdd_base.array) #the callback is called almost immediately
            normal = self.queryNormal.normal

            # get closest point on virtual spline
            distToSpline = np.linalg.norm(out - robotPos, axis=1)
            idx = (np.abs(distToSpline)).argmin() # scale from distance to spline control -- auto protects against segfault
            pt_target = out[idx]
            robotToTarget =  pt_target - robotPos
            sign = np.sign( np.dot( normal, robotToTarget)) * directionToChisel
            error_line = np.linalg.norm(robotToTarget) * sign
            #error_line = np.clip(error_line, -0.02, 0.02) #clip the error distance from line
            PID_line.update([error_line])
            dAng_line = PID_line.getControl()

            #print "dAng_line", dAng_line*57. #convert rad -> deg

            # calculate control due to distance ALONG virtual spline
            t = time.time() - t0
            #if (t / timeToComplete) > 0.5:
            #    pdb.set_trace()
            dist = t / timeToComplete * distMax # scale from time to distance
            idx = (np.abs(dists - dist)).argmin() # scale from distance to spline control -- auto protects against segfault
            pt_target = out[idx] # extract xyz values from evaluated spline
            robotToTarget =  pt_target - robotPos
            fwd = np.cross(normal, y_tool_WRT_base)
            if np.isclose(np.linalg.norm(fwd), 0.0, 1e-3, 1e-3): #near zero
                if idx < out.shape[0]:
                    fwd = out[idx+1] - out[idx]
                else:
                    fwd = out[idx] - out[idx-1]
            else:
                fwd = unit_vector(fwd)
            sign = np.sign( np.dot( fwd, robotToTarget)) # direction to chisel taken into account because of y_tool and that 'shallower' control is opposite in opposite directions
            error_target = np.linalg.norm(robotToTarget) * sign
            #if abs(error_target) > error_target_limit:
                #print("Robot not advancing quick enough. Assuming Stuck. Exiting")
                #return (False, times, controls) #still want to back up, so "b_sphere" = False
            depth = (initPose.inverse * robotPose.pos)[2] # transform robot position into initial frame, and take the depth. Depth should be the z-direction
            #print "depth: ", depth
            if depth > depth_limit: # signed
                print("Chisel is too deep. Assuming Stuck. Exiting")
                return (False, times, controls) #still want to back up, so "b_sphere" = False

            PID_target.update([error_target])
            dAng_target = PID_target.getControl()

            # force-sensor control
            error_force = self.forceSensorError(ropeRob)
            PID_force.update(error_force) # tool-y rotation only
            dAng_force = PID_force.getControl()
            #print 'force ang: ',dAng_force*57., 'tool-y torque [shrunk]: ', error_force

            # final angle
            #dAng_line[0] = 0.0         # see what happens when ignore depth-follower
            dAng_target = 0.0         #ignore target for now
            #nominal_angle = 0.0
            angle = directionToChisel*nominal_angle + dAng_line + dAng_target

            # add the local normal (because nominal_angle is w.r.t. normal angle)
            normal_tool = rotate3V(rob, normal, to_TCP=True) # has the effect of subtracting robotPose's current y-rotation .....  I think
            normalAngle_tool = np.arctan(normal_tool[0] / normal_tool[2])
            angleWithNormal = angle + normalAngle_tool #y-dir
            preClip = np.clip(angleWithNormal, -math.pi/2., math.pi/2.) # pre-clipping at -90 to 90 deg to prevent angle wraparound
            #print 'preClip', preClip*57.

            # limit it
            delta_angle = self.checkAngleLimits(robotPose, preClip[0])[1] # have to index preClip because it's a numpy array
            #print '\ndelta_angle', delta_angle*57.
            # rosbag ROS publish
            publishFloat(self.pubDeltaAngle, [delta_angle])
            #
            delta_angle += dAng_force # add force control here, so that it doesn't get drowned out by dAng_line
            if math.isnan(delta_angle):
                timedOut = (time.time()-t0) > timeOut
                distancedOut = checkSphere.check()
                continue
            delta_angle = self.checkAngleLimits(robotPose, delta_angle[0])[1] # limit again in case force is in wrong direction
            #print 'delta_angle w/force', delta_angle*57.

            #if (t / timeToComplete) > 0.5:
            #    pdb.set_trace()
            # call force scoop with corrective delta-angle
            arr = np.array([0, 0, 1.] + [0, delta_angle, 0])
            PID_angle.update(arr)
            torque, limits = PID_angle.getForceModeInput()
            #print 'torque: ', torque, ' limits: ', limits
            #pdb.set_trace()
            rob.URSafetyEvent.wait()
            force_mode(ropeRob, robotPose, torque, limits, setRemote=True) # in the base frame


            #print "Time for this loop: ", time.time() - tLoop
            #print "Total time: ", time.time() - t0
            time.sleep(0.05)
            # outs
            times.append(time.time() - t0)
            timedOut = times[-1] > timeOut
            distancedOut = checkSphere.check()

        publishBool(self.pubActive, False)
        return (distancedOut, times, controls)
        
    def main_straight_attack(self, directionToChisel):
        """ This function doesn't change the attack-poses angle-of-attack
        """
        Debug = self.Debug
        # params
        t0 = time.time() # this t0 used in the timeOut
        # ---
        timeOut = self.timeOut
        distanceOut = self.distanceOut
        
        controls = []
        times = []

        # check sphere object
        checkSphere = CheckSphere(self.arm_client, distanceOut)

        print "DynamicNormal: Seconds to prepare: ", time.time() - t0
        if Debug:
            pdb.set_trace()
        publishBool(self.pubActive, True)
        #t0 = time.time()
        distancedOut = checkSphere.check() # just in case force_touch3 slipped
        checkSphere.reset()
        timedOut = False
        while not timedOut and not distancedOut:
            robotPose = self.arm_client.get_pose()

            self.arm_client.wait()
            # 0 = compliant axis (can move in that diretion), 1 = non-compliant axis (will use force to maintain the limit)
            selection_vector = [0, 0, 1, 0, 0, 0]
            torque = [0, 0, 225.0, 0, 0, 0]
            limits = [0.1, 0.1, 0.2, 0.2, 0.2, 0.2]
            self.arm_client.force_mode(pose=robotPose, torque=torque, limits=limits, selection_vector=selection_vector, setRemote=True) # in the base frame

            time.sleep(0.05)
            # outs
            times.append(time.time() - t0)
            timedOut = times[-1] > timeOut
            distancedOut = checkSphere.check()

        publishBool(self.pubActive, False)
        return (distancedOut, times, controls)

    def main_simple(self, mw, directionToChisel):
        """ The difference with main_simple is that the arm rotation is time-based
        """
        Debug = self.Debug
        # params
        t0 = time.time() # this t0 used in the timeOut
        # ---
        timeOut = self.timeOut
        distanceOut = self.distanceOut
        nominal_angle = self.nominal_angle
        
        PID_line = self.PID_line
        PID_target = self.PID_target
        PID_force = self.PID_force
        PID_angle = self.PID_angle

        controls = []
        times = []

        # check sphere object
        checkSphere = CheckSphere(self.arm_client, distanceOut)

        if not Debug:
            initPose = force_touch3(self.arm_client, checkSphere,  120, 50)
            time.sleep(0.1)

        print "DynamicNormal: Seconds to prepare: ", time.time() - t0
        if Debug:
            pdb.set_trace()
        publishBool(self.pubActive, True)
        #t0 = time.time()
        distancedOut = checkSphere.check() # just in case force_touch3 slipped
        checkSphere.reset()
        timedOut = False
        firstLoop = True
        while not timedOut and not distancedOut:
            tLoop = time.time()
            robotPose = self.arm_client.get_pose()
            robotPos = robotPose.pos.array
            vecToAdd_tcp = m3d.Vector(0.0, 0.0, -0.03)
            vecToAdd_base = robotPose.orient * vecToAdd_tcp

            # calculate control due to distance FROM virtual spline
            self.queryNormal.publish(robotPos+vecToAdd_base.array) #the callback is called almost immediately
            normal = self.queryNormal.normal
            normal = np.array([0.0, -1.0, 0.0])

            # force-sensor control
            error_force = self.forceSensorError()
            PID_force.update(error_force) # tool-y rotation only
            dAng_force = PID_force.getControl()

            angle = directionToChisel * nominal_angle
            #print("\n\nangle: {}".format(angle*57.))

            # add the local normal (because nominal_angle is w.r.t. normal angle)
            normal_tool = rotate3V(normal, robotPose, to_TF=True) # has the effect of subtracting robotPose's current y-rotation .....  I think
            normalAngle_tool = np.arctan(normal_tool[0] / normal_tool[2])
            angleWithNormal = angle + normalAngle_tool #y-dir
            preClip = np.clip(angleWithNormal, -math.pi/2., math.pi/2.) # pre-clipping at -90 to 90 deg to prevent angle wraparound
            #print 'preClip', preClip*57.

            # limit it
            delta_angle = mw.checkAngleLimits(robotPose, preClip)[1] # have to index preClip because it's a numpy array
            #print 'delta_angle', delta_angle*57.
            # rosbag ROS publish
            publishFloat(self.pubDeltaAngle, [delta_angle])
            #
            delta_angle += dAng_force # add force control here, so that it doesn't get drowned out by dAng_line
            if math.isnan(delta_angle):
                timedOut = (time.time()-t0) > timeOut
                distancedOut = checkSphere.check()
                continue
            delta_angle = mw.checkAngleLimits(robotPose, delta_angle[0])[1] # limit again in case force is in wrong direction
            #print 'delta_angle w/force', delta_angle*57.

            # call force scoop with corrective delta-angle
            arr = np.array([0, 0, 1.] + [0, delta_angle, 0])
            PID_angle.update(arr)
            torque, limits = PID_angle.getForceModeInput()
            #print 'torque: ', torque, ' limits: ', limits
            #pdb.set_trace()
            self.arm_client.wait()
            self.arm_client.force_mode(pose_base=robotPose, torque=torque, limits=limits, selection_vector=np.ones(6), setRemote=True) # in the base frame

            time.sleep(0.05)
            # outs
            times.append(time.time() - t0)
            timedOut = times[-1] > timeOut
            distancedOut = checkSphere.check()

        if timedOut:
            print "Dyno main_simple: Timed Out"
        if distancedOut:
            print "Dyno main_simple: Distanced Out"

        publishBool(self.pubActive, False)
        return (distancedOut, times, controls)

    def save_max_q(self, msg):
        self.max_q_value = msg.data
        #print self.max_q_value

    def mainML(self, arm_client, chiselbot_pub_status, directionToChisel):
        '''
        basically, this works so that we don't start chiseling unless the chiseler is ready to start giving angles
        chiseler says: I'm active
        mainML receives "I'm active" and responds with CHISELBOT_STATUS.ACTIVE
        chiseler receives CHISELBOT_STATUS.ACTIVE and begins publishing angles at a predefined rate.
        mainML starts executing angles at a pre-defined rate
        after it time's out, mainML says: CHISELBOT_STATUS.ATTACK_COMPLETE
        chiseler receives CHISELBOT_STATUS.ATTACK_COMPLETE and finishes the episode

        Failure modes: mainML never publishes CHISELBOT_STATUS.ATTACK_COMPLETE
        '''
        self.ml_client_waiter.wait()
        self.ml_client_waiter.reset() # need to reset this so that the chiseler, chiselbot handshake completes correctly
        self.angle_waiter.reset() # make sure I'm not using values from previous attack
        t0 = time.time() # this t0 used in the timeOut
        Debug = self.Debug
        # --- 
        timeOut = self.timeOut
        distanceOut = self.distanceOut
        timeToComplete = self.timeToComplete
        depth_limit = 0.08 #+ (0.02 + offset)# depth from initPose, which is taken from the position BEFORE force_touch3
        
        PID_force = self.PID_force
        PID_angle = self.PID_angle
        self.zforce = 1.0 # start with full force
        self.preClip = 0.0
        self.min_allowable_q = 0.05

        controls = []
        times = []

        # check sphere object
        checkSphere = CheckSphere(arm_client, distanceOut)
        # this allows the network to early-exit by not commanding any force
        # human feedback will stop it from abusing this in order to reduce total negative reward
        zforceTIMEOUT = TimeOut(timeOut=1.0) 
        zforceThreshold = 50.0 / 225.0

        print "DynamicNormal: Seconds to prepare: ", time.time() - t0
        if Debug:
            pdb.set_trace()
        t0 = time.time() # this t0 used in the timeOut
        self.arm_client.set_standard_digital_out(id1=0, val=1) #turn on drill
        if False: #not Debug:
            # setup takes ~0.11 seconds, which is close enough amount of time we'd want to ensure coupling
            initPose = force_touch3(self.arm_client, checkSphere, 120, 50)
            time.sleep(0.2)
        else:
            initPose = self.arm_client.get_pose()
        robotPose = initPose
        initPitch_tool = self.anglesInToolFrame(initPose, initPose)[1] # y-dir
        # ----- reset classes for the attack -----
        chiselbot_pub_status.publish(CHISELBOT_STATUS(type=CHISELBOT_STATUS.ACTIVE))
        publishBool(self.pubActive, True)
        distancedOut = checkSphere.check() # just in case force_touch3 slipped
        checkSphere.reset()
        timedOut = False
        pdb.set_trace()
        while not timedOut and not distancedOut: # and self.max_q_value > self.min_allowable_q:
            robotPose = self.arm_client.get_pose()
            depth = (initPose.inverse * robotPose.pos)[2]
            if depth > depth_limit: # signed
                finalPitch_tool = self.anglesInToolFrame(robotPose, initPose)[1] # y-dir
                directionChiseled = 1.0 if (finalPitch_tool - initPitch_tool) > 0.0 else -1.0 
                print("DynamicNormal: chisel is too deep. Assuming Stuck. Exiting")
                break


            # -------------------------------------
            if self.angle_waiter.msg is None:
                #print("Angle not ready.")
                timedOut = (time.time()-t0) > timeOut
                distancedOut = checkSphere.check()
            else:
                preClip, zforce = self.angle_waiter.msg.data
                if self.b_ML_toggle: preClip *= -1.0 # need to toggle the sign so that camera-left is the same regardless of if camera is oriented up or down
                self.angle_waiter.reset() # make sure we only update when ready, and don't block the control loop

                self.zforce = np.clip(zforce, 0.0, 1.0)
                self.preClip = np.clip(preClip, -np.pi/2.0, np.pi/2.0)
                #print("zForce: {}".format(self.zforce))
                #print '\npreClip (deg)', self.preClip*57.
                #print getMaxAngles(robotPose)

            # ----------------------- post process neural net output -----------------------
            # limit it
            delta_angle = self.checkAngleLimits(robotPose, self.preClip)
            #print "Clipped angle: ", delta_angle*57.
            # force-sensor control
            error_force = self.forceSensorError()
            PID_force.update(error_force) # tool-y rotation only
            dAng_force = PID_force.getControl()
            delta_angle += dAng_force # add force control here, so that it doesn't get drowned out by dAng_line
            delta_angle = self.checkAngleLimits(robotPose, delta_angle[0]) # limit again in case force is in wrong direction
            if np.isnan(delta_angle):
                timedOut = (time.time()-t0) > timeOut
                distancedOut = checkSphere.check()
                continue
            # call force scoop with corrective delta-angle
            #print("Final delta-angle: {}".format(delta_angle*57.))
            arr = np.array([0, 0, self.zforce] + [0, delta_angle, 0])
            PID_angle.update(arr)
            torque, limits = PID_angle.getForceModeInput()
            #print 'torque: ', torque, ' limits: ', limits
            #pdb.set_trace()
            self.arm_client.wait()
            # -------------------------------- Execute Force Mode --------------------------------
            self.arm_client.force_mode(pose=robotPose, torque=torque, limits=limits, selection_vector=np.ones(6), setRemote=True) # in the base frame

            # --- housekeeping ---
            times.append(time.time() - t0)
            timedOut = times[-1] > timeOut
            distancedOut = checkSphere.check()
            time.sleep(0.05)

        if timedOut:
            print("DynamicNormal: timedOut.")
        if distancedOut:
            print("DynamicNormal: distancedOut.")

        chiselbot_pub_status.publish(CHISELBOT_STATUS(type=CHISELBOT_STATUS.ATTACK_COMPLETE))

        publishBool(self.pubActive, False)
        finalPitch_tool = self.anglesInToolFrame(robotPose, initPose)[1] # y-dir
        directionChiseled = 1.0 if (finalPitch_tool - initPitch_tool) > 0.0 else -1.0 
        return (distancedOut, directionChiseled, times, controls)
    

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def getCamModel():
    topic = "/camera_02/depth/camera_info" # need depth info for depth input
    msg = rospy.wait_for_message(topic, CameraInfo, timeout=10.0)
    camModel = image_geometry.PinholeCameraModel()
    camModel.fromCameraInfo(msg)
    return camModel

class ValidAttacks(object):
    def __init__(self, frame_id="base", ns=""):
        self.lock = False
        self.lastPos = np.zeros(3)
        self.reset(1)
        self.queryNormal = QueryNormal(ns=ns)
        self.rayIntersect = RayIntersect(ns=ns)
        self.centroidFinder = CentroidFinder()
        self.peakFinder = PeakFinder()
        self.camModel = getCamModel()
        self.sub = rospy.Subscriber("wall/chiselIdentification/attackPos", Float32MultiArray, self.cb)
        self.frame_id_ = frame_id

    def cb(self, arr):
        self.lastPos = np.array(arr.data[:])
    
    def getValidAttacks(self, nominal_angle, useML=False):
        # for each candidate attack, see if it's valid through motion planning & geometry & bbox
        minDist = 0.05 # 7.5cm minimum distance away
        bools = np.zeros(len(self.optimalPts), dtype=bool)

        for idx in range(len(self.optimalPts)):
            optimalPt = self.optimalPts[idx]
            normal = self.normals[idx]
            centroid = self.centroids[idx]

            if np.isnan(optimalPt).any():
                bools[idx] = False
                #print("getValidAttacks: any was isnan.")
                continue

            dist = np.linalg.norm(optimalPt - self.lastPos)
            if dist < minDist:
                pass
                bools[idx] = False
                print "Attack too close to the previous. {}".format(dist)
                continue

            bValid, self.attackPose, self.directionToChisel = checkAttack(optimalPt, normal, centroid, self.queryNormal, useML)
            bools[idx] = bValid
        #print("Got Valid Attacks.")
        return bools
    
    def reset(self, length):
        # reset optimal pt
        self.optimalPts = np.NaN*np.ones([length, 3])
        self.normals = np.NaN*np.ones([length, 3])
        self.centroids = np.NaN*np.ones([length, 3])
        self.attackPose = None
        self.directionToChisel = None
    
    def setFromPeakFinder(self, origin, coords, transformer):
        self.reset(len(coords))
        cx = self.camModel.cx()
        cy = self.camModel.cy()
        for idx, (x, y) in enumerate(coords):
            u = x * (2.*cx)
            v = y * (2.*cy)
            unitDirection = self.camModel.projectPixelTo3dRay((u, v)) # outputs unit-direction vector of ray
            unitDirection_base = transformPointM3D(self.camModel.tfFrame(), self.frame_id_, transformer, unitDirection, freeVector=True)
            self.peakFinder.publish(origin+list(unitDirection_base)) # list concat
            if np.isnan(self.peakFinder.peak).any():
                continue
            self.rayIntersect.publish(origin+list(unitDirection_base)) # list concat
            if np.isnan(self.rayIntersect.intersectPt).any():
                continue
            # add to targeter
            self.optimalPts[idx] = np.array(self.rayIntersect.intersectPt)
            self.centroids[idx] = np.array(self.peakFinder.peak)
            self.queryNormal.publish( self.rayIntersect.intersectPt )
            self.normals[idx] = np.array(self.queryNormal.normal)

    def setFromPtsCentroidFinder(self, origin, coords, transformer):
        self.reset(len(coords))
        cx = self.camModel.cx()
        cy = self.camModel.cy()
        for idx, (x, y) in enumerate(coords):
            u = x * (2.*cx)
            v = y * (2.*cy)
            unitDirection = self.camModel.projectPixelTo3dRay((u, v)) # outputs unit-direction vector of ray
            unitDirection_base = transformPointM3D(self.camModel.tfFrame(), self.frame_id_, transformer, unitDirection, freeVector=True)
            #pdb.set_trace()
            self.centroidFinder.publish(origin+list(unitDirection_base)) # list concat
            if np.isnan(self.centroidFinder.intersectPt).any():
                continue
            # add to targeter
            self.optimalPts[idx] = np.array(self.centroidFinder.intersectPt)
            self.centroids[idx] = np.array(self.centroidFinder.centroid)
            self.queryNormal.publish( self.centroidFinder.intersectPt )
            self.normals[idx] = np.array(self.queryNormal.normal)
        #print("Set from pts done")
    '''
    def setFromPts(self, origin, coords, centroid_coords, transformer):
        self.reset()
        cx = self.camModel.cx()
        cy = self.camModel.cy()
        for x, y in coords:
            u = x * (2.*cx)
            v = y * (2.*cy)
            unitDirection = self.camModel.projectPixelTo3dRay((u, v)) # outputs unit-direction vector of ray
            unitDirection_base = transformPointM3D(self.camModel.tfFrame(), self.frame_id_, transformer, unitDirection, freeVector=True)
            self.rayIntersect.publish(origin+unitDirection_base) # list concat
            # add to targeter
            self.optimalPts.append( np.array(self.rayIntersect.intersectPt) )
            self.queryNormal.publish( self.rayIntersect.intersectPt )
            self.normals.append( np.array(self.queryNormal.normal) )
        for x, y in centroid_coords:
            u = x * (2.*cx)
            v = y * (2.*cy)
            unitDirection = self.camModel.projectPixelTo3dRay((u, v)) # outputs unit-direction vector of ray
            unitDirection_base = transformPointM3D(self.camModel.tfFrame(), self.frame_id_, transformer, unitDirection, freeVector=True)
            self.rayIntersect.publish(origin+unitDirection) # list concat
            self.centroids.append( np.array(self.rayIntersect.intersectPt) )
        #print("Set from pts done")
    '''


def checkAttack(optimalPt, normal, initial_angle, centroid, qn, useML=False):
    bbox = getExpandedBoundingBox(0.0) #same as chiselID
    if not checkBox(optimalPt, bbox):
        #print("checkAttack: not in bbox.")
        return False, None, None

    # TODO: do this right
    nominal_angle = initial_angle

    # calculate attack pose
    attackPose = m3d.Transform()
    #attackPose.pos = self.optimalPt # should be same as queryNormal.contact
    attackPose.pos = optimalPt
    attackPose.orient = getHorizontalOrient()
    normalAngles = getAngleLimit(normal) #is w.r.t. horizontal +y-axis

    limitsMin, limitsMax = getMaxAngles(attackPose)
    if (normalAngles < limitsMin).any() or (normalAngles > limitsMax).any():
        # return false if the starting orientation is out-of-range
        pass
        #return False, None, None
    normalAnglesLimited = normalAngles.clip(limitsMin, limitsMax)
    attackPose.orient.rotate_xb(normalAnglesLimited[0])
    attackPose.orient.rotate_zb(normalAnglesLimited[2])

    # calculate chisel roll based on the optimal point's location w.r.t. the convex hull's centroid
    ptToCentroid = centroid - optimalPt
    normal = -1.* (attackPose.orient * m3d.Vector.e2) # tool-z to base, but in opposite direction cuz it's the normal
    #self.normal = unit_vector(self.normal)
    alignmentNormal = np.cross(ptToCentroid, normal.array)
    alignmentNormal = unit_vector(alignmentNormal)

    #pdb.set_trace()
    # direction of chisel
    ptToCentroid_tool = attackPose.orient.inverse * ptToCentroid
    directionToChisel = 1.0 if ptToCentroid_tool[0] > 0.0 else -1.0 #single line if-else statement
    delta_angle = checkAngleLimits(attackPose, directionToChisel * nominal_angle)
    attackPose.orient.rotate_yt( delta_angle )

    # check if directionToChisel is valid in mode of attack position
    directionMode, _, _ = getChiselModeArr(attackPose.pos.array)
    if useML: # this check is only valid when we're using hard-coded low-level chiselling
        bReturn = True
    else:
        bReturn = False
        if (attackPose.orient * m3d.Vector.e1).z > 0.0:
            directionToChisel_base = -1.0 * directionToChisel
        else:
            directionToChisel_base = directionToChisel
        #print("base-chisel-dir: {}".format(directionToChisel_base))
        if directionMode == 0: # valid regardless
            bReturn = True
        elif directionMode == 1 and directionToChisel_base > 0.0:
            bReturn = True
        elif directionMode == -1 and directionToChisel_base < 0.0:
            bReturn = True

        if bReturn == False:
            pass
            #print("checkAttack: dir_mode: {}, dirToChisel: {}".format(directionMode, directionToChisel))

    return bReturn, attackPose, directionToChisel

class ChiselIdentification(object):
    def __init__(self, peak_targeter_timeout=60.0, chisel_id_timeout=60.0, ns=""):
        self.optimalPts = []
        self.normal = []
        self.centroid = []
        self.lastPos = [0,0,0]
        self.lock = False
        self.pub_counter = 1
        self.queryNormal = QueryNormal(ns=ns)
        self.rayIntersect = RayIntersect(ns=ns)
        self.subSetup = rospy.Subscriber("wall/chiselIdentification/output", numpy_msg(Float32MultiArray), self.cb)
        # "over-cut"
        self.oc_topic = ns+"wall/chiselIdentification"
        self.pubSetupOc = rospy.Publisher(self.oc_topic+"/input", Float32MultiArray, queue_size=1)

        # "under-cut"
        self.uc_topic = ns+"wall/peakTargeter"
        self.pubSetupUc = rospy.Publisher(self.uc_topic+"/input", Float32MultiArray, queue_size=1)

        self.timeouter1 = TimeOut(timeOut = peak_targeter_timeout) 
        self.timeouter2 = TimeOut(timeOut = chisel_id_timeout) 
        self.switch = 1.0



    def calcPose(self, mw, nominal_angle, useML=False):
        bTimedOut = False
        if self.lock: # wait until unlocked
            print("Waiting until ChiselID is unlocked.")
            t0 = time.time()
            timeOut = 5
            while self.lock and not bTimedOut:
                time.sleep(0.1)
                bTimedOut = (time.time() - t0) > timeOut
            if bTimedOut:
                print("ChiselID:calcPose. TimedOut. Returning.")
                return (bTimedOut, None, None)

        if len(self.optimalPts) == 0:
            print("ChiselID:calcPose. No optimal points.")
            return (True, None, None)

        valid = False
        poses = []
        directions = []
        for idx in range(len(self.optimalPts)):
            optimalPt = self.optimalPts[idx]
            normal = self.normals[idx]
            centroid = self.centroids[idx]

            valid, attackPose, directionToChisel = mw.check_attack(optimalPt, normal, nominal_angle, centroid, self.queryNormal)
            if valid:
                poses.append(attackPose)
                directions.append(directionToChisel)

        if len(poses) == 0:
            return (True, None, None)
        else:
            max_len = 5
            length = np.min([max_len, len(poses)])
            idx = np.random.randint(length)
            return (bTimedOut, poses[idx], directions[idx])
            #if valid:
            #    self.lastPos = attackPose.pos.array.tolist()
            #    return (bTimedOut, attackPose, directionToChisel)
        # this only executes if no valid attack pose was found
        return (True, None, None)

    def reset(self):
        # reset optimal pt
        self.optimalPts = []
        self.normals = []
        self.centroids = []
        

    def cb(self, array):
        self.reset()
        # array is now a 9*n long 
        data = array.data
        assert(len(data)%9 == 0 ) # make sure it's a multiple of 9
        n = len(data) / 9
        for i in range(n):
            dataN = data[(i*9):(i*9+9)] 
            optimalPt = dataN[0:3]
            normal = dataN[3:6]
            centroid = dataN[6:9]
            self.optimalPts.append(optimalPt)
            self.normals.append(normal)
            self.centroids.append(centroid)
        
        # randomize first 5 locations --> make sure you're using the 'sandstonePeakTargeter'
        if len(self.optimalPts) > 5:
            idxs = np.random.permutation(5)
            self.optimalPts[0:5] = [self.optimalPts[idx] for idx in idxs]
            self.normals[0:5]    = [self.normals[idx]    for idx in idxs]
            self.centroids[0:5]  = [self.centroids[idx]  for idx in idxs]

        self.lock = False

    def wait(self, topic):
        print("waiting on chiselID.")
        rospy.wait_for_message(topic + "/output", Float32MultiArray, timeout=10.0)
        while self.lock and not rospy.is_shutdown():
            time.sleep(0.1) #wait for callback to complete

    def make_data(self, arm_client, scaleSmall, scaleLarge, donThreshold, segmentRadius):
        pose = arm_client.get_pose()
        data = pose.pos.list
        data += pose.orient.quaternion.inverse.list # transform to tcp requires the inverse orientation
        data.append(scaleSmall)
        data.append(scaleLarge)
        data.append(donThreshold)
        data.append(segmentRadius)
        data += self.lastPos
        return data

    def main_uc(self, arm_client, scaleSmall, scaleLarge, donThreshold, segmentRadius, wait=False):
        """ main under-cut
        """
        data = self.make_data(arm_client, scaleSmall, scaleLarge, donThreshold, segmentRadius)
        self.lock = True
        publishFloat(self.pubSetupUc, data)
        if wait: self.wait(self.oc_topic)

    def main_oc(self, arm_client, scaleSmall, scaleLarge, donThreshold, segmentRadius, wait=False):
        """ main over-cut
        """
        data = self.make_data(arm_client, scaleSmall, scaleLarge, donThreshold, segmentRadius)
        self.lock = True
        publishFloat(self.pubSetupOc, data)
        if wait: self.wait(self.oc_topic)

    def main(self, arm_client, scaleSmall, scaleLarge, donThreshold, segmentRadius, wait=False):
        # ---------------- PCL ------------------
        # run difference-of-normals alg
        # run magnitude threshold
        # run euclidean distance clustering
        # run recursive convex hull generator
        # filter out empty hulls
        # project onto neutral plane
        # filter for edge points
        # rank points according to criteria (closest to horizontal)
        # determine direction to chisel
        # --------------- End PCL ---------------

        # trigger chiselIdentification cpp code
        print("chiselid: Last attack pos: {}".format(self.lastPos))
        
        if self.switch > 0.0:
            self.main_uc(arm_client, scaleSmall, scaleLarge, donThreshold, segmentRadius, wait)
            if self.timeouter1.check():
                self.switch *= -1.0
                self.timeouter2.reset()
        else:
            self.main_oc(arm_client, scaleSmall, scaleLarge, donThreshold, segmentRadius, wait)
            if self.timeouter2.check():
                self.switch *= -1.0
                self.timeouter1.reset()

def calcMidPoint(pose_i, pose_f, offset, offset_dir=None):
    e2 = m3d.Vector.e2

    e2_base_i = pose_i.orient * e2
    e2_base_f = pose_f.orient * e2

    avg_e2 = ((e2_base_i + e2_base_f) / 2.0)
    avg_e2.normalize()
    midPt = (pose_i.pos + pose_f.pos) / 2.0

    """
    line_direction = pose_f.pos.array - pose_i.pos.array
    line_direction /= np.linalg.norm(line_direction)
    avg_e2_projection = np.dot(avg_e2.array, line_direction) * line_direction
    avg_e2_rejection = avg_e2.array - avg_e2_projection
    avg_e2_rejection /= np.linalg.norm(avg_e2_rejection)
    """
    if offset_dir is None:
        offset_dir = avg_e2
    assert( isinstance(offset_dir, m3d.Vector) )
    #
    splineMidPt = midPt - offset * offset_dir
    #check to be safe
    if splineMidPt.length_squared > midPt.length_squared:
        splineMidPt = midPt + offset * offset_dir
    pts = np.vstack([pose_i.pos.array, splineMidPt.array, pose_f.pos.array])
    return pts


class ForceSpline_DEPRECATED(object):
    def __init__(self, ns=""):
        # ------ PARAMS ------
        self.offset = 0.1 # offset in negative avg_e2 direction (should be towards base)
        self.timeOut = 1.5
        self.distanceOut = 0.2
        self.timeToComplete = 1.0 # assume we want constant arc-speed
        self.Debug = True

        self.startPt = np.zeros(3)
        self.endPt = np.zeros(3)

        self.queryNormal = QueryNormal(ns=ns)
        self.pub_poseArray = rospy.Publisher("wall/forceSpline/poseArray", PoseArray, queue_size=1)
        #time.sleep(0.5) # must sleep so that ROS stuff has time to connect

    def zieglerNicholsGainTuning(self, arm_client):
        # https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method

        notStable = True
        while notStable:
            pass

        """
        P = 0.0; I = 0.0; D = 0.0
        if controlType == 'PD':
            P = 0.8*Ku; D = Tu / 8.0
        """

        return #(P,I,D)


    def main(self, arm_client, targetPose): #input must be a full m3d transform
        if not isinstance(targetPose, m3d.Transform):
            print("ForceSpline:Main. Must input m3d.Transform for targetPose")
        # ------ PARAMS ------
        t0 = time.time()
        offset = self.offset # w.r.t. NORMAL
        timeOut = self.timeOut
        distanceOut = self.distanceOut
        timeToComplete = self.timeToComplete
        Debug = self.Debug
        controls = []
        times = []

        # check sphere object
        checkSphere = CheckSphere(arm_client, distanceOut)

        # --- setup calculations ---
        e2 = m3d.Vector.e2

        # assuming real-time
        pose_i = arm_client.get_pose()
        pose_f = targetPose 
        pts = calcMidPoint(pose_i, pose_f, offset)

        # set up spline
        tck, u = interpolate.splprep(pts.T, k=2) # tck = (t,c,k) = knots, coefficients, degree
        # simple way to calculate arc-length
        unew = np.arange(0, 1.01, 0.01) # 100 values
        dists = np.zeros_like(unew)
        out = np.array(interpolate.splev(unew, tck)).T # gets into correct form
        # limit with bounding box
        bbox = getBoundingBox()
        bbox[1] += 0.03 # increase y a little so that we don't overshoot (unlikely, but still)
        out = out.clip(bbox[0:3], bbox[0:3]+bbox[3:6])
        #for i, u in enumerate(unew):
        #    dists[i] = interpolate.splint(0.0, u, tck) #idk what this actually integrates
        dists = unew # for now, assume the distance is fairly linear between control points (decent assumption due to PointCloud data)
        distMax = 1.0

        # spherical linear interp (slerp) the orientation
        slerp = Slerp(pose_i.orient, pose_f.orient)

        if Debug: #plotting
            distToRobot = np.linalg.norm(out - pose_i.pos.array, axis=1) #make sure distance is increasing as fxn of index --> correct direction
            fig = plt.figure()
            plt.plot(distToRobot)
            plt.show() 

            fig = plt.figure()
            ax = plt.axes(projection = '3d')
            x, y, z = out.T
            ax.plot3D(x, y, z)
            plt.show()

            # ROS
            poseArray = PoseArray()
            poseArray.header.frame_id = "base"
            poseArray.header.stamp = rospy.Time.now()
            for idx in range(out.shape[0]):
                x, y, z = out[idx]
                orient = slerp.orient(float(idx) / out.shape[0])
                # because tool is z-forward
                orient.rotate_yt(-90./180.*math.pi)
                quat = orient.quaternion
                # fill in object
                pose = Pose()
                pose.position.x = x
                pose.position.y = y
                pose.position.z = z
                pose.orientation.w    = quat[0]
                pose.orientation.x    = quat[1]
                pose.orientation.y    = quat[2]
                pose.orientation.z    = quat[3]
                poseArray.poses.append(pose)
            self.pub_poseArray.publish(poseArray)

        # set up control
        Kp_pos = 400.0 # distance -> force
        Kd_pos = 50.0
        Kp_rot = 0.5 # distance -> speed, not speed -> speed
        Kd_rot = 0.25
        Kp = np.array([Kp_pos]*3+[Kp_rot]*3, dtype='float')
        Kd = np.array([Kd_pos]*3+[Kd_rot]*3, dtype='float')

        # limits
        maxForce = 225.0 # Newtons
        maxTorque = 0.2 #is actually the rotation speed
        maxControl = np.array([maxForce]*3+[maxTorque]*3, dtype='float')
        deadForce = 5.0
        deadTorque = 0.05
        deadband = np.array([deadForce]*3+[deadTorque]*3, dtype='float')
        limitSpeed = 0.2
        limitAngSp = 0.05
        limitsDefault = np.array([limitSpeed]*3+[limitAngSp]*3, dtype='float')

        print "Seconds to prepare: ", time.time() - t0
        if Debug:
            pdb.set_trace()
        t0 = time.time()
        checkSphere.reset()
        timedOut = False
        distancedOut = False
        while not timedOut and not distancedOut:
            tLoop = time.time()
            robotPose = arm_client.get_pose()
            robotPos = robotPose.pos.array
            robotEuler = robotPose.orient.rotation_vector

            # calculate control due to distance ALONG virtual spline
            t = time.time() - t0
            alpha = t / timeToComplete
            alpha = np.clip(alpha, 0.0, 1.0)
            dist =  alpha * distMax # scale from time to distance
            idx = (np.abs(dists - dist)).argmin() # scale from distance to spline control -- auto protects against segfault
            pt_target = out[idx] # extract xyz values from evaluated spline
            
            targetEuler = slerp.quat(alpha).rotation_vector
            error_pos =  pt_target - robotPos
            error_ang =  targetEuler - robotEuler
            wrapAround(error_ang)
            error = np.concatenate([error_pos, error_ang])

            #if abs(np.linalg.norm(error[0:3])) > error_target_limit:
            #    print("Robot not advancing quick enough. Assuming Stuck. Exiting")
            #    return (times, controls)

            control = np.multiply(error, Kp)
            control = control.clip(-1.*maxControl, maxControl) # saturate the controls for safety
            controls.append(control.tolist())
            torque, limits = URcontrolDeadband(control, limitsDefault, deadband)

            #if alpha > 0.5:
            #    pdb.set_trace()
            #pdb.set_trace()
            #print('finish debugging before continuing')
            # call force scoop with corrective delta-angle
            arm_client.force_mode(pose_base=m3d.Transform(), torque=torque, limits=limits, setRemote=True)
            #ropeRob.set_force_remote([0.0]*6, [1]*6, torque, limits, 2) # in the base frame

            #print "Time for this loop: ", time.time() - tLoop
            #print "Total Time: ", time.time() - t0
            time.sleep(0.05)
            # outs
            times.append(time.time() - t0)
            timedOut = times[-1] > timeOut
            distancedOut = checkSphere.check()

        return (times, controls)



class Mapping(object):
    def __init__(self, ns=""):
        self.queryNormal = QueryNormal(ns=ns)
        self.pub = rospy.Publisher('wall/reconstructed/flagGo', Bool, queue_size=5)        
        self.pubReset = rospy.Publisher('wall/reconstructed/resetMap', Bool, queue_size=5) 
        self.pubBBox = rospy.Publisher('robot/boundingBox', Float32MultiArray, queue_size=5, latch=True) 
        

    def calibrate(self, arm_client):
        # tell user to face robot toward a vertical FLAT wall
        # 
        inp = raw_input('Put chisel against flat wall. Base-frame must be square with wall.\nEnter "yes" to continue. ').rstrip('\n')
        if inp == 'yes':
            OFFSET, ANGLE_OFFSET, CAM_DIR = getOffsets()
            robotPose = arm_client.get_pose()
            #robotPose.orient.rotate_zt(ANGLE_OFFSET/180.*math.pi) #get back to original TCP orientation
            self.queryNormal.publish(robotPose.pos.array)
            normal = self.queryNormal.normal
            all_zeros = not np.any(normal)
            if all_zeros:
                print 'Could not get normals. Retry'
            # nominally, should be square with wall
            mainDir = np.argmax(abs(normal))
            otherDir = 1 - mainDir # mainDir = 1 --> otherDir = 0
            pitch = np.arctan( normal[2] / normal[mainDir])
            yaw = np.arctan( normal[otherDir] / normal[mainDir])
            vec = m3d.Vector([0.0, pitch, yaw])
            vec_tool = rotate3V(vec, arm_client.get_pose(), to_TF=True)
            vec_tool_deg = vec_tool / math.pi*180.

            print '\nCamera Calibrate: '
            print 'Pitch (about tool-y-dir) offset (deg): ', -1.*vec_tool_deg[1]
            print 'Yaw (about tool-x-dir) offset (deg): ', -1.*vec_tool_deg[2]
            print 'Add these values to your current offset angles'
            print ''

        else:
            return


    def buildMap(self, arm_client, acc=0.2, vel=0.2, angle=10.):
        # rotate in cone around TCP
        # ----- parameters -------
        Amplitude = angle / 180. * math.pi
        # ----- setup -------
        t = np.arange(0, 1.1, .1)
        yaw = Amplitude*np.sin(2*np.pi*t)
        pitch = Amplitude*np.cos(2*np.pi*t)
        #roll  =  

        initPose = arm_client.get_pose()
        pose_list = []
        for index, val in enumerate(t):
            pose = initPose.copy()
            pose.orient.rotate_xt(yaw[index])
            pose.orient.rotate_yt(pitch[index])
            pose_list.append(pose.pose_vector.tolist())
            arm_client.movel(pose=pose, acc=acc, vel=vel)
       
    def capture(self):
        b = Bool()
        b.data = True
        self.pub.publish(b)

    def resetMap(self):
        b = Bool()
        b.data = True
        self.pubReset.publish(b)

    def pushBoundingBox(self, bbox=None):
        if bbox is None: bbox = getBoundingBox()
        publishFloat(self.pubBBox, bbox)

class TimeOut(object):
    def __init__(self, timeOut=10.0):
        self.timeOut = timeOut
        self.reset()

    def check(self):
        self.bTimedOut = (time.time()-self.t0) > self.timeOut
        return self.bTimedOut

    def reset(self):
        self.t0 = time.time()
        self.bTimedOut = False

    def check_and_reset(self):
        timed_out = (time.time()-self.t0) > self.timeOut
        if timed_out:
            self.reset()
        return timed_out

class WaitForClosed(object):
    ''' 
    Base class that also sets event clear when a message is posted to a special threading topic
    This ensures that exiting with ctrl+c also exits the Event wait
    '''
    def __init__(self):
        self.sub = rospy.Subscriber("/threading/close", Empty, self.close_cb, queue_size=10)
        self.active = True
        self.event = threading.Event()

    def close(self):
        self.raiseKeyboard = True
        self.event.set()
        del self.event

    def close_cb(self, msg):
        self.event.set()

    def activate(self):
        self.active = True

    def deactivate(self):
        self.active = False

    def clear(self):
        self.event.clear()

class WaitForEnum(WaitForClosed):
    def __init__(self, topic, msg):
        '''
        Callback to wait until an enum msg published on a ROS topic is equal to the type to wait for.
        Assumes that all enum msgs use keyword 'type' to signify which enum type they are.
        NOTE: Do NOT change the callback on ctrl+c here, because it'll fuck up the ROS.on_shutdown callback
        and never call it.
        '''
        super(WaitForEnum, self).__init__()
        self.raiseKeyboard = False
        self.reset()
        self.sub = rospy.Subscriber(topic, msg, self.cb, queue_size=10)

    def quit(self, signo, _frame):
        #print("Interrupted by %d, shutting down" % signo)
        self.raiseKeyboard = True
        self.event.set()

    def reset(self):
        self.enumtype = 0 # I use 1 as minimum on my enums
        self.waitenum = 0

    
    def check(self):
        if self.enumtype == self.waitenum:
            self.event.set()

    def cb(self, msg):
        self.enumtype = msg.type       
        self.check() 

    def wait(self, waitenum):
        self.waitenum = waitenum
        self.check()
        while self.active and (not self.raiseKeyboard) and (not self.event.is_set()):
            self.event.wait(5.0)
        if self.raiseKeyboard:
            raise KeyboardInterrupt
        self.clear()

class WaitForTopic(WaitForClosed):
    def __init__(self, topic, msg):
        '''
        Callback to wait until an enum msg published on a ROS topic is equal to the type to wait for.
        Assumes that all enum msgs use keyword 'type' to signify which enum type they are.
        '''
        super(WaitForTopic, self).__init__()
        self.raiseKeyboard = False
        self.reset()
        self.sub = rospy.Subscriber(topic, msg, self.cb, queue_size=10)

    def quit(self, signo, _frame):
        #print("Interrupted by %d, shutting down" % signo)
        self.raiseKeyboard = True
        self.event.set()

    def reset(self):
        self.msg = None

    def cb(self, msg):
        self.msg = msg
        self.event.set()

    def wait(self):
        while self.active and (not self.raiseKeyboard) and (not self.event.is_set()):
            self.event.wait(5.0)
        if self.raiseKeyboard:
            raise KeyboardInterrupt
        self.clear()

class WaitForActive(WaitForClosed):
    def __init__(self, namespace):
        '''
        callback to wait until a ROS topic is active and ready to accept input values
        '''
        super(WaitForActive, self).__init__()
        topic = os.path.join(namespace, "active")
        self.raiseKeyboard = False
        self.reset()
        self.sub = rospy.Subscriber(topic, Bool, self.cb, queue_size=10)


    def quit(self, signo, _frame):
        #print("Interrupted by %d, shutting down" % signo)
        self.raiseKeyboard = True
        self.event.set()

    def reset(self):
        self.topic_active = False
        self.clear()

    def cb(self, BoolMsg):
        self.topic_active = BoolMsg.data 
        if self.topic_active:
            self.event.set()
        else:
            self.event.clear()

    def wait(self):
        while self.active and (not self.raiseKeyboard) and (not self.event.is_set()):
            self.event.wait(5.0)
        if self.raiseKeyboard:
            raise KeyboardInterrupt

def getDist(pose, qn):
    
    pos_array = pose.pos.array
    qn.publish(pos_array)
    contact = qn.contact
    v1 = contact - pos_array
    sign = np.sign(np.dot( pos_array, v1))
    dist = sign * np.linalg.norm(pos_array - contact)
    return dist

class CheckSphere(object):
    def __init__(self, get_tf_base_tcp, radius, waitTime=10.0):
        self.get_tf_base_tcp = self.get_tf_base_tcp
        self.radius = radius
        self.radius2 = radius**2
        self.waitTime = waitTime
        # reset
        self.reset()

    def reset(self):
        self.t0 = time.time()
        self.pos0 = self.get_tf_base_tcp().pos.array

    def check(self):
        b_sphere = False
        pos = self.get_tf_base_tcp().pos.array #x,y,z position
        diff = pos - self.pos0
        pos2 = np.inner(diff, diff)
        if pos2 > self.radius2:
            b_sphere = True
        return b_sphere

class CheckDistance():
    def __init__(self, arm_client, radius, pos_final, waitTime=10.0):
        self.arm_client = arm_client
        self.radius = radius
        self.radius2 = (radius**2) * 2.5
        self.waitTime = waitTime
        self.pos_final = pos_final
        self.pose_history = []
        # reset
        self.reset()

    def reset(self):
        self.t0 = time.time()
    
    def update_pose_history(self, pose_history):
        self.pose_history = pose_history

    def check(self, tol = None):
        b_inside = False
        pos = self.arm_client.get_pose().pos.array #x,y,z position
        diff = pos - self.pos_final
        pos2 = np.inner(diff, diff)
        
        # Allow for input tolerance to override radius2
        if tol is None:
            tol = self.radius2
        if self.arm_client.arm_type == "ur10":
            if len(self.pose_history) > 15:
                pos_last = self.pose_history[len(self.pose_history) - 1].pos.array
                pos_first = self.pose_history[0].pos.array
                diff = pos_last - pos_first
                print("First pose to Last pose distance = {}".format(np.inner(diff, diff)))
                pose_history_criteria = np.inner(diff,diff) < (0.001**2)
                if pos2 < tol or pose_history_criteria:
                    if pose_history_criteria:
                        print("Setting plunge/cut complete based on distance between current pose and oldest pose!!")
                    elif pos2 < tol:
                        print("Setting plunge/cut complete based on tolerance criteria!!")
                    b_inside = True
                    pos2 = 0 # For percentage completion purposes we can assign this to zero when b_inside has become True                
            else:
                if pos2 < tol:
                    b_inside = True
                    pos2 = 0
        return b_inside, pos2

def check_sphere(arm_client, radius, waitTime, alpha=0.15):
    t0 = time.time()
    radius2 = radius**2
    pos0 = arm_client.get_pose().pos.array
    b_sphere = False
    bbox = getExpandedBoundingBox( alpha ) #intermediate box, halfway between inner-most and final box
    bbox[1] += 0.04 # remember, "expanded" box doesn't change y-dir limit
    while (time.time() - t0) < waitTime:
        pos = arm_client.get_pose().pos.array #x,y,z position
        pos2 = np.linalg.norm(pos - pos0)
        b_box = not checkBox(pos.array, bbox)
        if pos2 > radius2 or b_box:
            b_sphere = True
            print('check_sphere: Out of Range, or outside BBox.')
            break
    return b_sphere, b_box


@deprecated
def force_scoop_DEPRECATED(arm_client, force, maxAngle, maxSpeed=0.45, timeOut=10.0, gains=[0.05, 0.5, 0.25], force_touch=False, check_sphere=False, radius=0.05):
    # NOTE: rotation component must be large enough (max 50 Nm) to overcome dynamic moments
    # otherwise you get cyclical behavior. See: blooper reel
    # Note2: confirmed the tool_pose() "locks in" the frame, it doesn't update as the chisel rotates

    # Note3: control loop is slow, 3 seconds -> 11 loops --> ~4 hz (not that good)

    # check max angle, assume > 90 deg in radians is a fault
    if abs(maxAngle) > 90./180.*math.pi:
        print('Max Angle above 90 deg (in radians), assuming wrong units. Converting to radians.')
        maxAngle *= math.pi / 180.0

    # strategy: rotate until reach max angle
    initPoseGlobal = arm_client.get_pose().pose_vector #confirmed this is global frame
    initPose = rotate6V(initPoseGlobal, arm_client.get_pose(), to_TF=True) #transform into TCP frame
    initAngle = initPose.pose_vector[4] #because of calibration, we scoop about the tool 'y' axis
    diffAngle = 0.0
    t0 = time.time()
    radius2 = radius**2
    pos0 = initPoseGlobal[0:3] #xyz initial

    #force mode inputs
    sel_vector = [1.,1.,1.,1.,1.,1.] # 'z' & 'pitch' directions
    torque = [0., 0., force, 0., 999.0*np.sign(maxAngle), 0.] # linear is max 225, but angular is anything.
    limits = [0.,0.,0.,0.,0.,0.] #angular behavior is much more dependent on max rot/s, must be careful.

    #linear ramp-up & ramp-down of angular speed, as a percentage of maxAngle
    # the control for ramp-up will saturate at the maxAngSpeed value, and then will be brought back down 
    # to 0.0 by the ramp-down controller
    def angFxnUp():
        poseGlobal = arm_client.get_pose().pose_vector
        poseTCP = rotate6V(poseGlobal, arm_client.get_pose(), to_TF=True)
        return poseTCP[4]
    def angFxnDown():
        Ry = angFxnUp()
        if abs(Ry - initAngle) < abs(rampDown*maxAngle):
            return initAngle + rampDown*maxAngle
        else:
            return Ry

    rampUp, rampDown, maxAngSpeed = gains
    #linController = linearControl
    torqRampUpController   = linearControl(angFxnUp(), rampUp*maxAngle, angFxnUp, 0.05, maxAngSpeed) #linear in angles
    torqRampDownController = linearControl(angFxnDown(), (1.0-rampDown)*maxAngle, angFxnDown, 0.0, -1.*maxAngSpeed+.05) #linear in angles

    delay = 0.05
    tol = 1e-1 # ~5.7 degrees
    b_sphere = False

    #first loop is making contact with wall
    if force_touch:
        dir1 = [0, 1, 0] # + global-y
        sel1 = abs(np.array(dir1))
        torque1 = 200*np.array(dir1) #how much force to push with, in Newtons
        robPose = arm_client.get_pose()
        force_mode(ropeRob, robPose, [0,0,225.0,0,0,0], [0,0,0.2,0,0,0], setRemote=False)
        while abs(rob.get_force()) < 150:
            time.sleep(0.05)
        pos0 = robPose.pos.array #xyz initial
    time.sleep(0.3)

    while abs(diffAngle) < abs(maxAngle):
        if (time.time() - t0) > timeOut:
            print('force_scoop: timeout!')
            break
        #if abs(ropeRob.robotConnector.RobotModel.dataDir['urPlus_force_torque_sensor'][2]) > 275.0: #z-direction
        #    print ropeRob.robotConnector.RobotModel.dataDir['urPlus_force_torque_sensor'][2]
        #    print('force_scoop: too much z-force (usually means shear force')
        #    break
        task_frame = arm_client.get_pose() # is actually a pose
        if check_sphere:
            pos = task_frame.pos.array #x,y,z position
            pos2 = (pos[0]-pos0[0])**2 + (pos[1]-pos0[1])**2 + (pos[2]-pos0[2])**2
            #print 'Position deviation: ', pos2**0.5
            if pos2 > radius2:
                b_sphere = True
                print('force_scoop check_sphere: Out of Range.')
                break
        angSpeed1 = torqRampUpController.getControl() #update y-torque
        angSpeed2 = torqRampDownController.getControl()
        angSpeed = angSpeed1 + angSpeed2 #positive --> will rotate, negative --> no rotate
        if angSpeed > maxAngSpeed: angSpeed = maxAngSpeed
        #print 'angSpeed: ', angSpeed
        #deadband
        if abs(force) < 10.0:
            force = 0.0
            limits[2] = 0.0
        else:
            torque[2] = force
            limits[2] = maxSpeed
        if angSpeed < 0.05:
            angSpeed = 0.0
            limits[4] = 0.0
        else:
            limits[4] = angSpeed
        #update force command
        force_mode(ropeRob, task_frame, torque, limits, setRemote=True)
        diffAngle = angFxnUp() - initAngle #difference between current angle and initial angle
        #print 'diffAng', diffAngle
        #time.sleep(delay)
    #test
    #force_stop(rob, ropeRob, [-225.0, -50.0]) #wait time built in
    #time.sleep(2.0)
    #diffAngle = angFxnUp() - initPose[4] #difference between current angle and initial angle
    #print 'Final diffAng', diffAngle
    #print('\n\n\n')
    return b_sphere

def force_touch3(arm_client, checkSphere, threshold, contactThreshold, cmd_speed = 0.12):
    if contactThreshold > threshold:
        print("force_touch3: ContactThreshold must be less than threshold. Returning.")
        return

    def func():
        fz = arm_client.get_force()[2]
        return fz
    check_time = TimeOut(10.0)
    lc1 = linearControl(0.0, threshold, func, cmd_speed, cmd_speed) # 0.125 to 0.05
    contactPose = arm_client.get_pose()
    contactFlag = False
    while abs(func()) < threshold: # and (not checkSphere.check()):
        speed = lc1.getControl()
        # print 'force_touch3: speed', speed
        pose = arm_client.get_pose()
        arm_client.force_mode(pose_base=pose, torque=[0,0,+225.0,0,0,0], limits=[0,0,speed,0,0,0], selection_vector=np.ones(6), setRemote=True)
        # print 'force_touch3: force', abs(func())
        if not contactFlag and abs(func()) > contactThreshold:
            contactPose = pose
            contactFlag = True
        if check_time.check():
            return None
        #time.sleep(0.05)

    # arm_client.end_force_mode()
    return contactPose


def checkBox(pts, box):
    # pts = numpy array [pt index, dimension]
    # box = numpy array [x, y, z, w, h, d]
    # output: b_inside: numpy boolean array of whether each pt is outside the bounding box
    pts2d = np.atleast_2d(pts)
    ll = box[0:3]
    ul = ll + box[3:6]
    b_inside = np.logical_and(np.all(ll < pts2d, axis=1), np.all(pts2d < ul, axis=1))

    if not b_inside:
        pass
        #print("Outside bounding box.")
    #    pdb.set_trace()
    return b_inside

def wrapAround(inputArray):
    idx = abs(inputArray) > math.pi
    while np.sum(idx) > 0:
        inputArray[idx] -= np.array([2.*math.pi]*np.sum(idx))*np.abs(inputArray[idx]) # wrap-around
        idx = abs(inputArray) > math.pi

def deltaHorizontal(pose):
    pdb.set_trace()
    horizontalOrient = getHorizontalOrient()
    horizontalEuler_base = horizontalOrient.rotation_vector
    orientDelta = pose.orient.rotation_vector - horizontalEuler_base
    wrapAround(orientDelta)
    return orientDelta

def checkAngle(pose, horizontalEuler_base, limitsMin, limitsMax):
    orientDelta = pose.orient.rotation_vector - horizontalEuler_base
    wrapAround(orientDelta)

    b_inside_angle = np.all( np.logical_and( limitsMin < orientDelta, orientDelta < limitsMax) )
    #pdb.set_trace()
    return b_inside_angle

def ctrl2velocity(ropeRob, ctrl, linear_velocity, rotational_velocity):
    # unpack ctrl list
    left, right, up, down, fwd, bwd, lr, rr, upp, dnp, ly, ry, chisl = ctrl

    base_speed = np.zeros(6)
    tool_speed = np.zeros(6)

    #get base linear speed (no rotation)
    base_speed[2] += (up - down) * linear_velocity #global z

    #get tool linear speed from joystick
    tool_speed[1] += (right - left) * linear_velocity #reversed
    tool_speed[2] += (fwd - bwd) * linear_velocity #move along chisel

    #get tool rotational speed from joystick
    tool_speed[3] -= (ry - ly) * rotational_velocity #yaw
    tool_speed[4] += (upp - dnp) * rotational_velocity #pitch is reversed
    tool_speed[5] += (rr - lr) * rotational_velocity #roll

    ### rotations ###
    ts_mount = TCP_MOUNT_TF(ropeRob, tool_speed, toTCP=False)
    base_mount = BASE_MOUNT_TF(base_speed, toMount=True)

    final_speed = [base_mount[i] + ts_mount[i] for i in xrange(len(base_speed))] #add together velocities
    return final_speed

