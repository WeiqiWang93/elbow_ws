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
import tf2_ros
import shelve
#
import yaml
from enum import Enum

from itertools import groupby
from bot_common_ros.ur_utils import TCP_BASE_TF, getHorizontalOrient, m3d_to_ros, linearControl, rotate3V, rotate6V, UR_FORCE_PID, rosParamToClassObject, closest_pt_to_array, SensorFeedbackControl, SensorFeedbackControl2, ROStoM3d, fill_pose_stamped
from bot_common_ros.ur_utils import *

from bot_arm_planner.constrained_traj_sampler import WayPointConstraints, ConstrainedTrajSampler

from bot_perception_ros.srv import updatePC, mergePC, queryNormal

from dynamic_reconfigure.server import Server

from rospy_message_converter import message_converter
import random

from bot_overseer_api import OverseerAPI

from bot_common_ros.sia20_driver import *
from bot_common_ros.sia20_driver import SIA20Driver

class MapManager(object):

    def __init__(self):

        self.tf_listener = ROS_TF.TransformListener()
        self.state = 'UNIT'
        self.static_tfs = []
        

    
    def get_closest_point(self, ns="/ctm"):
        """ Returns the closest point to tcp using query normal.
        """
        qn = rospy.ServiceProxy(ns+'/queryNormal', queryNormal)

        (trans, rot) = self.tf_listener.lookupTransform("chisel_base", "chisel_tip_link", rospy.Time(0))
        
        req = Float32MultiArray()
        req.data = trans
        resp = qn(req)

        if resp.SUCCESS:
            arr = np.array(resp.output.data[:])
            normal = arr[0:3]
            contact = arr[3:6]
            sqDist = arr[6]
            return contact.tolist()
        else:
            trans = np.array(trans)
            return trans.tolist()


    def get_state(self):
        return self.state

    def update_pc(self, ns):

        self.state = 'INCOMPLETE'
        self.upc = rospy.ServiceProxy('/' + ns +'/wall/updatePC', updatePC)
        (trans, rot) = self.tf_listener.lookupTransform("chisel_base", "camera_chisel_tool_link", rospy.Time(0))
        tf_base_to_camera = ROStoM3d(trans, rot)
        resp = self.upc(m3d_to_ros(tf_base_to_camera))
        self.state = 'COMPLETE'
        return True

    def capture_pc(self, ns, cam_id=0):

        self.state = 'INCOMPLETE'
        self.merge_pc = rospy.ServiceProxy('/' + ns +'/wall/mergePC', mergePC)
        self.merge_pc(True, cam_id)
        self.state = 'COMPLETE'
        return True
    
    def update_platform_tf(self, pts_world_frame=[[0, 0.95, 0.0], [2.15, 0.83, 0.0]], pts_platform_frame=[[-0.71, 0.0, 0.0], [1.443, 0.0, 0.0]],
                                world_frame="world", platform_frame="platform_frame", publish=True):

       
        pts_world = [np.array(pt) for pt in pts_world_frame]
        pts_plat = [np.array(pt) for pt in pts_platform_frame]

        tf_world_to_sample = m3d.Transform()
        tf_world_to_sample.pos = pts_world[0]

        vec_sample = m3d.Vector(pts_world[1] - pts_world[0])
        yaw = m3d.Vector(1.0, 0.0, 0.0).signed_angle(vec_sample)

        tf_world_to_sample.orient.rotate_zb(yaw)

        tf_platform_to_sample = m3d.Transform()
        tf_platform_to_sample.pos = pts_plat[0]

        vec_sample_plat = m3d.Vector(pts_plat[1] - pts_plat[0])
        yaw = m3d.Vector(1.0, 0.0, 0.0).signed_angle(vec_sample_plat)
        tf_platform_to_sample.orient.rotate_zb(yaw)

        tf_world_to_platform = tf_world_to_sample * tf_platform_to_sample.inverse
        
        if publish:
            self.pub_tf(tf_world_to_platform.copy(), world_frame, platform_frame)
        else:
            return tf_world_to_platform
    

    def localize(self, pts_platform_frame=[[-0.61, 1.475, 0.0], [0.0, 1.475, 0.0]], pts_arm_frame=[[0.42184, 0.32482, -0.31583], [1.046, 0.36194, -0.3113]],
                                    platform_frame="platform_left_frame", arm_frame="chisel_base"):
        
        tf_platform_arm_base = self.update_platform_tf(pts_world_frame=pts_platform_frame, pts_platform_frame=pts_arm_frame,
                                    world_frame=platform_frame, platform_frame=arm_frame, publish=False)

        # Mobile base to arm base frame
        time.sleep(3.0)
        (trans, rot) = self.tf_listener.lookupTransform("world", arm_frame, rospy.Time(0))
        tf_mobile_base_arm_frame = ROStoM3d(trans, rot)

        tf_platform_mobile_base = tf_platform_arm_base * tf_mobile_base_arm_frame.inverse

        self.pub_tf(tf_platform_mobile_base, platform_frame, "world")
    
    def publish_boulder_frame(self, tf_world_to_boulder, world_frame):
        self.pub_tf(tf_world_to_boulder.copy(), world_frame, 'boulder_frame')
    
        
    def pub_tf(self, tf, parent, child):

        tf_world_to_platform = tf
        platform_frame = TransformStamped()
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        platform_frame.header.stamp = rospy.Time.now()
        platform_frame.header.frame_id = parent
        platform_frame.child_frame_id = child

        platform_frame.transform.translation.x = tf_world_to_platform.pos.x
        platform_frame.transform.translation.y = tf_world_to_platform.pos.y
        platform_frame.transform.translation.z = tf_world_to_platform.pos.z

        quat = list(tf_world_to_platform.orient.get_unit_quaternion())
        platform_frame.transform.rotation.x = quat[1]
        platform_frame.transform.rotation.y = quat[2]
        platform_frame.transform.rotation.z = quat[3]
        platform_frame.transform.rotation.w = quat[0]
        self.static_tfs.append(platform_frame)
        broadcaster.sendTransform(self.static_tfs)






if __name__ == "__main__":
    rospy.init_node("map_manager")
    
    
    map_manager = MapManager()
    sia20 = SIA20Driver()
    
    time.sleep(1) 
   
    # map_manager.update_platform_tf(world_frame="locust_world", platform_frame="platform_left_frame")

    # map_manager.update_platform_tf(pts_world_frame=[[0, 0.95, 0.0], [2.15, 0.83, 0.0]], pts_platform_frame=[[-0.72, 0.0, 0.0], [1.433, 0.0, 0.0]],
    #                                    world_frame="locust_world", platform_frame="platform_right_frame")

    # map_manager.update_platform_tf(pts_world_frame=[[-0.61, 1.475, 0.0], [0.0, 1.475, 0.0]], pts_platform_frame=[[0.42184, 0.32482, -0.31583], [1.046, 0.36194, -0.3113]],
    #                                 world_frame="platform_left_frame", platform_frame="chisel_base")

    # map_manager.localize(pts_platform_frame=[[-0.61, 1.475, 0.0], [0.0, 1.475, 0.0]], pts_arm_frame=[[0.42184, 0.32482, -0.31583], [1.046, 0.36194, -0.3113]],
    #                                 platform_frame="platform_left_frame", arm_frame="chisel_base")
    
    bbox = Marker()
    floor_z = -0.21
    roof_z = 0.49
    xmin = -0.96
    xmax = -0.5
    bbox.header.frame_id = "saw_base"
    bbox.pose.position.x = 0.5 * (xmin + xmax)
    bbox.pose.position.y = 1.10
    bbox.pose.position.z = 0.5 * (floor_z + roof_z) 
    bbox.pose.orientation.w = 1.0
    bbox.scale.x = abs(xmax - xmin)
    bbox.scale.y = 0.40
    bbox.scale.z = roof_z - floor_z
    
    sia20.initialize_robot(sim=False, arm_params_path="/sia20_driver/arm_1_saw/params/", tool_params_path="robo_saw_goelz/arm/", overseer=False)
    # scan_poses = map_manager.get_scan_poses(bbox=bbox, cam_fov=0.18, cam_dist=0.6, cam_frame="camera_saw_tool_front_link", tool="saw")
    # js_traj = map_manager.sample_scan_traj(scan_poses)

    
    sf = shelve.open('scan_state.dat', writeback=True)
    js_traj = sf['tool_front']
    sf.close()

    
    sia20.smart_move(joints_goal=list(js_traj[0]), velocity=0.1, acceleration=0.2)
    
    # api = OverseerAPI()
    jlist = js_traj

    for js in jlist:
        sia20.movels(jointList=[js], vel_rot=np.deg2rad(5.0), wait=True, poseList_base=[0.0])
        try:
            map_manager.capture_pc("saw")
        except:
            pass

    # api.run_private_op('CHISEL.CAPTURE_PC', 'ctm')
            
        
        