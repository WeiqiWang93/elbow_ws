import urx, URBasic, sys
import math3d as m3d
import numpy as np
import math, time
import pdb

from scipy import interpolate
import matplotlib.pyplot as plt # plots


import rospy, roslib, cv2
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped, PointStamped, Pose, TransformStamped, Quaternion
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
import sensor_msgs.point_cloud2 as pc2
import message_filters
import threading, socket
import tf as ros_tf
import tf.msg 
from math3d.interpolation import SE3Interpolation
import tf2_ros

## ---------------------------------------------------------------------------------------------------------
import warnings
import functools

def deprecated(func):
    """This is a decorator which can be used to mark functions
    as deprecated. It will result in a warning being emitted
    when the function is used."""
    @functools.wraps(func)
    def new_func(*args, **kwargs):
        warnings.simplefilter('always', DeprecationWarning)  # turn off filter
        warnings.warn("Call to deprecated function {}.".format(func.__name__),
                      category=DeprecationWarning,
                      stacklevel=2)
        warnings.simplefilter('default', DeprecationWarning)  # reset filter
        return func(*args, **kwargs)
    return new_func

## ---------------------------------------------------------------------------------------------------------

'''
import logging
logger = logging.getLogger('root')
logging.basicConfig(
    format="%(asctime)-15s [%(levelname)s] %(funcName)s: %(message)s")
logger.setLevel(logging.INFO)
'''
horizontalMount = False # aka chiseling up-down
verticalMount = True # aka chiseling sideways
if (horizontalMount and verticalMount) or (not horizontalMount and not verticalMount):
    raise Exception("bot_common_ros.ur_utils: horizontalMount/verticalMount exactly one must be true. Exiting.")

if horizontalMount:
    xr =  -00.0   / 180.*math.pi
    yr = 0.0   / 180.*math.pi
    zr = 180.0   / 180.*math.pi
elif verticalMount:
    xr =  180.0   / 180.*math.pi # previous was 0.0
    yr = -90.0   / 180.*math.pi
    zr = 0.0   / 180.*math.pi # previous was 90.0

MOUNT_TO_BASE = m3d.Transform() # default is zero in the vector
MOUNT_TO_BASE.orient.rotate_xb(xr)
MOUNT_TO_BASE.orient.rotate_yt(yr)
MOUNT_TO_BASE.orient.rotate_zt(zr)
#MOUNT_TO_BASE.orient.rotate_zb( 180.0 / 180.0 * np.pi)
GRAVITY_VEC_mount = 9.81*(MOUNT_TO_BASE.inverse * m3d.Vector.e2) # POSITIVE z-direction, NOT NEGATIVE

# input:    list or np.array of [x, y, z, rx, ry, rz] 
# output:   math3d.transform.Transform
def list_to_tf(pose):
    if isinstance(pose, np.ndarray):
        pose = pose.tolist()
    pos = pose[0:3]
    rot = m3d.Orientation.new_euler(pose[3:6], "xyz")
    tf = m3d.Transform(rot, pos)
    return tf

# input:    math3d.transform.Transform
# output:   np array [x, y, z, rx, ry, rz]
def tf_to_arr(tf):
    pos = tf.get_pos().get_array()
    rot = tf.get_orient().to_euler("xyz")
    return np.concatenate([pos, rot])

# input:    math3d.transform.Transform
# output:   list [x, y, z, rx, ry, rz]
def tf_to_list(tf):
    pos = tf.get_pos().get_array()
    rot = tf.get_orient().to_euler("xyz")
    return pos.tolist() + rot.tolist()

def pose_msg_to_dict(pose):

    pose_dict = {}
    pose_dict['pos_x'] = pose.position.x
    pose_dict['pos_y'] = pose.position.y
    pose_dict['pos_z'] = pose.position.z
    pose_dict['rot_x'] = pose.orientation.x
    pose_dict['rot_y'] = pose.orientation.y
    pose_dict['rot_z'] = pose.orientation.z
    pose_dict['rot_w'] = pose.orientation.w

    return pose_dict

def pose_dict_to_msg(pose_dict):

    pose = Pose()
    pose.position.x = pose_dict['pos_x']
    pose.position.y = pose_dict['pos_y']
    pose.position.z = pose_dict['pos_z']
    pose.orientation.x = pose_dict['rot_x']
    pose.orientation.y = pose_dict['rot_y']
    pose.orientation.z = pose_dict['rot_z']
    pose.orientation.w = pose_dict['rot_w']

    return pose

def pose_msg_to_list(pose):

    plist = [0]*7
    plist[0] = float(pose.position.x)
    plist[1] = float(pose.position.y)
    plist[2] = float(pose.position.z)
    plist[3] = float(pose.orientation.x)
    plist[4] = float(pose.orientation.y)
    plist[5] = float(pose.orientation.z)
    plist[6] = float(pose.orientation.w)

    return plist

def list_to_pose_msg(plist):

    pose = Pose()
    pose.position.x = plist[0]
    pose.position.y = plist[1]
    pose.position.z = plist[2]
    pose.orientation.x = plist[3]
    pose.orientation.y = plist[4]
    pose.orientation.z = plist[5]
    pose.orientation.w = plist[6]

    return pose

def linear_interp_tf(tf1, tf2, num_poses=None, pose_spacing=None, add_end=False):
        """ Given two poses, example last spline pose and extract pose
            Linearly interpolates the poses
            num_poses is the number if poses added after tf1 excluding tf2
            Returns a list of list of poses [tf1, tf1a, .., tf1z]            
            """

        # Create the tf interpolation object
        tf_interpolator = SE3Interpolation(tf1, tf2)
        dist = np.linalg.norm(tf1.pos.array - tf2.pos.array)
        # tf interpolated list contains tf's from tf1 to tf2
        tf_interpolated_list = []
        if num_poses is None:
            num_poses = int(np.floor(0.5 * dist / pose_spacing))
        total_num_poses = num_poses + 1 # including first and not last

        # Depending on number of poses append the linearly interpolated TFs to the list
        for i in range(0, total_num_poses):
            tf_interpolated_list.append(tf_interpolator(float(i)/total_num_poses))
        
        if add_end:
            tf_interpolated_list.append(tf2)
        
        return tf_interpolated_list

def getOffsets():
    """get chisel offset from base-TCP coordinate system, angle offset in degrees from the nominal UR10 TCP frame, and camera orientation (1.0 == camera up, -1.0 == camera down)
    
    ```python
    Input: void  
    Output: float OFFSET, float ANGLE_OFFSET, float NOMINAL_CAM_DIR
    ```"""
    # true value from chisel-tip to base-TCP
    true_value = 0.60 # africa campaign length 
    OFFSET = true_value - 0.0 #0.01 # meters, usually works better if we subtact 1 cm
    ANGLE_OFFSET = -0.0 # degrees. calculate using Left-hand-rule. use -90deg for camera vertical-up
    NOMINAL_CAM_DIR = 1.0 # 1.0 == camera up, -1.0 == camera down
    # in polyscope, -90 rotate rotated an axis neg z-tool
    return OFFSET, ANGLE_OFFSET, NOMINAL_CAM_DIR

# rotation matrix TCP to non-rotated TCP
_, angle_offset, _ = getOffsets()
TCP_TO_SENSOR_orient = m3d.Orientation()
TCP_TO_SENSOR_orient.rotate_zb(angle_offset/180.*np.pi)



# input:    list or np.array of [x, y, z, rx, ry, rz] 
# output:   math3d.transform.Transform
def list_to_tf(pose):
    if isinstance(pose, np.ndarray):
        pose = pose.tolist()
    pos = pose[0:3]
    rot = m3d.Orientation.new_euler(pose[3:6], "xyz")
    tf = m3d.Transform(rot, pos)
    return tf

# input:    math3d.transform.Transform
# output:   np array [x, y, z, rx, ry, rz]
def tf_to_arr(tf):
    pos = tf.get_pos().get_array()
    rot = tf.get_orient().to_euler("xyz")
    return np.concatenate([pos, rot])

# input:    math3d.transform.Transform
# output:   list [x, y, z, rx, ry, rz]
def tf_to_list(tf):
    pos = tf.get_pos().get_array()
    rot = tf.get_orient().to_euler("xyz")
    return pos.tolist() + rot.tolist()

def pose_msg_to_dict(pose):

    pose_dict = {}
    pose_dict['pos_x'] = pose.position.x
    pose_dict['pos_y'] = pose.position.y
    pose_dict['pos_z'] = pose.position.z
    pose_dict['rot_x'] = pose.orientation.x
    pose_dict['rot_y'] = pose.orientation.y
    pose_dict['rot_z'] = pose.orientation.z
    pose_dict['rot_w'] = pose.orientation.w

    return pose_dict

def pose_dict_to_msg(pose_dict):

    pose = Pose()
    pose.position.x = pose_dict['pos_x']
    pose.position.y = pose_dict['pos_y']
    pose.position.z = pose_dict['pos_z']
    pose.orientation.x = pose_dict['rot_x']
    pose.orientation.y = pose_dict['rot_y']
    pose.orientation.z = pose_dict['rot_z']
    pose.orientation.w = pose_dict['rot_w']

    return pose

def pose_msg_to_list(pose):

    plist = [0]*7
    plist[0] = float(pose.position.x)
    plist[1] = float(pose.position.y)
    plist[2] = float(pose.position.z)
    plist[3] = float(pose.orientation.x)
    plist[4] = float(pose.orientation.y)
    plist[5] = float(pose.orientation.z)
    plist[6] = float(pose.orientation.w)

    return plist

def list_to_pose_msg(plist):

    pose = Pose()
    pose.position.x = plist[0]
    pose.position.y = plist[1]
    pose.position.z = plist[2]
    pose.orientation.x = plist[3]
    pose.orientation.y = plist[4]
    pose.orientation.z = plist[5]
    pose.orientation.w = plist[6]

    return pose


def getGlobalVelocities():
    linVel = 0.1
    angVel = 0.1
    acc = 0.4
    return linVel, angVel, acc

def getObstacleBox():
    """DEPRECATED"""
    # mining stope obstacle box
    # x & y are effectively infinite, only the z matters (65 cm stope height)
    corner = np.array([ -10., -10., -0.22])
    size = np.array([ 20., 20., 0.65])

    ObstacleBox = np.concatenate([corner, size])
    return ObstacleBox

def getBoundingBox():
    """user set 3d rectangular bounding box used to define operational area of chiselbot
    
    ```python
    Input: void  
    Output: np.array [cornerx, cornery, cornerz, width, length, height]
    ```"""
    if horizontalMount:
        corner = np.array([ -0.23, 1.0, 0.08])
        size = np.array([ 0.4, 1.087+0.15-corner[1], 0.45+.05])
    elif verticalMount:
        corner = np.array([ -0.2, 0.9, -0.2])
        size = np.array([ 0.4 - corner[0], 1.1+0.2-corner[1], 0.35 - corner[2]]) # 40cm seems to be max, with 20deg

        corner = np.array([ -0.195, 0.9, -0.15])
        size = np.array([ 0.22 - corner[0], 1.1+0.4-corner[1], 0.25 - corner[2]]) 

        _, _, CAM_DIR = getOffsets()
        if False and int(round(CAM_DIR)) == -1: 
            if False:
                # mirror the bounding box
                t1 = -1.0*(corner[2] + size[2])
                corner[2] = t1
            else:
                # directly set upside-down bbox
                corner = np.array([ -0.30, 0.95, -0.20])
                size = np.array([ 0.35 - corner[0], 1.1+0.10-corner[1], 0.15 - corner[2]]) # 40cm seems to be max, with 20deg

        totalVolume = np.prod(size)
        #print("Total Bounding Box Volume: %s" % totalVolume)

    boundingBox = np.concatenate([corner, size])
    return boundingBox

def getChiselModeArr(arr, percentage_upper, percentage_lower, bbox=None):
    """splits up bounding box into three sections in each direction. Axis direction corresponds to -1, 0, 1 in the output. Used to determine if the arm is constrained to chisel in only one direction (left or right mode), or both directions (middle mode).
    Includes hard-set percentage which user must set [TO CHANGE].
    
    ```python
    Input: np.array (x, y, z) position
    output: np.array (3,)
    ```"""
    if bbox is None:
        bbox = getBoundingBox()
    # TODO: add x-percentage as linear controller fxn of depth (z)
    size = bbox[3:6]
    mins  = bbox[0:3]
    maxs  = mins + size
    upperQ = maxs - size * percentage_upper
    lowerQ = mins + size * percentage_lower
    mode = np.zeros(3)
    # if I include the arr < maxs and mins < arr, then if the chisel goes outside the range, it automatically thinks it's mode 0 ........ VERY BAD
    mode[ upperQ < arr ] = 1
    mode[ arr < lowerQ ] = -1
    return mode

def getChiselMode(arr):
    """DEPRECATED"""
    x, y, z = arr # in base coord system
    bbox = getBoundingBox()
    percentage = 0.2 # expanded bbox region
    xSize = bbox[3]
    minx  = bbox[0]
    maxX  = minx + xSize
    upperQ = maxX - xSize * percentage
    lowerQ = minx + xSize * percentage
    chiselMode = 0
    if upperQ < x: # and x < maxX: if I include the x< maxX, then if the chisel goes outside the range, it automatically thinks it's mode 0 ........ VERY BAD
        chiselMode = 1 # positive rotation about tool-y (upward chiseling allowed, only)
    elif x < lowerQ:
        chiselMode = -1 # negative rotation about tool-y (downward chiseling allowed, only)
    return chiselMode

def getExpandedBoundingBox(percentage):
    """expands the bounding box by an additional percentage amount. DOES NOT expand in the -y direction for safety, because that's in the direction toward the robot base. This makes it less likely to self-collide with the base.
    
    ```python
    Input: float  
    Output: np.array [cornerx, cornery, cornerz, width, length, height]
    ```"""
    # percentage is how much ADDITIONAL size to add on
    boundingBox = getBoundingBox()
    expansion = boundingBox[3:6] * percentage # by 50% on either side
    boundingBox[0] -= expansion[0]
    boundingBox[3] += 2.*expansion[0]
    boundingBox[4] += 2.*expansion[1] #expand y-depth, but not the starting spot
    boundingBox[2] -= expansion[2]
    boundingBox[5] += 2.*expansion[2]
    return boundingBox


def getAlphaOfBoundingBox(box, alpha):
    """expand bounding box by a factor of alpha"""
    center = box[0:3] + alpha * box[3:6]
    return center

def getMaxAngles_DEPRECATED(robotPose):
    """Given the robot position and the min/max Includes hard-set maxAngle_base & minAngle_base & percentage which user must set [TO CHANGE].
    
    ```python
    Input: math3d.Transform object
    Output: np.array (3,), np.array (3,)
    ```"""
    if horizontalMount:
        maxAngle_base = np.array( [ 30,  5, 10], dtype='float') * math.pi / 180.0 # [roll, pitch, yaw] limits w.r.t. base frame
        minAngle_base = np.array( [-45, -5, -7], dtype='float') * math.pi / 180.0 # [roll, pitch, yaw] limits w.r.t. base frame
    elif verticalMount:
        # maxAngle_base = np.array( [ 10,  5, 55], dtype='float') * math.pi / 180.0 # [roll, pitch, yaw] limits w.r.t. base frame
        # minAngle_base = np.array( [ -17.5, -5, -55], dtype='float') * math.pi / 180.0 # [roll, pitch, yaw] limits w.r.t. base frame
        # for the integration tests:
        maxAngle_base = np.array( [ 15,  5, 45], dtype='float') * math.pi / 180.0 # [roll, pitch, yaw] limits w.r.t. base frame
        minAngle_base = np.array( [ -7, -5, -55], dtype='float') * math.pi / 180.0 # [roll, pitch, yaw] limits w.r.t. base frame

    # get nominal mode -- 
    hMode, _, vMode = getChiselModeArr(robotPose.pos.array)
    #print('H-mode: {}'.format(hMode))
    if hMode == 1: maxAngle_base[2] = 0
    if hMode == -1: minAngle_base[2] = 0

    if vMode == 1: minAngle_base[0] = maxAngle_base[0]
    if vMode == -1: maxAngle_base[0] = minAngle_base[0]
    if vMode == 0: 
        minAngle_base[0] = 0.0
        maxAngle_base[0] = 0.0

    def indptFxn():
        return robotPose.pos[2]

    bbox = getBoundingBox()
    o1 = minAngle_base[0]
    o2 = maxAngle_base[0]
    percentage = 0.35
    start = 1.0 - percentage
    cl_max = linearControl(bbox[2], percentage * bbox[2+3], indptFxn, o1, o2)
    cl_min = linearControl(bbox[2] + start * bbox[2+3], percentage * bbox[2+3], indptFxn, o1, o2)
    aMax = cl_max.getControl()
    aMin = cl_min.getControl()
    maxAngle_base[0] = aMax
    minAngle_base[0] = aMin

    return minAngle_base, maxAngle_base

def rotate180List(orient, direction = 1.0):
    # meant for use by tool, in tool-frame
    # rotate 180 while controlling which direction
    # direction = 1.0 => clockwise
    ots = []
    copy1 = orient.copy()
    copy1.rotate_zt(direction * math.pi / 2.0)
    ots.append( copy1.copy() )
    copy1.rotate_zt(direction * math.pi / 2.0)
    ots.append( copy1.copy() )
    return ots

def getHorizontalOrient(inv = False, cam_horizontal=False):
    """get a horizontal orientation in the base frame. 
    Sets the defualt saw blade oritentation with respect to base frame assuming the current operating condition. 
    That the  saw is mounted physically(its probably not the slot direction but we are not sure) either horizontal or vertical with respect to the world coordinates.
    With the blade facing the wall which is y axis in base frame currently

    ```python
    Input: bool whether to inverse, bool if the camera is horizontal
    Output: math3d.Orientation
    ```"""
    OFFSET, ANGLE_OFFSET, CAM_DIR = getOffsets()
    orient = m3d.Orientation() # x-dir
    if horizontalMount:
        orient.rotate_zb( 90.0 / 180.0 * math.pi)
        orient.rotate_yt( 90.0 / 180.0 * math.pi)
        orient.rotate_zt( 180.0 / 180.0 * math.pi)
    elif verticalMount:
        orient.rotate_zb( 90.0 / 180.0 * math.pi)
        orient.rotate_yt( 90.0 / 180.0 * math.pi)
        orient.rotate_zt( -90.0 / 180.0 * math.pi)
        if CAM_DIR == -1.0:
            orient.rotate_zt( 180.0 / 180.0 * math.pi)
        if inv:
            orient.rotate_zt( 180.0 / 180.0 * math.pi)
    # orient.rotate_zt(-ANGLE_OFFSET/180.*math.pi)
    return orient

def wrapAngles(angles):
    wrappedAngles = []
    for angle in angles:
        angle = (angle*180/np.pi) % 360
        wrappedAngles.append(angle*np.pi/180)
    return wrappedAngles

def clipToPi(val,limit):
    # if val>0 and (val-limit) < 0:
    #     lower = 0
    # else:
    #     lower = val-limit

    # if val<(2*np.pi) and (val+limit) > (2*np.pi):
    #     upper = (2*np.pi)
    # else:
    #     upper = val+limit
    lower = val-limit
    upper = val+limit
    
    return [lower,upper]

def rosParamToClassObject(class1, rosParams, must_already_exist=False):
    """Sets class attributes according to keys in the input dict, using the value of that key. Used to generically import yaml config
    
    ```python
    Input: python class, dict of parameters
    Output: void
    ```"""
    print("Updating Values to class: {}".format(class1))
    for key, value in rosParams.iteritems():
        if must_already_exist and hasattr(class1, key):
            valid = True
        elif must_already_exist and not hasattr(class1, key):
            valid = False
        else:
            valid = True
        if valid:
            print("     {} to value: {}".format(key, value))
            setattr(class1, key, value)

def movel(robot=None, tf=MOUNT_TO_BASE, pose=None, acc=0.5, vel=None, wait=False, timeout=5, threshold=None):
    """python wrapper for move-in-linear-space"""
    pose_mount = tf.inverse * pose # base -> mount
    if isinstance(robot, urx.Robot):
        robot.movel(pose_mount.pose_vector, acc=acc, vel=vel, wait=False)
        if wait:
            robot._wait_for_move(pose_mount.pose_vector[:6], timeout=timeout, threshold=threshold)
    else:
        robot.movel(pose_mount.pose_vector, a=acc, v=vel, wait=wait)
        
def movels(URXrob=None, tf=MOUNT_TO_BASE, poseList_base=None, acc=0.5, vel=None, radius=0.01, wait=False, timeout=5, threshold=None):
    """python wrapper for move-in-linear-space with a spline meaning multiple poses to traverse through
    
    ```python
    Input: robot object, list-like (6, x) of poses in the base frame
    Output: void
    ```"""
    poseList_mount = []
    for pose in poseList_base:
        pose_mount = tf.inverse * pose # base -> mount
        poseList_mount.append(pose_mount.pose_vector)

    if isinstance(URXrob, urx.Robot):
        URXrob.movels(poseList_mount, acc=acc, vel=vel, radius=radius, wait=False, threshold=threshold)
        if wait:
            URXrob._wait_for_move(target=poseList_mount[-1], timeout=timeout, threshold=threshold)
    else:
        print("movels: Input must be python-urx.")
        return

def movexs(URXrob=None, command="", pose_list=[], acc=0.01, vel=0.01, radius=0.01, wait=True, timeout=5, threshold=None, prefix="p"):
    """python wrapper for move with a spline meaning multiple poses to traverse through
    
    ```python
    Input: python-urx robot object, movement command string, list-like (6, x) of poses in the base frame
    Output: list (3,) position
    ```"""
    if not isinstance(URXrob, urx.Robot):
        print("movexs: Input must be python-urx.")
        return
        
    """
    Concatenate several movex commands and applies a blending radius
    pose_list is a list of pose.
    This method is usefull since any new command from python
    to robot make the robot stop
    """
    header = "def myProg():\n"
    end = "end\n"
    prog = header
    for idx, pose in enumerate(pose_list):
        if idx == (len(pose_list) - 1):
            radius = 0
        prog += URXrob._format_move(command, pose, acc, vel, radius, prefix=prefix) + "\n"
    prog += end
    URXrob.send_program(prog)
    if wait:
        if prefix=="p":
            URXrob._wait_for_move(target=pose_list[-1], threshold=threshold)
        else:
            URXrob._wait_for_move(target=pose_list[-1], threshold=threshold,joints=True, timeout=3.0)
        return URXrob.getl()

def movejs(URXrob=None, jointList=None, acc=0.5, vel=0.25, radius=0.0025, wait=True, timeout=5, threshold=None):
    """python wrapper for move-in-joint-space with a spline meaning multiple poses to traverse through
    
    ```python
    Input: python-urx robot object, list-like (6, x) of joint angles
    Output: void
    ```"""
    assert(type(jointList) == list )
    assert (len(jointList) > 0)
    assert (type(jointList[0]) == list)
    if isinstance(URXrob, urx.Robot):
        movexs(URXrob, "movej", jointList, acc=acc, vel=vel, radius=radius, wait=False, threshold=threshold, prefix="")
        if wait:
            URXrob._wait_for_move(target=jointList[-1], timeout=timeout, threshold=threshold, joints=True)
    else:
        print("movels: Input must be python-urx.")
        return

def movej(robot=None, joints=[], acc=0.1, vel=0.05, wait=True, relative=False, threshold=None):
    """python wrapper for move-in-joint-space
    
    ```python
    Input: python-urx robot object, list-like (6,) of joint angles
    Output: list (6,) joints
    ```"""
    if relative:
        l = robot.getj()
        joints = [v + l[i] for i, v in enumerate(joints)]
    prog = robot._format_move("movej", joints, acc, vel)
    robot.send_program(prog)
    if wait:
        robot._wait_for_move(joints[:6], threshold=threshold, joints=True)
        return robot.getj()

def get_pose(robot=None, tf=MOUNT_TO_BASE, inMountFrame=False):
    """python wrapper for get robot pose
    
    ```python
    Input: robot object, bool if the pose should be in mount frame or base frame
    Output: math3d.Transform object
    ```"""
    if isinstance(robot, urx.Robot):
        tf_mount = robot.get_pose()
    else:
        tf_mount = m3d.Transform(robot.get_actual_tcp_pose())
    if not inMountFrame:
        tf_base = tf * tf_mount
        return tf_base
    else:
        return tf_mount

def get_joints(robot=None):
    """python wrapper for get robot joint angles

    ```python
    Input: robot object
    Output: list (6,) joint angles
    ```"""
    if isinstance(robot, urx.Robot):
        joints = robot.getj()
    else:
        joints = np.array(robot.get_actual_joint_positions())
    return joints

def get_pos(robot=None, inMountFrame=False):
    """python wrapper for get robot position
    
    ```python
    Input: robot object
    Output: list (3,) position
    ```"""
    pose = get_pose(robot, inMountFrame=inMountFrame)
    return pose.pos.array

def get_force(robot=None, tf=TCP_TO_SENSOR_orient):
    """python wrapper for get TCP force in TCP frame
    
    ```python
    Input: ur-interface robot object
    Output: np.array (6,) force in TCP frame
    ```
    """
    if isinstance(robot, urx.Robot):
        print("get_force: Input must be rope-robotics.")
        return
    # 'arr' is in the sensor frame, which is not rotated w.r.t. ANGLE_OFFSET
    arr = np.array(robot.robotConnector.RobotModel.dataDir['urPlus_force_torque_sensor'])
    if arr is None:
        print("Error: Check FT300 interface.")
        return None
    force = arr[0:3]
    torque = arr[3:6]
    # force_TCP = tf.inverse * force
    # torque_TCP = tf.inverse * torque
    # this is in the non-rotated TCP frame. Must rotate it into TCP
    return np.concatenate([force, torque])

def force_mode_tool(robot, torque, limits, setRemote=True):
    """BUGGED"""
    # ### bugged I think
    # use this if you're specifying the force mode w.r.t. the tool frame, and want to skip
    # the extra calculations below (quicker control rate)
    pose = robot.get_actual_tcp_pose()
    if setRemote:
        robot.set_force_remote(pose, [1]*6, torque, limits, 2)
    else:
        robot.force_mode(pose, [1]*6, torque, 2, limits)

def force_mode(robot=None, tf=MOUNT_TO_BASE, pose_base=None, torque=None, limits=None, selection_vector=None, setRemote=True):
    """python wrapper to enter UR force mode. Use setRemote True if you want the force mode to be updated rather than [re]started.
    
    ```python
    Input: ur-interface robot object, target pose in base frame, list-like (6,) torque vector, list-like (6,) limits, bool if to set remote
    Output: void
    ```"""
    pose_mount = tf.inverse * pose_base # pose vector is always w.r.t. base, so must transform into mount-frame
    if selection_vector is None:
        selection_vector = [1]*6

    if isinstance(robot, urx.Robot):
        print("force_mode: Input must be rope-robotics.")
        return
    else:
        if setRemote:
            robot.set_force_remote(pose_mount.pose_vector, selection_vector, torque, limits, 2)
        else:
            robot.force_mode(pose_mount.pose_vector, selection_vector, torque, 2, limits)

def get_speed(robot=None, tf=MOUNT_TO_BASE, inMountFrame=False):
    """python wrapper for get TCP speed
    
    ```python
    Input: ur-interface robot object
    Output: np.array (6,) velocity in base frame
    ```"""
    if isinstance(robot, urx.Robot):
        print("Input must be a rope-robotics. Returning.")
        return
    else:
        speed_mount = np.array(robot.get_actual_tcp_speed())
    if not inMountFrame:
        speed_base = tf.orient * speed_mount # must be the orientation because speed_mount is VECTOR6D
        return speed_base
    else:
        return speed_mount

def force_stop2(rob, ropeRob, timeout=1.0):
    # stop the robot by controlling the max speed allowed
    # linear interpolation of speed from max to zero in timeout
    initSpeed = get_speed(ropeRob, inMountFrame=True)


    t0 = time.time()
    def fxn():
        return time.time() - t0

    controller = linearControl(fxn(), timeout, fxn, initSpeed, np.zeros(6))
    while fxn() < timeout: 
        limits = controller.getControl()
        force_mode(robot=ropeRob, tf=m3d.Transform(), pose_base=m3d.Transform(), torque=[0.]*6, limits=limits, setRemote=True)

    # clear the force mode.
    force_mode(robot=ropeRob, tf=m3d.Transform(), pose_base=m3d.Transform(), torque=[0.]*6, limits=[0.]*6, setRemote=True) 
    time.sleep(0.05)
    #ropeRob.stopl(a=0.5)
    print("force_stop2: returning normally.")

def fill_pose_stamped(pose, frame_id):
    """ fills pose stamped with time now
    Assumes input is either m3d.Transform or a ros Pose()
    """
    if isinstance(pose, m3d.Transform):
        msg = PoseStamped(pose = m3d_to_ros(pose))
    else:
        msg = PoseStamped(pose = pose)
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    return msg

def fillTF(vec_pos, quat, t):
    """fills a ROS tf from position and quat data

    ```python
    Input: list-like (3,) of position, list-like (4,) of quaternion, ROS geometry_msgs/TransformStamped Message
    Output: void
    ```"""
    if isinstance(t, TransformStamped):
        t.transform.translation.x = vec_pos[0] 
        t.transform.translation.y = vec_pos[1]
        t.transform.translation.z = vec_pos[2]
        t.transform.rotation.w    = quat[0]
        t.transform.rotation.x    = quat[1]
        t.transform.rotation.y    = quat[2]
        t.transform.rotation.z    = quat[3]
    elif isinstance(t, Pose):
        t.position.x = vec_pos[0] 
        t.position.y = vec_pos[1]
        t.position.z = vec_pos[2]
        t.orientation.w    = quat[0]
        t.orientation.x    = quat[1]
        t.orientation.y    = quat[2]
        t.orientation.z    = quat[3]


'''
class ReferenceSystemExt(ReferenceSystem):

    def update_frame(self, frame_name, xform):
        if frame_name in self._frames:
            raise self.Error('Can not find frame name.')
        self._frames[frame_name]._xform = xform
'''   

def ROStoM3d_orient(rot):
    """ input is a quaternion in ROS units """
    if isinstance(rot, list):
        quat = m3d.UnitQuaternion(rot[3], rot[0], rot[1], rot[2]) # s, xyz
    elif isinstance(rot, Quaternion):
        quat = m3d.UnitQuaternion(rot.w, rot.x, rot.y, rot.z)
    return m3d.Orientation(quat)

def ROStoM3d(trans, rot):
    """converts ROS tf to m3d tf. Careful because ROS defines quats as (x, y, z, s), and m3d uses (s, x, y, z) notation

    ```python
    Input: list-like (3,) translation (position), list-like (4,) quaternion
    Output: math3d.Transform object
    ```"""
    quat = ROStoM3d_orient(rot)
    t = m3d.Transform(quat, trans)
    return t

def ros_to_m3d(pose):
    trans = [pose.position.x, pose.position.y, pose.position.z]
    rot = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    return ROStoM3d(trans, rot)


def geometry_msg_to_m3d(pose):
    """ converts geometry msg to m3d transform"""
    tf = m3d.Transform()
    tf.pos= (pose.position.x, pose.position.y, pose.position.z)
    quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    tf.set_orient(m3d.UnitQuaternion(quat[3], quat[0], quat[1], quat[2]).orientation)
    return tf

def transform_to_m3d(pose):
    tf = m3d.Transform()
    tf.pos= (pose.translation.x, pose.translation.y, pose.translation.z)
    quat = [pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w]
    tf.set_orient(m3d.UnitQuaternion(quat[3], quat[0], quat[1], quat[2]).orientation)
    return tf
    

def bbox_to_ros(bbox):
    msg = Marker( type = Marker.CUBE )
    center = np.array(bbox[0:3]) + 0.5 * np.array(bbox[3:6])
    msg.pose.position.x = center[0] # center
    msg.pose.position.y = center[1]
    msg.pose.position.z = center[2]
    #
    msg.scale.x = bbox[3] # size
    msg.scale.y = bbox[4]
    msg.scale.z = bbox[5]
    return msg

def m3d_to_ros(tf):
    pose = Pose()
    fillTF(tf.pos, tf.orient.quaternion, pose)
    return pose

def transformPoint(topic1, topic2, transformer, ptARR):
    """ all in ROS """
    ps = PointStamped()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = topic1
    ps.point.x = ptARR[0]; ps.point.y = ptARR[1]; ps.point.z = ptARR[2]
    transformer.waitForTransform(topic1, topic2, ps.header.stamp, rospy.Duration(0.1))
    psOut = transformer.transformPoint(topic2, ps)
    pts = psOut.point
    outARR = (pts.x, pts.y, pts.z)
    return outARR

def transformPointM3D(topic1, topic2, transformer, ptARR, freeVector=False):
    # transforms ptARR in frame topic1 to the frame topic2
    (trans, rot) = transformer.lookupTransform(topic1, topic2, rospy.Time(0))
    topic2_to_topic1 = ROStoM3d(trans, rot)
    vector = m3d.Vector(ptARR)
    if freeVector:
        out = topic2_to_topic1.inverse.orient * vector # free vectors don't have position, only orientation
    else:
        out = topic2_to_topic1.inverse * vector
    return tuple(out)

def transformPoseM3D(topic1, topic2, transformer, pose):
    # transforms pose in frame topic1 to the frame topic2
    (trans, rot) = transformer.lookupTransform(topic1, topic2, rospy.Time(0))
    topic2_to_topic1 = ROStoM3d(trans, rot)
    out = tf_helper(topic2_to_topic1, pose, to_TF=True)
    return out

def rotate3V(arr, tf, to_TF=False):
    if not isinstance(arr, m3d.Vector):
        sv = m3d.Vector(arr)
    else:
        sv = arr
    if to_TF: # rotate vec3 into tool-frame from base-frame
        sv_out = tf.orient.inverse * sv
    else:
        sv_out = tf.orient * sv
    return sv_out.array
    
def rotate6V(vec6, tf, to_TF):
    a1 = vec6[0:3]
    a2 = vec6[3:6]
    out = np.append(rotate3V(a1, tf, to_TF), rotate3V(a2, tf, to_TF))
    return out

def rotate6V(vec6, tf, to_TF):
    a1 = vec6[0:3]
    a2 = vec6[3:6]
    out = np.append(rotate3V(a1, tf, to_TF), rotate3V(a2, tf, to_TF))
    return out

def transform_vec(vec3, to_TF):
    """
        Transform vector between two frames using m3d transfom
    """
    p_a = m3d.Vector(vec3)
    T_AB = to_TF.pos  
    R_AB = to_TF.orient

    p_b = R_AB*(p_a - T_AB)

    return p_b

def pub_tf(tf, parent, child):
        """
        Publishes a static transform between parent and child
        Args:
            parent: Name of the parent frame
            child: Name of the child frame
        Returns
        """
        tf_child_to_parent = tf
        parent_frame = TransformStamped()
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        parent_frame.header.stamp = rospy.Time.now()
        parent_frame.header.frame_id = parent
        parent_frame.child_frame_id = child

        parent_frame.transform.translation.x = tf_child_to_parent.pos.x
        parent_frame.transform.translation.y = tf_child_to_parent.pos.y
        parent_frame.transform.translation.z = tf_child_to_parent.pos.z

        quat = list(tf_child_to_parent.orient.get_unit_quaternion())
        parent_frame.transform.rotation.x = quat[1]
        parent_frame.transform.rotation.y = quat[2]
        parent_frame.transform.rotation.z = quat[3]
        parent_frame.transform.rotation.w = quat[0]
        static_tfs = []
        static_tfs.append(parent_frame)
        broadcaster.sendTransform(static_tfs)

def lookup_transform_m3d(tf_AB, tf_BC):
    """
        Given tf between frame A and B, and frame B and C, the function generates
        tf between frame A and C.
    """
    tf_AC = m3d.Transform()

    tf_AC.orient = tf_AB.orient * tf_BC.orient
    tf_BA = tf_AB.inverse
    # tf_AC.pos = transform_vec(tf_BC.pos, tf_BA)
    tf_AC.pos = rotate3V(tf_BC.pos, tf_BA, to_TF = True)
    tf_AC.pos = tf_AB.pos + tf_AB.orient * tf_BC.pos 

    return tf_AC

def tf_array(tf, arr, to_TF=False):
    """ assumes input array is order [n, 3]
    """
    v = np.ones([arr.shape[0], 1])
    new_arr = np.concatenate([arr, v], axis=1)
    sv = np.array(new_arr).T
    if to_TF: # rotate vec3 into tool-frame from base-frame
        mat = tf.inverse.array
    else:
        mat = tf.array
    sv_out = np.matmul(mat, sv).T
    return sv_out[:, 0:3] # get rid of last column of all ones

def tf_helper(tf, inp, to_TF=False):
    """ Take the input (inp) and transform it w.r.t. 'tf'.
    Assumes that if input is NOT a transform, then it's a free-vector
    which means that ONLY rotation is performed, and NOT translation too
    invert False means that input is already in the tf frame, and must be transformed out of it
    """
    if isinstance(inp, m3d.Transform):
        if to_TF:
            pose_out = tf.inverse * inp
        else:
            pose_out = tf * inp
        return pose_out
    else:
        if to_TF:
            out = tf.orient.inverse * inp # base -> mount
        else:
            out = tf.orient * inp # mount -> base
        if isinstance(out, m3d.Vector):
          return out.array
        else:
          return out

def TCP_BASE_TF(ropeRobOrTF, vec, toTCP=False):
    if not isinstance(ropeRobOrTF, m3d.Transform):
        tr = get_pose(ropeRobOrTF, inMountFrame=False) # tool -> base
    else:
        tr = ropeRobOrTF
    return tf_helper(tr, vec, to_TF=toTCP)

def TCP_MOUNT_TF(ropeRobOrTF, vec, toTCP=False):
    if not isinstance(ropeRobOrTF, m3d.Transform):
        tr = get_pose(ropeRobOrTF, inMountFrame=True) # tool -> mount
    else:
        tr = ropeRobOrTF
    return tf_helper(tr, vec, to_TF=toTCP)

def BASE_MOUNT_TF(inp, toMount=False):
    return tf_helper(MOUNT_TO_BASE, inp, to_TF=toMount)

"""Ivmech PID Controller is simple implementation of a Proportional-Integral-Derivative (PID) Controller in the Python Programming Language.
More information about PID Controller: http://en.wikipedia.org/wiki/PID_controller

https://github.com/ivmech/ivPID
"""

def URcontrolDeadband(control, limitsDefault, deadband, torqueMag=50.0):
    """Generate the torque and limits subject to the deadband - minimum allowed control before clipping to zero

    ```python
    Input: np.array (6,) [Fx, Fy, Fz, Mx, My, Mz], np.array (6,) default UR10 limits vector, np.array (6,) deadband
    Output: np.array (6,) [Fx, Fy, Fz, Mx, My, Mz], np.array(6,) limits vector
    ```"""
    torqueMag = np.clip(torqueMag, -150.0, 150.0)
    torque = np.zeros_like(control)
    limits = np.zeros_like(limitsDefault)
    np.copyto(torque, control)
    np.copyto(limits, limitsDefault)
    limits[3:6] = abs(torque[3:6]) #convert to UR5/10 input
    torque[3:6] = torqueMag*np.sign(control[3:6]) # linear is max 225, but angular is anything.
    #deadband
    idxDB = abs(control) < deadband
    torque[idxDB] = 0.0 # dead band
    limits[idxDB] = 0.0
    return torque, limits

class UR_FORCE_PID:
    """PID Controller
    """

    def __init__(self, numParams = 6):
        self.numParams = numParams
        self.Kp = np.zeros(numParams)
        self.Ki = np.zeros(numParams)
        self.Kd = np.zeros(numParams)
        self.maxCmd = None

        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time
        self.windup_guard = None

        # limits
        maxForce = 225.0 # Newtons
        maxTorque = 0.5 #is actually the rotation speed
        self.maxControl = np.array([maxForce]*3+[maxTorque]*3, dtype='float')
        self.deadband = np.zeros(numParams)
        limitSpeed = 0.15 # in m/s
        limitAngSp = 0.1 #this number always gets overwritten. Doesn't matter.
        self.limitsDefault = np.array([limitSpeed]*3+[limitAngSp]*3, dtype='float')

        # specify rotation torque too
        self.setTorque = False
        self.torquePTerm = 1.0

        # time step between errors
        self.delta_time = 0.0

        self.clear()

    def clear(self):
        """Clears PID computations and coefficients"""
        numParams = self.numParams
        self.PTerm = np.zeros(numParams)
        self.ITerm = np.zeros(numParams)
        self.DTerm = np.zeros(numParams)
        self.control_p = None
        self.control_i = None
        self.control_d = None
        self.last_error = np.zeros(numParams)

        self.current_time = time.time()
        self.last_time = self.current_time

        # Windup Guard
        #self.int_error = 0.0

        self.control = np.zeros(numParams)
        self.controls = [] #fill with control outputs
        self.times = []
        self.errors = []

    def update(self, error):
        """Calculates PID value for given reference feedback
        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}
        .. figure:: images/pid_1.png
           :align:   center
           Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)
        """
        if len(error) != self.numParams:
            print("UR_FORCE_PID: Incorrect error size. Returning")
            return
        else:
            error = np.array(error) # can input lists, but not single numbers

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = error
            self.ITerm += error * delta_time

            # if self.windup_guard is not None:
            #     self.ITerm = np.clip(self.ITerm, -1.0 * self.windup_guard, self.windup_guard) # acts as a saturation on the integral
            

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / float(delta_time)

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            if self.setTorque:
                torqueMag = abs(self.torquePTerm * self.PTerm[3:6])
            else:
                torqueMag = 40.0

            mult = np.multiply

            self.control_p = mult(self.Kp, self.PTerm)
            self.control_i = mult(self.Ki, self.ITerm)
            self.control_d = mult(self.Kd, self.DTerm)
            self.control =  self.control_p + self.control_i + self.control_d
            self.control = self.control.clip(-1.*self.maxControl, self.maxControl) # saturate the controls for safety
            if len(self.control) == 6:
                self.torque, self.limits = URcontrolDeadband(self.control, self.limitsDefault, self.deadband, torqueMag=torqueMag)
            else: # still need to do the deadband
                idxDB = abs(self.control) < self.deadband
                self.control[idxDB] = 0.0
            self.controls.append(self.control)
            self.times.append(self.current_time)
            self.errors.append(error)
    
    def update_new(self, error):
        """Calculates PID value for given reference feedback
        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}
        .. figure:: images/pid_1.png
           :align:   center
           Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)
        """
        if len(error) != self.numParams:
            print("UR_FORCE_PID: Incorrect error size. Returning")
            return
        else:
            error = np.array(error) # can input lists, but not single numbers

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        self.delta_time = delta_time
        # rospy.loginfo("DELTA_TIME = {}".format(self.delta_time))
        delta_error = error - self.last_error
        # rospy.loginfo("ERROR = {}".format(error))

        if (delta_time >= self.sample_time):
            self.PTerm = error
            self.ITerm += error * delta_time

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / float(delta_time)

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            mult = np.multiply

            self.control_p = mult(self.Kp, self.PTerm)
            # rospy.loginfo("CONTROL P = {}".format(self.control_p))
            self.control_i = mult(self.Ki, self.ITerm)
            self.control_d = mult(self.Kd, self.DTerm)
            self.control =  self.control_p + self.control_i + self.control_d
            # rospy.loginfo("TOTAL CONTROL = {}".format(self.control))
            self.controls.append(self.control)
            self.times.append(self.current_time)
            self.errors.append(error)

    def getForceModeInput(self):
        return self.torque, self.limits

    def getControl(self):
        return self.control

    def setKp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        if len(proportional_gain) != self.numParams:
          print("UR_PID: Wrong Size. Must be %s long."% self.numParams)
        else:
          self.Kp = np.array(proportional_gain)

    def setKi(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        if len(integral_gain) != self.numParams:
          print("UR_PID: Wrong Size. Must be %s long."% self.numParams)
        else:
          self.Ki = np.array(integral_gain)

    def setKd(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        if len(derivative_gain) != self.numParams:
          print("UR_PID: Wrong Size. Must be %s long."% self.numParams)
        else:
          self.Kd = np.array(derivative_gain)

    def setWindup(self, windup):
        """Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        """
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sample_time = sample_time

class linearControl(object):
    """Linear controller
    """

    def __init__(self, Imin, rampVal, indptFxn, minCommand, maxCommand):
        """After initialize, call self.getControl() for the control.
        must provide a callable function, indptFxn, which provides the current indepent value.

        ```python
        Imin: independent minimum. If None, is created from indptFxn
        rampVal: add to Imin to get Imax
        indptFxn: callable function to generate the indpt value.
        minCommand: minimum command (dependent value)
        maxCommand: maximum command (dependent value)
        ```"""
        self.rampVal = rampVal
        self.indptFxn = indptFxn
        if Imin is not None:
            self.Imin = Imin
        else:
            self.Imin = self.indptFxn()
        self.Imax = self.Imin + self.rampVal
        self.minCommand = minCommand
        self.maxCommand = maxCommand


    def getControl(self):
        """Call to get the new control"""
        total = self.indptFxn()
        control = linear_interp(self.Imin, self.Imax, self.minCommand, self.maxCommand, total)
        return control


def linear_interp(Imin, Imax, Omin, Omax, x):
    """Linear interpolation. Saturated to Omin and Omax when x  < Imin or x > Imax respectively

    ```python
    Imin: independent minimum
    Imax: independent maximum
    Omin: output minimum
    Omax: output maximum
    x: current independent value
    ```"""
    #make sure all are numpy
    Imin = np.array(Imin, dtype='float64')
    Imax = np.array(Imax, dtype='float64')
    Omin = np.array(Omin, dtype='float64')
    Omax = np.array(Omax, dtype='float64')
    x    = np.array(x,    dtype='float64')
    if abs(Imax - Imin) < 0.001: #divide by zero
        alpha = 1.0
    else:
        alpha = (x - Imin) / (Imax - Imin)
    if alpha > 1.0:
        alpha = 1.0
    elif alpha < 0.0:
        alpha = 0.0
    #linear slider
    out = Omin * (1.0-alpha) + Omax * alpha
    return out.tolist()


def factorization(n):
  """
  """
  for i in range(int(math.sqrt(float(n))), 0, -1):
    if n % i == 0:
      return (i, int(n / i))



class MultiImage(object):
    """
    """
    def __init__(self):
        """
        """
        self.parula = get_parula()

        #ROS callback
        self.numCams = 2
        imageSubs = []
        for i in range(1, self.numCams+1):
            string = "/camera_0"+str(i)+"/rgb/image_raw"
            imageSubs.append( message_filters.Subscriber(string, Image) )

        self.ts = message_filters.ApproximateTimeSynchronizer(imageSubs, 10, 1.0)
        self.ts.registerCallback(self.imageListener)

        #video
        self.fourcc = cv2.cv.CV_FOURCC(*'XVID')
        #self.out = cv2.VideoWriter('./output.avi',self.fourcc, 30.0, (640,480))

    def close(self):
        """
        """
        #self.out.release()
        cv2.destroyAllWindows()

    def imageListener(self, *args):
        """
        """
        bridge = CvBridge()
        images = []
        for data in args: #variable input arg list, but each one should be an 'image' data object
            try:
                cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)

            (h, w) = cv_image.shape[:2]
            center = (w / 2, h / 2)
             
            # rotate the image by 180 degrees
            M = cv2.getRotationMatrix2D(center, 180, 1.0)
            rotated = cv2.warpAffine(cv_image, M, (w, h))
            images.append( rotated )


        (grid_Y, grid_X) = factorization (len(images)) #concat.shape[0])
        final = []
        count = 0
        for i in range(grid_X):
            row = []
            for j in range(grid_Y):
                row.append(images[count])
                count += 1
            final.append(row)

        concat = np.hstack(final)
        squeezed = np.squeeze(concat)

        cv2.namedWindow("Smart Chisel", cv2.cv.CV_WINDOW_NORMAL)
        #cv2.setWindowProperty("Smart Chisel",cv2.WND_PROP_FULLSCREEN, 1)
        cv2.imshow("Smart Chisel", squeezed)
        k = cv2.waitKey(3) % 2**16 
        if k == 27:
            rospy.signal_shutdown("User Pressed Escape.")


def depthToCV2Depth(depth_array):
    """
    """
    depth_output = np.zeros_like(depth_array)
    cv2.normalize(depth_array, depth_output, 0, 255, cv2.NORM_MINMAX) # range 0 to 255
    return depth_output

class image_depth:
    """
    """
    def __init__(self):
        """
        """
        #make pygame window -- for now just try out cv2 w/ make circle
        self.parula = get_parula()
        
        #ROS callback
        image_sub = message_filters.Subscriber("camera/rgb/image_raw", Image)
        depth_sub = message_filters.Subscriber("camera/depth/image", Image)

        self.ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], 10, 0.5)
        self.ts.registerCallback(self.image_depth_listener)

        #video
        self.fourcc = cv2.cv.CV_FOURCC(*'XVID')
        self.out = cv2.VideoWriter('./output.avi',self.fourcc, 30.0, (640,480))

    def close(self):
        """
        """
        self.out.release()
        cv2.destroyAllWindows()
        
    def image_depth_listener(self, rgb_data, depth_data):
        """
        """
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(rgb_data, "bgr8")
            depth_array = bridge.imgmsg_to_cv2(depth_data, "passthrough")
        except CvBridgeError as e:
            print(e)
        depth_array = np.nan_to_num(depth_array)
        depth_array -= 0.3937 #offset from orbec to chisel tip
        depth_array[depth_array < 0.0] = 0.0
        depth_array[depth_array > 1.0] = 1.0 #1 meter, should be good

        #images
        cv2.namedWindow("Smart Chisel", cv2.cv.CV_WINDOW_NORMAL)
        cv2.setWindowProperty("Smart Chisel",cv2.WND_PROP_FULLSCREEN, 1)
        parula_map = self.parula
        lenCM = len(parula_map)
        #draw sphere
        width, height = depth_array.shape
        nums = 20
        dw = int(width / nums)
        dh = int(height / nums)
        Ws = range(dw/2, width, dw)
        Hs = range(dh/2, height, dh)
        subD = depth_array[np.ix_(Ws, Hs)]
        minD = np.min(subD)
        maxD = np.max(subD)

        for w in Ws:
            for h in Hs:
                colorIdx = int(linear_interp(minD, maxD, 0, lenCM-1, depth_array[w,h]))
                color = parula_map[colorIdx]
                cv2.circle(cv_image, (h, w), 2, tuple(color), -1)
              
        #pdb.set_trace()
        cv2.imshow("Smart Chisel", cv_image)
        self.out.write(cv_image)
        #cv2.waitKey(3)

        depth_output = depthToCV2Depth(depth_array)
        #cv2.imshow("Depth", depth_output)
        k = cv2.waitKey(3) % 2**16 
        if k == 27:
            rospy.signal_shutdown("User Pressed Escape.")

def xyOverlayImage(cv_image, x, y, color=(0, 255, 0)):
    """
    """
    (h, w) = cv_image.shape[:2] # here I switch around height and width
    sx = int(x*w)
    sy = int(y*h)
    cv2.circle(cv_image, (sx, sy), 4, color, -1)  # green & filled
    return cv_image
        
def showImage(cv_image):
    """
    """
    cv2.imshow("image", cv_image)
    k = cv2.waitKey(3) % 2**16 
    if k == 27:
        rospy.signal_shutdown("User Pressed Escape.")

def rotateImage180(cv_image):
    """
    """
    (h, w) = cv_image.shape[:2]
    center = (w / 2, h / 2)
    # rotate the image by 180 degrees
    M = cv2.getRotationMatrix2D(center, 180, 1.0)
    rotated = cv2.warpAffine(cv_image, M, (w, h))
    return rotated
        
def image_listener(data):
    """
    """
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    rotated = rotateImage180(cv_image)

    #cv2.namedWindow("Smart Chisel", cv2.WINDOW_NORMAL)
    #cv2.setWindowProperty("Smart Chisel",cv2.WND_PROP_FULLSCREEN, 1)
    showImage(rotated)


def get_parula():
    """
    """
    #from matplotlib.colors import LinearSegmentedColormap

    cm_data = np.array([[0.2081, 0.1663, 0.5292], [0.2116238095, 0.1897809524, 0.5776761905], 
     [0.212252381, 0.2137714286, 0.6269714286], [0.2081, 0.2386, 0.6770857143], 
     [0.1959047619, 0.2644571429, 0.7279], [0.1707285714, 0.2919380952, 
      0.779247619], [0.1252714286, 0.3242428571, 0.8302714286], 
     [0.0591333333, 0.3598333333, 0.8683333333], [0.0116952381, 0.3875095238, 
      0.8819571429], [0.0059571429, 0.4086142857, 0.8828428571], 
     [0.0165142857, 0.4266, 0.8786333333], [0.032852381, 0.4430428571, 
      0.8719571429], [0.0498142857, 0.4585714286, 0.8640571429], 
     [0.0629333333, 0.4736904762, 0.8554380952], [0.0722666667, 0.4886666667, 
      0.8467], [0.0779428571, 0.5039857143, 0.8383714286], 
     [0.079347619, 0.5200238095, 0.8311809524], [0.0749428571, 0.5375428571, 
      0.8262714286], [0.0640571429, 0.5569857143, 0.8239571429], 
     [0.0487714286, 0.5772238095, 0.8228285714], [0.0343428571, 0.5965809524, 
      0.819852381], [0.0265, 0.6137, 0.8135], [0.0238904762, 0.6286619048, 
      0.8037619048], [0.0230904762, 0.6417857143, 0.7912666667], 
     [0.0227714286, 0.6534857143, 0.7767571429], [0.0266619048, 0.6641952381, 
      0.7607190476], [0.0383714286, 0.6742714286, 0.743552381], 
     [0.0589714286, 0.6837571429, 0.7253857143], 
     [0.0843, 0.6928333333, 0.7061666667], [0.1132952381, 0.7015, 0.6858571429], 
     [0.1452714286, 0.7097571429, 0.6646285714], [0.1801333333, 0.7176571429, 
      0.6424333333], [0.2178285714, 0.7250428571, 0.6192619048], 
     [0.2586428571, 0.7317142857, 0.5954285714], [0.3021714286, 0.7376047619, 
      0.5711857143], [0.3481666667, 0.7424333333, 0.5472666667], 
     [0.3952571429, 0.7459, 0.5244428571], [0.4420095238, 0.7480809524, 
      0.5033142857], [0.4871238095, 0.7490619048, 0.4839761905], 
     [0.5300285714, 0.7491142857, 0.4661142857], [0.5708571429, 0.7485190476, 
      0.4493904762], [0.609852381, 0.7473142857, 0.4336857143], 
     [0.6473, 0.7456, 0.4188], [0.6834190476, 0.7434761905, 0.4044333333], 
     [0.7184095238, 0.7411333333, 0.3904761905], 
     [0.7524857143, 0.7384, 0.3768142857], [0.7858428571, 0.7355666667, 
      0.3632714286], [0.8185047619, 0.7327333333, 0.3497904762], 
     [0.8506571429, 0.7299, 0.3360285714], [0.8824333333, 0.7274333333, 0.3217], 
     [0.9139333333, 0.7257857143, 0.3062761905], [0.9449571429, 0.7261142857, 
      0.2886428571], [0.9738952381, 0.7313952381, 0.266647619], 
     [0.9937714286, 0.7454571429, 0.240347619], [0.9990428571, 0.7653142857, 
      0.2164142857], [0.9955333333, 0.7860571429, 0.196652381], 
     [0.988, 0.8066, 0.1793666667], [0.9788571429, 0.8271428571, 0.1633142857], 
     [0.9697, 0.8481380952, 0.147452381], [0.9625857143, 0.8705142857, 0.1309], 
     [0.9588714286, 0.8949, 0.1132428571], [0.9598238095, 0.9218333333, 
      0.0948380952], [0.9661, 0.9514428571, 0.0755333333], 
     [0.9763, 0.9831, 0.0538]])
    cm_data *= 255.0 #scale

    #parula_map = LinearSegmentedColormap.from_list('parula', cm_data)
    return cm_data.tolist()

class SensorFeedbackControl(object):
    """DEPRECATED
    """
    def __init__(self, arm_client):
        # NOTE: the target is w.r.t. the tool-frame

        numParams = 6
        self.numParams = numParams
        self.target = np.zeros(numParams) # default is you bring the force sensor to zero
        assert (arm_client is not None)
        self.arm_client = arm_client


        self.PID = UR_FORCE_PID()
        self.PID.setTorque = True # set the force mode torque as well as rotation speed
        self.PID.torquePTerm = 6.0
        # gains
        Pl = 0.5
        Pa = 0.01
        P = np.array([Pl]*3+[Pa]*3, dtype='float')
        self.PID.setKp(P)
        deadForce = 15. # N
        deadTorque = 0.05 # rad/s
        self.PID.deadband = np.array([deadForce]*3+[deadTorque]*3, dtype='float')
        limitSpeed = 0.1 # in m/s
        limitAngSp = 0.1 #this number always gets overwritten. Doesn't matter.
        self.PID.limitsDefault = np.array([limitSpeed]*3+[limitAngSp]*3, dtype='float')
        maxForce = 225.0 # Newtons
        maxTorque = 0.3 #is actually the rotation speed
        self.maxControl = np.array([maxForce]*3+[maxTorque]*3, dtype='float')

    def reset(self): # is done by URmonitor at beginning
        self.arm_client.reset_FT300()

    def update(self):
        force = self.arm_client.get_force()
        self.error = self.target - -1.*force # reverse force direction because want to move in direction of force vector
        self.PID.update(self.error)

    def main(self, timeOut = 1.0):
        """ Bring total torque to zero through PID control
        """

        self.update()
        torque, limits = self.PID.getForceModeInput()

        t0 = time.time()
        tol = 1e-4
        while (time.time()-t0) < timeOut and np.sum(abs(torque)) > tol: #while at least one direction is not deadband
            self.update()
            torque, limits = self.PID.getForceModeInput()
            print '\nerror: ', self.error
            print 'torque', torque, 'limits', limits
            time.sleep(0.1)
            # TODO: pass in mount -> base tf
            ### force_mode(self.ropeRob,  tf=MOUNT_TO_BASE, pose_base=self.arm_client.get_pose(), torque=torque, limits=limits, selection_vector=np.ones(6), setRemote=True)

class SensorFeedbackControl2(SensorFeedbackControl):
    """ sets error to 'target - force'
    """
    def update(self):
        force = self.arm_client.get_force()
        self.error = self.target - force
        self.PID.update(self.error)

def closest_pt_to_array(array, pt):
    # get closest point on virtual spline
    distToSpline = np.linalg.norm(array - pt, axis=1) # broadcasts
    idx = (np.abs(distToSpline)).argmin() # scale from distance to spline control -- auto protects against segfault
    return idx, distToSpline
