# ROS messages
import sensor_msgs
from sensor_msgs.msg import Image, PointCloud2 as pc2, PointField
from geometry_msgs.msg import PoseStamped, PointStamped, PoseArray, Wrench
from visualization_msgs.msg import Marker
import std_msgs
from std_msgs.msg import Float32MultiArray, Bool, Int32
from std_msgs.msg import Empty as Empty_msg
from std_srvs.srv import Empty as Empty_srv
import image_geometry

#
import yaml
from enum import Enum

from itertools import groupby
import tf as ROS_TF

# actionlib server stuff
from bot_common_ros.msg import ARMAction as Action, ARMGoal as Goal, ARMFeedback as Feedback, ARMResult as Result
from bot_common_ros.msg import ActionType, ProcessStatus
from bot_common_ros.actions import FSMActionServer
from bot_common_ros.arm_client import ArmClient
from bot_chisel.msg import ChiselStatus

import URBasic
import urx
from bot_common_ros.ur_control import *
from bot_common_ros.ur_control import URMiddleWare, getAngleLimit, TimeOut, Mapping, QueryNormal, WaitForActive, CheckDistance, getDist, CheckSphere, RayIntersect, RayIntersect_ARR, force_touch3
from bot_common_ros.ur_utils import TCP_BASE_TF, getHorizontalOrient, m3d_to_ros, linearControl, rotate3V, rotate6V, UR_FORCE_PID, rosParamToClassObject, closest_pt_to_array, SensorFeedbackControl, SensorFeedbackControl2, ROStoM3d, fill_pose_stamped
from bot_common_ros.ur_utils import *


class ChiselForceControl(object):

    def __init__(self):
                
        self.wrench_tcp = Wrench()
        self.tf_listener = ROS_TF.TransformListener()

    def get_tool_euler_angles(self, tf_target, orient):
        """ Method to return euler angles to rotate from target-frame to tcp frame
        """
        euler = ((tf_target.inverse).orient * orient).to_euler('YXZ')
        return euler

    def get_tf_base_tcp(self):

        (trans, rot) = self.tf_listener.lookupTransform("chisel_base", "chisel_TCP", rospy.Time(0))
        return ROStoM3d(trans, rot)

    def set_rotation_extrema(self, min, max):
        self.min_angles = min
        self.max_angles = max

    def init_PID_controllers(self):
        """ Method that initializes the controller objects based on param calcuclations.
        """

        ctrl_gains = rospy.get_param('/chisel_ctrl/attack_ctrl_gains')
        K_row_follower = ctrl_gains['row_follower']['P']
        K_force = ctrl_gains['force']['P']
        Kp_angle = ctrl_gains['angle_yaw']['P']
        Kp_angle_pitch = ctrl_gains['angle_pitch']['P']
        Kd_angle_pitch = ctrl_gains['angle_pitch']['D']
        
        PID_row_follower  = UR_FORCE_PID(numParams = 1) # protection against over-torqueing, operate on tool-y torque only
        PID_force  = UR_FORCE_PID(numParams = 2) # protection against over-torqueing, operate on tool-y torque only
        PID_angle  = UR_FORCE_PID() # lower level angle --> force_mode input
        #
        PID_row_follower.setKp([K_row_follower])
        PID_row_follower.maxControl = np.array([30./180.*math.pi])
        #
        PID_force.setKp([K_force, K_force])
        PID_force.maxControl = np.array([40./180.*math.pi])
        #
        # angle control limits
        maxForce = 150.0 # Newtons
        maxTorque = 0.55 #is actually the rotation speed
        maxControl = np.array([maxForce]*3+[maxTorque]*3, dtype='float')
        deadForce = 5.0
        deadTorque = 0.025  # is actually rotation speed
        deadband = np.array([deadForce]*3+[deadTorque]*3, dtype='float')
        limitSpeed = 0.15 # in m/s
        limitAngSp = 0.15 #this number always gets overwritten. Doesn't matter.
        limitsDefault = np.array([limitSpeed]*3+[limitAngSp]*3, dtype='float')
        PID_angle.setKp( np.array([0,0,225.0]+[Kp_angle_pitch, Kp_angle, 0.0]) )
        PID_angle.setKd( np.array([0, 0, 0] + [Kd_angle_pitch, 0, 0]))
        PID_angle.maxControl    = maxControl
        PID_angle.deadband      = deadband
        PID_angle.limitsDefault = limitsDefault

        self.PID_row_follower = PID_row_follower
        self.PID_force = PID_force
        self.PID_angle = PID_angle 
        

    def checkAngleLimits(self, pose, maxAngle, tf_target, dir='y'):
        """ check angle limits using euler angles 
        RECALL: euler angles are form "YXZ", hence why idx are 1, 0, 2
        """

        euler_original = self.get_tool_euler_angles(tf_target, pose.orient)
        rospy.logdebug("current tool yaw = {}".format(euler_original[0] * 57.0))
        new_pose = pose.copy()
        if dir == "x":
            new_pose.orient.rotate_xt( maxAngle)
            idx = 1
        elif dir == "y":
            rospy.logdebug("Rotating about tool y by {} degrees".format(maxAngle * 57.0))
            new_pose.orient.rotate_yt( maxAngle )
            idx = 0
        elif dir == "z":
            new_pose.orient.rotate_zt( maxAngle )
            idx = 2
        else:
            raise Exception("Invalid 'dir' input")
        #
        euler_new = self.get_tool_euler_angles(tf_target, new_pose.orient)
        
        euler_new_clipped = np.clip(euler_new, self.min_angles, self.max_angles)
        diff_euler = euler_new_clipped - euler_original
        if True:
            # rospy.logdebug("Euler-minn {}".format(57.0*minn))
            # rospy.logdebug("Euler-maxx {}".format(57.0*maxx))
            rospy.logdebug("Euler-Original {}".format(57.0*euler_original))
            rospy.logdebug("euler_new {}".format(57.0*euler_new))
            rospy.logdebug("euler_new_clipped {}".format(57.0*euler_new_clipped))
            rospy.logdebug("diff_euler {}".format(57.0*diff_euler))
        return diff_euler[idx]



    # force sensor error fxn
    def shrinkOperator(self, array, shrinkValue):
        sign = np.sign(array)
        array -= sign * shrinkValue # origin translation. Subtact if positive, add if negative (bring closer to zero)
        idx = (sign*array) < 0.0
        array[idx] = 0.0 
        return array

    def forceSensorError(self):
        shrinkValue = 5.0 # newton*m
        yTorq = -1.0 * self.wrench_tcp.torque.y
        xTorq = -1.0 * self.wrench_tcp.torque.x
        xTorq = self.shrinkOperator(np.array(xTorq), shrinkValue)
        yTorq = self.shrinkOperator(np.array(yTorq), shrinkValue)
        return xTorq, yTorq

    def tool_x_error(self):
        shrinkValue = 10.0 # newton*m
        tool_x_force = self.wrench_tcp.force.x
        tool_x_force = self.shrinkOperator(np.array(tool_x_force), shrinkValue)
        return tool_x_force

    def get_force_inputs(self, dir_to_chisel, nominal_angle, row_z_value, tf_target):
        """TO-DO: Must take nominal angle and direction to chisel as inputs
                    self.row_z_value as input as well
        """
        def angFxnUp():
            poseGlobal = self.get_tf_base_tcp().pose_vector
            poseTCP = rotate6V(poseGlobal, self.get_tf_base_tcp(), to_TF=True)
            return poseTCP[4]

        robotPose = self.get_tf_base_tcp()
        robotPos = robotPose.pos.array
        vecToAdd_tcp = m3d.Vector(0.0, 0.0, -0.03)
        vecToAdd_base = robotPose.orient * vecToAdd_tcp

        # calculate control due to distance FROM virtual spline
        normal = np.array([0, -1.0, 0])

        # force-sensor control
        force_error = np.array(self.forceSensorError()).flatten()
        self.PID_force.update(force_error) # tool-y and tool-x rotation
        [dAng_force_x, dAng_force_y] = self.PID_force.getControl()

        # transform into TCP frame -- delta-angle is in the base frame, so we must calculate the error in the tool frame
        euler_angles = self.get_tool_euler_angles(tf_target, robotPose.orient)  # order YXZ
        yaw = euler_angles[0]
        # target_yaw = dir_to_chisel * nominal_angle
        target_yaw = nominal_angle
        angle_tcp = target_yaw - yaw
        rospy.logdebug("angle-tcp: {}".format(angle_tcp * 57.0))
        rospy.logdebug("yaw: {}".format(yaw * 57.0))


        # limit it
        delta_angle = self.checkAngleLimits(robotPose, angle_tcp, tf_target)
        # delta_angle = angle_tcp
        rospy.logdebug("delta_angle: {}".format(delta_angle * 57.0))
        # add force control here, so that it doesn't get drowned out by dAng_line
        delta_angle += dAng_force_y 
        if math.isnan(delta_angle):
            return None, None
        delta_angle = self.checkAngleLimits(robotPose, delta_angle, tf_target) # limit again in case force is in wrong direction
        rospy.logdebug("delta_angle2: {}".format(delta_angle * 57.0))

        # calculate the angular error from the tangential drift
        tangential_drift_error = row_z_value - robotPos[2]
        self.PID_row_follower.update([tangential_drift_error])
        [dAng_drift] = self.PID_row_follower.getControl()
        rospy.logdebug("drift: {}".format(tangential_drift_error))
        rospy.logdebug("dAng_drift: {}".format(dAng_drift * 57.0))
        # Clamp it between a certain value, if drift isnt too large maybe 30 degrees?
        np.clip(dAng_drift, -np.deg2rad(30), np.deg2rad(30))

        delta_angle_x = dAng_drift
        # limit it
        delta_angle_x = self.checkAngleLimits(robotPose, dAng_drift, tf_target, dir="x")
        rospy.logdebug("delta_angle_x: {}".format(delta_angle_x * 57.0))

        # add force control here, so that it doesn't get drowned out by dAng_line
        delta_angle_x += dAng_force_x 
        if math.isnan(delta_angle_x):
            return None, None
        delta_angle_x = self.checkAngleLimits(robotPose, delta_angle_x, tf_target, dir="x") # limit again in case force is in wrong direction
        rospy.logdebug("delta_angle_x_post: {}".format(delta_angle_x * 57.0))

        # call force scoop with corrective delta-angle
        arr = np.array([0, 0, 1.] + [delta_angle_x, delta_angle, 0])
        self.PID_angle.update(arr)
        torque, limits = self.PID_angle.getForceModeInput()
        rospy.logdebug("torque: {}. limits: {}".format(torque, limits))
        rospy.logdebug("\n\n")
        return torque, limits