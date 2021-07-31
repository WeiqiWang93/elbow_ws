from bot_overseer_api import OverseerAPI
from bot_common_ros.ur_utils import list_to_tf, tf_to_list
import math3d as m3d
import time, pdb
import rospy

class ArmOverseer(object):
    """This class is a wrapper around overseer commands for the SIA20 driver plugin.
    Motion commands (smart_move, movejs/movels, stepl/stepj) are inserted into the robosaw_sia20_motions activity
    Instantaneous commands (set_speed, get_last_pt, stop) are ran as private_ops
    Critical commands (stop) will be run as critical_ops

    Initialize this class within an application (i.e. robo_saw_action_server) and run initialize_robot() to begin sending and receiving data.
    This class has been structured to work with existing code in the application layer so driver calls from the application layer do not need to be re-written.

    Usage example:
        arm = ArmOverseer()
        arm.initialize_robot(overseer=True)
        pose = arm.get_pose()
        arm.movejs(jointList=joints)
    """
    def __init__(self):
        self.arm_pose = [0.0] * 6
        self.arm_joints = [0.0] * 7
        self.arm_tcp_speed = [0.0] * 3
        self.arm_torque_percent = [0] * 7
        self.last_pt_list = [None] * 4
        self.overseer_api = None
        self.arm_is_robot_moving = False
        self.arm_type = "saw"
        self.is_sim = False
        self.arm_motion_alarm = False
        self.arm_last_pt_list = []
        self.joint_names = ['joint_s', 'joint_l', 'joint_e', 'joint_u', 'joint_r', 'joint_b', 'joint_t']

        self.use_public_ops = False

    def initialize_robot(self, sim=False, overseer=True):
        """Initialize robot & tm callbacks.
        Always call initialize_robot() to start tm, and always call power_off() to stop_tm, before exiting.
        SIA20_Driver will check it's state (ie IDLE, INTIAILZED) so it will not re-initialize if already active
        """
        self.overseer_api = OverseerAPI()
        self.setup_tm_subscribers()
        self.overseer_api.start_tm()
        self.is_sim = sim
        self.arm_type = self.overseer_api.get_tm("SAW.ARM.ARM_TYPE")

        # Load and run the init_sia20 activity
        self.overseer_api.load_activity("init_sia20", remove_loaded=True)
        self.overseer_api.start_activity("init_sia20")

        # Load the robosaw_sia20_motions activity
        self.overseer_api.load_activity("robosaw_sia20_motions", remove_loaded=False)

    def setup_tm_subscribers(self):
        """Setup overseer telemetry subscribers.
        """
        self.tm_arm_pose_list = 'SAW.BLADE.POSE_LIST'
        self.tm_arm_joints = 'SAW.ARM.JOINT_STATES'
        self.tm_arm_tcp_speed = 'SAW.BLADE.VEL_LIST'
        self.tm_arm_torque = 'SAW.ARM.TORQUE_JOINTS'
        self.tm_arm_last_pt= 'SAW.ARM.GET_LAST_PT'
        self.tm_arm_is_robot_moving = "SAW.ARM.IS_ROBOT_MOVING"
        self.tm_arm_motion_alarm = "SAW.ARM.MOTION_ALARM"
        self.tm_arm_last_mp = "SAW.ARM.GET_LAST_MP"
        rospy.sleep(0.1)

        # Check that overseer_api has setup the tm_sample for relevant TM before subscribing
        warn_msg = False
        while not rospy.is_shutdown():
            if self.overseer_api.get_tm_sample(self.tm_arm_pose_list) == None:
                rospy.sleep(0.1)

                if warn_msg == False:
                    rospy.logwarn("SIA20 Plugin is waiting for SAW.BLADE.POSE_LIST telemetry...")
                    warn_msg = True
            else:
                break

        self.overseer_api.subscribe_tm_sample(self.tm_arm_pose_list, self.saw_arm_cb)
        self.overseer_api.subscribe_tm_sample(self.tm_arm_joints, self.saw_arm_cb)
        self.overseer_api.subscribe_tm_sample(self.tm_arm_tcp_speed, self.saw_arm_cb)
        self.overseer_api.subscribe_tm_sample(self.tm_arm_torque, self.saw_arm_cb)
        self.overseer_api.subscribe_tm_sample(self.tm_arm_last_pt, self.saw_arm_cb)  
        self.overseer_api.subscribe_tm_sample(self.tm_arm_is_robot_moving, self.saw_arm_cb)
        self.overseer_api.subscribe_tm_sample(self.tm_arm_motion_alarm, self.saw_arm_cb)
        self.overseer_api.subscribe_tm_sample(self.tm_arm_last_mp, self.saw_arm_cb)

    def saw_arm_cb(self, sample):
        """Callback function for telemetry subscribers.
        """
        if sample.name == self.tm_arm_pose_list:    self.arm_pose = sample.value
        elif sample.name == self.tm_arm_joints:      self.arm_joints = sample.value
        elif sample.name == self.tm_arm_tcp_speed:   self.arm_tcp_speed = sample.value
        elif sample.name == self.tm_arm_torque:      self.arm_torque_percent = sample.value
        elif sample.name == self.tm_arm_last_pt:     self.arm_last_pt_list = sample.value
        elif sample.name == self.tm_arm_is_robot_moving: self.arm_is_robot_moving = sample.value
        elif sample.name == self.tm_arm_motion_alarm: self.arm_motion_alarm = sample.value
        elif sample.name == self.tm_arm_last_mp: self.last_mp = sample.value

    def get_pose(self):
        """Returns arm pose as a m3d.Transform
        """
        if isinstance(self.arm_pose, list):
            return list_to_tf(self.arm_pose)
        else:
            return self.arm_pose

    def get_joints(self):
        """Returns arm joint-states as a list
        """
        return self.arm_joints
    
    def get_tcp_speed(self):
        """Returns TCP speed as a list
        """
        return self.arm_tcp_speed
    
    def get_torque_percentage(self):
        """Returns arm joint-torque percentages (normalized to 1.0) as a list
        """
        return self.arm_torque_percent
    
    def get_last_pt(self):
        """Returns info about the last succcessfully executed arm motion command. Converts list to dict.
        Order is [jointList, poseList_base, vel, wait]
        """
        self.overseer_api.run_private_op("SAW.ARM.UPDATE_LAST_PT") # Updates self.arm_last_pt_list variable
        last_pt = {}
        try:
            last_pt["jointList"] = self.arm_last_pt_list[0]
            last_pt["poseList_base"] = self.arm_last_pt_list[1]
            last_pt["vel"] = self.arm_last_pt_list[2]
            last_pt["wait"] = bool(self.arm_last_pt_list[3])
        except:
            last_pt["jointList"] = [[]]
            last_pt["poseList_base"] = [[]]
            last_pt["vel"] = 0
            last_pt["wait"] = False
        return last_pt
    
    def is_robot_moving(self):
        """Returns True is the robot is currently executing a motion
        """
        return self.arm_is_robot_moving

    def get_motion_alarm(self):
        """Returns True if the SIA20 has a motion alarm
        """
        try:
            return self.arm_motion_alarm
        except:
            return False

    def smart_move(self, **kwargs):
        """Command to execute a smart_move using a motion plan. Requires jointList[] or poseList_base[] in kwargs.
        Inserts op into robosaw_sia20_motions activity and resume the activity
        """
        kwargs = self.check_kwargs(**kwargs)
        kwargs["index"] = OperationIndex.AFTER_ACTIVE.value

        if self.use_public_ops:
            self.overseer_api.set_working_activity("robosaw_sia20_motions")
            self.overseer_api.insert_op("SAW.ARM.SMART_MOVE", **kwargs, index=OperationIndex.AFTER_ACTIVE.value)
            self.overseer_api.resume_activity("robosaw_sia20_motions")
        else:
            self.overseer_api.run_private_op("SAW.ARM.SMART_MOVE", **kwargs)

        if kwargs["wait"] == True:
            self.wait_for_arm_status("INCOMPLETE")
            self.wait_for_arm_status("COMPLETE")

    def movejs(self, **kwargs):
        """Command to execute an open-loop joint-motion, requires jointList[] in kwargs.
        Inserts op into robosaw_sia20_motions activity and resume the activity
        """
        kwargs = self.check_kwargs(**kwargs)
        kwargs["index"] = OperationIndex.AFTER_ACTIVE.value

        if self.use_public_ops:
            self.overseer_api.set_working_activity("robosaw_sia20_motions")
            self.overseer_api.insert_op("SAW.ARM.MOVEJS", **kwargs, index=OperationIndex.AFTER_ACTIVE.value)
            self.overseer_api.resume_activity("robosaw_sia20_motions")
        else:
            self.overseer_api.run_private_op("SAW.ARM.MOVEJS", **kwargs)

        if kwargs["wait"] == True:
            self.wait_for_arm_status("INCOMPLETE")
            self.wait_for_arm_status("COMPLETE")


    def movels(self, **kwargs):
        """Command to execute an open-loop linear-motion, requires poseList_base[] in kwargs.
        Inserts op into robosaw_sia20_motions activity and resume the activity
        """
        kwargs = self.check_kwargs(**kwargs)
        kwargs["index"] = OperationIndex.AFTER_ACTIVE.value

        if self.use_public_ops:
            self.overseer_api.set_working_activity("robosaw_sia20_motions")
            self.overseer_api.insert_op("SAW.ARM.MOVELS", **kwargs, index=OperationIndex.AFTER_ACTIVE.value)
            self.overseer_api.resume_activity("robosaw_sia20_motions")
        else:
            self.overseer_api.run_private_op("SAW.ARM.MOVELS", **kwargs)

        if kwargs["wait"] == True:
            self.wait_for_arm_status("INCOMPLETE")
            self.wait_for_arm_status("COMPLETE")


    def set_speed(self,  **kwargs):
        """Command to set speed percentage (1-255%), used in the control loop
        """
        if not self.is_sim:
            kwargs = self.check_kwargs(**kwargs)
            self.overseer_api.run_private_op("SAW.ARM.SET_SPEED", **kwargs)

    def STOP(self):
        """Immediately stop all motion and remove any queued motions.
        """
        self.overseer_api.run_private_op("SAW.ARM.STOP")
        # self.overseer_api.run_critical_op("SAW.ARM.STOP")

    def stepl(self, **kwargs):
        """Command to perform linear-step motion.
        kwargs include:
        x, y, z in linear velocity [m/s] 
        roll, pitch, yaw in angular velocity [deg/s]
        duration [s]
        Inserts op into robosaw_sia20_motions activity and resume the activity
        """
        kwargs = self.check_kwargs(**kwargs)
        kwargs["index"] = OperationIndex.AFTER_ACTIVE.value

        if self.use_public_ops:
            self.overseer_api.set_working_activity("robosaw_sia20_motions")
            self.overseer_api.insert_op("SAW.ARM.STEPL", **kwargs, index=OperationIndex.AFTER_ACTIVE.value)
            self.overseer_api.resume_activity("robosaw_sia20_motions")
        else:
            self.overseer_api.run_private_op("SAW.ARM.STEPL", **kwargs)

    def stepj(self, **kwargs):
        """Command to perform joint-step motion. Currently HMI supports 1 joint at a time.
        kwargs include:
        joint="joint_name", ang_velocity [deg/s], duration [s] 
        Inserts op into robosaw_sia20_motions activity and resume the activity
        """
        kwargs = self.check_kwargs(**kwargs)
        kwargs["index"] = OperationIndex.AFTER_ACTIVE.value

        if self.use_public_ops:
            self.overseer_api.set_working_activity("robosaw_sia20_motions")
            self.overseer_api.insert_op("SAW.ARM.STEPJ", **kwargs, index=OperationIndex.AFTER_ACTIVE.value)
            self.overseer_api.resume_activity("robosaw_sia20_motions")
        else:
            self.overseer_api.run_private_op("SAW.ARM.STEPJ", **kwargs)

    def update_last_pt(self):
        """Command drive to update info about the last succcessfully executed arm motion command.
        """
        self.overseer_api.run_private_op("SAW.ARM.UPDATE_LAST_PT")
    
    def power_off(self):
        """Power off arm and stop telemetry subscribers.
        """
        self.overseer_api.stop_tm()

    def check_kwargs(self, **kwargs):
        """Check kwargs for proper formatting
        """

        # convert poseList_base from tf to list
        if "poseList_base" in kwargs:
            if isinstance(kwargs["poseList_base"][0], m3d.Transform):
                poseList = [tf_to_list(p) for p in kwargs["poseList_base"][:]]
                kwargs["poseList_base"] = poseList[:]

        if "pose" in kwargs:
            if isinstance(kwargs["pose"], m3d.Transform):
                kwargs["pose"] = tf_to_list(kwargs["pose"])

        # convert numpy to float
        if "vel_rot" in kwargs:
            kwargs["vel_rot"] = float(kwargs["vel_rot"])

        # Set wait=True by default if not specified
        if "wait" not in kwargs:
            kwargs["wait"] = True

        return kwargs

    def wait_for_arm_status(self, status):
        """ Waits indefinitely until the SIA20 plugin reaches the given status.
        """
        while(self.overseer_api.get_tm('SAW.ARM.PROCESS_STATUS') != status) and not rospy.is_shutdown():
            rospy.sleep(0.01)

    def get_last_mp(self):
        """ Returns last attempted motion plan (list of joint-states)
        If last attempted motion plan failed, then this returns an empty list [[]]
        """
        return self.last_mp