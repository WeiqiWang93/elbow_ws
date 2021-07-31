import sys, yaml
import roslib; roslib.load_manifest('motoman_driver')
import rospy, rosbag
import time
from std_srvs.srv import Trigger, TriggerRequest
from industrial_msgs.srv import StopMotion, StopMotionRequest
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math3d as m3d
import pdb
from motoman_ros_constants import *
import math
import numpy as np


class MotomanROS(object):

    def __init__(self):
    
        # Services
        try:
            # wait for services to be on
            rospy.wait_for_service('/stop_motion')
            rospy.wait_for_service('/robot_enable')
            rospy.wait_for_service('/robot_disable')

            # stopj service
            self.srv_stopj = rospy.ServiceProxy('/stop_motion', StopMotion)

            # Create the connection to the service. Remember it's a Trigger service
            self.srv_enable = rospy.ServiceProxy('/robot_enable', Trigger)
            self.srv_disable = rospy.ServiceProxy('/robot_disable', Trigger)

            sub_joints = rospy.Subscriber('/saw/joint_states', JointState, self.save_joints_cb)

        except rospy.ServiceException, e:
            print "Service call failed: %s"
        


    ############### Functions for ArmServer#####################

    # wrapper around joint_states topic
    # get_joints() function returns list of joint data
    # def get_joints(self):
    #     try:
    #         data = rospy.wait_for_message("/saw/joint_states", JointState)
    #         return data.position
    #     except (rospy.ROSException, rospy.ROSInterruptException):
    #         rospy.logerr('Unable to read current position')

    #         raise

    def get_joints(self):
        return self.joints

    def save_joints_cb(self, msg):
        self.joints = np.array(msg.position[:]).tolist()


    # wrapper around joint_states topic
    # get_joint_velocities() function returns list of joint velocities
    def get_joint_velocities(self):
        try:
            data = rospy.wait_for_message("/joint_states", JointState)
            return data.velocity
        except (rospy.ROSException, rospy.ROSInterruptException):
            rospy.logerr('Unable to read current position')
            raise
    
    # Wrapper around move_to_joint, which uses build_traj_time or build_traj_vel to generate 2-point traj
    # enables servos/comms, completes move, disables comms
    def movej(self, joints=[], acc=None, vel=None, wait=True, timeout=None, threshold=0.3):
        """ move-in-joint-space
        Inputs: 
            joints[rad] : List of 7 joint angles [rad]
            acc         : Currently ignored (no easy way to set accel on FS100) [rad/s^2]
            vel[rad/s]  : Max joint velocity (OVERRIDES timeout)
            wait        : Function return when movement is finished
            time[s]     : Sets time for movej, IGNORED if vel is set
            threshold   : Passes to wait_for_move, recommended to leave as None
        Outputs: 
            list (6,) joints

        """
        rospy.loginfo("Begin movej to joints[]: {}".format(joints))

        self.enableMoveJ()
        (end_pos, duration) = self.parse_args([str(joints), timeout])

        # if timeout and velocity are both not given, then use velocity = 0.087 rad/s (5 deg/s)
        if vel <= 0 and timeout <= 0:
            rospy.logwarn("MoveJ was not given velocity or timeout, so use default vel {} rad/s ({} deg/s)".format(MOVEJ_DEFAULT_VEL, math.degrees(MOVEJ_DEFAULT_VEL)))
            vel = MOVEJ_DEFAULT_VEL
            
        # traj with velocity
        if vel >= 0.0:
            (traj,timeout) = self.build_traj_vel(self.get_cur_pos(), end_pos, vel)

        # traj with time
        else:
            traj = self.build_traj_time(self.get_cur_pos(), end_pos, duration)

        # call move_to_joint on traj
        self.move_to_joint(traj)

        # Block return if wait is True 
        if wait == True:
            self.wait_for_move(joints_goal=joints,vel=vel,timeout=timeout,threshold=threshold)
            self.disableMoveJ()

        return self.get_cur_pos()


    def movejs(self, jointList=[], acc=None, vel=None, wait=True, timeout=None, threshold=0.3):
        if type(jointList) == np.ndarray:
            rospy.loginfo("Convert numpy array to list")
            jointList = jointList.tolist()
            pdb.set_trace()


        self.enableMoveJ()
        
        startTime = time.time()

        for joints in jointList:
            self.movej(joints=joints,acc=acc,vel=vel,wait=wait,timeout=timeout,threshold=threshold)
            
            '''
            Relationship between START_MAX_PULSE_DEVIATION (Controller.h) and t_between_movej
                AND start_pos_tol_
            pulses/deg: SLEU 1024.0, RB 1149.2, T 580.3 

            Deviation=50,  t_between_movej=0.7, start_pos_tol_ = 1e-4
            Deviation=100, t_between_movej=0.6, start_pos_tol_ = 1e-4
            Deviation=1000,t_between_movej=0.6, start_pos_tol_ = 1e-4

            Deviation=1000,t_between_movej=0.5, start_pos_tol_ = 1e-2
            '''

            t_between_movej = 0.0
            if t_between_movej > 0.0:
                rospy.loginfo("Sleep between movej's of {} sec".format(t_between_movej))
                time.sleep(t_between_movej)
        
        # If wait is true, then disable MoveJ after all commands are done
        if wait == True:
            self.disableMoveJ()

    # Wrapper around robot_disable service
    # stop motion immediately
    def stopj(self):
        # rospy.loginfo("stopj begin")
        '''
        joints_vel = self.get_joint_velocities()
        avg_joint_vel = abs (sum(joints_vel) / len(joints_vel))
        # print avg_joint_vel

        while avg_joint_vel > 0.0001:
            try:
                result = self.srv_stopj(StopMotionRequest())
                rospy.loginfo("stopj")
            except:
                rospy.logerr('Unable to stop. Aborting...')
                raise

            joints_vel = self.get_joint_velocities()
            avg_joint_vel = abs (sum(joints_vel) / len(joints_vel))
            # print avg_joint_vel
            time.sleep(0.1) # loop at 10 Hz 
        '''  
        # rospy.loginfo("IGNORE stopj called")
        # self.srv_stopj(StopMotionRequest())
        # time.sleep(0.5) # give 1.5 second for robot to fully stop
            # best performance with 1.0 sleep, then 0.5 sleep, then 0.0 sleep
        return
    
    # Helper function to determine if robot has arrived to joint goal, and stop upon arrival
    def wait_for_move(self, joints_goal, vel, timeout=None, threshold=0.0):
        startTime = time.time()
        extra_time = 10
        MOTION_TIMEOUT = 10
        JOINT_DELTA = 0.01

        # give 0.2 for motion to start
        time.sleep(0.5)

        if timeout > 0:
            MOTION_TIMEOUT = timeout + extra_time # give 3 extra seconds before timeout
            # rospy.loginfo("wait_for_move for {} s".format(MOTION_TIMEOUT))

        if threshold > 0:
            JOINT_DELTA = threshold
            # rospy.loginfo("wait_for_move threshold set to {}".format(JOINT_DELTA))

        success = False
        sleep_time = 0.01
        while success == False:
            # time.sleep(sleep_time)

            joint_data = self.get_joints()
            joint_deltas = [x1 - x2 for (x1, x2) in zip(joint_data, joints_goal)]
            sum_joint_delta = sum(map(abs, joint_deltas))
            
            joints_vel = self.get_joint_velocities()
            avg_joint_vel = abs (sum(joints_vel) / len(joints_vel))

            currentTime = time.time()

            if sum_joint_delta < JOINT_DELTA and avg_joint_vel < JOINT_DELTA/100:
                rospy.loginfo("MoveJ arrived after {} s".format(round(currentTime-startTime, 4)))
                success = True
                break

            # if robot is not moving (most likely due to rejected trajectory) then send move goal again
            if sum_joint_delta > 0.1 and avg_joint_vel < 0.0001:
                #self.movej(joints=joints_goal,vel=vel,time=timeout,wait=False) # no wait
                rospy.logwarn("re-sending moveJ since last request was rejected")
                time.sleep(0.1)
                self.movej(joints=joints_goal,acc=None,vel=vel,wait=False,timeout=timeout,threshold=0.0) # no wait


            # timeout
            if currentTime - startTime > MOTION_TIMEOUT:
                rospy.logerr("Failed to arrive at movej goal, return after timeout {}s".format(MOTION_TIMEOUT))
                self.stopj()
                rospy.logwarn("re-sending moveJ since last request was rejected")
                self.movej(joints=joints_goal,acc=None,vel=vel,wait=False,timeout=timeout,threshold=0.0) # no wait
                return
            

        self.stopj()


    # enable servos and comms
    def enableMoveJ(self):
        result = self.srv_enable(TriggerRequest())
        
        '''
        time.sleep(0.1)
        if result.sucess == False:
            rospy.logwarn("MotomanROS Error: rosservice call /robot_enable failed, trying again.")
            self.enableMoveJ()
        '''
        # time.sleep(0.1)
        # rospy.loginfo("enableMoveJ: {}".format(result.success))

        return result.success

    # disable comms
    def disableMoveJ(self):
        result = self.srv_disable(TriggerRequest())

        '''
        time.sleep(0.1)
        if result.sucess == False:
            rospy.logwarn("MotomanROS Error: rosservice call /robot_disable failed, trying again.")
            self.disableMoveJ()
        '''

        # time.sleep(0.1)
        # rospy.loginfo("disableMoveJ: {}".format(result.success))

        return result.success

    ############### Helper functions from move_to_joint.py #####################

    # build a simple trajectory from the start to end position
#   - for the FS100, we can get by with a simple 2-point trajectory
#   - the controller handles the necessary accel/decel to make smooth motion
    def build_traj_time(self, start, end, duration):

        if sorted(start.name) <> sorted(end.name):
            rospy.loginfo(sorted(start.name))
            rospy.loginfo(sorted(end.name))
            rospy.logerr('Start and End position joint_names mismatch')
            # raise

        # assume the start-position joint-ordering
        joint_names = start.name
        
        start_pt = JointTrajectoryPoint()
        start_pt.positions = start.position
        start_pt.velocities = [0]*len(start.position)
        start_pt.time_from_start = rospy.Duration(0.0)

        # enforce 5.0 sec min duration
        # if duration < MIN_DURATION:
        #     rospy.logwarn("motoman_ros build_traj_time enforcing a {} sec min duration".format(MIN_DURATION))
        #     duration = MIN_DURATION

        end_pt = JointTrajectoryPoint()
        for j in joint_names:
            idx = end.name.index(j)
            end_pt.positions.append(end.position[idx])  # reorder to match start-pos joint ordering
            end_pt.velocities.append(0)
        end_pt.time_from_start = rospy.Duration(duration)

        rospy.loginfo("MoveJ input time duration {} sec".format(duration))

        return JointTrajectory(joint_names=joint_names, points=[start_pt, end_pt])
    
    # build simple trajectory with vel/acc specified, not time
    # IMPORTANT: difference in behavior between end_pt.velocities.append(vel) and end_pt.velocities.append(0)
    def build_traj_vel(self, start, end, vel):
        if sorted(start.name) <> sorted(end.name):
            rospy.loginfo(sorted(start.name))
            rospy.loginfo(sorted(end.name))
            rospy.logerr('Start and End position joint_names mismatch')
            # raise

        # assume the start-position joint-ordering
        joint_names = start.name
        
        start_pt = JointTrajectoryPoint()
        start_pt.positions = start.position
        start_pt.velocities = [0 for i in range(6)]
        start_pt.time_from_start = rospy.Duration(0.0)

        # check for max vel (30 deg/s)
        if vel > MOVEJ_MAX_VEL_DEG:
            rospy.logwarn("Lowered velocity from {} deg/s to {} deg/s".format(vel, MOVEJ_MAX_VEL_DEG))
            vel = MOVEJ_MAX_VEL_DEG

        # calculate time
        joint_deltas = [x1 - x2 for (x1, x2) in zip(start.position, end.position)]
        duration = abs(max(joint_deltas, key=abs) / vel)
        # rospy.loginfo("{}s = {} max_delta /{} vel".format(round(duration, 3), round(max(joint_deltas), 3), round(vel,3 )))

        # enforce 5.0 sec min duration
        # if duration < MIN_DURATION:
        #     rospy.logwarn("motoman_ros build_traj_time enforcing a {} sec min duration".format(MIN_DURATION))
        #     duration = MIN_DURATION

        # Check the velocity per joint
        joint_vels = [float(x)/vel for x in joint_deltas]
        joint_vels_str = ["{}".format(round(i,3)) for i in joint_vels]
        rospy.loginfo("Calculating joint_vels [deg]: {}".format(joint_vels_str))

        end_pt = JointTrajectoryPoint()
        for j in joint_names:
            idx = end.name.index(j)
            end_pt.positions.append(end.position[idx])  # reorder to match start-pos joint ordering
            end_pt.velocities.append(0)
        end_pt.time_from_start = rospy.Duration(duration)

        rospy.loginfo("MoveJ calculated time duration {} sec".format(round(duration, 3)))

        return JointTrajectory(joint_names=joint_names, points=[start_pt, end_pt]), duration

    # read the current robot position from the "joint_states" topic
    def get_cur_pos(self):
        try:
            return rospy.wait_for_message("/saw/joint_states", JointState, 5.0)
        except (rospy.ROSException, rospy.ROSInterruptException):
            rospy.logerr('Unable to read current position')
            raise

        # wait for subscribers
    def wait_for_subs(self, pub, num_subs, min_time, timeout):
        end = rospy.Time.now() + rospy.Duration(timeout)
        rospy.sleep(min_time)

        r = rospy.Rate(10)  # check at 10Hz
        while (pub.get_num_connections() < num_subs) and (rospy.Time.now() < end) and not rospy.is_shutdown():
            r.sleep()

        return (pub.get_num_connections() >= num_subs)

    # move the robot to the specified position
    #   - read the current robot position
    #   - generate a trajectory from the current position to target position
    #   - publish the trajectory command
    def move_to_joint(self, traj):
        # wait for subscribers to connect
        pub = rospy.Publisher('/joint_path_command', JointTrajectory, queue_size=1)
        if not self.wait_for_subs(pub, 1, 0.5, 2.0):
            rospy.logwarn('Timeout while waiting for subscribers.  Publishing trajectory anyway.')

        pub.publish(traj)

        # extract the first JointTrajectory point from a bag file
        #   - if multiple messages, use the first message on the specified topic
    def get_pos_from_bag(self, bag_file, topic_name):
        with rosbag.Bag(bag_file) as bag:
            msgs = list(bag.read_messages(topic_name))

        if len(msgs) <> 1:
            rospy.logwarn("Multiple trajectories found.  Exporting first trajectory only")

        traj = msgs[0][1]

        return JointState(name=traj.joint_names, position=traj.points[0].positions)

    # get the typical list of motoman joint-names, based on DOF-count
    #   - override default list with ROS param '~joint_names', if present
    def get_joint_names(self, num_joints):

        if num_joints == 6:
            default_names = ['joint_'+j for j in ['s','l','u','r','b','t']]
        elif num_joints == 7:
            default_names = ['joint_'+j for j in ['s','l','e','u','r','b','t']]
        else:
            default_names = ''

        return rospy.get_param('/controller_joint_names', default_names)

    # parse the input arguments
    def parse_args(self, args):
        if len(args) < 1 or len(args) > 2:
            print_usage()
            raise ValueError("Illegal number of arguments")

        # check if first argument is bag-file, otherwise assume array of joint-positions
        try:
            end_pos = self.get_pos_from_bag(args[0], 'joint_path_command')
        except:
            pos = yaml.load(args[0])
            names = self.get_joint_names(len(pos))
            end_pos = JointState(name=names, position=pos)

        duration = float(args[1]) if len(args)>1 else 10.0

        return (end_pos, duration)