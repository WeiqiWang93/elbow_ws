class MotionPlanner(object):
    """ Class that implements a client for the Motion Planning service.
        Also executes the plan on the robot.
        Set class member 'use_ur_driver' to true for executing pre-interpolated
        Trajectories on the robot. 
    """
    def __init__(self, group_name, frame_id="base"):
        """ Valid group-names: 'chisel', 'saw'
        """
        self.group_name = group_name
        self.frame_id_ = frame_id

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

        self.tf = TransformListener()
        ### self.logfile = open(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")+".txt","w")
        self.avg_planning_time = 0.0
        self.count = 0.0
        self.success_rate = 0.0
        self.success_count = 0.0
        self.total_time = 0.0
        self.goal_wall_offset = 0.02
        self.joint_prefix = self.group_name
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

    def get_plan(self, attack_pose, joints, joints_goal=None):
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

            # goal in cartesian space
            goal_pose = geometry_msgs.msg.PoseStamped()
            goal_pose.header.frame_id = self.frame_id_
            if isinstance(attack_pose, m3d.Transform):
                goal_pose.pose = m3d_to_ros(attack_pose)
            elif isinstance(attack_pose,  geometry_msgs.msg.Pose):
                goal_pose.pose = attack_pose

            # Planning result
            path = trajectory_msgs.msg.JointTrajectory()
            
            # Planning to a cartesian pose goal
            if joints_goal is None:

                # Visualize goal
                goal_pose.header.stamp = rospy.Time.now()
                self.pub_goal.publish(goal_pose)     

                # IK input needs to be in the world frame. 
                pose_world = PoseStamped()
                pose_world.header.stamp = rospy.Time.now()
                pose_world = self.tf.transformPose("/world", goal_pose)
                group = String()
                group.data = self.group_name
                      
                # obtain a joint state through IK service
                ik = self.get_ik(pose_world.pose, group, rstate)

                if ik.found_ik:
                    joints_goal_msg = ik.joints
                else:
                    joints_goal_msg.data = []
                
                # Calling path server
                planner_resp = self.mps[self.group_name](rstate, goal_pose, ik.joints)

            # Planning to a joints goal
            else:
                joints_goal_msg.data = joints_goal
                planner_resp = self.mps[self.group_name](rstate, goal_pose, joints_goal_msg)
            
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

                return (planner_resp.magic_points, planner_resp.translations)

            else:
                self.pub_goal.publish(goal_pose)
                path.points = []
                return (path, path)

        except rospy.ServiceException, e:
            rospy.logerr("Planning Service call failed: {}".format(e))

    def plan(self, start_js, input_pose=None, input_joints=None):
        """ The public method to be invoked by the end user. 
            Call this method from your script to navigate the end-effector
            to goalPose.
        """ 
        # VEL deg/s
        # ACC deg/s^2
        goalPose = None
        temp_joints = start_js
        temp_joints = (temp_joints)

        # Goal 
        if input_joints is None:
            goalPose = input_pose.copy()
            result = self.get_plan(goalPose, temp_joints)
        else:
            result = self.get_plan(goalPose, temp_joints,
                                   joints_goal=input_joints
                                   )
        if result is None:
            final_pts = []
            return final_pts

        (way_points, trans) = result

        # Post-process the trajectory to downsample way-points
        if len(way_points.points) == 0:
            final_pts = []
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
        return final_pts

    def plan_and_move(self, input_joints=None, input_pose=None, ACC=2, VEL=1):
        """ plans and moves to target """

        final_pts = []
        if input_joints is not None:
            final_pts = self.plan(input_joints=input_joints)
        elif input_pose is not None:
            final_pts = self.plan(input_pose=input_pose)
        else:
            rospy.logerr("Invalid input to the motion planner.")
        return final_pts

    def get_ik_srv(self, group, ik_seed_js, start_pose):
        if isinstance(start_pose, m3d.Transform):
            first_pose = m3d_to_ros(start_pose)
        else:
            first_pose = start_pose

        # Transform the poses to /world frame
        start_pose_world = PoseStamped()
        start_pose_world.header.stamp = rospy.Time.now()
        start_pose_world.header.frame_id = self.frame_id_
        start_pose_world.pose = first_pose
        start_pose_world = self.tf.transformPose("/world", start_pose_world)

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
        if isinstance(goal_pose, m3d.Transform):
            last_pose = m3d_to_ros(goal_pose)
        else:
            last_pose = goal_pose
        assert(isinstance(last_pose, geometry_msgs.msg.Pose))
        
        if isinstance(start_pose, m3d.Transform):
            first_pose = m3d_to_ros(start_pose)
        else:
            first_pose = start_pose
        assert(isinstance(first_pose, geometry_msgs.msg.Pose))

        # TODO: Check validity of excluded links.

        # Transform the poses to /world frame
        start_pose_world = PoseStamped()
        start_pose_world.header.stamp = rospy.Time.now()
        start_pose_world.header.frame_id = self.frame_id_
        start_pose_world.pose = first_pose
        start_pose_world = self.tf.transformPose("/world", start_pose_world)

        # Transform the poses to /world frame
        last_pose_world = PoseStamped()
        last_pose_world.header.stamp = rospy.Time.now()
        last_pose_world.header.frame_id = self.frame_id_
        last_pose_world.pose = last_pose
        last_pose_world = self.tf.transformPose("/world", last_pose_world)

        # The following function overrides the planning server's default planning group
        gp = String()
        gp.data = group

        # Obtain joint state of the start pose from Ik
        # With given seed state.
        ik = self.get_ik(start_pose_world.pose, gp, ik_seed_js)
        start_state = self.get_robot_state_msg(ik.joints.data)
        
        # Perform collision checking and obtain result
        resp = self.services[self.group_name](last_pose_world.pose, first_pose,
                                              gp, start_state, excluded_links)
        in_collision = resp.result
        poses = [self.tf.transformPose(self.frame_id_, PoseStamped(resp.pose_array.header, pose)) for pose in resp.pose_array.poses]

        return in_collision, poses

    def check_collision_list(self, group, ik_seed_js, input_poses, excluded_links=[], attached_markers=None):
        """ Checks collision of the given group defined in diggerbot.srdf
            groups can be "saw", "chisel", "saw_guard"
            ik_seed_js: List of joint values representing arm's current joint state.
            excluded_links: This arg takes a list of links that are to be
            excluded from collision checking against the octomap.
            input_poses: List of poses to be checked for collision.

        returns 
            in_collision: a list of boolean in_collision flags
            poses: list of ROS pose-stamped messages of each pose tested
        """
        input_poses_world = []
        for pose in input_poses:
            last_pose = geometry_msgs.msg.Pose()
            if isinstance(pose, m3d.Transform):
                last_pose = m3d_to_ros(pose)
            else:
                last_pose = pose

            # TODO: Check validity of excluded links.

            # Transform the poses to /world frame
            last_pose_world = PoseStamped()
            last_pose_world.header.stamp = rospy.Time.now()
            last_pose_world.header.frame_id = self.frame_id_
            last_pose_world.pose = last_pose
            last_pose_world = self.tf.transformPose("/world", last_pose_world)
            input_poses_world.append(last_pose_world.pose)

        ik_seed_js = self.get_robot_state_msg(ik_seed_js)

        # Attach collision objects
        # convert visualization_msgs.marker.cube to moveit_msgs/AttachedCollisionObject
        if attached_markers:
            attached_objects = self.viz_markers_to_aco(attached_markers)
            rstate.attached_collision_objects.append(attached_objects)

        # Perform collision checking and obtain result
        gp = String()
        gp.data = group
        ip = geometry_msgs.msg.PoseArray()
        ip.poses = input_poses_world
        resp = self.services_ccl[self.group_name](ip, gp, ik_seed_js, excluded_links)
        in_collision = resp.result
        poses = [self.tf.transformPose(self.frame_id_, PoseStamped(resp.pose_array.header, pose)) for pose in resp.pose_array.poses]

        return in_collision, poses

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
        start_pose_world = self.tf.transformPose("/world", start_pose_world)

        # The following function overrides the planning server's default planning group
        gp = String()
        gp.data = self.group_name
        ik_seed_state = self.get_robot_state_msg(ik_seed_js)

        ik = self.get_ik(start_pose_world.pose, gp, ik_seed_state)
        return ik.joints.data
    
    
