import time, argparse
import sys
import copy
import math
import numpy as np
import pdb
import socket
import os
import math3d as m3d
#
import ruamel.yaml
import rospy
from visualization_msgs.msg import Marker
import actionlib
from rospy_message_converter import message_converter
from geometry_msgs.msg import PoseStamped
# actionlib server stuff
from bot_common_ros.msg import ARMAction as Action, ARMGoal as Goal, ARMFeedback as Feedback, ARMResult as Result
from bot_saw_slot_manager.msg import SlotManagerResult as SlotResult, SlotManagerAction as SlotAction, SlotManagerGoal as SlotGoal
from bot_common_ros.msg import ActionType, ProcessStatus, ArmStatus
#
from tf import TransformListener
from rospy_message_converter import message_converter

#
from bot_chisel.msg import CHISELAction, CHISELGoal, CHISELFeedback, CHISELResult

done = False
status = None


class DualClientCmd():
    def __init__(self):
        self.__version__ = "_1_0"
        rospy.init_node('DualClientCmd'+self.__version__, anonymous=True)

        self.bbox = None

        self.clients = {}
        # set up saw client
        self.clients["saw"] = actionlib.SimpleActionClient('/saw/RoboSaw', Action)
        self.clients["saw"].wait_for_server()
        s1 = rospy.Subscriber("/saw/RoboSaw/result", Result, self.saw_update_status)        

        # set up chisel client
        self.clients["chisel"] = actionlib.SimpleActionClient('/chisel/RoboChisel', CHISELAction)
        self.clients["chisel"].wait_for_server()
        s2 = rospy.Subscriber("/chisel/RoboChisel/result", CHISELResult, self.chisel_update_status)

        self.clients["slot_manager"] = actionlib.SimpleActionClient('/saw/SlotManager', SlotAction)
        self.clients["slot_manager"].wait_for_server()
        s3 = rospy.Subscriber("/saw/SlotManager/result", SlotResult)

        cwd_path = os.getcwd()
        home_path = cwd_path[0:cwd_path.find('swarm-robotic-mining')]

        self.bbox_yaml_path = home_path + "swarm-robotic-mining/catkin_ws/src/bot_common_ros/config/params/current_bbox.yaml"
        self.global_bbox_yaml_path = home_path + "swarm-robotic-mining/catkin_ws/src/bot_common_ros/config/params/global_bbox_saw.yaml"
        self.global_chisel_bbox_yaml_path = home_path + "swarm-robotic-mining/catkin_ws/src/bot_common_ros/config/params/global_bbox_chisel.yaml"
        self.global_bbox = None
        # self.bbox subscriber
        s4 = rospy.Subscriber('/bot_digger_chisel/operating_box', Marker, self.bbox_subscriber, queue_size=10)
        self.pub_bbox =  rospy.Publisher("/bbox_viz", Marker, queue_size = 10)
        rospy.sleep(1)
        self.done = {"saw": False, "slot_manager": False, "chisel": False}
        self.status = {"saw": 0, "slot_manager": 0, "chisel": 0}
        self.tf = TransformListener()
    
    def load_global_bbox(self, tool):
        # read in global operating box 
        if tool == "chisel":        
            global_bbox_dict, _, _ = ruamel.yaml.util.load_yaml_guess_indent(open(self.global_chisel_bbox_yaml_path))
        else:
            global_bbox_dict, _, _ = ruamel.yaml.util.load_yaml_guess_indent(open(self.global_bbox_yaml_path))
        if global_bbox_dict:
            self.global_bbox = message_converter.convert_dictionary_to_ros_message('visualization_msgs/Marker', global_bbox_dict)        
        else:
            rospy.logerr("Check global bbox yaml file path")
            return

        self.global_bbox = None        

        
        

    def reset(self, tool):
        self.done[tool] = False
        self.status[tool] = None

    def update_status(self, msg, tool):
        self.status[tool] = msg.result.arm_status.status

    def saw_update_status(self, msg):
        self.update_status(msg, "saw")
    
    # def slot_manager_update_status(self, msg):
    #     self.update_status(msg, "slot_manager")

    def chisel_update_status(self, msg):
        self.update_status(msg, "chisel")

    def wait_until_status(self, status_done, tool):
        while not self.status[tool] == status_done:
            time.sleep(0.5)
        self.reset(tool)
    
    def hard_box(self):
        # INPUT GEOMETRY
        y_value = 1.10 # 0.97 #0.92 # Wall distance
        ll = np.array([  0.0, y_value, -0.15]) # set ll z value based on uu z and slot length
        uu = np.array([ 0.55, y_value, 0.60])

        # Don't Touch
        scale = uu - ll
        center = ll + 0.5 * scale
        self.bbox = Marker( type = Marker.CUBE )
        self.bbox.pose.position.x = center[0]
        self.bbox.pose.position.y = center[1]
        self.bbox.pose.position.z = center[2]
        #
        self.bbox.scale.x = scale[0]
        self.bbox.scale.y = scale[1]
        self.bbox.scale.z = scale[2]
        # End Don't Touch
        return self.bbox

    def min_max_bbox(self, bbox, dim = 0):
        x_center = bbox.pose.position.x
        y_center = bbox.pose.position.y
        z_center = bbox.pose.position.z
        x_scale = bbox.scale.x
        y_scale = bbox.scale.y
        z_scale = bbox.scale.z

        x_min = x_center - x_scale / 2.0
        x_max = x_center + x_scale / 2.0

        y_min = y_center - y_scale / 2.0
        y_max = y_center + y_scale / 2.0

        z_min = z_center - z_scale / 2.0
        z_max = z_center + z_scale / 2.0

        if dim == 0:
            return x_min, x_max
        elif dim == 1:
            return y_min, y_max
        elif dim == 2:
            return z_min, z_max

    
    def split_bbox(self, margin):
        """ Function for dividing the current bbox (if required) into two separate bboxes. One for chiseling right to left
        and other for chiseling left to right
        margin: Defines the percent of the bbox from the left side after which the chisel will operate from left to right
        """

        try:
            global_bbox_data, _, _ = ruamel.yaml.util.load_yaml_guess_indent(open(self.global_bbox_yaml_path))
            self.global_bbox = message_converter.convert_dictionary_to_ros_message('visualization_msgs/Marker', global_bbox_data)
        except IOError:
            print("Error: Global bbox file does not appear to exist. Please write it by using write_global_bbox()")
        

        if margin == 1.0:
            return self.bbox, None
        elif margin == 0.0:
            return None, self.bbox
        
        x_min, x_max = self.min_max_bbox(self.global_bbox, 0)        
        x_threshold = x_min + margin * self.global_bbox.scale.x
        
        bbox_left = copy.deepcopy(self.bbox)
        bbox_right = copy.deepcopy(self.bbox)
        

        curr_x_min, curr_x_max = self.min_max_bbox(self.bbox, 0)
        if curr_x_max <= x_threshold:
            return bbox_left, None
        if curr_x_min >= x_threshold:
            return None, bbox_right
        curr_x_center = self.bbox.pose.position.x
        curr_x_scale = self.bbox.scale.x

        left_bbox_scale = abs(curr_x_min - x_threshold)
        right_bbox_scale = abs(curr_x_max - x_threshold)
        
        if left_bbox_scale < 0.05:
            # left bbox too small
            # Only do left to right chisel
            return None, self.bbox
        if right_bbox_scale < 0.05:
            # right bbox too small
            # only chisel right to left
            return self.bbox, None
        
        
        right_bbox_x = (curr_x_max + x_threshold) / 2.0
        left_bbox_x = (curr_x_min + x_threshold) / 2.0

        bbox_left.pose.position.x = left_bbox_x
        bbox_left.scale.x = left_bbox_scale

        bbox_right.pose.position.x = right_bbox_x
        bbox_right.scale.x = right_bbox_scale

        return bbox_left, bbox_right
    
    def write_bbox(self, path = None):
        """Writes the currently drawn bbox in rviz to current_bbox.yaml file"""
        if self.bbox:
            bbox = self.bbox
        else:
            rospy.logerr("You must first set the bbox.") 
        bbox_data = message_converter.convert_ros_message_to_dictionary(self.bbox)        

        if path is None:
            path = self.bbox_yaml_path

        with open(path ,'w') as yaml_file:
                ruamel.yaml.round_trip_dump(bbox_data, yaml_file)
    
    def set_from_global(self):
        self.bbox = self.global_bbox
    
    def write_global_bbox(self):
        print("Writing currently drawn operating box to global bbox file...")
        self.write_bbox(self.global_bbox_yaml_path)
        


    def run(self, goal, tool):
        self.clients[tool].send_goal(goal)
        self.clients[tool].wait_for_result()
        result = self.clients[tool].get_result()
        return result

    def op_init(self, tool, index = None):
        if index is None:
            index = 0
        goal = Goal()
        goal.action_type.type = ActionType.INIT        
        goal.vertical_pitch = 0.05
        goal.quadrant = self.bbox
        # Check the frame and transform bbox pose
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.bbox.header.frame_id
        pose.pose = self.bbox.pose
        pose_transformed = self.tf.transformPose(tool+"_base", pose)
        self.bbox.pose = pose_transformed.pose
        self.bbox.header.frame_id = tool+"_base"
        goal.quadrant.pose = pose_transformed.pose
        goal.slots_z0 = self.bbox.pose.position.z - 0.5 * self.bbox.scale.z
        

        # self.saw_goal = goal
        self.saw_goal.start_index = index
        self.chisel_goal = goal
        self.run(goal, tool)    
    
    def sm_init(self, quad_id=None, delete=False):
        if quad_id is None:
            quad_id = 1
        goal = SlotGoal()
        goal.quadrant_id = quad_id
        goal.delete = delete
        return self.run(goal, "slot_manager")

    def op_reset(self, tool):
        goal = Goal()
        goal.action_type.type = ActionType.RESET
        goal.quadrant = self.bbox
        self.saw_goal = goal
        self.chisel_goal = goal
        self.run(goal, tool)
        
    def read_bbox(self):
        """
        function to read ONLY the bbox pose and scale from yaml file and write it to the existing bbox
        
        """
        try:
            bbox_data, _, _ = ruamel.yaml.util.load_yaml_guess_indent(open(self.bbox_yaml_path))
        except IOError:
            print("Error: File does not appear to exist.")
            return False
        
        self.bbox = message_converter.convert_dictionary_to_ros_message('visualization_msgs/Marker', bbox_data)
        


    def saw_execute(self, **kwargs):
    # def execute(start_index = 0, plunge_vel = 0.005, cutting_vel = 0.01, skip_slot = False, skip_spline = False, slot_percent = 0.0, num_slots = 0):
        # run op_execute
        goal = self.saw_goal
        goal.action_type.type = ActionType.EXECUTE
        
        return self.run(goal, "saw")
    
    def sm_execute(self, index):
        # this is the query operation
        goal = SlotGoal()        
        goal.start_index = index
        goal.action_type.type = ActionType.EXECUTE

        return self.run(goal, "slot_manager")

    def chisel_execute(self):
        # chisel execute shave
        goal = self.chisel_goal
        goal.options = "SHAVER"
        goal.action_type.type = ActionType.EXECUTE
        self.clients["chisel"].send_goal(goal)
    
    def create_chisel_goal(self, bbox, tool):
        goal = Goal()
        goal.action_type.type = ActionType.INIT        
        goal.vertical_pitch = 0.025
        goal.quadrant = bbox
        # Check the frame and transform bbox pose
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = bbox.header.frame_id
        pose.pose = bbox.pose
        pose_transformed = self.tf.transformPose(tool+"_base", pose)
        bbox.pose = pose_transformed.pose
        bbox.header.frame_id = tool+"_base"
        goal.quadrant.pose = pose_transformed.pose
        goal.slots_z0 = bbox.pose.position.z - 0.5 * bbox.scale.z

        return goal
    
    def op_chisel_split(self):
        # splits bbox if required and performs init and execute on the boxes sequentially
        bbox_left, bbox_right = self.split_bbox(margin = 0.60)
                
        if bbox_left:
            goal_left = self.create_chisel_goal(bbox_left, tool = "chisel")
            self.pub_bbox.publish(bbox_left)            
            self.op_init(tool = "chisel", goal = goal_left)
            self.chisel_execute()
            print("Completed execute action on LEFT BBOX. Moving to transition POSE.")
        if bbox_right:
            goal_right = self.create_chisel_goal(bbox_right, tool = "chisel")
            self.pub_bbox.publish(bbox_right)
            self.op_init(tool = "chisel", goal = goal_right)
            self.chisel_execute()
            print("Completed execute action on RIGHT BBOX. Moving to transition POSE.")

        # self.op_init(goal_right, "chisel")
        # self.chisel_execute()
        
    
    def chisel_cleanup(self):
        # run op_execute
        goal = self.chisel_goal
        goal.options = "CLEANUP"
        goal.action_type.type = ActionType.EXECUTE
        self.clients["chisel"].send_goal(goal)


    def op_pause(self, tool):
        # run op-pause
        goal = Goal()
        goal.action_type.type = ActionType.PAUSE
        self.run(goal, tool)

    def aco(self, tool):
        goal = Goal()
        goal.action_type.type = ActionType.CONTINUE
        self.clients[tool].send_goal(goal)

    def shutdown(self, tool):
        goal = Goal()
        goal.action_type.type = ActionType.SHUTDOWN
        self.run(goal, tool)

    def bbox_subscriber(self, msg):
        """ This subscriber callback goes through the minimum sequence for
        sawing and shaving the input self.bbox.

        To be used on the dual-arm rig in Locust
        """
        self.bbox = msg
        # overwrite the yaml with the bbox drawn in rviz
        # if self.bbox:
        #     self.write_bbox()

    def shift_bbox(self, shift):
        """ function to shift the center of bbox while maintaining the scale of the box
        shift is a 3 element list (x shift, y shift, z shift)
        """
        bbox = self.bbox
        bbox.pose.position.x += shift[0]
        bbox.pose.position.y += shift[1]
        bbox.pose.position.z += shift[2]
        self.bbox = bbox 
    
    def resize_bbox(self, dir, size):
        """ function to increase the size of bbox along a direction"""
        if dir == "l":
            # If we want to move size (m) to the left, move the center size/2 to the left and decrease x scale by 2
            self.shift_bbox([-size/2, 0, 0])
            self.bbox.scale.x += size
        elif dir == "r":
            self.shift_bbox([size/2, 0, 0])
            self.bbox.scale.x += size
        elif dir == "u":
            self.shift_bbox([0, 0, size/2])
            self.bbox.scale.z += size
        elif dir == "d":
            self.shift_bbox([0, 0, -size/2])
            self.bbox.scale.z += size

            
        # if self.bbox:
        #     self.write_bbox()
        
            

        
    def vb(self):
        self.pub_bbox.publish(self.bbox)

    def saw_and_chisel(self):
        
        # init saw
        self.op_init("saw")

        # pre-condition
        self.saw_execute()

        # chisel 
        self.op_init("chisel")

        # start chiselling
        self.chisel_execute()

    def cut_quadrant(self, quadrant_id = None, start_index = 0, perform_4_slot_test = True):
        """ Function which runs the slot trajectory generation and cutting operations"""
        sm_init_result = self.sm_init(quadrant_id, False) # initializes the collision checking thread
        index = start_index
        INIT_COMPLETE = False
        nb_vert_slots = sm_init_result.nb_vert_slots
        nb_horiz_slots = sm_init_result.nb_horiz_slots
        slot_indices = [0, nb_vert_slots-1, nb_vert_slots, nb_vert_slots + nb_horiz_slots - 1]
        if perform_4_slot_test:
            rospy.loginfo("PERFORMING 4 SLOT TEST")
            for index in slot_indices:
                sm_result = self.sm_execute(index) # queries the slot trajectory and slot wall points
                goal = Goal()
                if not sm_result.slot_trajectory or not sm_result.slot_wall_points: # exit condition for while loop
                    rospy.logwarn("Either slot trajectory or slot wall points was not retrieved") 
                    break
                goal.slot_trajectory = sm_result.slot_trajectory
                goal.slot_wall_points = sm_result.slot_wall_points
                goal.start_index = index
                self.saw_goal = goal
                # Executes the cut  
                if not INIT_COMPLETE:
                    # only need to INIT once per quadrant
                    self.op_init("saw", index)  
                    INIT_COMPLETE = True  
                    
                saw_result = self.saw_execute()
                while True:
                    if saw_result is not None:
                        break
            rospy.loginfo("4 SLOT TEST COMPLETE")
        else:
            # cut all the slots
            while True:
                sm_result = self.sm_execute(index) # queries the slot trajectory and slot wall points
                goal = Goal()
                if not sm_result.slot_trajectory or not sm_result.slot_wall_points: # exit condition for while loop
                    rospy.logwarn("Either slot trajectory or slot wall points was not retrieved") 
                    break
                goal.slot_trajectory = sm_result.slot_trajectory
                goal.slot_wall_points = sm_result.slot_wall_points
                goal.start_index = index
                self.saw_goal = goal
                # Executes the cut  
                if not INIT_COMPLETE:
                    # only need to INIT once per quadrant
                    self.op_init("saw", index)  
                    INIT_COMPLETE = True                                        
                saw_result = self.saw_execute()
                while True:
                    if saw_result is not None:
                        break
                index += 1            
            rospy.loginfo("Finished cutting all slots of quadrant: %d", quadrant_id)

    def main(self):
        def pc():
          self.op_pause("chisel")
        def ps():
          self.op_pause("saw")
        def se(ind):
            self.saw_execute(start_index=ind)
        def si(ind = None):
            self.op_init("saw", ind)
        def ci():
            self.op_init("chisel")
        def ce():
            self.chisel_execute()
        def cs():
            self.shutdown("chisel")

        def find(name, path):
            for root, dirs, files in os.walk(path):
                if name in files:
                    if os.path.join(root, name) is not None:
                        return os.path.join(root, name)
            return False
        
        self.read_bbox()
        self.cut_quadrant(1, 0)

        resp_bbox = raw_input("\nWOULD YOU LIKE TO DRAW AN OPERATING BOX? (Y/N) \n")
        if resp_bbox[0].lower() == 'y':
            print("ALLOWING 60 SECONDS TO DRAW OPERATING BOX, OTHERWISE READING FROM FILE")
            time.sleep(60.0)
        else:
            self.read_bbox()
        if not self.bbox:            
            self.read_bbox()
        resp = raw_input("\nWOULD YOU LIKE TO WRITE CURRENT BBOX AND GLOBAL BBOX TO FILE? (Y/N)? \n")
        if (resp[0].lower() == 'y') or (not find('global_bbox_saw.yaml', os.path.expanduser("~/"))):                
            self.write_bbox()
            self.write_global_bbox()
        
        resp_quad = raw_input("\nRUNNING CUT QUADRANT OPERATION. ENTER QUADRANT ID: \n")
        resp_start_index = raw_input("\n ENTER STARTING SLOT INDEX: \n")
        
        self.cut_quadrant(int(resp_quad[0]), int(resp_start_index[0]))
        # self.cut_quadrant(1)
        

if __name__ == '__main__':
    cmder = DualClientCmd()
    if len(sys.argv) > 2:
        arg1 = sys.argv[1]
        arg2 = sys.argv[2]
    
    cmder.main()