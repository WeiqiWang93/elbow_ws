#! /usr/bin/env python

import os, sys, pdb, traceback, time, collections
import numpy as np
from enum import Enum, unique
import rospy, actionlib, threading
from bot_common_ros.msg import ActionType, ProcessStatus


class ActionServer(object):
    """An action lib server for offworld projects.

    This action lib server primarily interacts with the overseer

    Attributes:
    """    

    def __init__(self, action_name, action_type):
        self._action_name           = action_name # full node name
        # pdb.set_trace()
        self._as                    = actionlib.SimpleActionServer(self._action_name, action_type, execute_cb=self.new_goal_callback, auto_start=False)
        self._cbs                   = None
        self._action_type_enum      = self._enumerate_message('ActionType', ActionType)
        self._process_status_enum   = self._enumerate_message('ProcessStatus', ProcessStatus)
        self._feedback              = None
        self._result                = None

    def _enumerate_message(self, name, message_class):
        """Convert string to Enums

        Accepts a message type object, converts a message type object to string,
        parses the string line after line and converts key, value pairs into 
        enums.

        Args:
            name: name of the Enum
            message: a message type object

        Returns:
            An enum created from the message object

        Raises:
            AssertionError
        """
        key_value_pair = []
        assert(message_class is not None)    
        assert(dir(message_class) is not None)    
        
        # TODO: to review, hack
        for key in dir(message_class):
            if key.isupper():
            
                # set the class attributes to the Enum values
                key_value_pair.append((key, getattr(message_class, key)))
        
        return Enum(name, key_value_pair)

    def start(self):
        """Start the action lib server

        Raises:
            AssertionError
        """
        assert(self._cbs is not None)
        self._as.start() # start the server

    def register_callbacks(self, cbs): 
        """Registers the call back function

        The action lib server needs a call back function whenever is goal is sent 
        to the server. This registers the call back function for the action lib server.

        Args:
            cbs:
                A dictionnary with each pair as follows:
                   key as an ActionType value (eg ActionType.
                   value as a call back function 

        Raises:
            AssertionError
        """
        assert isinstance(cbs, dict) # cbs must be a dictionary
        cbs_key_list = list(cbs.keys())
        enum_key_list = [enum.value for enum in self._action_type_enum]
        
        # TODO replace by check for compulsory keys only, all action types do not apply to all actions
        #assert(len(set(enum_key_list) - set(cbs_key_list)) == 0)

        self._cbs = {}
        for key in cbs_key_list: # check syntax
            print "Registered callback for action %s" % key
            self._cbs[key] = cbs[key]
        rospy.logdebug("Callbacks registered.")

    def _get_action_from_goal(self, goal): 
        """Find enum for the goal type

        The goal.action_type field contains the index of the action to take. This finds the 
        enum that represents the action based on the goal.action_type
oa
        Args:
            goal:
                the action goal received from the action lib client

        Returns:
            An enum representing the goal action or None
        """
        # assume the goal msg has attribute ActionEnum which is an int32
        try:
            type_enum = self._action_type_enum(goal.action_type.type)
            #rospy.loginfo("Action server requested action {}".format(type_enum))
            return type_enum
        except:
            return None

    def _fill_feedback(self):
        """ Must be overrriden in the child class
        """
        # To be overrided in child
        raise Exception("Must override in child.")
    
    def _fill_result(self):
        """ Must be overrriden in the child class
        """
        # To be overrided in child
        raise Exception("Must override in child.")

    def send_feedback(self):
        self._fill_feedback()
        self._as.current_goal.publish_feedback(self._feedback)

    def _finish(self, goal_handle):
        self._fill_result()
        action_type = self._get_action_from_goal(goal_handle.get_goal())
        rospy.logdebug('{}: completed requested action -- {}'.format(self._action_name, action_type))
        goal_handle.set_succeeded(self._result)

    def _preempted(self, goal_handle):
        # self._fill_result()
        action_type = self._get_action_from_goal(goal_handle.get_goal())
        rospy.logdebug('{}: preemption complete of action -- {}'.format(self._action_name, action_type))
        # goal_handle.set_canceled(self._result)

    def _aborted(self, goal_handle):
        self._fill_result()
        action_type = self._get_action_from_goal(goal_handle.get_goal())
        rospy.logdebug('{}: aborted requested action -- {}'.format(self._action_name, action_type))
        goal_handle.set_aborted(self._result)

    def get_goal_msg(self):
        """ Use action-server API to obtain current goal msg
        """
        goal_handle = self._as.current_goal
        
        goal = goal_handle.get_goal()
        return goal
     
    def new_goal_callback(self, goal_handle):
        """Excecute call back function based on the goal type

        The goal.type contains index of the action to execute, based on 
        the type find an appropriate call back function and execute it.

        Args:
            goal_handle: ServerGoalHandle
                goal sent from an action lib client
        """
        goal_handle = self._as.current_goal
        success = False
        goal = goal_handle.get_goal()
        
        # get the enumeration
        action_type = self._get_action_from_goal(goal)
        if action_type is not None and action_type.value in self._cbs.keys():
            
            rospy.logdebug('Executing action %s %s' % (self._action_name, action_type))
            # call the correct callback
            success, preempted = self._cbs[action_type.value](goal)

            # fill out feedback
            self._fill_feedback()
            goal_handle.publish_feedback(self._feedback)

            # return if preempted, takes precedence over 'success' state
            if preempted:
                self._preempted(goal_handle)
            else:
                # return if finished
                if success:
                    self._finish(goal_handle)
                else:
                    # specify failure
                    self._aborted(goal_handle)
                
# ---------------------------------------------------------------------------------------------------------------------
class ActionServerError(Exception):
    """Action Server Exception"""
    pass

class FSMActionServer(ActionServer):
    """ This is an action-server AND an FSM
        User defines transitions in an FSM, and a mapping from 'operations' and 'states'
        This mapping is not necessarily 1-to-1, meaning that the inheritor may have
        many states and few operations or vice-versa.

        Paradigm:
            Action-server operations update the FSM state, and the FSM callbacks are where all the logic lives.
            The infinite loop is run on the main thread which periodically checks for blocking flags at evenly spaced "gates" in the code
    """
    def __init__(self, action_name, action_type, state_structure, initial_state):
        super(FSMActionServer, self).__init__(action_name, action_type)
        self.pub_status = rospy.Publisher(self._action_name+"/FSM/state", state_structure, queue_size=5, latch= True)
        rospy.sleep(0.5)
        #
        self._as.register_preempt_callback(self._save_preempt_status)
        #
        self.STATES = state_structure
        self._state_enums = self._enumerate_message('states', state_structure)
        #
        self.current_state = None
        self.pause_state = None # used in psuedo "pause" state to allow quick return to state prior to pausing
        self.continue_state = None
        self.previous_states = collections.deque(maxlen=50)
        self._goal = None
        # Add a variable to store the previous goal
        self._prev_goal = None
        self._recent_goal_continued = False
        self._fsm_recent_goal_preempted = False
        self._as_recent_goal_preempted = False
        self._fsm_aborted = False
        self._op_wait_states = None
        self._status_mapping = None
        self._update_action = False
        self._update_action_without_pause = False
        self.fsm_gate = threading.Event()
        #
        self.update_status(initial_state)
        self.fsm_gate.set()

    # def variable_checker(self, var_name, goal):
    #     var_limits = self._var_limits
    #     var_types = self._var_types
    #     var_obj = getattr(goal, var_name)
    #     if var_obj <= var_limits[(var_name, 0)] or var_obj > var_limits[(var_name, 1)] or not isinstance(var_obj, var_types[var_name]):
    #         rospy.loginfo("{} Invalid.".format(var_name))
    #         return False
    #     else:
    #         return True

    # def goal_valid(self, goal):
    #     """ Checks if an incoming goal is valid
    #     """
    #     action_type = self._get_action_from_goal(goal)
    #     var_list = self._vars[action_type.value] # get the list of variables associated with that operation
    #     for var_name in var_list:
    #         if not variable_checker(self, var_name, goal):
    #             return False
    #     return True
        
    def update_status(self, status, publish=False):
        """ update class status with checks
        """
        #assert( isinstance(status, self.STATES) )
        if self.get_state() is not None:
            self.previous_states.append( self.get_state() )
        enum = self._state_enums(status)
        self.current_state = status
        self.pub_status.publish(self.STATES(status = status, text = enum.name))
        rospy.logdebug("Transitioned to state: {}".format(enum))
    
    def revert_state(self):
        """ revert state to previous """
        if self.previous_states > 0: # checks for empty
            self.update_status(self.previous_states.pop())

    def print_prior_states(self):
        """ print stack of prior states """
        rospy.logdebug("Prior States:")
        for state in self.previous_states:
            enum = self._state_enums(state)
            rospy.logdebug(enum)

    def check_status(self, status):
        """ allows list inputs. Outputs True if current state is any listed state
        """
        #assert( isinstance(status, self.STATES) )
        # print self.current_state
        if isinstance(status, list):
            return np.any( [self.current_state == ss for ss in status] )
        elif isinstance(status, int):
            return self.current_state == status
        else:
            raise Exception("Unknown state type")

    def check_flags(self):
        """ Check operational flags
        """
        self.gate() # check the gate first, so that shutdown may occur immediately afterwards+
        preempted = self.fsm_is_preempted()
        return preempted

    def get_status_change_from_action(self, action_type_value):
        if self._status_mapping is None:
            return None
        else:
            state = self._status_mapping[action_type_value]
            return state

    def get_valid_op_transitions(self, state):
        """Get valid operation transitions for 'state'
        """
        #assert( isinstance(state, self.STATES) )
        if state in self._tns.keys():
            out = self._tns[state]
        else:
            out = None
        return out

    def get_valid_op_wait_states(self, state):
        """Get valid operation transitions for 'state'
        """
        #assert( isinstance(state, self.STATES) )
        if state in self._op_wait_states.keys():
            out = self._op_wait_states[state]
        else:
            rospy.logdebug("Caution, get_valid_op_wait_states returned None.")
            out = None
        return out

    def check_op_transition_valid(self, status):
        """Ensure the state transition due to an operation is valid in the FSM
        """
        if self.check_status(self.STATES.PAUSED):
            state = self.pause_state
        else:
            state = self.get_state()
        valid = False
        if state in self._tns.keys():
            valid_op_transitions = self.get_valid_op_transitions(state)
            if valid_op_transitions is not None:
                valid = status in valid_op_transitions
        #
        if not valid:
            rospy.logdebug("Invalid state transition. Returning.")
        return valid

    def get_state(self):
        return self.current_state

    def is_locked(self):
        locked = not self.fsm_gate.is_set()
        return locked

    def lock_gate(self):
        """ Abstraction on the threading object
        """
        self.fsm_gate.clear()
    
    def as_pause(self, goal, wait = True):
        """ Called when user clicks 'pause'. Acts as a psuedo-state
        Called by AS thread
        """
        self.lock_gate()
        if wait:
            success, preempted = self.wait_for_complete(self.STATES.PAUSED)
        else:
            success, preempted = True, False
        return success, preempted

    def unlock_gate(self):
        """ Abstraction on the threading object
        """
        self.fsm_gate.set()
    
    def execute_continue(self, goal, wait = True):
        """
        when user performs execute -> pause -> execute
        """
        self._recent_goal_continued = True
        self.unlock_gate()

        if wait:
            valid_final_states = self.get_valid_op_wait_states(self.pause_state)
            success, preempted = self.wait_for_complete(valid_final_states)
        else:
            success, preempted = True, False
        return success, preempted

    
    def as_continue(self, goal, wait=True):
        """ Called when user clicks 'continue'. Acts as a psuedo-state
        Called by AS thread
        """
        self._recent_goal_continued = True
        rospy.loginfo("Updating previous goal")
        self._prev_goal = self._goal
        self.unlock_gate()

        if wait:
            valid_final_states = self.get_valid_op_wait_states(self.pause_state)
            success, preempted = self.wait_for_complete(valid_final_states)
        else:
            success, preempted = True, False
        return success, preempted

    def PAUSED(self):
        """ Called by FSM thread
        """
        self.pause_state = self.get_state() # the state FSM was in before 'op-pause' was called
        self.continue_state = self.pause_state
        self.update_status(self.STATES.PAUSED)
    
    def CONTINUE(self):
        """ Called by FSM thread
        """
        self.update_status(self.continue_state)
        self.pause_state = None

    def _save_preempt_status(self):
        """BUG: this is called immediately, while the transition to PAUSED state takes longer,
        so the code preempts but doesn't catch that it should be waiting at the gate.
        Used by main-loop to exit a callback early. DO NOT USE FROM ACTION-SERVER THREAD.
        Instead use self._as.is_preempt_requested() since this is specific to the current request
        """
        """ Sets to True unless new action is PAUSE, then it passes
        """
        rospy.logdebug("------------------ save preempt status called -----------------")
        # pdb.set_trace()
        gh = self._as.next_goal.get_goal()
        if gh is None:
            rospy.logdebug("Next Goal was None")
            return
        else:
            action_type = self._get_action_from_goal(gh)
            rospy.logdebug("action-type: {}".format(action_type.value))
            # check for continued 
            if action_type.value == ActionType.CONTINUE or action_type.value == ActionType.PAUSE or \
                        self.check_op_transition_valid(self.get_status_change_from_action(action_type.value)):
                rospy.logdebug("Recent goal preempted: {}".format(self._get_action_from_goal(self.get_goal_msg())))
                self._as.set_preempted()

    def is_continued(self):
        """ check for valid exit from PAUSED state """
        return self._recent_goal_continued
    
    def is_update_active(self):
        """ check if we have updated and executed with pause"""
        return self._update_action
    
    def is_update_active_no_pause(self):
        """check if we have updated and executed without pause"""
        return self._update_action_without_pause

    def reset_continued(self):
        """ reset boolean for valid exit from PAUSED state """  
        self._recent_goal_continued = False
        self._update_action = False
        self._update_action_without_pause = False

    def fsm_is_preempted(self):
        """ return if the FSM has been preempted. Assumes incoming action is 'valid' and not 'pause' nor 'continue'
        """
        return self._fsm_recent_goal_preempted

    def reset_fsm_preempted(self):
        """ reset fsm-preemption flag. To be ran from the FSM side """
        self._fsm_recent_goal_preempted = False

    def as_assert(self, b_flag):
        """ takes a boolean flag as input, and if the flag is false, it aborts the current action.
        Usage: same as built-in 'assert', but this way the server simply aborts instead of crashes.
        example: self.as_assert( important_variable > 2.0 )
        """
        if not b_flag:
            # exc_type, exc_value, exc_traceback = sys.exc_info()
            # traceback.print_tb(exc_traceback, limit=2, file=sys.stdout)
            rospy.logerr("{}: Assertion failed. Action Aborting. Server still running. Please correct parameter as outlined in traceback and re-run.".format(self._action_name))
            #
            self._fsm_aborted = True
            raise ActionServerError

    def gate(self):
        """ check gated transition between state[x] and PAUSED through ops pause & continue
        """
        locked = self.is_locked()
        if locked:
            self.PAUSED() # pause at locked gate
            self.fsm_gate.wait() # wait for gate to unlock
            self.CONTINUE() # continue through open gate

    def fsm_register_op_variables(self, mapping):
        assert isinstance(mapping, dict) # mapping must be a dictionary
        mapping_key_list = list(mapping.keys())

        self._vars = {}
        for key in mapping_key_list:
            val = mapping[key]
            if val is not None:
                if not isinstance(val, list): val = [val]
                self._vars[key] = val[:] # shallow copy
        rospy.logdebug("Op Variables registered.")
    
    def fsm_register_variable_limits(self, mapping):
        assert isinstance(mapping, dict) # mapping must be a dictionary
        mapping_key_list = list(mapping.keys())

        self._var_limits = {}
        for key in mapping_key_list:
            val = mapping[key]
            if val is not None:
                if not isinstance(val, tuple): val = (val[0], val[1])
                self._var_limits[key] = val[:] # shallow copy
        rospy.logdebug("Variable Limits registered.")

    def fsm_register_variable_types(self, mapping):
        assert isinstance(mapping, dict) # mapping must be a dictionary
        mapping_key_list = list(mapping.keys())

        self._var_types = {}
        for key in mapping_key_list:
            val = mapping[key]
            if val is not None:
                if isinstance(val, type):
                    self._var_types[key] = val # shallow copy
        rospy.logdebug("Variable Types registered.")

    def fsm_register_op_transitions(self, mapping):
        """Register the valid transitions for operations in the FSM
        """
        assert isinstance(mapping, dict) # mapping must be a dictionary
        mapping_key_list = list(mapping.keys())

        self._tns = {}
        for key in mapping_key_list:
            val = mapping[key]
            if val is not None:
                if not isinstance(val, list): val = [val]
                self._tns[key] = val[:] # shallow copy
        rospy.logdebug("Op Transitions registered.")

    def wait_for_complete(self, valid_final_states):
        """ wait for the FSM to complete its state transition, while allowing _as preemption
        """
        # BUG: if itself is valid state, then returns immediately
        rr = rospy.Rate(1.0)
        # print "valid states here", valid_final_states
        #
        done = False
        preempted = False
        success = True
        while not done and not rospy.is_shutdown():
            if not self._as.is_active():
                preempted = True
                break
            done = self.check_status(valid_final_states)
            # if done:
            #     print "Reached a valid final state"
            # else:
            #     print "following final states not reached"
            #     print valid_final_states
            done = done or self._fsm_aborted
            rr.sleep()
        if self._fsm_aborted:
            # reset abort status
            success = False
            self._fsm_aborted = False
        return success, preempted

    def fsm_register_op_waits(self, mapping):
        """Register the final state to wait for when processing an operation
        """
        assert isinstance(mapping, dict) # mapping must be a dictionary
        mapping_key_list = list(mapping.keys())
        
        self._op_wait_states = {}
        for key in mapping_key_list:
            val = mapping[key]
            if val is not None:
                if not isinstance(val, list): val = [val]
                self._op_wait_states[key] = val[:] # shallow copy
        rospy.logdebug("Op Wait States registered.")

    def fsm_register_status_changes(self, mapping):
        """ Register the mapping from operations to state changes. Keys are ActionType.type, Values are status_msg.status
        Assumes 1 state transition per operation
        Convert state changes to "callbacks" in order to interface with 
        register_callbacks function naturally
        """
        assert isinstance(mapping, dict) # mapping must be a dictionary
        mapping_key_list = list(mapping.keys())
        self._status_mapping = mapping

        def fcn_maker(key):
            def fcn(goal):
                state_values = self.get_status_change_from_action(key)
                success = False; preempted = False # default values
                if self.check_op_transition_valid(state_values): 
                    b_fsm_preempted = True
                    # set so that wait_for_complete will set fsm_preemption
                    self.continue_state = state_values
                    # save the previous goal if possible
                    if self._goal is not None:
                        rospy.loginfo("Updating previous goal")
                        self._prev_goal = self._goal
                    self._goal = goal
                    # set the start index same as previous goal for convenience for python commander
                    # TODO: move this into application (saw)
                    # pdb.set_trace()
                    # if self._goal.skip_slot is True:
                    #     self._goal.start_index = self._prev_goal.start_index
                    # The check is for convenience to switch directly from PAUSED to a different valid current state
                    if self.check_status(self.STATES.PAUSED):
                        if self.continue_state == self.pause_state:
                            # ex: execute -> pause -> execute or execute -> pause -> aco
                            self._update_action = True
                            b_fsm_preempted = False
                        # wait for completion of prior state
                        print("pause state: {}".format(self.pause_state))
                        valid_final_states = self.get_valid_op_wait_states(self.pause_state)
                        self.execute_continue(goal, wait=False)
                    else:
                        if self.check_status(self.continue_state):
                            # ex: execute -> execute
                            self._update_action_without_pause = True
                            b_fsm_preempted = False
                        
                        self.update_status(state_values)
                        valid_final_states = self.get_valid_op_wait_states(self.get_state())
                        
                        # TO-DO: perform update status before obtaining valid final states.
                        
                    # want to change _fsm_recent_goal_preempted after a new state has been set, so that the fsm-preemption has a new state to go to
                    if b_fsm_preempted:
                        self._fsm_recent_goal_preempted = True

                    success, preempted = self.wait_for_complete(valid_final_states)                     
                else:
                    rospy.logdebug("_as: Transition not valid. Returning.")
                return success, preempted
            return fcn
        
        cbs = {}
        for key in mapping_key_list:
            fcn = fcn_maker(key)
            cbs[key] = fcn

        # special lock and release mechanism for pause / continue
        cbs[ActionType.PAUSE] = self.as_pause
        cbs[ActionType.CONTINUE] = self.as_continue

        super(FSMActionServer, self).register_callbacks(cbs)

    def fsm_register_state_functions(self, fxns):
        """Registers one function per state to be called in the FSM-loop
        """
        assert isinstance(fxns, dict) # cbs must be a dictionary
        fxns_key_list = list(fxns.keys())

        self._fxns = {}
        for key in fxns_key_list: # check syntax
            print("Registered function for state {}".format(key))
            self._fxns[key] = fxns[key]

    def fsm_loop(self):
        """ Run the FSM game-loop
        """
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            self.reset_fsm_preempted()
            self.reset_continued()

            state = self.get_state()
            if state is None:
                rospy.logerr("State is None. Error")
                continue
            #
            if state not in self._fxns.keys():
                rospy.logdebug("{}: no function for given state. May be a bug".format(state))
                continue
            ###
            # call the correct callback
            try:
                self._fxns[state](self._goal)
            except ActionServerError as error:
                # current state aborted, so return to the previous state
                rospy.logerr(traceback.format_exc())
                rospy.logdebug("{}: Aborted, reverting to previous state".format(self._state_enums(state)))
                # self.print_prior_states()
                # pdb.set_trace()
                self.revert_state()

            # set result
            self._fill_result()
