



'''
actionserver.py notes:
ArmStatus isn't finite state, it can be both deployed and stuck
execute_callback should set self.goal_id, self.text from the action msg
line 53 Feedback() doesn't exist --> I corrected by importing [ACTION]Feedback as Feedback
line 54 Result() doesn't exist --> I corrected by importing [ACTION]Result as Result
line 101 goal contains attribute action_type which has attribute type, so this should read goal.action_type.type
line 136 let's assume goal msg has an attribute 'config', and pass that into the callback
line 143 replace 'success' with 'done'
line 143 you should call a self._done which must be overwritten in the child to set 'done'
'''

from bot_common_ros.msg import CHISELGoal as Goal, CHISELFeedback as Feedback, CHISELResult as Result
from bot_common_ros.msg import ActionType, ProcessStatus, ArmStatus
from bot_common_ros.scripts.actionserver import ActionServer

from bot_perception_ros.msg import CHISELBOT_STATUS, CHISELBOT_COMMAND

class ChiselActionServer(ActionServer):
    def __init__(self, **kwargs):
        super(ChiselActionServer, self).__init__(**kwargs)

        '''
        this interfaces with the chiselbot server over ROS
        this allows chiselbot and chiselactionserver to operate on different threads, so the chiselbot
        can be running continuous actions and then pause / continue when the action server says so.
        '''

        self.ACTIVE = False
        self.command = CHISELBOT_COMMAND()
        self.command_pub = rospy.Publisher("/chiselbot/command", CHISELBOT_COMMAND, queue_size=10)
        sub_status = rospy.Subscriber("/chiselbot/status", CHISELBOT_STATUS, self.save_status)

        callbacks = {ActionType.INIT:     self.INIT,
                     ActionType.RESET:    self.RESET,
                     ActionType.GOAL:     self.GOAL,
                     ActionType.CONTINUE: self.CONTINUE,
                     ActionType.PAUSE:    self.PAUSE,
                     ActionType.SHUTDOWN: self.SHUTDOWN
                    }
        self.register_callbacks(callbacks)

    def save_status(self, msg):
        self.chiselbot_status = msg.type

    def action_server_done(self):
        done = self._as.is_active()
        return done

    def INIT(self, config={}):
        useML = <> #TODO
        start_chiselbot(useML) #TODO

    def RESET(self, config={}):
        self.command.type = CHISELBOT_COMMAND.RESET
        self.command_pub.publish(self.command)

    def GOAL(self, config={}):
        done = self.action_server_done() # I think this is right?
        rospy.loginfo("Start Chiseling.")
        while not done
            self.action()
            done = self.action_server_done()
            self._fill_feedback()
            self._as.publish_feedback(self._feedback)

    def CONTINUE(self, config={}):
        if self.chiselbot_status == CHISELBOT_STATUS.PAUSED:
            self.GOAL(config=config)

    def PAUSE(self, config={}):
        if self.chiselbot_status != CHISELBOT_STATUS.PAUSED:
            self.command.type = CHISELBOT_COMMAND.PAUSE #TODO
            self.command_pub.publish(self.command)

    def SHUTDOWN(self, config={}):
        self.publish_status(CHISELBOT_STATUS.CLOSED)
        self.close()

    def getStatus(self):
        status = self.chiselbot.ARM_STATUS.type #TODO
        pstatus = self.chiselbot.PROCESS_STATUS.type #TODO
        return status, pstatus

    def _fill_feedback(self):
        status, pstatus = self.getStatus()

        self._feedback.arm_status.status = status
        self._feedback.process_status.status = pstatus
        self._feedback.totalVolume = self.chiselbot.totalVolume #TODO

    def _fill_result(self):
        status, pstatus = self.getStatus()
        
        self._result.arm_status.status = status
        self._result.process_status.status = pstatus

    def _done(self):
        ''' determine if server is in terminal state '''
        done = False
        if self.chiselbot_status == CHISELBOT_STATUS.CLOSED: done = True
        if < nothing left to chisel here >: done = True #TODO
        return done