import socket
import select
import struct
import time
import sys
import threading
import Queue
import math3d as m3d
import numpy as np
import math, tf
import pdb
import commands
import re
import os
import signal
import ntpath

import sys, yaml
import roslib; roslib.load_manifest('motoman_driver')
import rospy, rosbag
from sensor_msgs.msg import JointState 
from enum import IntEnum

from motoman_hse_constants import *
from bot_common_ros.ur_utils import list_to_tf, tf_to_list, tf_to_arr

'''
This class creates a wrapper for the Motoman FS100 High-Speed Ethernet (HSE server)
Implements ArmServer commands that are not already supported by MotoROS
movel, get_pose, servo_on, power_off, get_torque, stepl

sia20_driver.py acts as a simple wrapper around the functions
in this class

motoman_hse_constants.py 
'''


''' MOUNT_TO_BASE stuff for movel tf'''
xr = -90.0 / 180.*math.pi 
yr = 0.0   / 180.*math.pi
zr = 90.0  / 180.*math.pi

MOUNT_TO_BASE = m3d.Transform() # default is zero in the vector
MOUNT_TO_BASE.orient.rotate_zt(zr)
MOUNT_TO_BASE.orient.rotate_xb(xr)
MOUNT_TO_BASE.orient.rotate_yt(yr)

#MOUNT_TO_BASE.orient.rotate_zb( 180.0 / 180.0 * np.pi)
GRAVITY_VEC_mount = 9.81*(MOUNT_TO_BASE.inverse * m3d.Vector.e2) # POSITIVE z-direction, NOT NEGATIVE

# TODO: currently throttled to 20 Hz but can probably improve? requires testing
UDP_DELAY = 0.05

class GoalStateEnums(IntEnum):
    """ Defines enums for goal_state_queue
    STATE_PRIMARY: list of "primary" goal, ie pose-goal for movel, joint-goal for movej
    STATE_SECONDARY: list of "secondary" goal, ie pose-goal for movelj
    MOTION_TYPE: string of motion type, ie "movel", "movej", "movelj"
    TIMEOUT: timeout value for motion_complete
    """

    STATE_PRIMARY = 0
    STATE_SECONDARY = 1
    MOTION_TYPE = 2
    TIMEOUT = 3

class MotomanHSE(object):

    
    def __init__(self, debug_mode=False):

        # Network settings
        self.UDP_TX_IP = rospy.get_param("/robot_ip_address", "192.168.50.40")
        self.UDP_RX_IP = "0.0.0.0"
        self.UDP_PORT = 10040
        self.UDP_FILE_PORT = 10041

        # setup UDP socket
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Internet, HSE
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # self.udp_socket.setblockingblocking(0)

        # tx_queue 
        self.tx_queue = Queue.Queue(maxsize=0)
        self.move_queue = Queue.Queue(maxsize=0)

        # Errors
        self.added_status_code = 0

        # Alarms
        self.alarm_code = -1
        self.abandon_motion = False

        # Goal pose stuff
        self.goal_state_queue = Queue.Queue(maxsize=0)
        self.current_pose_list = [0,0,0,0,0,0,0]

        self.pose_hist = []
        self.pose_deriv_hist = []
        self.pos_deriv = [0,0,0,0,0,0,0]
        self.time_deriv_hist = []
        self.velocity = [0,0,0,0,0,0,0]

        self.goal_state    = [0,0,0,0,0,0,0]
        self.last_msg = ""
        self.last_motion_msg = ""
        self.joint_torque = [0,0,0,0,0,0,0]
        self.joint_torque_prev = []
        self.force_data = [0,0,0,0,0,0]
        self.motion_done = True
        self.move_queue_override = None
        self.motion_type = None

        # Get status info
        self.in_teach_mode = False
        self.in_hold = False
        self.alarm_triggered = False
        self.alarm_reset_in_progress = False

        self.is_robot_moving = False
        self.move_cmd_already_sent = False
        self.speed = 0
        self.last_estop_time = 0
        self.last_hold_time = 0
        self.hold_timer_counter = 0
        self.wait_for_motion = False
        self.servos_off_counter = 0
        self.estop_enabled = False

        # Jobs
        self.job_name = None
        self.line_num = None 
        self.step_num = None 
        self.speed_override_value = None

        # throttling cmd requests
        self.debug_mode = debug_mode
        self.last_torque_time = 0
        self.torque_cmd_hz = 20.0 # throttle torque cmd requests to 5 Hz
        self.last_status_time = 0
        self.status_cmd_hz = 20.0 # throttle status cmd requests to 10 Hz

        self.HSE_REQ_ID_TX = [0]
        self.HSE_REQ_ID_MOVE = [0]

        self.exit = False
        self.form_byte = -1

        # movej joint params
        self.pulse_params = rospy.get_param("/pulse_limits_sia20d")
        self.joint_names = ["s","l","e","u","r","b","t"]
        self.pulse_per_deg = []
        self.joints_max_speed = []
        self.max_speed = dict.fromkeys(self.joint_names, 0)
        for n in self.joint_names:
            self.pulse_per_deg.append(self.pulse_params["joint_" + n]["pulse_per_deg"])
            self.joints_max_speed.append(self.pulse_params["joint_" + n]["max_speed_rad"])
        # rospy.loginfo(self.pulse_per_deg)

        self.manipulator_operating_warning = False
        self.send_attempts = 1

        # Bind UDP socket
        self.udp_bind = False
        try:
            # Check the socket
            (status, text) = commands.getstatusoutput('netstat -tulpn | grep 10040')

            # Parse netstat text output to find pid and kill process
            if text.find('10040') != -1:
                integers = map(int, re.findall(r'\d+', text))
                pid = integers[-1]
                rospy.logwarn("Running 'kill -9 {}' to terminate last HSE UDP socket".format(pid))
                os.kill(int(pid), signal.SIGKILL)
                time.sleep(1.0)

            self.udp_socket.bind((self.UDP_RX_IP, self.UDP_PORT))
            self.udp_bind = True
        except:
            rospy.logerr("FATAL ERROR: HSE Driver udp_socket failed to bind. SIA20 driver failed to initialize.")
            self.exit = True
            self.power_off()
            return

        # Create rx thread
        self.udp_rx = threading.Thread(target=self.rx_thread,args=(self.UDP_PORT,))   
        self.udp_rx.setDaemon(True) 
        self.udp_rx.start()
        self.current_time = rospy.get_time()

        # create TX thread
        self.udp_tx = threading.Thread(target=self.tx_thread,args=(self.tx_queue,))  
        self.udp_tx.setDaemon(True)  
        self.udp_tx.start()

        # create move thread
        self.udp_move = threading.Thread(target=self.move_thread,args=(self.move_queue,))  
        self.udp_move.setDaemon(True)  
        self.udp_move.start()

        # locks 
        self._rx_lock = threading.Lock()
        self._tx_lock = threading.Lock()
        self._rx_lock.acquire()

        self.prev_joints_goal = None
        self.prev_pose_goal = None

        sub_joints = rospy.Subscriber('/saw/joint_states', JointState, self.save_joints)

    ''' UTILITY/HELPER FUNCTIONS '''

    # append data to msg (data can be list or int)
    def append_msg(self, msg, data):
        try:
            if isinstance(data, list):
                for x in range(len(data)):
                    msg.append(data[x])
            else:
                msg.append(data)
        except:
            self.print_debug("MotomanHSE Warning: Dropped byte in append_msg()", lvl="logwarn")

    # returns list of info from the cmd packet
    # cmd_info = [req_id, cmd_id, cmd_srv]
    # indices defined as CMD_INFO_REQ_ID = 0, CMD_INFO_ CMD_ID = 1, CMD_INFO_SRV = 2
    def cmd_info(self, msg):

        # req_id = 0
        req_id = self.struct_unpack("<B", msg[11:12], "cmd_info")
        cmd_id = self.struct_unpack("<h", msg[24:26], "cmd_info")
        cmd_srv = self.struct_unpack("<B", msg[29:30], "cmd_info")

        cmd_info = [req_id, cmd_id, cmd_srv]
        return cmd_info

    # increments req_id for all outgoing messages
    def increment_req_id(self, id):
        id[0] += 1
        id[0] = id[0] % 256
        return

    # Convert 32-bit int to little-endian hex packet
    def int_to_hex(self, num):
        num = int(round(num))
        msg = bytearray()
        hex_pack = struct.pack('<i', num)
        for i in hex_pack:
            msg.append(i)
        # print ','.join('{:02x}'.format(x) for x in msg)
        return msg

    # Convert 32-bit little-endian hex packet to an int
    def hex_to_int(self, msg):
        num = self.struct_unpack('<i', msg)
        return num

    # Set up default header and apply specific settings (ie data size, cmd id, instance, attribute, service)
    def format_msg_header(self, msg, data_size, cmd_id):
        data_bytes = data_size # TODO 
        cmd_bytes = cmd_id # TODO 

        packet = HSE_HDR + HSE_HDR_SIZE + data_bytes + HSE_RES_1 + \
                HSE_PROCESS_ROBOT_CTRL + HSE_ACK_REQUEST + self.HSE_REQ_ID_TX + HSE_BLOCK_ID + \
                HSE_RES_2 + cmd_bytes + HSE_INST_MOVEL_2 + HSE_ATTR_GETPOS + \
                HSE_SRV_GETPOS + HSE_PADDING

        self.append_msg(msg, packet)
        return msg
    
    # Convert robot pose list (x,y,z,rx,ry,rz) to m3d.transform
    # x,y,z need to be converted mm to meters
    # rx,ry,rz are stored in degrees and need to be converted to radians
    def list_to_transform(self, pose):
        _rx,_ry,_rz = pose[3:6]

        # verify rotations are in acceptable range, apply modulo if necessary
        if _rx < -180 or _rx > 180:
            _rx = _rx % 180
        if _ry < -90 or _ry > 90:
            rospy.logerr("ry must be -pi/2 < ry < pi/2")
            return
        if _rz < -180 or _rz > 180:
            _rz = _rz % 180
        pose[3:6] = _rx, _ry, _rz

        rx,ry,rz = [math.radians(x) for x in pose[3:6]] # converting to radians
        position = [x/1000.0 for x in pose[0:3]] # convert mm to meters
        matrix = tf.transformations.euler_matrix(rx,ry,rz)[0:3, 0:3]
        rotation = m3d.Orientation(matrix)
        transform = m3d.Transform(rotation, position)

        return transform
    
    # Convert transform to robot pose list (x,y,z,rx,ry,rz)
    # x,y,z need to be converted meters to mm
    # rx,ry,rz need to be converted to degrees for robot motion
    def transform_to_list(self, transform):
        pos = transform.get_pos()[0:3]*1000.0 # convert meters to mm
        rot = tf.transformations.euler_from_matrix(transform.get_orient().get_matrix())
        rot = [math.degrees(x) for x in rot] # converting to degrees 
        pose_list = np.append(pos,rot) 
        return pose_list  

    ''' ROBOT COMMANDS (SRM TO ROBOT) '''
    def get_pose(self):
    # get robot position (use tx_queue)
        # rospy.loginfo("get_pose")
        msg = bytearray()
        packet = HSE_HDR + HSE_HDR_SIZE + HSE_DATA_SIZE_GET_POS + HSE_RES_1 + \
                HSE_PROCESS_ROBOT_CTRL + HSE_ACK_REQUEST + self.HSE_REQ_ID_TX + HSE_BLOCK_ID + \
                HSE_RES_2 + HSE_READ_ROBOT_POS_CMD_ID + HSE_INST_GETPOS_R1 + HSE_ATTR_GETPOS + \
                HSE_SRV_GETPOS + HSE_PADDING

        self.append_msg(msg, packet)

        # send to tx_queue
        self.increment_req_id(self.HSE_REQ_ID_TX)
        self.tx_queue.put(msg)

        # Returns a transform as required by SRM
        pose = self.list_to_transform(self.current_pose_list)
        return pose

    def get_joints(self):
        return self.joints

    def save_joints(self, msg):
        self.joints = np.array(msg.position[:]).tolist()

    # enable/disable robot servos (use tx_queue)
    def servo_on(self, status):
        # if status == True:
        #    rospy.loginfo("Turn servo on")
        # else:
        #    rospy.loginfo("Turn servo off")

        msg = bytearray()

        if (status):
            packet = HSE_HDR + HSE_HDR_SIZE + HSE_DATA_SIZE_SERVO + HSE_RES_1 + \
                    HSE_PROCESS_ROBOT_CTRL + HSE_ACK_REQUEST + self.HSE_REQ_ID_TX + HSE_BLOCK_ID + \
                    HSE_RES_2 + HSE_SERVO_CMD_ID + HSE_INST_SERVO_ON + HSE_ATTR_SERVO + \
                    HSE_SRV_SET_ATTR_SINGLE + HSE_PADDING + HSE_DATA_SERVO_ON
            self.append_msg(msg, packet)
        else:
            packet = HSE_HDR + HSE_HDR_SIZE + HSE_DATA_SIZE_SERVO + HSE_RES_1 + \
                    HSE_PROCESS_ROBOT_CTRL + HSE_ACK_REQUEST + self.HSE_REQ_ID_TX + HSE_BLOCK_ID + \
                    HSE_RES_2 + HSE_SERVO_CMD_ID + HSE_INST_SERVO_ON + HSE_ATTR_SERVO + \
                    HSE_SRV_SET_ATTR_SINGLE + HSE_PADDING + HSE_DATA_SERVO_OFF
            self.append_msg(msg, packet)
        
        # send to tx_queue
        self.tx_queue_clear()
        self.increment_req_id(self.HSE_REQ_ID_TX)
        self.tx_queue.put(msg)

        return msg

    # set_speed to vel (vel is int between 0-100%)
    def set_speed(self, vel=100):

        # Cast to int and clip to 1 <= vel <= 255
        vel = int(round(vel))
        if vel < 1:
            vel=1
        if vel > 255:
            vel=255

        data = [struct.pack('<B', vel)]        
        msg = bytearray()

        packet = HSE_HDR + HSE_HDR_SIZE + HSE_DATA_SIZE_SET_IO + HSE_RES_1 + \
                HSE_PROCESS_ROBOT_CTRL + HSE_ACK_REQUEST + self.HSE_REQ_ID_TX + HSE_BLOCK_ID + \
                HSE_RES_2 + HSE_IO_CMD_ID + HSE_INST_SPD_IO + HSE_ATTR_IO + \
                HSE_SRV_IO + HSE_PADDING + data

        self.append_msg(msg, packet)

        # CLEAR txqueue and send to tx_queue
        self.tx_queue_clear()
        self.increment_req_id(self.HSE_REQ_ID_TX)
        self.tx_queue.put(msg)

        # keep track of the speed
        self.speed = vel

        return msg
    
    # Return velocity in x,y,z,rx,ry,rz
    def get_speed(self):

        if isinstance(self.velocity, np.ndarray):
            self.velocity = self.velocity.to_list()

        return self.velocity

    # wrapper around reset_error() and servo_on()
    def power_on(self):
        # Enable servo
        self.exit = False
        self.reset_error()
        self.servo_on(True)
        self.hold(False)
        rospy.sleep(0.5)
        return  

    # on shutdown, clear tx_queue and power off servos
    def power_off(self):
        if self.exit == True:
            rospy.logwarn("MotomanHSE already powered off.")
            return
            
        rospy.logwarn("MotomanHSE: Power off")

        if (self.tx_queue.qsize() > 0):
            rospy.logwarn("MotomanHSE: Clearing tx_queue buffer len {:d}".format(self.tx_queue.qsize()))
        else:
            rospy.logwarn("MotomanHSE: No tx_queue buffer to clear!")

        self.stop()
        self.servo_on(False)
        rospy.sleep(0.2) # give time for servo off to be sent before ending threads

        self.exit = True 
        rospy.sleep(0.5) # give time for threads to exit

        rospy.logwarn("HSE driver power_off completed")

        return

    # get robot torque
    def get_torque(self):

        # Throttle torque cmd requests to 5 Hz
        if rospy.get_time() - self.last_torque_time > (1.0 / self.torque_cmd_hz):
            self.print_debug("qsize {}".format(self.tx_queue.qsize()))
            self.last_torque_time = rospy.get_time()

            msg = bytearray()
            packet = HSE_HDR + HSE_HDR_SIZE + HSE_DATA_SIZE_GET_POS + HSE_RES_1 + \
                    HSE_PROCESS_ROBOT_CTRL + HSE_ACK_REQUEST + self.HSE_REQ_ID_TX + HSE_BLOCK_ID + \
                    HSE_RES_2 + HSE_TORQUE_CMD_ID + HSE_INST_TORQUE_R1 + HSE_ATTR_TORQUE_NON_AXIS + \
                    HSE_SRV_TORQUE + HSE_PADDING
            self.append_msg(msg, packet)

            # send to tx_queue
            self.increment_req_id(self.HSE_REQ_ID_TX)
            self.tx_queue.put(msg)

        return self.joint_torque

    # reset_error (0x82)
    def reset_error(self):
        # CANCEL
        msg = bytearray()
        packet = HSE_HDR + HSE_HDR_SIZE + HSE_DATA_SIZE_ERROR_RESET + HSE_RES_1 + \
                HSE_PROCESS_ROBOT_CTRL + HSE_ACK_REQUEST + self.HSE_REQ_ID_TX + HSE_BLOCK_ID + \
                HSE_RES_2 + HSE_ERROR_RESET_CMD_ID + HSE_INST_ERROR_CANCEL + HSE_ATTR_ERROR_RESET + \
                HSE_SRV_SET_ATTR_SINGLE + HSE_PADDING + HSE_ERROR_RESET_DATA
        self.append_msg(msg, packet)

        # send to tx_queue
        self.increment_req_id(self.HSE_REQ_ID_TX)
        self.tx_queue.put(msg)

        # RESET
        msg = bytearray()
        packet = HSE_HDR + HSE_HDR_SIZE + HSE_DATA_SIZE_ERROR_RESET + HSE_RES_1 + \
                HSE_PROCESS_ROBOT_CTRL + HSE_ACK_REQUEST + self.HSE_REQ_ID_TX + HSE_BLOCK_ID + \
                HSE_RES_2 + HSE_ERROR_RESET_CMD_ID + HSE_INST_ERROR_RESET + HSE_ATTR_ERROR_RESET + \
                HSE_SRV_SET_ATTR_SINGLE + HSE_PADDING + HSE_ERROR_RESET_DATA
        self.append_msg(msg, packet)

        # send to tx_queue
        self.increment_req_id(self.HSE_REQ_ID_TX)
        self.tx_queue.put(msg)

        return

    def create_send_file_chunk(self, data, block_num=0):

        # Encode the data and get data size
        data = data.encode(encoding='utf-8')
        data_len = struct.pack('<H', len(data))
        data_size = [ord(data_len[0]), ord(data_len[1])]

        # Create block num
        block_num_struct = struct.pack('<I', block_num)
        block_num_list = [ord(block_num_struct[0]), ord(block_num_struct[1]), ord(block_num_struct[2]), ord(block_num_struct[3])]

        # Set ack
        if block_num == 0:
            ack = HSE_ACK_REQUEST
        else:
            ack = HSE_ACK_SEND

        self.print_debug("Chunk ack {}, block_num {:04x}".format(ack[0], block_num))

        self.HSE_REQ_ID_TX = [0]

        # Pack everything into list
        msg = bytearray()
        packet = HSE_HDR + HSE_HDR_SIZE + data_size + HSE_RES_1 + \
                HSE_PROCESS_FILE_CTRL + ack + self.HSE_REQ_ID_TX + block_num_list + \
                HSE_RES_2 + HSE_FILE_IO_CMD_ID + HSE_INST_FILE_IO + HSE_ATTR_FILE_IO + \
                HSE_SRV_LOAD_FILE + HSE_PADDING

        # Convert list to bytearray
        self.append_msg(msg, packet)
        msg += data

        return msg

    def send_file(self, filename):
        """Send a local file to pendant
        Args:
            filename (str): Path of the local file
        Raises:
            ValueError: Empty content of the local file
        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """

        # Open file and verify it's valid (should be a JBI file for now)
        try:
            with open(filename, 'rb') as f:
                context = f.read()
        except FileNotFoundError:
            rospy.logwarn("file not found: '{}'".format(filename))
            return False

        if len(context) == 0:
            raise ValueError('An empty file')
        
        # Initialize variables
        CHUNK_SIZE = 400
        block_no = 0
        send_file_msg_list = []
        sent_last_chunk = False

        # Create the initial request, send just the filename as data
        data = ntpath.basename(filename).encode(encoding='utf-8')   # removes filepath and just includes filename
        msg = self.create_send_file_chunk(data, block_num=0)
        send_file_msg_list.append(msg)

        while not rospy.is_shutdown():
            if sent_last_chunk:
                break

            # Update block_no for each iteration
            block_no += 1

            # If-statement triggered indicates this it the final chunk
            if CHUNK_SIZE * block_no >= len(context):
                data = context[(block_no - 1) * CHUNK_SIZE:]
                block_no |= 0x80000000
                sent_last_chunk = True

            # Else: send up to 400 bytes of data
            else:
                data = context[(block_no - 1) * CHUNK_SIZE:block_no * CHUNK_SIZE]
            
            # Create the data
            msg = self.create_send_file_chunk(data, block_num=block_no)
            send_file_msg_list.append(msg)

        # Put the packets into the queue
        for msg in send_file_msg_list:
            self.tx_queue.put(msg)

        return


    def delete_file(self, filename):
        """Delete a file in pendant
        Args:
            file_name (str): Name of the file to be deleted
        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        # Create the packet
        # HSE_DELETE_FILE_CMD_ID = 0x00
        # HSE_INST = 0
        # HSE ATTR 0
        # SRV 0x09
        # data = file_name.encode(encoding='utf-8')
        # HSE_DATA_SIZE = len(data)

        # Parse file name
        data = filename.encode(encoding='utf-8')
        data_size = [len(data), 0] # TODO: pack into struct

        msg = bytearray()
        packet = HSE_HDR + HSE_HDR_SIZE + data_size + HSE_RES_1 + \
                HSE_PROCESS_FILE_CTRL + HSE_ACK_REQUEST + self.HSE_REQ_ID_TX + HSE_BLOCK_ID + \
                HSE_RES_2 + HSE_FILE_IO_CMD_ID + HSE_INST_FILE_IO + HSE_ATTR_FILE_IO + \
                HSE_SRV_DELETE_FILE + HSE_PADDING

        self.append_msg(msg, packet)
        msg += data

        # send to tx_queue
        self.increment_req_id(self.HSE_REQ_ID_TX)
        self.tx_queue.put(msg)


    def select_job(self, job_name, line_num=0):
        """Select a job in pendant for later playing
        Args:
            job_name (str): Name of the job file (without .JBI extension)
            line_num (int, optional): The beginning line number when playing. Defaults to 0.
        Raises:
            ValueError: Length of the job name exceeds the maximum 32 characters.
        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        self.print_debug("select_job {} and line_num {}".format(job_name, line_num))

        # Create the packet
        # HSE_SELECT_JOB_CMD_ID = 0x87
        # HSE_INST = 1
        # HSE ATTR 1 for job name, 2 for line number, 0 for both?
        # SRV 0x02
        # HSE_DATA_SIZE = len(data) --> fixed to 36 for this

        # Parse job name and line num
        data = job_name.encode(encoding='utf-8')
        if len(data) > 32:
            raise ValueError('Job name is too long')
        data += bytearray(32 - len(data))
        data += struct.pack('<I', line_num)

        msg = bytearray()
        packet = HSE_HDR + HSE_HDR_SIZE + HSE_DATA_SIZE_SELECT_JOB + HSE_RES_1 + \
                HSE_PROCESS_ROBOT_CTRL + HSE_ACK_REQUEST + self.HSE_REQ_ID_TX + HSE_BLOCK_ID + \
                HSE_RES_2 + HSE_SELECT_JOB_CMD_ID + HSE_INST_SELECT_JOB + HSE_ATTR_SELECT_JOB + \
                HSE_SRV_SELECT_JOB + HSE_PADDING

        self.append_msg(msg, packet)
        msg = msg + data

        # send to tx_queue
        self.increment_req_id(self.HSE_REQ_ID_TX)
        self.tx_queue.put(msg)

        return

    def play_job(self):
        """Start playing a job in pendant
        Note:
            select_job() should be performed before this method.
        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        # Create the packet
        # HSE_PLAY_JOB_CMD_ID = 0x86
        # HSE_INST = 1
        # HSE ATTR = 1
        # SRV 0x10 (Set attribute single)   HSE_SRV_SET_ATTR_SINGLE
        # Data = struct.pack('<I', 1)
        # HSE_DATA_SIZE = len(data) --> fixed to 4 for this
        self.print_debug("play job")

        msg = bytearray()
        packet = HSE_HDR + HSE_HDR_SIZE + HSE_DATA_SIZE_PLAY_JOB + HSE_RES_1 + \
                HSE_PROCESS_ROBOT_CTRL + HSE_ACK_REQUEST + self.HSE_REQ_ID_TX + HSE_BLOCK_ID + \
                HSE_RES_2 + HSE_PLAY_JOB_CMD_ID + HSE_INST_PLAY_JOB + HSE_ATTR_PLAY_JOB + \
                HSE_SRV_SET_ATTR_SINGLE + HSE_PADDING + HSE_PLAY_JOB_DATA

        self.append_msg(msg, packet)

        # send to tx_queue
        self.increment_req_id(self.HSE_REQ_ID_TX)
        self.tx_queue.put(msg)

    def update_job_info(self):
        """Start playing a job in pendant
        Note:
            select_job() should be performed before this method.
        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        # Create the packet
        # HSE_PLAY_JOB_CMD_ID = 0x86
        # HSE_INST = 1
        # HSE ATTR = 1
        # SRV 0x10 (Set attribute single)   HSE_SRV_SET_ATTR_SINGLE
        # Data = struct.pack('<I', 1)
        # HSE_DATA_SIZE = len(data) --> fixed to 4 for this

        data_size = HSE_DATA_SIZE_GET_POS

        msg = bytearray()
        packet = HSE_HDR + HSE_HDR_SIZE + data_size + HSE_RES_1 + \
                HSE_PROCESS_ROBOT_CTRL + HSE_ACK_REQUEST + self.HSE_REQ_ID_TX + HSE_BLOCK_ID + \
                HSE_RES_2 + HSE_JOB_INFO_CMD_ID + HSE_INST_JOB_INFO_IO + HSE_ATTR_JOB_INFO_IO + \
                HSE_SRV_JOB_INFO  + HSE_PADDING

        self.append_msg(msg, packet)

        # send to tx_queue
        self.increment_req_id(self.HSE_REQ_ID_TX)
        self.tx_queue.put(msg)

        return True

    def get_job_name(self):
        return self.job_name

    def get_job_line_num(self):
        return self.job_line_num
    
    def get_job_step_num(self):
        return self.job_step_num

    def get_speed_override(self):
        return self.speed_override_value

    def parse_get_job_info(self, data):
        try:
            self.job_name = data[32:64].decode('utf-8').rstrip('\x00')
            # self.job_name = self.struct_unpack("<i", data[0:32], "parse_get_job_info")
            self.job_line_num = self.struct_unpack("<I", data[64:68], "parse_get_job_info")
            self.job_step_num = self.struct_unpack("<I", data[68:72], "parse_get_job_info")
            self.speed_override_value = self.struct_unpack("<I", data[72:76], "parse_get_job_info") / 100.0
            # rospy.loginfo("{}, {}, {}, {}".format(self.job_name, self.job_line_num, self.job_step_num, self.speed_override_value))
        except:
            rospy.logwarn("MotomanHSE Warning: Dropped byte in parse_get_job_info()")
            return

    def calculate_movej_vel(self, joints=[], target_vel=None):
        """ Calculates the link % speed to be set to hit the target_vel
        Compares the goal joint-state with the last joint-state in move_queue (if empty, then compare with current joint-state)
        Args: 
            joints: List of joints (rad)
            vel : Max velocity for a joint (rad/s)
        Returns: 
            Link % speed (0.01%)
        """
        try:
            if not self.goal_state_queue.empty():
                last_goal_state = self.goal_state_queue[-1]
            
                if last_goal_state[GoalStateEnums.MOTION_TYPE.value] == "movej" or last_goal_state[GoalStateEnums.MOTION_TYPE.value] == "movelj":
                    # Set the last_joints to be the last_goal_state
                    last_joints_arr = np.asarray(last_goal_state[GoalStateEnums.STATE_PRIMARY.value])
                else:
                    # Set the last_joints to be the current goal_state
                    last_joints_arr = np.asarray(self.joints)
            else:
                # Set the last_joints to be the current goal_state
                last_joints_arr = np.asarray(self.joints)
        except:
            last_joints_arr = np.asarray(self.joints)

        # Create some numpy arrays to simplify calculation
        joints_goal_arr = np.asarray(joints)
        joints_max_speed_arr = np.asarray(self.joints_max_speed)

        # Calculate the slowest_joint_idx
        slowest_joint_idx = np.argmax(np.abs(joints_goal_arr - last_joints_arr) / joints_max_speed_arr)

        # Calculate link_speed (float)
        link_speed = target_vel / self.joints_max_speed[slowest_joint_idx]

        # print time
        self.print_debug("Expected travel time: {}".format(np.abs(joints_goal_arr - last_joints_arr)[slowest_joint_idx]/target_vel))

        # Convert link_speed to 0.1% by multiplying by 1000.0 (empirically found to be 0.1 instead of 0.01%)
        link_speed *= 1000.0

        return link_speed

    def movej(self, joints=[],  acc=None, vel_rot=0, wait=True, relative=False, threshold=None, timeout=None):
        """ move-in-joint-space
        Inputs: 
            joints [rad]: List of joints 
            acc         : Currently ignored (no easy way to set accel on FS100) 
            vel_rot [rad/s] : Rotational velocity (takes priority over vel)
            wait        : Function return when movement is finished (if True)
            threshold   : Passes to motion_complete (not yet implemented)
            timeout     : timeout for motion_complete; if None given, it's calculated in this function
        Outputs: 
            transform of current pos
        """
        # Calculate velocities (0.01%)
        _vel = self.calculate_movej_vel(joints=joints, target_vel=vel_rot)

        # calculate timeout
        if timeout == None:
            # if vel_rot is not provided (case of vel given, but not vel_rot), give it a default value
            if vel_rot <= 0:
                vel_rot = np.deg2rad(5.0)

            # Calculate the timeout to be 2x of the max joint distance velocity
            joint_distances = np.abs(np.asarray(self.joints) - np.asarray(joints))
            timeout = 3.0 * np.divide(np.max(np.abs(joint_distances)), vel_rot)
            # Set min timeout of 5.0 sec
            if timeout < 5.0:
                timeout = 5.0

        # pack the ints and apply units conversions
        j_s = self.int_to_hex(math.degrees(joints[0]) * self.pulse_per_deg[0])  # convert rad to deg
        j_l = self.int_to_hex(math.degrees(joints[1]) * self.pulse_per_deg[1])
        j_u = self.int_to_hex(math.degrees(joints[3]) * self.pulse_per_deg[3])
        j_r = self.int_to_hex(math.degrees(joints[4]) * self.pulse_per_deg[4])  # unit 0.0001 deg
        j_b = self.int_to_hex(math.degrees(joints[5]) * self.pulse_per_deg[5])
        j_t = self.int_to_hex(math.degrees(joints[6]) * self.pulse_per_deg[6])
        j_e = self.int_to_hex(math.degrees(joints[2]) * self.pulse_per_deg[2])
        _spd = self.int_to_hex(_vel*10.0) # 0.1 mm/s or 0.1 deg/s
       
        # Fill out goal_state and add to queue
        goal_state = [None] * 4
        goal_state[GoalStateEnums.STATE_PRIMARY.value] = joints
        goal_state[GoalStateEnums.STATE_SECONDARY.value] = None
        goal_state[GoalStateEnums.MOTION_TYPE.value] = "movej"
        goal_state[GoalStateEnums.TIMEOUT.value] = timeout
        self.goal_state_queue.put(goal_state)

        # Construct header
        hdr = HSE_HDR + HSE_HDR_SIZE + HSE_DATA_SIZE_MOVEJ + HSE_RES_1 + \
                HSE_PROCESS_ROBOT_CTRL + HSE_ACK_REQUEST + self.HSE_REQ_ID_MOVE + HSE_BLOCK_ID + \
                HSE_RES_2 + HSE_PULSE_MOVE_CMD_ID + HSE_INST_MOVEJ_1  + HSE_ATTR_MOVEJ + \
                HSE_SRV_SETPOS + HSE_PADDING
                # HSE_INST_MOVEJ_1 for % link operation instead of speed/absolute opreation

        # Construct data
        movej_data_1 = HSE_MOVEJ_CTRL_GROUP + HSE_MOVEJ_STATION + HSE_MOVEJ_SPD_CLASS_P             # index 1-3
        movej_data_2 = _spd + j_s + j_l + j_u + j_r + j_b + j_t + j_e
        padding = bytearray() # 11 bytes of padding for unused indexes 12-22 
        for x in range(11):
            self.append_msg(padding,HSE_MOVEJ_RES)

        # Construct msg
        msg = bytearray()
        msg.extend(hdr)
        msg.extend(movej_data_1)
        msg.extend(movej_data_2)
        msg.extend(padding)

        # send to move_queue
        self.abandon_motion = False
        self.motion_done = False
        self.increment_req_id(self.HSE_REQ_ID_MOVE)
        self.move_queue.put(msg)     # move_queue needs to know its a movej

        # if wait is true, wait for move_queue to be emptied and no motion
        if wait == True:   
            self.wait_for_motion = True
            while self.wait_for_motion and not rospy.is_shutdown() == True:
                rospy.sleep(UDP_DELAY)

                if self.abandon_motion:
                    break

        # Return false if there was some error
        if self.abandon_motion:
            return False
        else:
            return True

        return

    def movejs(self, jointList=[],  acc=None, vel_rot=None, wait=True, relative=False, threshold=None, auto_recover=True):
        retval = True

        print_message = True
        while self.in_teach_mode and not rospy.is_shutdown():
            if print_message:
                rospy.logwarn("Robot in TEACH, reset to REMOTE mode. Motion command on hold.")
                print_message = False
            rospy.sleep(0.5)

        print_message = True
        while self.estop_enabled and not rospy.is_shutdown():
            if print_message:
                rospy.logwarn("Robot E-STOP enabled, please reset. Motion command on hold.")
                print_message = False
            rospy.sleep(0.5)

        # wait - if False, interrupt current movement and clear queue, then add new movement
        if self.motion_done == False and wait == False:
            rospy.logwarn("Still in motion. Stop movement and clear queue.")
            self.hold(True)
            rospy.sleep(0.05)

            rospy.logwarn("Done stopping last movejs, release hold and continue")
            self.hold(False)
            rospy.sleep(0.05)

            # Clear move queues
            self.move_queue_clear()
            self.goal_state_queue.queue.clear()
            self.motion_done = True

        for j in jointList:
            success = self.movej(joints=j, acc=acc, vel_rot=vel_rot, wait=wait, relative=relative, threshold=threshold)
            
            # Abandon motion
            if success == False:
                retval = False 
                break

        if success == False and auto_recover == True and self.prev_joints_goal is not None:
            rospy.logwarn("Attempting automated recovery to prev_joints_goal {}.".format(self.prev_joints_goal))

            # Wait for alarm_triggered to be cleared
            while self.alarm_triggered and not rospy.is_shutdown() == True:
                # rospy.logwarn("Waiting another 0.1 seconds for self.alarm_triggered to be cleared...")
                rospy.sleep(0.1)

            # Get last_joint_goal and move
            self.movejs(jointList=[self.prev_joints_goal], vel_rot=np.deg2rad(5.0), wait=True)
        
            rospy.logwarn("Automated recovery complete")

        return retval

    """ step-in-joint-space
    Inputs: 
        joints [rad]: array of step increment value for each joint (in order)
        vel_rot [rad/s] : Rotational velocity (takes priority over vel)
    """
    def stepj(self, joints=[], vel_rot=5.0, timeout=None, wait=False):

        print_message = True
        while self.in_teach_mode and not rospy.is_shutdown():
            if print_message:
                rospy.logwarn("Robot in TEACH, reset to REMOTE mode. Motion command on hold.")
                print_message = False
            rospy.sleep(0.5)

        print_message = True
        while self.estop_enabled and not rospy.is_shutdown():
            if print_message:
                rospy.logwarn("Robot E-STOP enabled, please reset. Motion command on hold.")
                print_message = False
            rospy.sleep(0.5)

        cur = self.joints
        if isinstance(joints, np.ndarray):
            step = joints.tolist()  # remove deg2rad
        else:
            step = joints
        goal = [sum(x) for x in zip(cur, step)]

        # send movej command
        self.print_debug("stepj vel_rot {} deg/s".format(np.rad2deg(vel_rot)))
        self.movej(joints=goal, vel_rot=vel_rot, timeout=timeout, wait=wait)
        return


    def movel(self, tf=MOUNT_TO_BASE, joints=[], pose=None, re=None, acc=None, vel=None, vel_rot=None, form=None, wait=False, timeout=None, threshold=None):
        """ move-in-linear-space (with pose or joints goal)
        Inputs: 
            joints [rad]: Joint-state of goal, HSE driver will use linear-move with pulse goal, this OVERRIDES the pose goal
            pose [m,rad]: Transform of pose
            re [deg]    : Elbow angle 
            acc         : Currently ignored (no easy way to set accel on FS100) 
            vel [m/s]   : Velocity (overrides vel_rot)
            vel_rot [rad/s]: Rotational velocity (used by default, but gets overrided by vel if vel is set)
            wait        : Function return when movement is finished (if True)
            timeout     : Timeout (not yet implemented)
            threshold   : Passes to motion_complete (not yet implemented)
        Outputs: 
            transform of current pos
        """
        # if joints[] and pose are both supplied, use movel_pulse
        if len(joints) > 0:
            self.movel_pulse(tf=tf, joints=joints, pose=pose, vel=vel, vel_rot=vel_rot, wait=wait, timeout=timeout, threshold=threshold)
        
        # if joints[] not supplied, use movel_cart
        else:
            self.movel_cart(tf=tf, pose=pose, re=re, vel=vel, vel_rot=vel_rot, form=form, wait=wait, timeout=timeout, threshold=threshold)

        return

    def movels(self, tf=MOUNT_TO_BASE, jointList=[], poseList_base=None, reList=None, acc=None, vel=None, vel_rot=None, form=None, wait=False, timeout=5, threshold=None, override=False):

        print_message = True
        while self.in_teach_mode and not rospy.is_shutdown():
            if print_message:
                rospy.logwarn("Robot in TEACH, reset to REMOTE mode. Motion command on hold.")
                print_message = False
            rospy.sleep(0.5)

        print_message = True
        while self.estop_enabled and not rospy.is_shutdown():
            if print_message:
                rospy.logwarn("Robot E-STOP enabled, please reset. Motion command on hold.")
                print_message = False
            rospy.sleep(0.5)

        # wait - if False, interrupt current movement and clear queue, then add new movement
        if self.motion_done == False and wait == False:
            rospy.logwarn("Still in motion. Stop HSE motion and clear queue.")
            self.hold(True)
            rospy.sleep(0.05)

            # Clear move queues
            self.move_queue_clear()
            self.goal_state_queue.queue.clear()
            rospy.loginfo("move_queue has been cleared")

            rospy.logwarn("Done stopping last movels, release hold and continue")
            self.hold(False)
            rospy.sleep(0.1) # give 100 ms for the hold to be released


            # Abandon movement
            rospy.logwarn("Abandoning motion in movels function")
            self.abandon_motion = True # tells motion_complete loop to abandon motion and assume the motion is done
            self.move_queue_override = None
            # self.motion_done = True

            rospy.logwarn("Done stopping last movels/movelj, release hold and continue")
            self.hold(False)
            rospy.sleep(0.05)

        if reList is None:
            reList = [0.0] * len(poseList_base)

        # Create empty lists if poseList_base, jointList, reList are empty
        if poseList_base is None and jointList is not None:
           poseList_base = [0.0] * len(jointList)        
        if jointList is None and poseList_base is not None:
           jointList = [0.0] * len(poseList_base)

        self.print_debug("movelj called on len: {}".format(len(poseList_base)))
        for i in range(len(poseList_base)):
            retval = self.movel(tf=MOUNT_TO_BASE, joints=jointList[i], pose=poseList_base[i], re=reList[i], acc=acc, vel=vel, vel_rot=vel_rot, form=form, wait=wait)
            rospy.sleep(0.01)
            # Abandon motion
            if retval == False:
                rospy.logerr("Abandon movels")
                return False 
        
        return True

    def movel_cart(self, tf=tf, pose=None, re=None, vel=None, vel_rot=None, form=None, wait=False, timeout=None, threshold=None):
        """ move-in-linear-space (movel)
        Inputs: 
            pose [m,rad]: Transform of pose
            re [deg]    : Elbow angle 
            acc         : Currently ignored (no easy way to set accel on FS100) 
            vel [m/s]   : Velocity (overrides vel_rot)
            vel_rot [rad/s]: Rotational velocity (used by default, but gets overrided by vel if vel is set)
            wait        : Function return when movement is finished (if True)
            timeout     : Timeout (not yet implemented)
            threshold   : Passes to motion_complete (not yet implemented)
        Outputs: 
            transform of current pos
        """

        if isinstance(pose, m3d.Transform) == False:
            rospy.logerr("movel pose input must be of type math3d.Transform")
            return
        
        # if elbow angle not provided then keep cur rent re
        if re == None or re == 0.0:
            re = self.current_pose_list[6] # keep the current re

        # Convert transform to pose_list (x,y,z,rx,ry,rz,re)
        _list = self.transform_to_list(pose)
        pose_list = np.append(_list, [re])

        # Default to 10 deg/sec if no velocities have been set
        if vel <= 0 and vel_rot <= 0:
            vel_rot = np.deg2rad(10)

        # if no form provided, then use current form
        if form is None:
            form = self.form_byte

        # Set rotational or translational velocity
        if vel > 0:
            # Cap speed at 100.0 mm/s
            if vel > 0.1:
                rospy.logwarn("Lowering movel velocity to 0.1 m/s")
                vel = 0.1

            # Set translational velocity
            _vel = vel*1000.0 # convert m/s to mm/s
            HSE_MOVEL_SPD_CLASS = HSE_MOVEL_SPD_CLASS_V
            self.print_debug("movel_cart vel [mm/s] {}".format(_vel))
        else:
            # Set rotational velocity
            _vel = math.degrees(vel_rot) # convert rad/s to deg/s
            HSE_MOVEL_SPD_CLASS = HSE_MOVEL_SPD_CLASS_VR
            self.print_debug("movel_cart vel_rot [deg/s] {}".format(_vel))

        # calculate timeout
        if timeout == None:
            # if vel is not provided (case of vel_rot given, but not vel), give it a default value
            if vel <= 0:
                vel = 0.003 # 3 mm/s

            # Calculate the timeout to be 2x of the max pose distance velocity
            pose_distances = np.abs(np.asarray(self.current_pose_list) - np.asarray(pose_list))
            timeout = 3.0 * np.divide(np.max(np.abs(pose_distances)), vel)
            # Set min timeout of 1.0 sec
            if timeout < 1.0:
                timeout = 1.0


        # pack the ints
        _x  = self.int_to_hex(pose_list[0]*1000.0)  # convert mm to mm/1000
        _y  = self.int_to_hex(pose_list[1]*1000.0)
        _z  = self.int_to_hex(pose_list[2]*1000.0) 
        _rx = self.int_to_hex(pose_list[3]*10000.0) # unit 0.0001 deg
        _ry = self.int_to_hex(pose_list[4]*10000.0)
        _rz = self.int_to_hex(pose_list[5]*10000.0)
        _re = self.int_to_hex(re*10000.0)
        _spd = self.int_to_hex(_vel*10.0)            # unit 0.1 mm/s or 0.1 deg/s
        _form = self.int_to_hex(form)

        # Fill out goal_state and add to queue
        goal_state = [None] * 4
        goal_state[GoalStateEnums.STATE_PRIMARY.value] = pose_list
        goal_state[GoalStateEnums.STATE_SECONDARY.value] = None
        goal_state[GoalStateEnums.MOTION_TYPE.value] = "movel"
        goal_state[GoalStateEnums.TIMEOUT.value] = timeout
        self.goal_state_queue.put(goal_state)

        # Construct header
        hdr = HSE_HDR + HSE_HDR_SIZE + HSE_DATA_SIZE_SET_POS_2 + HSE_RES_1 + \
                HSE_PROCESS_ROBOT_CTRL + HSE_ACK_REQUEST + self.HSE_REQ_ID_MOVE + HSE_BLOCK_ID + \
                HSE_RES_2 + HSE_CART_MOVE_CMD_ID + HSE_INST_MOVEL_2 + HSE_ATTR_MOVEL + \
                HSE_SRV_SETPOS + HSE_PADDING


        # Base coordinate
        _coord = bytearray()
        self.append_msg(_coord, HSE_MOVEL_BASE_COORD)

        # Construct data
        movel_data_1 = HSE_MOVEL_CTRL_GROUP + HSE_MOVEL_STATION + HSE_MOVEL_SPD_CLASS # list of fixed values
        movel_data_2 = _spd + _coord + _x + _y + _z + _rx + _ry + _rz + _re # bytearray of variable inputs
        movel_data_3 = HSE_MOVEL_RES    # index 12 (reserved)
        movel_data_4 = _form  # index 13 (type) 
        padding = bytearray() # 12 bytes of padding for unused indexes 14-25 (expanded type to station 6th axis)
        for x in range(12):
            self.append_msg(padding,HSE_MOVEL_RES)

        # Construct msg
        msg = bytearray()
        msg.extend(hdr)
        msg.extend(movel_data_1)
        msg.extend(movel_data_2)
        msg.extend(movel_data_3)
        msg.extend(movel_data_4)
        msg.extend(padding)

        # send to move_queue
        self.abandon_motion = False
        self.motion_done = False
        self.increment_req_id(self.HSE_REQ_ID_MOVE)
        self.move_queue.put(msg)

        # if wait is true, wait for move_queue to be emptied and no motion
        if wait == True:   
            self.wait_for_motion = True
            while self.wait_for_motion and not rospy.is_shutdown() == True:
                rospy.sleep(UDP_DELAY)

                if self.abandon_motion:
                    break

        # Return false if there was some error
        if self.abandon_motion:
            return False
        else:
            return True


    def movel_pulse(self, tf=None, pose=None, joints=[], vel=None, vel_rot=None, wait=False, timeout=None, threshold=None):
        """ move-in-linear-space with joint-goals (movelj)
        Inputs: 
            joints [rad]: Joint-state of goal, HSE driver will use linear-move with pulse goal, this OVERRIDES the pose goal
            pose [m,rad]: Transform of pose
            re [deg]    : Elbow angle 
            acc         : Currently ignored (no easy way to set accel on FS100) 
            vel [m/s]   : Velocity (overrides vel_rot)
            vel_rot [rad/s]: Rotational velocity (used by default, but gets overrided by vel if vel is set)
            wait        : Function return when movement is finished (if True)
            timeout     : Timeout (not yet implemented)
            threshold   : Passes to motion_complete (not yet implemented)
        Outputs: 
            transform of current pos
        """

        # Default to 10 deg/sec if no velocities have been set
        if vel <= 0 and vel_rot <= 0:
            vel_rot = np.deg2rad(10)

        # Set rotational or translational velocity
        if vel > 0.0:
            # Set translational velocity
            _vel = vel*1000.0 # convert m/s to mm/s
            HSE_MOVEL_SPD_CLASS = HSE_MOVEL_SPD_CLASS_V
            self.print_debug("movel_pulse vel [mm/s] {:.3f}".format(_vel))
        else:
            # Set rotational velocity
            _vel = math.degrees(vel_rot) # convert rad/s to deg/s
            HSE_MOVEL_SPD_CLASS = HSE_MOVEL_SPD_CLASS_VR
            # _vel = _vel / 10.0 # 6/18 edit to lower speed, documentation says units of 0.1 deg/s but unclear if this is correct
            self.print_debug("movel_pulse vel_rot [deg/s] {:.3f}".format(_vel))

        # calculate timeout
        if timeout == None:
            # if vel_rot is not provided (case of vel given, but not vel_rot), give it a default value
            if vel_rot <= 0:
                vel_rot = np.deg2rad(5.0)

            # Calculate the timeout to be 3x of the max joint distance velocity
            joint_distances = np.abs(np.asarray(self.joints) - np.asarray(joints))
            timeout = 3.0 * np.divide(np.max(np.abs(joint_distances)), vel_rot)

            # Set min timeout of 5.0 sec
            if timeout < 5.0:
                timeout = 5.0

        # Fill out goal_state and add to queue
        goal_state = [None] * 4
        goal_state[GoalStateEnums.STATE_PRIMARY.value] = joints
        goal_state[GoalStateEnums.STATE_SECONDARY.value] = pose
        goal_state[GoalStateEnums.MOTION_TYPE.value] = "movelj"
        goal_state[GoalStateEnums.TIMEOUT.value] = timeout
        self.goal_state_queue.put(goal_state)

        # pack the ints and apply units conversions
        j_s = self.int_to_hex(math.degrees(joints[0]) * self.pulse_per_deg[0])  # convert rad to deg
        j_l = self.int_to_hex(math.degrees(joints[1]) * self.pulse_per_deg[1])
        j_u = self.int_to_hex(math.degrees(joints[3]) * self.pulse_per_deg[3])
        j_r = self.int_to_hex(math.degrees(joints[4]) * self.pulse_per_deg[4])  # unit 0.0001 deg
        j_b = self.int_to_hex(math.degrees(joints[5]) * self.pulse_per_deg[5])
        j_t = self.int_to_hex(math.degrees(joints[6]) * self.pulse_per_deg[6])
        j_e = self.int_to_hex(math.degrees(joints[2]) * self.pulse_per_deg[2])
        _spd = self.int_to_hex(_vel*10.0) # 0.1 mm/s or 0.1 deg/s
       
        # Print pulses to check
        # for p in [j_s, j_l, j_u, j_r, j_b, j_t, j_e]: rospy.loginfo(self.hex_to_int(p))

        # Construct header
        hdr = HSE_HDR + HSE_HDR_SIZE + HSE_DATA_SIZE_MOVEJ + HSE_RES_1 + \
                HSE_PROCESS_ROBOT_CTRL + HSE_ACK_REQUEST + self.HSE_REQ_ID_MOVE + HSE_BLOCK_ID + \
                HSE_RES_2 + HSE_PULSE_MOVE_CMD_ID + HSE_INST_MOVEJ_2  + HSE_ATTR_MOVEJ + \
                HSE_SRV_SETPOS + HSE_PADDING

        # Construct data
        movej_data_1 = HSE_MOVEJ_CTRL_GROUP + HSE_MOVEJ_STATION + HSE_MOVEL_SPD_CLASS             # index 1-3
        movej_data_2 = _spd + j_s + j_l + j_u + j_r + j_b + j_t + j_e
        padding = bytearray() # 11 bytes of padding for unused indexes 12-22 
        for x in range(11):
            self.append_msg(padding,HSE_MOVEJ_RES)

        # Construct msg
        msg = bytearray()
        msg.extend(hdr)
        msg.extend(movej_data_1)
        msg.extend(movej_data_2)
        msg.extend(padding)

        # send to move_queue
        self.abandon_motion = False
        self.motion_done = False
        self.increment_req_id(self.HSE_REQ_ID_MOVE)
        self.move_queue.put(msg)     # move_queue needs to know its a movej

        # if wait is true, wait for move_queue to be emptied and no motion
        if wait == True:   
            self.wait_for_motion = True
            while self.wait_for_motion and not rospy.is_shutdown() == True:
                rospy.sleep(UDP_DELAY)

                if self.abandon_motion:
                    break

        # Return false if there was some error
        if self.abandon_motion:
            return False
        else:
            return True

        return

    def stepl(self, pose=None, vel=0, vel_rot=0, form=None, wait=False, timeout=None):
        """ step-in-linear-space
        Inputs:
            step_tf     : tf of the step
            vel [m/s]   : Velocity
            wait        : Function return when movement is finished (if True)
            timeout     : Timeout (not yet implemented)
            threshold   : Passes to motion_complete (not yet implemented)
        Outputs: 
            transform of current pos
        """
        print_message = True
        while self.in_teach_mode and not rospy.is_shutdown():
            if print_message:
                rospy.logwarn("Robot in TEACH, reset to REMOTE mode. Motion command on hold.")
                print_message = False
            rospy.sleep(0.5)

        print_message = True
        while self.estop_enabled and not rospy.is_shutdown():
            if print_message:
                rospy.logwarn("Robot E-STOP enabled, please reset. Motion command on hold.")
                print_message = False
            rospy.sleep(0.5)

        step_tf = pose
        current_pos = self.list_to_transform(self.current_pose_list[:])

        # Matrix math method
        trans = current_pos.get_pos() + step_tf.get_pos()
        rot = current_pos.get_orient() * step_tf.get_orient() # try using the sum method (convert tf_to_arr then add components)
        goal = m3d.Transform(rot, trans)
        
        self.print_debug("cur: {}".format(tf_to_list(current_pos)))
        self.print_debug("goal: {}".format(tf_to_list(goal)))
        self.print_debug("dist: {}".format(tf_to_arr(goal) - tf_to_arr(current_pos)))

        # If there's only rotation, neglect translational velocity
        if all(v==0 for v in step_tf.get_pos()):
            vel = None

            # Assign default of 5 deg/s for vel_rot
            if vel_rot == 0:
                vel_rot = np.deg2rad(5.0)

        # send movel command
        self.movel(pose=goal, vel=vel, vel_rot=vel_rot, form=form, wait=wait, timeout=timeout)
        return

    # Status is TRUE (hold on), or FALSE (hold off)
    def hold(self, status):
        msg = bytearray()
        if (status):
            packet = HSE_HDR + HSE_HDR_SIZE + HSE_DATA_SIZE_SERVO + HSE_RES_1 + \
                    HSE_PROCESS_ROBOT_CTRL + HSE_ACK_REQUEST + self.HSE_REQ_ID_TX + HSE_BLOCK_ID + \
                    HSE_RES_2 + HSE_SERVO_CMD_ID + HSE_INST_HOLD + HSE_ATTR_SERVO + \
                    HSE_SRV_SET_ATTR_SINGLE + HSE_PADDING + HSE_DATA_SERVO_ON
            self.append_msg(msg, packet)
        else:
            packet = HSE_HDR + HSE_HDR_SIZE + HSE_DATA_SIZE_SERVO + HSE_RES_1 + \
                    HSE_PROCESS_ROBOT_CTRL + HSE_ACK_REQUEST + self.HSE_REQ_ID_TX + HSE_BLOCK_ID + \
                    HSE_RES_2 + HSE_SERVO_CMD_ID + HSE_INST_HOLD + HSE_ATTR_SERVO + \
                    HSE_SRV_SET_ATTR_SINGLE + HSE_PADDING + HSE_DATA_SERVO_OFF
            self.append_msg(msg, packet)
        
        # send to tx_queue
        self.increment_req_id(self.HSE_REQ_ID_TX)
        self.tx_queue.put(msg)

        return msg

    def get_alarm(self):
        self.get_alarm_id(1)

    # Get current alarm (1-4)
    def get_alarm_id(self, alarm_id=1):
        # Clear tx queue
        self.tx_queue_clear()

        # Clear saved alarm data
        self.alarm_code = -1

        HSE_INST_ALARM = [alarm_id, 0x00]

        msg = bytearray()
        packet = HSE_HDR + HSE_HDR_SIZE + HSE_DATA_SIZE_ALARM + HSE_RES_1 + \
                HSE_PROCESS_ROBOT_CTRL + HSE_ACK_REQUEST + self.HSE_REQ_ID_TX + HSE_BLOCK_ID + \
                HSE_RES_2 + HSE_ALARM_CMD_ID + HSE_INST_ALARM + HSE_ATTR_ALARM + \
                HSE_SRV_ALARM + HSE_PADDING
        self.append_msg(msg, packet)

        # send to tx_queue
        self.increment_req_id(self.HSE_REQ_ID_TX)
        self.tx_queue.put(msg)

        return

    # Get alarm history
    def get_alarm_history(self, instance):
        return 
    
    # Clear txQueue, and executes fct(args) if provided as the top-priority function with mutex lock
    # test fct(args)
    def tx_queue_clear(self, fct=None, arg=None, thread_safe=False):

        if thread_safe:
            # Acquire lock
            self._tx_lock.acquire()

            # Clear queue
            with self.tx_queue.mutex:
                self.tx_queue.queue.clear() 

            # Run function and send msg
            if fct is not None:
                if arg is not None:
                    msg = fct(arg)
                else:
                    msg = fct()

                # Send the msg here, instead of tx_thread or move_thread
                self.increment_req_id(self.HSE_REQ_ID_TX)
                self.udp_socket.sendto(msg, (self.UDP_TX_IP, self.UDP_PORT))
                self.last_msg = msg

            # Release lock
            self._rx_lock.acquire() # waits for rx_thread to release lock before moving on
            self._tx_lock.release()
        else:
            # Clear queue
            with self.tx_queue.mutex:
                self.tx_queue.queue.clear() 


    def move_queue_clear(self):
        with self.move_queue.mutex:
            self.move_queue.queue.clear()

    # Stop motion by clearing tx_queue and move_queue
    def stop(self):

        # Set the motion_complete variables properly so the motion is aborted properly
        rospy.loginfo("Abandoning motion in stop function")
        self.abandon_motion = True
        self.move_queue_override = None
        self.motion_done = True  # to ignore the automatic hold reset

        self.goal_state_queue.queue.clear()
        self.move_queue_clear()

        rospy.loginfo("Queues cleared. Still in motion. Stopping movement...")
        self.tx_queue_clear(fct=self.hold, arg=True, thread_safe=True)
        rospy.sleep(0.2)
        
        rospy.loginfo("stop done, reset hold and get back into play mode")
        self.tx_queue_clear(fct=self.hold, arg=False, thread_safe=True)
        self.hold(False)

        rospy.loginfo("Stop operation completed.")
        return
    
    def set_digital_out(self, signal, value):
        if value <> 0 and value <> 1:
            print "digital out value must be 0 or 1"
            return

        io_signal_packed  = struct.pack('<H', signal)
        HSE_INST_IO = [self.struct_unpack('<B', io_signal_packed[0], "stop"), self.struct_unpack('<B', io_signal_packed[1], "stop")]

        msg = bytearray()
        packet = HSE_HDR + HSE_HDR_SIZE + HSE_DATA_SIZE_SET_IO + HSE_RES_1 + \
                HSE_PROCESS_ROBOT_CTRL + HSE_ACK_REQUEST + self.HSE_REQ_ID_TX + HSE_BLOCK_ID + \
                HSE_RES_2 + HSE_IO_CMD_ID + HSE_INST_IO + HSE_ATTR_IO + \
                HSE_SRV_IO + HSE_PADDING
        self.append_msg(msg, packet)
        self.append_msg(msg, [value])

        # send to tx_queue
        self.increment_req_id(self.HSE_REQ_ID_TX)
        self.tx_queue.put(msg)
        return

    # Returns self.error_status, which is True when in error, False when no error
    def get_status(self):
        # Throttle status cmd requests to 20 Hz
        if rospy.get_time() - self.last_status_time > (1.0 / self.status_cmd_hz):
            self.last_status_time = rospy.get_time()

            msg = bytearray()
            packet = HSE_HDR + HSE_HDR_SIZE + HSE_DATA_SIZE_GET_STATUS + HSE_RES_1 + \
                    HSE_PROCESS_ROBOT_CTRL + HSE_ACK_REQUEST + self.HSE_REQ_ID_TX + HSE_BLOCK_ID + \
                    HSE_RES_2 + HSE_GET_STATUS_CMD_ID + HSE_INST_GET_STATUS + HSE_ATTR_GET_STATUS_BOTH + \
                    HSE_SRV_GET_STATUS + HSE_PADDING
            self.append_msg(msg, packet)

            # send to tx_queue
            self.increment_req_id(self.HSE_REQ_ID_TX)
            self.tx_queue.put(msg)

        # Sleep to slow down arm_monitor thread or else UDP port will be overwhelmed
        # rospy.sleep(0.01)
        return 

    # Returns True if an alarm has been triggered or estop is enabled
    # # rsas check_stalled() checks this function in the control loop
    def get_motion_alarm(self):
        return self.alarm_triggered or self.estop_enabled

    # Level 1 -- 10 shock
    # Level 2 -- 50 shock
    # Level 3 -- 100 shock
    def set_shock_level(self, level):
        rospy.loginfo("MotomanHSE setting shock sense level to {}".format(level))

        job = HSE_SHOCK_LVL1_JOB
        if level == 1:
            job = HSE_SHOCK_LVL1_JOB
        elif level == 2:
            job = HSE_SHOCK_LVL2_JOB
        elif level == 3:
            job = HSE_SHOCK_LVL3_JOB
        else:
            rospy.logerr("Bad shock level specified (must be 1-3)")

        msg = bytearray()
        packet = HSE_HDR + HSE_HDR_SIZE + HSE_DATA_SIZE_JOB_SELECT + HSE_RES_1 + \
                HSE_PROCESS_ROBOT_CTRL + HSE_ACK_REQUEST + self.HSE_REQ_ID_TX + HSE_BLOCK_ID + \
                HSE_RES_2 + HSE_JOB_SELECT_CMD_ID + HSE_INST_JOB_SELECT + HSE_ATTR_JOB_SELECT + \
                HSE_SRV_JOB_SELECT + HSE_PADDING
        self.append_msg(msg, packet)

        data = [ord(c) for c in job] + [0,0]
        padding = [0 for i in range(20)] # 5 bytes of empty padding
        exe = HSE_SHOCK_EXE_LINE1

        self.append_msg(msg, data)
        self.append_msg(msg, padding)
        self.append_msg(msg, exe)


        # send to tx_queue
        self.increment_req_id(self.HSE_REQ_ID_TX)
        self.tx_queue.put(msg)

        # Give time to transmit
        # rospy.sleep(0.1)
        return

    ''' PARSE RESPONSE FROM ROBOT '''
    # Wrapper around struct.unpack to handle exceptions
    def struct_unpack(self, format, data, caller_name="caller"):
        try:
            msg = struct.unpack(format, data)[0]
            return msg
        except:
            self.print_debug("MotomanHSE Warning: Dropped byte in {}()".format(caller_name), lvl="logwarn")
            return None

    # Called constantly in rx_thread
    def parse_response(self, data):
        # rospy.loginfo("tx_queue.qsize() {} ".format(self.tx_queue.qsize()))

        # print "RX", ','.join(str(hex(ord(c))) for c in data)
        if data[0:4] <> "YERC":
            return -1

        # Parse the response
        hdr = self.struct_unpack("<H", data[4:6], "parse_response")
        length = self.struct_unpack("<H", data[6:8], "parse_response")
        ack = self.struct_unpack("<B", data[10], "parse_response")
        req_id = self.struct_unpack("<B", data[11], "parse_response")
        service = self.struct_unpack("<B", data[24], "parse_response") # should be command service + 0x80
        status = self.struct_unpack("<B", data[25], "parse_response")
        status_size = self.struct_unpack("<B", data[26], "parse_response")
        self.added_status_code = self.struct_unpack("<H", data[28:30], "parse_response")

        # get the cmd_info from last_msg
        last_msg_info = self.cmd_info(self.last_msg)

        # verify reponse matches the last cmd request
        if req_id <> last_msg_info[CMD_INFO_REQ_ID] and HSE_CART_MOVE_CMD_ID[0] == last_msg_info[CMD_INFO_CMD_ID]:
            # rospy.logwarn("MotomanHSE Error: bad request id on movel")
            # print "bad request id on movel, tx", last_msg_info[CMD_INFO_REQ_ID], "rx", req_id
            return -1

        elif req_id <> last_msg_info[CMD_INFO_REQ_ID]:
            # rospy.logwarn("MotomanHSE Error: bad request id on tx/rx")
            # print "bad request id, tx", last_msg_info[CMD_INFO_REQ_ID], "rx", req_id
            return -1

        if service <> last_msg_info[CMD_INFO_SRV] + 0x80:
            # rospy.logwarn("MotomanHSE Error: bad service")
            # print "bad service"
            return -1


        if status <> 0:
            if self.added_status_code == 0x3450:
                pass
                # rospy.logerr("MotomanHSE Error: Servo power cannot be turned on")
            elif self.added_status_code == 0x2060:
                rospy.logerr("FS100 Error: Error/alarm occuring, reset on teach pendant")
            elif self.added_status_code == 0x2010 or self.added_status_code == 0x2050:

                # Print manipulator operating warning once
                if not self.manipulator_operating_warning:
                    rospy.logwarn("MotomanHSE Error: Manipulator operating warning from FS100")
                self.manipulator_operating_warning = True

                if self.move_cmd_already_sent:
                    rospy.logwarn("Re-sending last HSE move command...")
                    self.move_queue_override = self.last_motion_msg
                    self.send_attempts += 1
                if self.added_status_code == 0x2050:
                    rospy.logwarn("Robot in HOLD, motion ignored, re-attempting motion")

            elif self.added_status_code == 0x2070:
                rospy.logwarn("MotomanHSE Error: 2070 means SERVO OFF. Please check E-Stop and robot in remote mode, and send motion command again.")
                self.abandon_motion = True
                self.stop()

            elif self.added_status_code == 0x2080:
                rospy.logwarn("MotomanHSE Error: 2080 means Incorrect mode, please check robot in remote mode, and send motion command again.")
                self.abandon_motion = True
                self.stop()
            
            elif self.added_status_code == 0x4040:
                rospy.logwarn("MotomanHSE Error: 4040 No Specified Job, JBI file transfer failed. Reload the robosaw activity in HMI.")
                self.abandon_motion = True
                self.alarm_triggered = True
                self.stop()
            else:
                rospy.logerr("MotomanHSE Error: Returned added status error code {:02x}".format(self.added_status_code))
                self.abandon_motion = True

        if ack <> 1:
            return -1

        # parse pose data
        if last_msg_info[CMD_INFO_CMD_ID] == HSE_READ_ROBOT_POS_CMD_ID[0]:
            self.parse_get_pose(data)

        # parse get_status
        if last_msg_info[CMD_INFO_CMD_ID] == HSE_GET_STATUS_CMD_ID[0]:
            self.parse_get_status(data)

        # parse torque data 
        if last_msg_info[CMD_INFO_CMD_ID] == HSE_TORQUE_CMD_ID[0]:
            self.parse_get_torque(data)

        # parse torque data 
        if last_msg_info[CMD_INFO_CMD_ID] == HSE_GET_FORCE_CMD_ID[0]:
            self.parse_get_force(data)

        # parse alarm data 
        if last_msg_info[CMD_INFO_CMD_ID] == HSE_ALARM_CMD_ID[0]:
            self.parse_get_alarm(data)

        # parse get_job_info
        if last_msg_info[CMD_INFO_CMD_ID] == HSE_JOB_INFO_CMD_ID[0]:
            self.parse_get_job_info(data)

        # file io
        if last_msg_info[CMD_INFO_CMD_ID] == HSE_FILE_IO_CMD_ID[0]:
            self.print_debug("Received ACK for file I/O")

        # play job
        if last_msg_info[CMD_INFO_CMD_ID] == HSE_PLAY_JOB_CMD_ID[0]:
            self.print_debug("Received ACK for play job")

        return



    # Called from parse_response when a robot coordinate is returned, update current_pose variable
    # TODO: this function is limited to 23 Hz, figure out why so slow
    def parse_get_pose(self, data):

        self.last_time = self.current_time
        self.current_time = rospy.get_time()

        # if time.time() - self.one_sec_time < 1.0:
        #     self.counter += 1
        # else:
        #     rospy.loginfo("parse_get_pose {} Hz".format(self.counter))
        #     self.counter = 0
        #     self.one_sec_time = time.time()

        try:
            coord = self.struct_unpack("<i", data[36:40], "parse_get_pose")
            form = self.struct_unpack("<i", data[40:44], "parse_get_pose")
            tool = self.struct_unpack("<i", data[44:48], "parse_get_pose")
            user_coord = self.struct_unpack("<i", data[48:52], "parse_get_pose")
            ex_form = self.struct_unpack("<i", data[52:56], "parse_get_pose")
            x = self.struct_unpack("<i", data[52:56], "parse_get_pose")  / 1000.0
            y = self.struct_unpack("<i", data[56:60], "parse_get_pose")  / 1000.0
            z = self.struct_unpack("<i", data[60:64], "parse_get_pose")  / 1000.0
            rx = self.struct_unpack("<i", data[64:68], "parse_get_pose") / 10000.0
            ry = self.struct_unpack("<i", data[68:72], "parse_get_pose") / 10000.0
            rz = self.struct_unpack("<i", data[72:76], "parse_get_pose") / 10000.0
            re = self.struct_unpack("<i", data[76:80], "parse_get_pose") / 10000.0
        except:
            self.print_debug("MotomanHSE Warning: Dropped byte in parse_get_pose()", lvl="logwarn")
            return
        # padding - 80:84

        # Return
        resp = [coord, form, tool, user_coord, ex_form, x, y, z, rx, ry, rz, re]
        if None in resp:
            return
        else:
            self.current_pose_list = resp[5:len(resp)]
            self.calc_velocity(self.current_pose_list)

            if form != self.form_byte:
                # rospy.loginfo("Form {}".format(form))
                self.form_byte = form
            # rospy.loginfo("self.pose_deriv sum is {}, self.pose_hist len: {}".format(self.pos_deriv, len(self.pose_hist)))

    # Calculates velocity based on pos_deriv, which is a running average of the last 10 poses
    # also take the time_deriv as an average of the last 10 timestamps
    # since pose data is updated at 20 Hz (thus calc_velocity will be avg of last 0.5 sec)
    def calc_velocity(self, pose, num_samples=10):

        self.rx_period = self.current_time - self.last_time
        
        if len(self.pose_hist) > 0:
            last_pose_arr = np.asarray(self.pose_hist[-1])
        else:
            last_pose_arr = np.asarray(pose)

        pose_arr = np.asarray(pose)
        
        self.pose_hist.append(pose)
        self.pose_deriv_hist.append(np.subtract(pose_arr, last_pose_arr))
        self.time_deriv_hist.append(self.rx_period)

        # keep max len to 3
        if(len(self.pose_hist) > num_samples):
            self.pose_hist = self.pose_hist[1:len(self.pose_hist)]
            self.pose_deriv_hist =  self.pose_deriv_hist[1:len( self.pose_deriv_hist)]
            self.time_deriv_hist = self.time_deriv_hist[1:len(self.time_deriv_hist)]

        # returns a numpy array of len 7 for pos_deriv in each axis
        self.pos_deriv = np.mean(self.pose_deriv_hist, axis=0)
        self.time_deriv = np.mean(self.time_deriv_hist)

        # save velocity
        self.velocity = np.divide(self.pos_deriv, self.time_deriv)
        self.velocity = [x/1000.0 for x in self.velocity[0:3]] # convert mm to meters

        # rospy.loginfo("deriv: {}, vel: {}".format(self.pos_deriv, self.velocity))

    def get_rx_period(self):
        return self.rx_period

    # Parse response to get_status (0x72), ie if robot is in teach mode, servo off, alarm
    # returns True if there is an error or servo off
    def parse_get_status(self, data):

        status1 = self.struct_unpack("<B", data[32], "parse_get_status")
        status2 = self.struct_unpack("<B", data[36], "parse_get_status")

        # rospy.loginfo("{:02x}, {:02x}".format(status1, status2))
        # print(format(status1, '08b'), format(status2, '08b')) 

        # if status1 & 0x04:
        #     rospy.loginfo("auto and continuous")
        
        # if status1 & 0x08:
        #     rospy.loginfo("robot running")

        # Check teach or play mode
        if status1 & 0x20:
            self.in_teach_mode = True
            return True # return since remaining statements don't need to be checked yet
        else:
            self.in_teach_mode = False

        # Check servos
        if not (status2 & 0x40):
            self.servo_on(True)
            self.hold(False)

            self.servos_off_counter += 1
            if self.servos_off_counter > 2:
                self.estop_enabled = True
        else:
            self.estop_enabled = False
            self.servos_off_counter = 0

        # Set alarm_triggered variable
        if status2 & 0x10:
            self.alarm_triggered = True
        else:
        # If alarm is no longer active, reset alarm_triggered variable
            if self.alarm_triggered == True:
                rospy.logwarn("Alarm has been reset")
            self.alarm_triggered = False
            self.alarm_reset_in_progress = False

        # Reset FS100 alarm
        if self.alarm_triggered == True:
            if self.alarm_reset_in_progress == False:
                rospy.logerr("FS100 ALARM!!!")
                rospy.logwarn("Alarm reset in progress...")

            self.get_alarm() # calls parse_get_alarm() which calls reset_error()
            self.alarm_reset_in_progress = True

        # TODO: robot in hold triggers too easily -- should have time-based counter, ie been in hold for at least 0.5 seconds, before this triggers
        # no need to use movequeue_override unneccesarily
        # if move_queue_override is used, then do NOT trigger auto_recovery
        # Robot in hold
        if status2 & 0x0E:
            if self.in_hold == False:
                self.hold_timer_counter = rospy.get_time()

            self.in_hold = True
            self.last_hold_time = rospy.get_time()

            # reset hold up to once per second
            # if rospy.get_time() - self.last_hold_time > 1.0:
            if self.last_hold_time - self.hold_timer_counter > 1.0 and self.motion_done == False:
                rospy.logwarn("Robot in hold, reset into play mode.")
                self.hold(False)
                self.hold_timer_counter = rospy.get_time()
                self.last_hold_time = rospy.get_time()

                if self.motion_done == False and self.abandon_motion == False and self.alarm_triggered == False:
                    rospy.logwarn("Robot was in hold, use override to re-send last move msg, now abandoning last motion")
                    rospy.logwarn("Abandoning motion in parse_get_status function")
                    self.abandon_motion = True
                    self.move_queue_override = self.last_motion_msg

        else:
            self.in_hold = False

        return

    # Parse torque data; sets self.joint_torque to the servo joint torques (Nm)
    def parse_get_torque(self, data):
        t_s = self.struct_unpack("<i", data[32:36], "parse_get_torque")
        t_l = self.struct_unpack("<i", data[36:40], "parse_get_torque") 
        t_u = self.struct_unpack("<i", data[40:44], "parse_get_torque")  
        t_r = self.struct_unpack("<i", data[44:48], "parse_get_torque")  
        t_b = self.struct_unpack("<i", data[48:52], "parse_get_torque") 
        t_t = self.struct_unpack("<i", data[52:56], "parse_get_torque") 
        t_e = self.struct_unpack("<i", data[56:60], "parse_get_torque")

        # report in robosaw order - SLEURBT
        torque = [t_s, t_l, t_e, t_u, t_r, t_b, t_t]

        try:
            self.joint_torque = [abs(a*b/100) for a,b in zip(SIA20_RATED_JOINT_SERVO_TORQUES, torque)]
        except:
            self.print_debug("Dropped byte in parse_get_torque(). Reporting all torques as 0.0.", lvl="logwarn")
            self.joint_torque = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def parse_get_alarm(self, data):
        """ SIA20Driver.get_safety_status()
            MotomanHSE.get_status()
            parse_get_status() reads an alarm (self.alarm_triggered) to call get_alarm()
            Finally parse_get_alarm() gets called
        """

        self.alarm_code = self.struct_unpack("<i", data[32:36], "parse_get_alarm") # 74 13 --> reverse to get 4980
        alarm_data = self.struct_unpack("<i", data[36:40], "parse_get_alarm")

        alarm_type = self.struct_unpack("<i", data[40:44], "parse_get_alarm") # 0x15 - 21 - MIN/MAX SLURBT
        if self.alarm_code == 0:
            # rospy.logwarn("get alarm requested but no alarm")
            return

        else:
            alarm_time_list = []
            alarm_str_list = []
            for i in xrange(44,60): 
                alarm_time_list.append(self.struct_unpack("c", data[i], "parse_get_alarm"))
            alarm_time = ''.join(alarm_time_list)

            for i in xrange(60,92):
                x =  self.struct_unpack("c", data[i], "parse_get_alarm")
                if x <> '\x00':
                    alarm_str_list.append(x)
            alarm_str = ''.join(alarm_str_list)

            # Ignore Drop-Value and Enable SW Signal Error
            if self.alarm_code == 4511 or self.alarm_code == 4833:
                pass

            # Alarm reset in progress, skip error messages
            # elif self.alarm_reset_in_progress:
            #     pass

            # Print alarm info
            else:
                rospy.logerr('SIA20 alarm! Alarm code {}, alarm data {:02x}, alarm type {}'.format(self.alarm_code, alarm_data, alarm_type))
                rospy.logerr('{}, {}'.format(alarm_time, alarm_str))

                # Overload error details
                if self.alarm_code == 4320:
                    min_axis = self.struct_unpack("<b", data[39], "parse_get_alarm")
                    max_axis = self.struct_unpack("<b", data[38], "parse_get_alarm")
                    rospy.logerr("Min Axis: " + format(min_axis, '07b') + ", Max Axis:" + format(max_axis, '07b') + " (Format: ETBRULS)")

            # Abandon motion and reset error status
            if self.alarm_reset_in_progress == False:
                rospy.logwarn("Abandoning motion in parse_get_alarm function")
                rospy.logwarn("Resetting the alarm")

            self.abandon_motion = True
            self.reset_error()

    def rx_thread(self, UDP_PORT):        
        while not self.exit and not rospy.is_shutdown():
            rospy.sleep(0.001)
            try:
                data, addr = self.udp_socket.recvfrom(2048) # buffer size is 1024 bytes
                self.parse_response(data)
                # if self.parse_response(data) == -1:
                #    print "would have quit()"
                    #   quit()

                self._rx_lock.release()    # release lock so tx_thread can do things

            # TODO: how to handle disconnection
            except socket.error:
                #If no data is received, you get here, but it's not an error
                #Ignore and continue
                pass

    # Queue & thread to handle all commands EXCEPT for movel
    def tx_thread(self, tx_queue):

        self.start_time = time.time()
        self.one_sec_time = time.time()
        self.counter = 0

        while not self.exit and not rospy.is_shutdown():
            rospy.sleep(0.001)
            if self.tx_queue.empty():
                continue
            else:
                self._tx_lock.acquire() # acquire _tx_lock so nothing else can transmit until this is done

                msg = self.tx_queue.get()

                # Check the msg to determine which port to use
                if msg[25:27] == '\x00\x00':
                    port = self.UDP_FILE_PORT
                    self.print_debug("Send using UDP file port")
                else:
                    port = self.UDP_PORT

                self.udp_socket.sendto(msg, (self.UDP_TX_IP, port))
                self.last_msg = msg

                self._rx_lock.acquire() # waits for rx_thread to release lock before moving on
                self._tx_lock.release() # release _tx_lock so move_thread can send msg

        return

    # Queue & thread to handle movel
    # blocks processing of move_queue until last command has been completed & checked by motion_complete
    def move_thread(self, move_queue):

        while not self.exit and not rospy.is_shutdown():
            rospy.sleep(0.001)
            # Passthrough if move_queue is empty AND no move_queue_override messages (ie attempting to re-send)
            if self.move_queue.empty() and self.move_queue_override is None:
                continue

            # First move cmd processed
            self.move_cmd_already_sent = True

            self._tx_lock.acquire()    # acquire _tx_lock so nothing else can transmit until this is done

            # Override case (if):  If move_queue_override has been set elsewhere, it means ignore move_queue and process the override
            #                      Override is triggered elsewhere to indicate a motion hasn't been completed yet and needs to be re-sent
            # Default case (else): Send the next goal_pos in the move_queue
            if self.move_queue_override is not None:
                self.print_debug("move_queue_override triggered")
                msg = self.move_queue_override
                self.move_queue_override = None # can set move_queue_override to none since msg has been set up

                if not self.manipulator_operating_warning:
                    self.print_debug("move_queue_override used to re-send goal_state", lvl="logwarn")

            else:
                msg = self.move_queue.get() # begin processing queue task
                # print "last_move tx:", ','.join('{:02x}'.format(x) for x in msg)
                self.goal_state = None # clear motion_complete() goal, until next motion_complete call
                self.print_debug("move_queue processed msg, move_queue.qsize(): {}".format(self.move_queue.qsize()))

            self.udp_socket.sendto(msg, (self.UDP_TX_IP, self.UDP_PORT))

            if not self.manipulator_operating_warning:
                self.print_debug("move_queue sent the UDP msg")

            self.last_msg = msg
            self.last_motion_msg = msg
            
            self._rx_lock.acquire() # waits for rx_thread to release lock before moving on
            self._tx_lock.release() # release _tx_lock so tx_thread can send msg

            # after rx_lock processed, then self.move_queue_override will be set to msg if msg rejected
            # need to verify motion was sent before entering motion_complete
            if self.move_queue_override is not None:
                rospy.sleep(0.02) # don't overload the sending attempts
                continue    # tx was rejected
            else: 
                self.motion_complete() # blocks move_thread/move_queue processing until goal is reached
                self.move_cmd_already_sent = False
                try:
                    self.move_queue.task_done()
                except ValueError:
                    rospy.logwarn("Warning: SIA20 driver move_thread task_done() called too many times.")
        return

    # Returns a tuple jointList[], transform
    # of the last completed motion command
    def get_last_pt(self):
        joints = self.prev_joints_goal

        if self.prev_pose_goal is None:
            pose = None

        elif isinstance(self.prev_pose_goal, m3d.Transform):
            # rospy.loginfo("m3d transform!!!")
            pose = self.prev_pose_goal
        else:
            # rospy.loginfo("converting {}".format(self.prev_pose_goal))
            # rospy.loginfo("type {}".format(type(self.prev_pose_goal)))
            pose = self.list_to_transform(self.prev_pose_goal)

        return [joints, pose]

    def print_debug(self, msg, lvl="loginfo"):
        if self.debug_mode:

            if lvl == "loginfo":
                rospy.loginfo(msg)
            elif lvl == "logwarn":
                rospy.logwarn(msg)

    # Holds move_thread processing until MoveL goal is reached (+- 0.5 mm and 0.5 deg per axis)
    def motion_complete(self, rate=100.0, timeout=30.0):
        retval = True
        
        if self.goal_state is None:
            goal_state = self.goal_state_queue.get()
            self.goal_state = goal_state[GoalStateEnums.STATE_PRIMARY.value]
            self.motion_type = goal_state[GoalStateEnums.MOTION_TYPE.value]
            self.timeout = max(goal_state[GoalStateEnums.TIMEOUT.value], 30.0) # at least 30 second timeout
            
            # movelj (joints primary, pose secondary)
            if self.motion_type == "movelj":
                self.joints_goal = goal_state[GoalStateEnums.STATE_PRIMARY.value]
                self.pose_goal = goal_state[GoalStateEnums.STATE_SECONDARY.value]
            # movej (joints primary)
            elif self.motion_type == "movej": 
                self.joints_goal = goal_state[GoalStateEnums.STATE_PRIMARY.value]
                self.pose_goal = goal_state[GoalStateEnums.STATE_SECONDARY.value] # None
            # movel (pose primary)
            elif self.motion_type == "movel":
                self.pose_goal = goal_state[GoalStateEnums.STATE_PRIMARY.value]
                self.joints_goal = goal_state[GoalStateEnums.STATE_SECONDARY.value] # None

        self.motion_done = False
        self.abandon_motion = False
        self.pos_deriv_zero_state = 0

        self.print_debug("before motion_complete, joint-state is {}".format(self.joints))
        rospy.loginfo("=== {} motion_complete timeout {:.3f} s, goal is {} ===".format(self.motion_type, self.timeout, map(float,['{:.6f}'.format(x) for x in self.goal_state])))
        self.is_robot_moving = True

        if self.manipulator_operating_warning:
            self.print_debug("Robot received motion after {} send_attempts".format(self.send_attempts), lvl="logwarn")

        self.manipulator_operating_warning = False
        self.send_attempts = 1

        r = rospy.Rate(rate)
        start_time = time.time()
        
        # START OF WHILE-LOOP
        while not rospy.is_shutdown():
            self.is_robot_moving = True

            # Begin by assuming goal_reached is True, then iterate thru current_pose or joints to check
            # if there's an axis/joint that isn't within MOVEL_COMPLETE_TOL then stop checking and continue while-loop
            goal_reached = True 

            if self.motion_type == "movel":
                goal = np.asarray(self.goal_state)
                curr = np.asarray(self.current_pose_list)
                sum_diff = np.sum(np.abs(goal-curr))
                
                if sum_diff > MOVEL_COMPLETE_TOL: # used to have more abs()
                    goal_reached = False

            elif self.motion_type == "movej" or self.motion_type == "movelj":
                goal = np.asarray(self.goal_state)
                curr = np.asarray(self.joints)
                sum_diff = np.sum(np.abs(goal-curr))

                if sum_diff > MOVEJ_COMPLETE_TOL:
                    goal_reached = False
                else:
                    goal_reached = True

            # goal_reached set to True each while-loop, but if it's not reached then set to False in for-loop
            if goal_reached == True:
                self.motion_done = True
                self.prev_joints_goal = self.joints_goal
                self.prev_pose_goal = self.pose_goal
                break

            # Check for timeout
            if time.time() - start_time > self.timeout:
                rospy.logwarn("Motion timeout after {:.3f} seconds.".format(self.timeout))
                self.abandon_motion = True

            # Check for motion alarm
            if self.get_motion_alarm() == True:
                self.abandon_motion = True

            # Check for abandon_motion call
            if self.abandon_motion == True:
                rospy.logerr("Abandoning last motion request")
                self.motion_done = True
                retval = False
                break

            # Check if motionDone
            if self.motion_done == True:
                break
            
            # increment pos_deriv_zero_state counter if robot isn't moving
            if abs(np.sum(self.pos_deriv)) < 0.001 and self.motion_done == False and self.speed > 1:
                self.pos_deriv_zero_state += 1
            else:
                self.pos_deriv_zero_state = 0

            # Run at defined rate (default 100 Hz)
            r.sleep()

        # END OF WHILE-LOOP

        # While-loop is over: reset to default values
        self.is_robot_moving = False
        self.pos_deriv_zero_state = 0
        self.motion_done = True

        if retval == True:
            rospy.loginfo("=== Motion complete! {:.3f} seconds elapsed ===".format(time.time() - start_time))
            self.print_debug("after motion_complete, joint-state is {}".format(self.joints))
            self.wait_for_motion = False
        else:
            rospy.logerr("=== Motion was aborted. ===")
            self.wait_for_motion = True

            if self.get_motion_alarm() == True:
                rospy.logerr("=== Alarm or Estop detected, stopping arm.")
                self.stop()

        return retval