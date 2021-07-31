''' '''
HSE_FIG_DEFAULT = 0x03 # front, up, no flip, S<180, R<180, T<180

MOVEL_COMPLETE_TOL = 0.5
MOVEJ_COMPLETE_TOL = 0.00087266 # np.deg2rad(0.05)

''' GENERIC HEADER '''
HSE_HDR = [ord(c) for c in "YERC"]           # index 0-3
HSE_HDR_SIZE = [0x20, 0x00]                      # index 4-5
# index 6-7 - Data size
HSE_RES_1 = [0x03]                            # index 8
HSE_PROCESS_ROBOT_CTRL = [1]                  # index 9
HSE_PROCESS_FILE_CTRL  = [2]                  # index 9
HSE_ACK_REQUEST = [0]                         # index 10
HSE_ACK_SEND = [1]                            # index 10
HSE_REQ_ID = [0x00]                           # index 11
HSE_BLOCK_ID = [0, 0, 0, 0]                 # index 12-15
HSE_RES_2 = [ord(c) for c in "99999999"]    # index 16-23

# Service (1 byte, index 29)
HSE_SRV_GET_ATTR_ALL = [0x01]
HSE_SRV_SET_ATTR_ALL = [0x02]
HSE_SRV_GET_ATTR_SINGLE = [0x0E]
HSE_SRV_SET_ATTR_SINGLE = [0x10]

# Header padding 
HSE_PADDING = [0x00, 0x00]              # Header padding (2 bytes, index 30-31)
HSE_PADDING_4BYTES = [0x00,0x00,0x00,0x00]


''' get pos '''
HSE_DATA_SIZE_GET_POS = [0x00, 0x00]    # Data size (2 bytes, index 6-7)
HSE_READ_ROBOT_POS_CMD_ID =[0x75, 0x00] # Command ID (2 bytes, index) 24-25
HSE_INST_GETPOS_R1 = [101, 0x00]        # Instance (2 bytes, index 26-27)
HSE_ATTR_GETPOS = [0x00]                # Attribute (1 byte, index 28)
HSE_SRV_GETPOS = [0x01]                 # Service (1 byte, index 29)


''' set pos / movel'''
HSE_DATA_SIZE_SET_POS = [0x34, 0x00]    # Data size (2 bytes, index 6-7)
HSE_DATA_SIZE_SET_POS_2 = [0x68, 0x00]  # Data size (2 bytes, index 6-7)
HSE_CART_MOVE_CMD_ID = [0x8A, 0x00]     # Command ID (2 bytes, index) 24-25
HSE_INST_MOVEL_INCREMENT = [0x03, 0x00]
HSE_INST_MOVEL_2  = [0x02, 0x00]        # Instance (2 bytes, index 26-27)
HSE_ATTR_MOVEL = [0x01]                 # Attribute (1 byte, index 28)
HSE_SRV_SETPOS = [0x02]                 # Service (1 byte, index 29)

# Data - MoveL
HSE_MOVEL_CTRL_GROUP = [0x01, 0x00, 0x00, 0x00]
HSE_MOVEL_STATION = [0x00, 0x00, 0x00 , 0x00]
HSE_MOVEL_SPD_CLASS_V = [0x01, 0x00, 0x00, 0x00]
HSE_MOVEL_SPD_CLASS_VR = [0x02, 0x00, 0x00, 0x00]
HSE_MOVEL_RES = [0x00,0x00,0x00,0x00]
HSE_MOVEL_BASE_COORD = [16,0,0,0]

''' set pos / movej'''
HSE_DATA_SIZE_MOVEJ = [0x58, 0x00]  # Data size (2 bytes, index 6-7)
HSE_PULSE_MOVE_CMD_ID = [0x8B, 0x00]     # Command ID (2 bytes, index) 24-25
HSE_INST_MOVEJ_1  = [0x01, 0x00]        # Instance (2 bytes, index 26-27)
HSE_INST_MOVEJ_2  = [0x02, 0x00]        # Instance (2 bytes, index 26-27)
HSE_ATTR_MOVEJ = [0x01]                 # Attribute (1 byte, index 28)
HSE_SRV_MOVEJ = HSE_SRV_SET_ATTR_ALL        # Service (1 byte, index 29)

# Data - MoveJ
HSE_MOVEJ_CTRL_GROUP = [0x01, 0x00, 0x00, 0x00]
HSE_MOVEJ_STATION = [0x00, 0x00, 0x00 , 0x00]
HSE_MOVEJ_SPD_CLASS_P = [0x00, 0x00, 0x00, 0x00]
HSE_MOVEJ_SPD_CLASS_V = [0x01, 0x00, 0x00, 0x00]
HSE_MOVEJ_SPD_CLASS_VR = [0x02, 0x00, 0x00, 0x00]
HSE_MOVEJ_RES = [0x00,0x00,0x00,0x00]
HSE_MOVEJ_BASE_COORD = [16,0,0,0]


''' enable/disable servo '''
HSE_DATA_SIZE_SERVO = [0x04, 0x00]      # Data size (2 bytes, index 6-7)
HSE_SERVO_CMD_ID = [0x83, 0x00]         # Command ID (2 bytes, index) 24-25
HSE_INST_SERVO_ON = [0x02, 0x00]        # Instance (2 bytes, index 26-27)
HSE_INST_HOLD = [0x01, 0x00]            # Instance (2 bytes, index 26-27)
HSE_ATTR_SERVO = [0x01]                 # Attribute (1 byte, index 28)
HSE_DATA_SERVO_ON  = [0x01, 0x00, 0x00, 0x00] # Data
HSE_DATA_SERVO_OFF = [0x02, 0x00, 0x00, 0x00] # Data


''' get torque '''
HSE_DATA_SIZE_TORQUE = [0x00, 0x00]     # Data size (2 bytes, index 6-7)
HSE_TORQUE_CMD_ID =[0x77, 0x00]         # Command ID (2 bytes, index) 24-25
HSE_INST_TORQUE_R1 = [0x01, 0x00]       # Instance (2 bytes, index 26-27)
HSE_ATTR_TORQUE_NON_AXIS = [0x00]       # Attribute (1 byte, index 28)
HSE_SRV_TORQUE = HSE_SRV_GET_ATTR_ALL   # Service (1 byte, index 29)


''' error reset '''
HSE_DATA_SIZE_ERROR_RESET = [0x04,0x00] # Data size (2 bytes, index 6-7)
HSE_ERROR_RESET_CMD_ID = [0x82, 0x00]   # Command ID (2 bytes, index) 24-25
HSE_INST_ERROR_RESET = [0x01, 0x00]     # Instance (2 bytes, index 26-27)
HSE_INST_ERROR_CANCEL = [0x02, 0x00]    # Instance (2 bytes, index 26-27)
HSE_ATTR_ERROR_RESET = [0x01]           # Attribute (1 byte, index 28)
HSE_SRV_ERROR_RESET = HSE_SRV_SET_ATTR_SINGLE   # Service (1 byte, index 29)
HSE_ERROR_RESET_DATA = [0x01, 0x00, 0x00, 0x00] # Data (4 bytes, index 32-35)

''' get_status '''
HSE_DATA_SIZE_GET_STATUS = [0x00,0x00] # Data size (2 bytes, index 6-7)
HSE_GET_STATUS_CMD_ID = [0x72, 0x00]   # Command ID (2 bytes, index) 24-25
HSE_INST_GET_STATUS = [0x01, 0x00]     # Instance (2 bytes, index 26-27)
HSE_ATTR_GET_STATUS_BOTH = [0x00]      # Attribute (1 byte, index 28)
HSE_ATTR_GET_STATUS_1 = [0x01]           # Attribute (1 byte, index 28)
HSE_ATTR_GET_STATUS_2 = [0x02]           # Attribute (1 byte, index 28)
HSE_SRV_GET_STATUS = HSE_SRV_GET_ATTR_ALL   # Service (1 byte, index 29)

''' set_shock_level (job select) '''
HSE_SHOCK_LVL1_JOB = "SHOCK-LVL1"
HSE_SHOCK_LVL2_JOB = "SHOCK-LVL2"
HSE_SHOCK_LVL3_JOB = "SHOCK-LVL3"
HSE_DATA_SIZE_JOB_SELECT = [0x24,0x00] # Data size (36 bytes, index 6-7)
HSE_JOB_SELECT_CMD_ID = [0x87, 0x00]   # Command ID (2 bytes, index) 24-25
HSE_INST_JOB_SELECT = [0x01, 0x00]     # Instance (2 bytes, index 26-27)
HSE_ATTR_JOB_SELECT = [0x00]           # Attribute (1 byte, index 28)
HSE_SRV_JOB_SELECT = HSE_SRV_SET_ATTR_ALL   # Service (1 byte, index 29)
HSE_SHOCK_EXE_LINE1 = [0x01,0x00,0x00,0x00] # Execute line 1 for data part

''' alarm data reading '''
HSE_DATA_SIZE_ALARM = [0x00, 0x00]
HSE_ALARM_CMD_ID = [0x70, 0x00]
# HSE_INST_ALARM = [0x01, 0x00] # instance can be 1,2,3,4
HSE_ATTR_ALARM = [0x00]
HSE_SRV_ALARM = HSE_SRV_GET_ATTR_ALL

''' alarm history '''
HSE_DATA_SIZE_ALARM_HIST = [0x00, 0x00]
HSE_ALARM_HIST_CMD_ID = [0x71, 0x00]
HSE_INST_ALARM_HIST = [0xE9, 0x03] # defined as 1001, 1002, etc
HSE_ATTR_ALARM_HIST = [0x00]
HSE_SRV_ALARM_HIST = HSE_SRV_GET_ATTR_ALL

''' write io (speed override) '''
HSE_DATA_SIZE_SET_IO = [0x01,0x00] # Data size (2 bytes, index 6-7)
HSE_IO_CMD_ID = [0x78, 0x00]   # Command ID (2 bytes, index) 24-25
HSE_INST_SPD_IO = [0xC5, 0x09]
HSE_ATTR_IO = [0x01]           # Attribute (1 byte, index 28)
HSE_SRV_IO = HSE_SRV_SET_ATTR_SINGLE   # Service (1 byte, index 29)


''' set svar '''
HSE_DATA_SIZE_SVAR = [0x10, 0x00]
HSE_SVAR_CMD_ID = [0x7E, 0x00]
HSE_ATTR_SVAR = [0x01]                  # Attribute (1 byte, index 28)

''' get_force '''
HSE_DATA_SIZE_GET_FORCE = [0x04, 0x00]     # Data size (2 bytes, index 6-7)
HSE_GET_FORCE_CMD_ID = [0x0403, 0x00]      # Command ID (2 bytes, index) 24-25
HSE_INST_GET_FORCE = [0x26, 0x00]       # Instance (2 bytes, index 26-27)
HSE_ATTR_GET_FORCE = [0x00]
HSE_SRV_GET_FORCE = [0x33]
HSE_FORCE_DATA_NUM = [0x06,0x00,0x00,0x00]

''' select_job '''
HSE_SELECT_JOB_CMD_ID = [0x87, 0x00]
HSE_INST_SELECT_JOB = [0x01, 0x00]
HSE_ATTR_SELECT_JOB = [0x00]
HSE_SRV_SELECT_JOB = [0x02]
HSE_DATA_SIZE_SELECT_JOB = [0x24,0x00] # Data size (36 bytes)

''' play_job '''
HSE_PLAY_JOB_CMD_ID = [0x86, 0x00]
HSE_INST_PLAY_JOB = [0x01, 0x00]
HSE_ATTR_PLAY_JOB = [0x01]
HSE_DATA_SIZE_PLAY_JOB = [0x04,0x00] # Data size (36 bytes)
HSE_PLAY_JOB_DATA = [1, 0, 0, 0]

''' file i/O: delete_file, load_file, get_file_list'''
HSE_FILE_IO_CMD_ID = [0x00, 0x00]
HSE_INST_FILE_IO = [0x00, 0x00]
HSE_ATTR_FILE_IO = [0x00]
HSE_SRV_DELETE_FILE = [0x09]
HSE_SRV_LOAD_FILE = [0x15]
HSE_SRV_GET_FILE_LIST = [0x32]
HSE_DATA_GET_FILE_LIST = "*.JBI"

'''get_job_info'''
HSE_JOB_INFO_CMD_ID = [0x73, 0x00]
HSE_INST_JOB_INFO_IO = [0x01, 0x00]
HSE_ATTR_JOB_INFO_IO = [0x00]
HSE_SRV_JOB_INFO = HSE_SRV_GET_ATTR_ALL

''' misc '''
# Command ID (2 bytes)
HSE_ROBOT_POS_CMD_ID = [0x7F, 0x00]     # index 24-25

# Data - "Hello World"
HSE_HELLO_WORLD_1 = [0x48, 0x65, 0x6C, 0x6C]
HSE_HELLO_WORLD_2 = [0x6F, 0x20, 0x57, 0x6F]
HSE_HELLO_WORLD_3 = [0x72, 0x6C, 0x64, 0x00]
HSE_HELLO_WORLD_4 = [0x00, 0x00, 0x00, 0x00]

# cmd_info() indices
CMD_INFO_REQ_ID = 0
CMD_INFO_CMD_ID = 1
CMD_INFO_SRV = 2

''' rated joint torques '''
SIA20_RATED_JOINT_SERVO_TORQUES = [6.0, 6.0, 2.5, 2.5, 1.0, 1.0, 1.0]