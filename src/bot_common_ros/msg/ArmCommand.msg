uint8 MOVEL = 1
uint8 MOVELS = 2
uint8 MOVEJ = 3
uint8 MOVEJS = 4
uint8 force_stop2 = 5
uint8 FORCE_MODE = 6
uint8 END_FORCE_MODE = 7
uint8 SHUTDOWN = 8
uint8 SET_SPEED = 9
uint8 STEPL = 10
uint8 STEPJ = 11
#
string type
geometry_msgs/Pose pose
geometry_msgs/Pose[] pose_list # NOT USED
geometry_msgs/Pose[] poseList_base

# joint mode stuff
std_msgs/Float32MultiArray joints
std_msgs/Float32MultiArray[] jointList

# force mode stuff
geometry_msgs/Pose pose_base
std_msgs/Float32MultiArray torque
std_msgs/Float32MultiArray limits
std_msgs/Float32MultiArray selection_vector
bool setRemote

# allowed to have default values
bool override
bool wait
float32 acc
float32 vel
float32 vel_rot
float32 radius
float32 timeout
float32 threshold
int32 id1
int32 val