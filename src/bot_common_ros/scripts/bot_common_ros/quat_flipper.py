#!/usr/bin/env python

import rospy, pdb
import tf as ros_tf
import tf2_ros

from bot_common_ros.ur_utils import ROStoM3d

from geometry_msgs.msg import TransformStamped, Pose
from rospy_message_converter import message_converter

def main():
    rospy.init_node('quat_flipper', anonymous=False)
    ns = rospy.get_param("/static_tfs")
    

    # loop through each camera
    t_list = []
    for key in ns:

        t = message_converter.convert_dictionary_to_ros_message('geometry_msgs/TransformStamped', ns[key])
        print t.child_frame_id
        tf = ROStoM3d(t.transform.translation, t.transform.rotation)

        # flip
        tf.orient.rotate_xt(3.1415)
        print tf.orient.quaternion

if __name__ == "__main__":
    main()