#!/usr/bin/env bash

# Start ROS bag recording
# things to record:
#   rgb image, next attack position, directionToChisel, camera pose, robot state (use quaternion), 
#   dynamic normal delta-angle (pre-clip), boolean 'is dynamic normal active', boolean 'is movement active'
#   to tell which mode we're in

# updated as per https://offworld-ai.slack.com/archives/C79JNTJGY/p1510185195000075
#  in order:
#  0) camera id, for identification purposes
#  1) camera feed
#  2) depth feed
#  3) original reconstructed point cloud feed
#  4) all transforms (listen to /tf)
#  5) delta-angle
#  6) is delta-angle active
#  7) next attack position expressed in base-frame (to be later transformed into camera-frame)
#  8) direction to chisel -- True: upwards, False: downwards
#  9) is movement active
# 10) volume excavated estimate


# some python code for transforms (might need to invert the tf):
#    (trans, quat) = self.listener.lookupTransform("base", "camera_02_depth_optical_frame", rospy.Time(0)) # latest transform available
#    m3dQuat = m3d.UnitQuaternion(quat[3],m3d.Vector(quat[0:3]))
#    tf = m3d.Transform(m3dQuat, trans) # m3d uses opposite convention (w,x,y,z) instead of ROS (x,y,z,s), s==w
#    attackPos_camera = tf * self.attackPose.pos

# WARNING! please record into an internal drive, otherwise the speed of copying can cause you to either overfill the infinite buffer, or drop important messages

#rosbag record -o "$OFFWORLD_ROOT"/Data2/rosbags/SmartChisel/ver1_3/chiselbot1_3 --split --duration=5m -b 0 \
rosbag record -o /media/offworld/easystore/boulder_rosbag --split --duration=20m -b 0 \
  /tf \
  /tf_static \
  /camera_chisel_track_down_left/depth/image_rect \
  /camera_chisel_track_up_left/depth/image_rect \
  /camera_chisel_track_up_left/depth/camera_info \
  /camera_chisel_track_down_left/depth/camera_info \
  /chisel/RoboChisel/FSM/state \
  /chisel/joint_states \
  /chisel/wrench \
  /chisel/attack_pose \
  