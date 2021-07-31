from bot_common_ros.sia20_driver import SIA20Driver
from bot_common_ros.arm_overseer import ArmOverseer
import rospy, pdb, time
import numpy as np
from bot_overseer_api import OverseerAPI

use_arm_overseer = True   

if use_arm_overseer == False:
  rospy.init_node("test")
  OA = OverseerAPI()
  OA.run_private_op("SAW.ARM.INIT_SIA20", sim=True, overseer=True)
  time.sleep(1.0)
  js = OA.get_tm("SAW.ARM.JOINT_STATES")
  pl = OA.get_tm("SAW.BLADE.POSE_LIST")
  # jt = OA.get_tm("SAW.ARM.TORQUE_JOINTS")
  vel = OA.get_tm("SAW.BLADE.VEL")

  print "Joint States: {}".format(js)
  print "Pose List: {}".format(pl)
  # print "Joint Torques: {}".format(jt)
  print "Velocities: {}".format(vel)
  pdb.set_trace()

  OA.run_private_op("SAW.ARM.STEPJ", joint="joint_s", ang_velocity=10.0, duration=1.0)
  OA.run_private_op("SAW.ARM.SMART_MOVE", jointList=[[-0.0034, 0.000827, 0.00089, -1.270705, -0.00022, 0.000145, 0.0012084]])
  ret = OA.run_private_op("SAW.ARM.GET_LAST_PT")

else:
  rospy.init_node("test")
  arm = ArmOverseer()
  arm.initialize_robot(sim=False)  # part of initialize_robot should be checking if initialized, if it already is then don't do it again
  js = arm.get_joints()
  pl = arm.get_pose()
  vel = arm.get_tcp_speed()

  print "Joint States: {}".format(js)
  print "Pose List: {}".format(pl)

  print "Velocities: {}".format(vel)


  j1 = [-2.658116102218628, 0.978338897228241, 0.265054851770401, -1.7802188396453857, 1.36278235912323, 1.2215194702148438, -1.3003345727920532]
  j2 = [-2.358116102218628, 0.978338897228241, 0.265054851770401, -1.7802188396453857, 1.36278235912323, 1.2215194702148438, -1.3003345727920532]

  pdb.set_trace()
  # arm.stepj(joint="joint_s", ang_velocity=10.0, duration=1.0)
  arm.movejs(jointList=[j1, j2], vel_rot=float(np.deg2rad(15.0)), wait=True)
  # arm.smart_move(jointList=[[-0.0034, 0.000827, 0.00089, -1.270705, -0.00022, 0.000145, 0.0012084], [-1.0034, 0.000827, 0.00089, -1.270705, -0.00022, 0.000145, 0.0012084]])
  pdb.set_trace()

  arm.update_last_pt()
  last_pt_list = arm.get_last_pt()