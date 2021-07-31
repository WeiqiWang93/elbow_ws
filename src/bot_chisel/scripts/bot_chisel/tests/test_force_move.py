import os, sys, time, pdb

import URBasic, urx

prg1 = """def force_move_ex():
  set_standard_analog_input_domain(0, 1)
  set_standard_analog_input_domain(1, 1)
  set_tool_analog_input_domain(0, 1)
  set_tool_analog_input_domain(1, 1)
  set_analog_outputdomain(0, 0)
  set_analog_outputdomain(1, 0)
  set_tool_voltage(0)
  set_input_actions_to_default()
  set_tcp(p[0.0,0.0,0.0,0.0,0.0,0.0])
  set_payload(8.0, [0.0, -0.069, 0.184])
  set_gravity([-9.82, 1.803904735144051E-15, 6.013015783813504E-16])
  # begin: URCap Installation Node
  #   Source: Robotiq_Force_Torque_Sensor, 1.0.3, Robotiq Inc.
  #   Type: FT Sensor
  ###############################################################
  # Script file used to communicate with Robotiq's ft sensor
  # Version: 0.0.1
  ###############################################################
  
  def rq_move_relative(P_from, P_to, Pi):
    return pose_trans(P_to, pose_trans(pose_inv(P_from), Pi))
  end
  
  def rq_elementWiseSub(list1, list2):
    global rq_return_list=list1
    i=0
    while i<length(list1):
      rq_return_list[i]=list1[i] - list2[i]
      i=i+1
    end
    return rq_return_list
  end
  
  if (not socket_open("127.0.0.1",63351,"stream")):
   popup("Can't connect to the sensor driver", "Robotiq's FT Sensor", error=True)
  end
  # end: URCap Installation Node
  while (True):
    $ 1 "Robot Program"
    sleep(0.02)
    force_mode(tool_pose(), [0, 0, 0, 0, 0, 0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 2, [0.5, 0.5, 0.5, 0.3490658503988659, 0.3490658503988659, 0.3490658503988659])
    $ 2 "Force"
    $ 3 "MoveL"
    $ 4 "Waypoint_2"
    movel([-2.063797646232839, -2.7944786173090232, -1.6035880194825785, 1.3494210743801096, 2.065506082254637, 0.03670016349681459], a=1.2, v=0.25)
    $ 5 "Waypoint_1"
    movel([-1.6816252441750619, -2.9335789882595833, -1.696584680303543, 1.4870729868993857, 1.6857872788075206, -0.00896890263943817], a=1.2, v=0.25)
    end_force_mode()
    stopl(5.0)
  end
end
"""


prg2 = """def force_move_ex():
  while (True):
    $ 1 "Robot Program"
    sleep(0.02)
    force_mode(tool_pose(), [0, 0, 0, 0, 0, 0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 2, [0.5, 0.5, 0.5, 0.3490658503988659, 0.3490658503988659, 0.3490658503988659])
    $ 2 "Force"
    $ 3 "MoveL"
    $ 4 "Waypoint_2"
    movel([-2.063797646232839, -2.7944786173090232, -1.6035880194825785, 1.3494210743801096, 2.065506082254637, 0.03670016349681459], a=1.2, v=0.25)
    $ 5 "Waypoint_1"
    movel([-1.6816252441750619, -2.9335789882595833, -1.696584680303543, 1.4870729868993857, 1.6857872788075206, -0.00896890263943817], a=1.2, v=0.25)
    end_force_mode()
    stopl(5.0)
  end
end
"""



prg3 = """def force_move_ex():
  while (True):
    $ 1 "Robot Program"
    sleep(0.02)
    force_mode(tool_pose(), [0, 0, 0, 0, 0, 0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 2, [1.0, 1.0, 1.0, 0.7, 0.7, 0.7])
    $ 2 "Force"
    $ 3 "MoveL"
    $ 5 "Waypoint_1"
    movel([-1.6816252441750619, -2.9335789882595833, -1.696584680303543, 1.4870729868993857, 1.6857872788075206, -0.00896890263943817], a=1.2, v=0.25)
    end_force_mode()
    stopl(5.0)
  end
end
"""

prg_test_movel = """def force_move_ex():
  movel([-1.6816252441750619, -2.9335789882595833, -1.696584680303543, 1.4870729868993857, 1.6857872788075206, -0.00896890263943817], a=1.2, v=0.25)
  stopl(5.0)
end
"""

prg_movel = """def force_move_ex():\n  {movestr_stripped}\nend\n"""

def get_str_force_move():
  pv = [-0.20453979, -0.6543287 , -0.12293676, -0.03324129,  1.99508058,
       -2.23358588]
  movestr = ropeRob._move(movetype='l', pose=pv, a=0.2, v=0.1, wait=False)
  idx = movestr.find("m")
  movestr_stripped = movestr[idx:]

        # make the URScript program
        # movel(get_actual_tcp_pose(), a=1.2, v=0.25)
  prg = '''def ur_force_move():
  while (True):
    sleep(0.02)
    force_mode(tool_pose(), [0, 0, 0, 0, 0, 0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 2, [1.0, 1.0, 1.0, 0.7, 0.7, 0.7])
    {movestr_stripped}
    end_force_mode()
    stopl(5.0)
  end
end
'''
  prg_formatted = prg.format(**locals())
  return prg_formatted

HOST = "192.168.2.101"
robotModel = URBasic.robotModel.RobotModel()
ropeRob = URBasic.urScriptExt.UrScriptExt(host=HOST, robotModel=robotModel, hasForceTorque=True)
rob = urx.Robot(HOST, use_rt=True)

def send_prg_rope(prg):
    ropeRob.robotConnector.RealTimeClient.SendProgram(prg)


def send_prg_rob(prg):
    rob.send_program(prg)

done = False
while not done:
    pdb.set_trace()

ropeRob.close()
rob.close()