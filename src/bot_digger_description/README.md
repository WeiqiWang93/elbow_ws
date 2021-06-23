**generate_ik.sh usage**

1) roscd bot_digger_description/scripts/bot_digger_description/
2) run ./generate_ik.sh <path to folder containing swarm_robotic_mining folder> <xacro file representing the urdf of arm+end-effector combination>
3) Example usage `./generate_ik.sh /home/offworld3/CodeBase ur16_hilti.urdf.xacro`
4) For BASE_LINK provide the index number corresponding to 'chisel_base_link' or 'saw_base_link' from the list displayed just above.
5) For EEF_LINK provide the index number corresponding to 'chisel_tip_link' or 'saw_tip_link' from the list displayed just above.

For SIA20 Goelz:     
    `./generate_ik.sh /home/offworld/ sia20_goelz.urdf.xacro`
    BASE_LINK=1, EEF_LINK=12, FREE_LINK=2, planning_group=saw, 7DOF=y