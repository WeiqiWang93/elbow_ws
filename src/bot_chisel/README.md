# Instructions for running Robochisel Simulator

## Changelog
* Last updated 11/18

## Running robochisel in Gazebo sim
1) roscd bot_chisel/data/ && ./save_chisel_sim_pcd.sh
  ** **CAUTION**: this will overwrite existing point cloud data in ~/.ros, not necessary if there is already valid PCD data saved
2) roscd bot_digger/scripts && ./launch_config.sh chisel_sim /home/offworld
3) Run mission 'chisel_initialize_ur10_sim'
4) Run mission 'chisel_moveto_base_state'
5) Run mission 'scan_quad_sim'
  ** would not recommend scanning in sim unless real_time_update_rate in amandebult_world.panel is set to 1000 and max_step_size set to 0.001
6) Run mission 'robochisel_sim'
  ** Does not perform attacks