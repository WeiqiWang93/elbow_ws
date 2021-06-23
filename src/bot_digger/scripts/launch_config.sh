#!/bin/bash
#######################################
# Copies terminator config from SRM to local and launches config
# For diggerbot operations, diggerbot_common should be launched first and always be running before any other configs are started
# For simulated operations, can launch chisel_sim or saw_sim directly without starting diggerbot_common 
#
# Arguments:
#   config_name: launches config_name immediately; if not provided, the bash script will prompt for input
#   SRM path: replaces default path /home/offworld with system-specific path to SRM 
# Example:
#   ./launch_config.sh  --> user will be prompted for config and SRM path
#   ./launch_config.sh diggerbot_common     --> diggerbot_common config will be launched using default SRM path (/home/offworld)
#   ./launch_config.sh diggerbot_common /home/offworld      --> diggerbot_common config will be launched using specified SRM path
# Config Files:
#   diggebot_common
#   vive_config (vive_localization, vive_world_calibration, vive_cihsel_calibration)
#   pc_capture_config (pointcloud_capture)
#   camera_calibration
#   chisel_ops
#   chisel_sim
#   saw_ops
#   sim_sim
#######################################

if [ $# = 0 ]
  then
    echo "Enter name of config [diggerbot_common, diggerbot_narrow_rack, vive_localization, pointcloud_capture, chisel_ops, chisel_sim, \
    robosaw_operations, robosaw_sim, vive_chisel_calibration, vive_world_calibration, camera_calibration, chisel_tool_cam_calib,\
    chisel_track_up_left_calib, chisel_track_up_right_calib, chisel_track_down_right_calib, chisel_track_down_left_calib, right_edge, cameras_feed_check, vive_and_motion_planning, right_edge_align, panel_manager]:"
    read config_name
    export CONFIG=$config_name

    # echo "Enter name of SRM path (ie /home/offworld):"
    # read srm_path
    export SRM_PATH=$OFFWORLD_HOME

elif [ $# = 1 ]
  then
    export CONFIG=$1
    export SRM_PATH=$OFFWORLD_HOME

elif [ $# = 2 ]
  then
    export CONFIG=$1
    export SRM_PATH=$2
fi

if [ "$CONFIG" = "diggerbot_common" ]
  then
    cp terminator_config/diggerbot_config ~/.config/terminator/config
    sed -i "s#/home/offworld#$SRM_PATH#g" ~/.config/terminator/config
    terminator -l diggerbot_common

elif [ "$CONFIG" = "diggerbot_startup" ]
  then
    cp terminator_config/diggerbot_config ~/.config/terminator/config
    sed -i "s#/home/offworld#$SRM_PATH#g" ~/.config/terminator/config
    terminator -l diggerbot_startup

elif [ "$CONFIG" = "diggerbot_common_logging" ]
  then
    cp terminator_config/diggerbot_config ~/.config/terminator/config
    sed -i "s#/home/offworld#$SRM_PATH#g" ~/.config/terminator/config
    terminator -l diggerbot_common_logging

elif [ "$CONFIG" = "diggerbot_narrow_rack" ]
  then
    cp terminator_config/diggerbot_narrow_config ~/.config/terminator/config
    sed -i "s#/home/offworld#$SRM_PATH#g" ~/.config/terminator/config
    terminator -l diggerbot_common

elif [ "$CONFIG" = "diggerbot_narrow_startup" ]
  then
    cp terminator_config/diggerbot_narrow_config ~/.config/terminator/config
    sed -i "s#/home/offworld#$SRM_PATH#g" ~/.config/terminator/config
    terminator -l diggerbot_startup

elif [ "$CONFIG" = "right_edge" ]
  then
    cp terminator_config/right_edge_reposition_config ~/.config/terminator/config
    sed -i "s#/home/offworld#$SRM_PATH#g" ~/.config/terminator/config
    terminator -l right_edge

elif [ "$CONFIG" = "chisel_tool_cam_calib" ]
  then
    cp terminator_config/chisel_camera_calib_config ~/.config/terminator/config
    sed -i "s#/home/offworld#$SRM_PATH#g" ~/.config/terminator/config
    terminator -l chisel_tool_camera_calibration

elif [ "$CONFIG" = "chisel_track_up_left_calib" ]
  then
    cp terminator_config/chisel_camera_calib_config ~/.config/terminator/config
    sed -i "s#/home/offworld#$SRM_PATH#g" ~/.config/terminator/config
    terminator -l chisel_up_left_camera_calibration

elif [ "$CONFIG" = "chisel_track_up_right_calib" ]
  then
    cp terminator_config/chisel_camera_calib_config ~/.config/terminator/config
    sed -i "s#/home/offworld#$SRM_PATH#g" ~/.config/terminator/config
    terminator -l chisel_up_right_camera_calibration

elif [ "$CONFIG" = "chisel_track_down_right_calib" ]
  then
    cp terminator_config/chisel_camera_calib_config ~/.config/terminator/config
    sed -i "s#/home/offworld#$SRM_PATH#g" ~/.config/terminator/config
    terminator -l chisel_down_right_camera_calibration

elif [ "$CONFIG" = "chisel_track_down_left_calib" ]
  then
    cp terminator_config/chisel_camera_calib_config ~/.config/terminator/config
    sed -i "s#/home/offworld#$SRM_PATH#g" ~/.config/terminator/config
    terminator -l chisel_down_left_camera_calibration

elif [ "$CONFIG" = "vive_localization" ]
  then
    cp terminator_config/vive_config ~/.config/terminator/config
    sed -i "s#/home/offworld#$SRM_PATH#g" ~/.config/terminator/config
    terminator -l vive_localization

elif [ "$CONFIG" = "pointcloud_capture" ]
  then
    cp terminator_config/pc_capture_config ~/.config/terminator/config
    sed -i "s#/home/offworld#$SRM_PATH#g" ~/.config/terminator/config
    terminator -l pointcloud_capture

elif [ "$CONFIG" = "chisel_ops" ]
  then
    cp terminator_config/chisel_ops_config ~/.config/terminator/config
    sed -i "s#/home/offworld#$SRM_PATH#g" ~/.config/terminator/config
    terminator -l chisel_ops
    
elif [ "$CONFIG" = "chisel_sim" ]
  then
    cp terminator_config/chisel_sim_config ~/.config/terminator/config
    sed -i "s#/home/offworld#$SRM_PATH#g" ~/.config/terminator/config
    terminator -l chisel_sim

elif [ "$CONFIG" = "robosaw_operations" ]
  then
    cp terminator_config/saw_ops_config ~/.config/terminator/config
    sed -i "s#/home/offworld#$SRM_PATH#g" ~/.config/terminator/config
    terminator -l robosaw_operations

elif [ "$CONFIG" = "robosaw_operations_logging" ]
  then
    cp terminator_config/saw_ops_config ~/.config/terminator/config
    sed -i "s#/home/offworld#$SRM_PATH#g" ~/.config/terminator/config
    terminator -l robosaw_operations_logging

elif [ "$CONFIG" = "robosaw_sim" ]
  then
    cp terminator_config/saw_sim_config ~/.config/terminator/config
    sed -i "s#/home/offworld#$SRM_PATH#g" ~/.config/terminator/config
    terminator -l robosaw_sim

elif [ "$CONFIG" = "vive_chisel_calibration" ]
  then
    cp terminator_config/vive_config ~/.config/terminator/config
    sed -i "s#/home/offworld#$SRM_PATH#g" ~/.config/terminator/config
    terminator -l vive_chisel_calibration

elif [ "$CONFIG" = "vive_world_calibration" ]
  then
    cp terminator_config/vive_config ~/.config/terminator/config
    sed -i "s#/home/offworld#$SRM_PATH#g" ~/.config/terminator/config
    terminator -l vive_world_calibration

elif [ "$CONFIG" = "camera_calibration" ]
  then
    cp terminator_config/camera_calib_config ~/.config/terminator/config
    sed -i "s#/home/offworld#$SRM_PATH#g" ~/.config/terminator/config
    terminator -l camera_calibration

elif [ "$CONFIG" = "cameras_feed_check" ]
  then
    cp terminator_config/chisel_camera_calib_config ~/.config/terminator/config
    sed -i "s#/home/offworld#$SRM_PATH#g" ~/.config/terminator/config
    terminator -l cameras_feed_check

elif [ "$CONFIG" = "right_edge_align" ]
  then
    cp terminator_config/right_edge_positioning_config ~/.config/terminator/config
    sed -i "s#/home/offworld#$SRM_PATH#g" ~/.config/terminator/config
    terminator -l right_edge_align

elif [ "$CONFIG" = "right_edge_positioning" ]
  then
    cp terminator_config/right_edge_positioning_config ~/.config/terminator/config
    sed -i "s#/home/offworld#$SRM_PATH#g" ~/.config/terminator/config
    terminator -l right_edge_positioning

elif [ "$CONFIG" = "panel_manager" ]
  then
    cp terminator_config/panel_manager_config ~/.config/terminator/config
    sed -i "s#/home/offworld#$SRM_PATH#g" ~/.config/terminator/config
    terminator -l panel_manager_sim

else
    echo "BAD INPUT"
fi