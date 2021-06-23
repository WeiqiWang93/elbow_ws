#!/bin/bash

source /opt/ros/kinetic/setup.bash
OFFWORLD_ROOT=$1
source $OFFWORLD_ROOT/swarm-robotic-mining/catkin_ws/devel/setup.bash --extend

roscd bot_digger_description/urdf

ls

export xacro_file=$2

echo "Generating .urdf file"

export urdf_file=$(cut -d '.' -f 1 <<< $xacro_file).urdf

rosrun xacro xacro --inorder -o $urdf_file $xacro_file 

export dae_file=$(cut -d '.' -f 1 <<< $urdf_file).dae
export ROBOT_NAME=$(cut -d '.' -f 1 <<< $urdf_file)

rosrun collada_urdf urdf_to_collada $urdf_file $dae_file

export IKFAST_PRECISION="5"
rosrun moveit_kinematics round_collada_numbers.py $dae_file $dae_file "$IKFAST_PRECISION"

echo "Reading .dae file "$dae_file
openrave-robot.py $dae_file --info links

echo "Specify the BASE_LINK"
read base_link
export BASE_LINK=$base_link

echo "Specify the EEF_LINK"
read eef_link
export EEF_LINK=$eef_link



echo "Specify the PLANNING GROUP. ex: saw, chisel"
read group
export PLANNING_GROUP=$group

roscd bot_arm_planner/../IKFast

export IKFAST_OUTPUT_PATH=$(pwd)/ikfast61_"$PLANNING_GROUP".cpp
echo $IKFAST_OUTPUT_PATH

echo "Does the robot comprise a 7-DOF arm? (y/n)"
read is_7dof

if [ "$is_7dof" = "y" ]
then
    echo "Specify the FREE INDEX:"
    read free_index
    export FREE_INDEX=$free_index

    roscd bot_digger_description/urdf
    python $(openrave-config --python-dir)/openravepy/_openravepy_/ikfast.py \
    --robot=$dae_file \
    --iktype=transform6d \
    --baselink="$BASE_LINK" \
    --eelink="$EEF_LINK" \
    --freeindex="$FREE_INDEX" \
    --savefile="$IKFAST_OUTPUT_PATH"
else
    roscd bot_digger_description/urdf
    python $(openrave-config --python-dir)/openravepy/_openravepy_/ikfast.py \
    --robot=$dae_file \
    --iktype="transform6d" \
    --baselink="$BASE_LINK" \
    --eelink="$EEF_LINK" \
    --savefile="$IKFAST_OUTPUT_PATH"
fi


export ik_plugin_pkg="$ROBOT_NAME"_ikfast_"$PLANNING_GROUP"_plugin

if [[ -d "$OFFWORLD_ROOT/swarm-robotic-mining/catkin_ws/src/$ik_plugin_pkg" ]]
then
    echo "Removing existing plugin pkg."
    roscd $ik_plugin_pkg
    rm -rf src/ && rm -rf include/
else
    echo "Creating IK Fast plugin pkg"
    cd $OFFWORLD_ROOT/swarm-robotic-mining/catkin_ws/src
    catkin_create_pkg "$ik_plugin_pkg"
    rospack find "$ik_plugin_pkg"
fi

export MOVEIT_IK_PLUGIN_PKG=$ik_plugin_pkg

rosrun bot_digger_description create_ikfast_moveit_plugin.py $ROBOT_NAME "$PLANNING_GROUP" "$MOVEIT_IK_PLUGIN_PKG" "$IKFAST_OUTPUT_PATH"

roscd
cd ..

