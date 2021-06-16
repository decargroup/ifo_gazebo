#!/bin/bash
#
# Setup environment to make PX4 visible to Gazebo.
#
# Note, this is not necessary if using a ROS catkin workspace with the px4
# package as the paths are exported.... EXCEPT THIS SEEMS TO BE BROKEN.
#
# THIS SCRIPT MUST BE PLACED IN THE SAME DIRECTORY ALONGSIDE THE PX4-Autopilot
# DIRECTORY. i.e.
#
# ifo_gazebo/
#     setup_ifo_gazebo.bash
#     PX4-Autopilot/
#     ...
#
# License: according to LICENSE.md in the root directory of the PX4 Firmware repository

THIS_SCRIPT_PATH=${BASH_SOURCE[0]}
PX4_PATH="$(dirname $THIS_SCRIPT_PATH)/PX4-Autopilot"

if [ "$#" != 1 ]; then
    echo -e "You can suppress this printout with \n >> source setup_ifo_gazebo.bash suppress \n"
    SUPPRESS_OUTPUT=false 
else
    SUPPRESS_OUTPUT=true
fi

# setup Gazebo env and update package path
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:${PX4_PATH}/build/px4_sitl_default/build_gazebo
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${PX4_PATH}/Tools/sitl_gazebo/models
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${PX4_PATH}/build/px4_sitl_default/build_gazebo
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${PX4_PATH}:${PX4_PATH}/Tools/sitl_gazebo

if [ "$SUPPRESS_OUTPUT" = false ]; then
    echo -e "GAZEBO_PLUGIN_PATH: $GAZEBO_PLUGIN_PATH"
    echo -e "GAZEBO_MODEL_PATH: $GAZEBO_MODEL_PATH"
    echo -e "LD_LIBRARY_PATH: $LD_LIBRARY_PATH"
    echo -e "ROS_PACKAGE_PATH: $ROS_PACKAGE_PATH"
fi