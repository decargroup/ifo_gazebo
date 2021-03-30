#!/bin/bash
#
# Setup environment to make PX4 visible to Gazebo.
#
# Note, this is not necessary if using a ROS catkin workspace with the px4
# package as the paths are exported.
#
# License: according to LICENSE.md in the root directory of the PX4 Firmware repository

if [ "$#" != 1 ]; then
    echo -e "usage: source setup_gazebo.bash px4_clone_dir \n"
    return 1
fi

PX4_CLONE_DIR=$1

# setup Gazebo env and update package path
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:${PX4_CLONE_DIR}/build/px4_sitl_default/build_gazebo
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${PX4_CLONE_DIR}/Tools/sitl_gazebo/models
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${PX4_CLONE_DIR}/build/px4_sitl_default/build_gazebo
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${PX4_CLONE_DIR}:${PX4_CLONE_DIR}/Tools/sitl_gazebo

echo -e "GAZEBO_PLUGIN_PATH $GAZEBO_PLUGIN_PATH"
echo -e "GAZEBO_MODEL_PATH $GAZEBO_MODEL_PATH"
echo -e "LD_LIBRARY_PATH $LD_LIBRARY_PATH"
echo -e "ROS_PACKAGE_PATH $ROS_PACKAGE_PATH"
