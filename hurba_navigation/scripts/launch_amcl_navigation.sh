#!/bin/bash

# build the workspace
# cd $(pwd)/../../..; catkin_make
cd $(pwd)/../../..;

source devel/setup.bash
#source /opt/ros/melodic/setup.bash
export HURBA_MAP_FILE="$(pwd)/src/Project-3-Navigation/hurba_navigation/maps/map.yaml"

xterm  -e  " roslaunch hurba_navigation amcl_localization.launch" &    
sleep 5
xterm  -e  " roslaunch hurba_navigation teleop.launch" &    
sleep 10
xterm  -e  " roslaunch hurba_navigation movebase_navigation.launch"