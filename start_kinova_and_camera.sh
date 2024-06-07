#!/bin/bash

cd /home/unicorn/catkin_ws/
conda deactivate 
source devel/setup.bash 
roslaunch kinova_bringup realsense_kinova_bringup.launch arm:=gen3_lite dof:=6
