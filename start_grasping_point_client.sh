#!/bin/bash

cd /home/unicorn/catkin_ws 

source devel/setup.bash 
source /home/unicorn/miniconda3/etc/profile.d/conda.sh 

conda activate kinovaconda 
rosrun realsense_segment2 find_grasping_point_client.py 
conda deactivate
