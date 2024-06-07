#!/bin/bash

cd /home/unicorn/catkin_ws/ 
source devel/setup.bash
conda env list
source /home/unicorn/miniconda3/etc/profile.d/conda.sh
conda activate kinovaconda 
python src/realsense_segment2/src/find_grasping_point_server.py
conda deactivate
