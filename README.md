# kinova

Normal Kinova:

to start kinova (outside conda): `$ cd ~/catkin_ws` 

`$ source devel/setup.bash`

`roslaunch kortex_driver kortex_driver.launch arm:=gen3_lite dof:=6` -> start basic bringup
	   
to start camera:
`roslaunch realsense2_camera rs_camera.launch`

*NOTE*
- More documentation can be found https://github.com/Kinovarobotics/ros_kortex
- Andre's repo can be found https://github.com/DreVinciCode/ROS_KinovaGen3LiteAR/ and has some good referance information

To start kinova and camera:
`roslaunch kinova_bringup realsense_kinova_bringup.launch arm:=gen3_lite dof:=6`

In another terminal (in ~/catkin/ws after sourcing terminal)
`$ conda activate kinovaconda`

`rosrun realsense_segment2 find_grasping_point_client.py`
this will run the client that takes in natural language string

In another terminal (in ~/catkin/ws after sourcing terminal)
`$ conda activate kinovaconda`

`python src/realsense_segment2/src/find_grasping_point_server.py`


In kinova movement example code;
`rosrun kortex_examples example_full_arm_movement.py robot_name:=my_gen3_lite`

>>`https://github.com/AABL-Lab/armpy` (AABL-Lab )

Visualize the poses in rviz

###### Top down pose: (change x,y,z)
`rosrun kortex_examples example_single_point.py `


## Shells 

In new terminals under base conda environment, directly bash shell files will enter the corresponding conda environment and move to ~/catkin/ directory.

# Few common errors:

1. CUDA out of memory: python processes did not die gracefully
solution: `sudo pkill -9 python`

2. CUDA not visible:
try doing `nvidia-smi`; if you don't see the driver
restart computer

3. try not to install driver again:

Wed Apr 17 17:09:17 2024       
+---------------------------------------------------------------------------------------+
| NVIDIA-SMI 535.171.04             Driver Version: 535.171.04   CUDA Version: 12.2     |
|-----------------------------------------+----------------------+----------------------+
| GPU  Name                 Persistence-M | Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp   Perf          Pwr:Usage/Cap |         Memory-Usage | GPU-Util  Compute M. |
|                                         |                      |               MIG M. |
|=========================================+======================+======================|
|   0  NVIDIA GeForce RTX 3080        Off | 00000000:01:00.0  On |                  N/A |
| 30%   37C    P8              20W / 320W |    631MiB / 10240MiB |     36%      Default |
|                                         |                      |                  N/A |
+-----------------------------------------+----------------------+----------------------+


4. In ros service; sometimes the code just fails:
rerun the client


# TO DO:

1. Add table in the URDF so that the robot does not hit the table
2. Add the camera pole in the URDF so that the robot does not hit the pole
3. Figure out why the pose sometimes is incorrect
4. Sometimes the server cannot find the object - it returns an error and everthing stops. Write an exception to catch the error and report the error
5. Clean the code (add comments)
6. Do pick and place using human generated NL (pick up the blue can; place in the orange bowl)

