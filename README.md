# Collision Avoidance in a Human-Robot Collaborative Workspace using two low-cost Intel D435 3D-Cameras
Point Cloud Library, Robot Operating System, Intel D435, OctoMaps, MoveIt!, Rviz

![alt text](https://raw.githubusercontent.com/nerovalerius/collision_avoidance/master/images/full_desk.jpg)

This package is built to filter two ROS PointCloud2 streams from two intel D435 Cameras, 
convert them into Octomaps and align the OctoMaps with a Franka Panda Robot Model inside
Rviz with MoveIt!

## Automated start script
### package/launch/start_everything.sh
This script starts MoveIt and Rviz with a demo robot. MoveIt gets the 3D data of the surroundings via the /move_group/monitored_planning_scene topic

Starts the following:
* Both 3D camera nodes from the realsense-ros package -> www.github.com/nerovalerius/registration_3d/launch/start_3d_cams.launch
* Alignment of both 3D images -> www.github.com/nerovalerius/registration_3d/preprocess_align_publish.cpp
* Manual Alignment between the 3D images and the MoveIt Panda cobot model via ros tf
* Preprocessing of the 3D images - passthrough filtering, outlier filtering, downsampling
* MoveIt with the 3D cobot model and Rviz -> https://github.com/nerovalerius/collision_avoidance/launch/collision_avoidance.launch

## Parameters to set
### panda_moveit_config/config/sensors_d435_pointcloud
* Point Cloud Topics and other params
### panda_moveit_config/launchn/sensors_manager.launch: OctoMaps params and the config file for the 3D cameras.





## Packages needed!
Takes both 3D-Camera images and aligns them with the ICP-algorithm:
www.github.com/nerovalerius/registration_3d

MoveIt Control for the "Franka Panda" collaborative robot
www.github.com/nerovalerius/panda_moveit_config

