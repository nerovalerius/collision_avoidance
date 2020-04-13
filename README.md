# Collision Avoidance in a Human-Robot Collaborative Workspace using 2 Intel D435 3D-Cameras
Point Cloud Library, Robot Operating System, Intel D435, OctoMaps, MoveIt!, Rviz


![alt text](https://raw.githubusercontent.com/nerovalerius/collision_avoidance/master/images/full_desk.jpg)


This package is built to filter two ROS PointCloud2 streams from two intel D435 Cameras, 
convert them into Octomaps and align the OctoMaps with a Franka Panda Robot Model inside
Rviz with MoveIt!

## Parameters to set
At package panda_moveit_config/config/sensors_d435_pointcloud: Point Cloud Topics and other params
At package panda_moveit_config/launchn/sensors_manager.launch: OctoMaps params and the config file for the 3D cameras

## To start everything automatically
At this package/launch/start_everything.sh

## Packages needed!
www.github.com/nerovalerius/registration_3d
www.github.com/nerovalerius/panda_moveit_config

