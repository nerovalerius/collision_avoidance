# Collision Avoidance in a Human-Robot Collaborative Workspace using two Intel D435 3D-Cameras and a Franka Panda Cobot
![alt text](https://raw.githubusercontent.com/nerovalerius/collision_avoidance/master/images/full_desk.jpg)

This package realizes a software prototype to let a franka panda cobot (collaborative robot) recognize its surroundings with two 3D cameras.
The cobot is controlled via ROS and Ubuntu.
The cobot's surroundings are sensed with two Intel D435 3D-Cameras, which are mounted above a human-robot-collaborative (HRC) workspace.
Their point cloud streams are first semi-automatically aligned with the iterative closest point algorithm (ICP),
such that the final 3D point cloud stream of the HRC workspace shows the whole workspace almost without any masked areas.
Afterwards, the point clouds are converted into Octomaps and visualized inside Rviz together with the 3D model of the cobot.

## Prerequisites
- [Ubuntu Xenial](http://releases.ubuntu.com/16.04/)
- [ROS Kinetic](http://wiki.ros.org/kinetic)

## Installation
- Install Ubuntu and ROS.
- Create a [catkin](http://wiki.ros.org/catkin) workspace (see [Creating a workspace for catkin](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)).
- Clone this repository into the workspace's `src` directory.
- Clone other, required repositories into the `src` directory:
    - [registration_3d](https://github.com/nerovalerius/registration_3d.git)
    - [franka_ros](https://github.com/nerovalerius/franka_ros.git)
    - [panda_moveit_config](https://github.com/nerovalerius/panda_moveit_config.git)
    - [realsense-ros](https://github.com/IntelRealSense/realsense-ros.git)
- Execute `rosdep install -i --from-paths src` from the workspace to install ROS dependencies.
- Execute `catkin_make` from the workspace to build all packages in the workspace.

## Usage
This script starts MoveIt and Rviz with a demo robot. MoveIt gets the 3D data of the surroundings via the /move_group/monitored_planning_scene topic
```sh
package/launch/start_everything.sh
```

The Following steps are conducted:
- Start both 3D camera nodes
- Align both 3D images and stream to a united output ros topic
- Manually align the united 3D point clouds with the MoveIt Panda cobot model via ros tf
- Preprocess the 3D point clouds - passthrough filter, outlier filter, voxel-grid fiilter
- Start MoveIt and Rvize together with the 3D cobot model
