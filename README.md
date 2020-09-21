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
    - [panda_moveit_config](https://github.com/nerovalerius/panda_moveit_config.git)
    - [realsense-ros](https://github.com/IntelRealSense/realsense-ros.git)
- [clone and build libfranka 0.8.0 and franka_ros ](https://frankaemika.github.io/docs/installation_linux.html#building-from-source)
- Execute `rosdep install -i --from-paths src` from the workspace to install ROS dependencies.
- Execute `catkin_make` from the workspace to build all packages in the workspace.
- Download the following rosbag file: [Link](https://drive.google.com/file/d/1eIEW_tNSs0p7Sgny7x9dS-HAtSRvAcDm/view?usp=sharing)
    - This is a 1 second sequence of both 3D cameras in our laboratory.
    - Please adapt the location of the rosbag file inside the start_rosbag.sh file
- If you want to use the franka panda without a realtime kernel


## Configuration
- panda_moveit_config/config/sensors_d435_pointcloud: Point Cloud Topics and configuration of your 3D Cameras
- panda_moveit_config/launchn/sensors_manager.launch: OctoMaps params, also loads the 3D Camera config yaml
- panda_moveit_config/launch/move_group.launch: Change the refreshing framerate of OctoMap/Planning Scene for MoveIt
    ```sh
    <param name="planning_scene_monitor/publish_planning_scene_hz" value="30.0" />
    ```

- franka::RealtimeConfig::kIgnore must be set when instantiating franka::Robot class.
    Otherwise the robot just runs with an FULL_PREEMPT_RT Kernel. Means: No Nvidia Drivers.
    To use franka panda without a realtime kernel two possibilites exist:
    - add the line `node_handle.setParam("realtime_config", std::string("ignore"));` inside `franka_ros/src/franka_control_node.cpp` after the node_handle is initalized and before franka_control.init() happens
    - OR if your `franka_control_node.cpp` initalizes the robot with `franka::Robot robot(robot_ip);` - then change this line to `franka::Robot robot(robot_ip, franka::RealtimeConfig::kIgnore);`

## Usage
This script starts MoveIt and Rviz with a demo robot. A rosbag file streams fake 3D point cloud data recorded in our laboratory.
```sh
rosrun collison_avoidance start_rosbag.sh
```

If you use your own intel D435(i) 3D cameras:
```sh
rosrun collison_avoidance start.sh
```
-   However, you need to change the serial numbers to your intel cameras inside the script. 
    Furthermore, the alignment of both 3D point cloud streams is realized with the ICP algorithm. 
    This algorithm needs a coarse manual alignment of both cameras. 
    If the mounting position of your cameras differs from the one as seen in [registration_3d](https://github.com/nerovalerius/registration_3d.git),
    you need to change the manual alignment parameters inside (registration_3d/src/preprocess_align_publish.cpp](https://github.com/nerovalerius/registration_3d/blob/master/src/preprocess_align_publish.cpp) - line 522


The Following steps are conducted:
- Start both 3D camera nodes / Rosbag - depending on which start script is used
- Align both 3D images and stream to a united output ros topic
- Manually align the united 3D point clouds to the MoveIt Panda cobot model via ros tf
- Preprocess the 3D point clouds - passthrough filter, outlier filter, voxel-grid fiilter
- Start MoveIt and Rviz together with the 3D cobot model


