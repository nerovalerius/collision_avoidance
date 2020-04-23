/*  ____      _ _ _     _                  _             _     _                      
*  / ___|___ | | (_)___(_) ___  _ __      / \__   _____ (_) __| | __ _ _ __   ___ ___ 
* | |   / _ \| | | / __| |/ _ \| '_ \    / _ \ \ / / _ \| |/ _` |/ _` | '_ \ / __/ _ \
* | |__| (_) | | | \__ \ | (_) | | | |  / ___ \ V / (_) | | (_| | (_| | | | | (_|  __/
*  \____\___/|_|_|_|___/_|\___/|_| |_| /_/   \_\_/ \___/|_|\__,_|\__,_|_| |_|\___\___|
*                                                                                    
*  https://github.com/nerovalerius/collision_avoidance                    
*
*  Armin Niedermüller
*  https://www.armin-niedermueller.net
*  https://www.linkedin.com/in/armin-niedermüller-317516159/ 
*
*
*/



// ROS
#include <ros/ros.h>
#include <ros/topic.h>
#include <ros/console.h>

// OctoMap
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

// Misc
#include <iostream>
#include <boost/scoped_ptr.hpp>
#include <pluginlib/class_loader.h>

int main(int argc, char** argv)
{

    // Init ROS Node
    ros::init(argc, argv, "planning_scene_operations");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;
    
    // Define the planning group and load the robot model
    const std::string PLANNING_GROUP = "panda_arm";
    robot_model_loader::RobotModelLoaderPtr robot_model_loader(
        new robot_model_loader::RobotModelLoader("robot_description"));
    robot_model::RobotModelPtr robot_model = robot_model_loader->getModel();

    // Get a Pointer to the robot pose and the planning group
    robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
    const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

    // Get a Pointer to the planning scene
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

    // Monitor the Planning Scene / Start the Planning Scene subscriber
    planning_scene_monitor::PlanningSceneMonitorPtr ps_monitor(
        new planning_scene_monitor::PlanningSceneMonitor(planning_scene, robot_model_loader));
    ps_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
    
    // Start the CurrentStateMonitor and the attached_collision_object_subscriber_.
    ps_monitor->startStateMonitor();
    ps_monitor->startSceneMonitor();
  
    // Start the collision_object_subscriber_, the planning_scene_world_subscriber_, and the OccupancyMapMonitor.
    ps_monitor->startWorldGeometryMonitor();

    // Get acces to the Planning Scene - Read Only
    planning_scene_monitor::LockedPlanningSceneRW locked_scene(ps_monitor);
    planning_scene::PlanningScenePtr scene = locked_scene;

    // Get current OctoMap with pose from Planning Scene
    octomap_msgs::OctomapWithPose octomap_pose;
    locked_scene->getOctomapMsg(octomap_pose);
    
    // Get the OctoMap
    octomap_msgs::Octomap octomap = octomap_pose.octomap;
    
    // Convert from OctoMap Message to Octree
    octomap::AbstractOcTree* abstract_map = octomap_msgs::msgToMap(octomap);
    octomap::OcTree* map = (octomap::OcTree*)abstract_map;
    octomap::OcTree octree = *map;
    
    // Write Octree to file 
    octree.writeBinary("octomap_frame.bt");
    
    ros::spinOnce();


  ros::shutdown();
  return 0;
}