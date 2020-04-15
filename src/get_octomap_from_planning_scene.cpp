
#include <ros/ros.h>

// MoveIt!
#include <planning_scene_monitor.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>



int main(int argc, char** argv)
{
    ros::init(argc, argv, "panda_arm_kinematics");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    

        // PANDA INIT
    static const std::string PLANNING_GROUP = "panda_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    move_group.getPlanningFrame()



    planning_scene_monitor::PlanningSceneMonitor planning_scene();

    planning_scene.

    // Starts the planning_scene_subscriber_
    startSceneMonitor 

    // starts the collision_object_subscriber_, the planning_scene_world_subscriber_, and the OccupancyMapMonitor.
    startWorldGeometryMonitor 

    // starts the CurrentStateMonitor and the attached_collision_object_subscriber_.
    startStateMonitor

    // starts another thread for publishing the entire planning scene on a provided topic for other PlanningSceneMonitor’s to subscribe to
    startPublishingPlanningScene

    // starts the get_scene_service_.
    providePlanningSceneService 






  ros::shutdown();
  return 0;
}