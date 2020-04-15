


/*  ____      _ _ _     _                  _             _     _                      
*  / ___|___ | | (_)___(_) ___  _ __      / \__   _____ (_) __| | __ _ _ __   ___ ___ 
* | |   / _ \| | | / __| |/ _ \| '_ \    / _ \ \ / / _ \| |/ _` |/ _` | '_ \ / __/ _ \
* | |__| (_) | | | \__ \ | (_) | | | |  / ___ \ V / (_) | | (_| | (_| | | | | (_|  __/
*  \____\___/|_|_|_|___/_|\___/|_| |_| /_/   \_\_/ \___/|_|\__,_|\__,_|_| |_|\___\___|
*                                                                                    
*                                                                                                
*  Armin Niedermüller
*
*/
                                                                                                                                                         
#include <iostream>
#include <ros/ros.h>
#include <ros/topic.h>
#include <ros/console.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <moveit_msgs/PlanningScene.h>

using namespace octomap;


// ------------------------------------------------------------------------------------------------------------
//                                                  THE MAIN
// ------------------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
  // Init ROS Node
  ros::init(argc, argv, "get_octomap_from_planning_scene_topic");
  std::cout << "get_octomap_from_planning_scene_topic" << std::endl;
  ros::NodeHandle nh;
  
  // Wait for the next Planning Scene Message 
  moveit_msgs::PlanningSceneConstPtr planning_scene = ros::topic::waitForMessage<moveit_msgs::PlanningScene>("/move_group/monitored_planning_scene", ros::Duration(10));
  moveit_msgs::PlanningSceneWorld planning_scene_world = (*planning_scene).world;

  // Extracting the OctoMap from the Planning Scene
  octomap_msgs::OctomapWithPose octomap_pose = planning_scene_world.octomap;
  octomap_msgs::Octomap octomap = octomap_pose.octomap;
  
  // Convert from OctoMap Message to Octree
  AbstractOcTree* abstract_map = octomap_msgs::msgToMap(octomap);
  
  OcTree* map = (OcTree*)abstract_map;
  OcTree octree = *map;
  
  // Write Octree to file 
  octree.writeBinary("octomap_frame.bt");
  
  ros::spinOnce();
}


