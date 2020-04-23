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


                                                                                                                                                        
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_config.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/surface/mls.h>
#include <pcl/surface/impl/mls.hpp>

#include <iostream>
#include <ros/ros.h>
#include <ros/topic.h>
#include <ros/console.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/visualization/pcl_visualizer.h>



// ------------------------------------------------------------------------------------------------------------
//                                                   MAIN CLASS
// ------------------------------------------------------------------------------------------------------------

class WorkspaceMapping final
{
  // Step counter
  uint step = 1;

  // Steps switch
  bool passthrough_active = false;
  bool downsampling_active = false;
  bool outlier_active = false;

public:
  int run(int argc, char **argv);

  // Container for original & filtered data
  pcl::PointCloud<pcl::PointXYZRGB> * input_cloud_1;
  pcl::PointCloud<pcl::PointXYZRGB>* input_cloud_2;
  pcl::PointCloud<pcl::PointXYZRGB> output_cloud_2, output_cloud_1;

  // Publishers and Subscibers
  ros::Subscriber sub_cloud_1, sub_cloud_2; 
  ros::Publisher pub_cloud_1, pub_cloud_2;

  // Callback Functions
  void Cloud1Callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  void Cloud2Callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

private:
  std::pair<bool, int> ProcessArguments(int argc, char **argv);
  void PassthroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
  void Downsampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
  void RemoveOutliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
};


// ----------------------------------------------------------
//                      ARGUMENT PARSING
// ----------------------------------------------------------
std::pair<bool, int> WorkspaceMapping::ProcessArguments(int argc, char **argv)
{
  if (argc < 2)
  {
    std::cout << " " << std::endl;
    return std::make_pair(false, -1);
  }

  // Argument-Vector
  std::vector<std::string> args;

  // Parse Arguments
  for (int i = 1; i < argc; i++)
  {
    args.push_back(std::string(argv[i]));
  }

  // Iterate through arguments
  for (const auto &arg : args)
  {
    // User needs HELP
    if (arg.find("help") != std::string::npos)
    {
      std::cout << "COLLISION_AVOIDANCE"
                << "\nPCL-Version: " << PCL_VERSION
                << "\nThis program is designed to:"
                << std::endl;
      return std::make_pair(false, 0);
    }  
    else if (arg.find("passthrough=true") != std::string::npos)
    {
      passthrough_active = true;

      // Activate downsampling
    }
    else if (arg.find("downsampling=true") != std::string::npos)
    {
      downsampling_active = true;

      // Activate outlier filtering
    }
    else if (arg.find("outlier=true") != std::string::npos)
    {
      outlier_active = true;

    }
    else if (arg.find("allsteps=true") != std::string::npos)
    {
      passthrough_active = true;
      downsampling_active = true;
      outlier_active = true;
    } else
    {
      std::cout << "No arguments given" << std::endl;
      return std::make_pair(false, -1);
    }
  }


  return std::make_pair(true, -1);
}

// ----------------------------------------------------------
//                       CALLBACK FOR CLOUD 1
// ----------------------------------------------------------

 
void WorkspaceMapping::Cloud1Callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_1_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

  // Convert to PCL data type
  pcl::fromROSMsg(*cloud_msg, *input_cloud_1_ptr);
  
  // Remove NaN Points from cloud - should speed up filter algorithms
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*input_cloud_1_ptr, *input_cloud_1_ptr, indices); 

  // Perform the actual filtering
  if (passthrough_active){
    PassthroughFilter(input_cloud_1_ptr);
  }
  if (downsampling_active){
    Downsampling(input_cloud_1_ptr);
  }
  if (outlier_active){
    RemoveOutliers(input_cloud_1_ptr);
  }

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*input_cloud_1_ptr, output);

  pub_cloud_1.publish(output);
}

// ----------------------------------------------------------
//                       CALLBACK FOR CLOUD 2
// ----------------------------------------------------------


void WorkspaceMapping::Cloud2Callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
   // Container for original & filtered data
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_2_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

  // Convert to PCL data type
  pcl::fromROSMsg(*cloud_msg, *input_cloud_2_ptr);
  
  // Remove NaN Points from cloud - should speed up filter algorithms
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*input_cloud_2_ptr, *input_cloud_2_ptr, indices); 

  // Perform the actual filtering
  if (passthrough_active){
    PassthroughFilter(input_cloud_2_ptr);
  }
  if (downsampling_active){
    Downsampling(input_cloud_2_ptr);
  }
  if (outlier_active){
    RemoveOutliers(input_cloud_2_ptr);
  }

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*input_cloud_2_ptr, output);

  pub_cloud_2.publish(output);
}


// ----------------------------------------------------------
//                       PASSTHROUGH FILTER
// ----------------------------------------------------------

void WorkspaceMapping::PassthroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  // Configure the filter
  pcl::PassThrough<pcl::PointXYZRGB> passthrough_filter;
  passthrough_filter.setFilterFieldName("z");

  // Set filter from 0.5m to 2.1m
  passthrough_filter.setFilterLimits(0.5, 2.4);

  // Apply passthrough filter
  passthrough_filter.setInputCloud(cloud);
  passthrough_filter.filter(*cloud);
}


// ----------------------------------------------------------
//                       VOXELGRID FILTER
// ----------------------------------------------------------

void WorkspaceMapping::Downsampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  // Downsampling of the point clouds using a voxelgrid filter
  // significantly reduces computing time in the next steps

  // Configure the filter
  pcl::VoxelGrid<pcl::PointXYZRGB> voxelgrid_filter;
  voxelgrid_filter.setLeafSize(0.02, 0.02, 0.02);

  voxelgrid_filter.setInputCloud(cloud);
  voxelgrid_filter.filter(*cloud);
}


// ----------------------------------------------------------
//                       OUTLIER FILTER
// ----------------------------------------------------------

void WorkspaceMapping::RemoveOutliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  // Configure the filter
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> outlier_filter;

  // Apply outlier filter
  outlier_filter.setInputCloud(cloud);
  outlier_filter.setMeanK(50);
  outlier_filter.setStddevMulThresh(1.0);
  outlier_filter.filter(*cloud);
}


// ----------------------------------------------------------
//                       RUN FUNCTION
// ----------------------------------------------------------
int WorkspaceMapping::run(int argc, char **argv)
{
  //auto [should_continue, result] = ProcessArguments(argc, argv); //requires c++17
  //if (!should_continue)
  //{
  //  return result;
  //}
  auto result = ProcessArguments(argc, argv);
  if (!result.first)
  {
    return result.second;
  }
    
  return 0;
}


// ------------------------------------------------------------------------------------------------------------
//                                                  THE MAIN
// ------------------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{

  // Init Preprocessing Class
  WorkspaceMapping workspaceMapping;
  workspaceMapping.run(argc, argv);

  // Init ROS Node
  ros::init(argc, argv, "pointcloud_preprocessing");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  workspaceMapping.sub_cloud_1 = nh.subscribe<sensor_msgs::PointCloud2> ("/cam_1/depth/color/points", 1, &WorkspaceMapping::Cloud1Callback, &workspaceMapping);
  workspaceMapping.pub_cloud_1 = nh.advertise<sensor_msgs::PointCloud2> ("/cam_1/depth/color/points_processed", 1);
  workspaceMapping.sub_cloud_2 = nh.subscribe<sensor_msgs::PointCloud2> ("/cam_2/depth/color/points", 1, &WorkspaceMapping::Cloud2Callback, &workspaceMapping);
  workspaceMapping.pub_cloud_2 = nh.advertise<sensor_msgs::PointCloud2> ("/cam_2/depth/color/points_processed", 1);


  std::cout << "pointcloud_preprocessing" << std::endl;

  ros::spin();


}


