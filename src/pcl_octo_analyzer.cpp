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
*  This program subscibes to the planning_scene topic from MoveIt! and extracts 1 frame of 
*  the included octomap into a file.
*/



#include    <assert.h>
#include    <iostream>
#include	<ros/ros.h>
#include    <ros/topic.h>
#include    <ros/console.h>
#include    <pcl/io/pcd_io.h>
#include	<pcl/point_cloud.h>	
#include    <pcl/point_types.h>
#include    <octomap/octomap.h>
#include    <octomap/octomap_types.h>
#include    <octomap/ColorOcTree.h>
#include    <pcl/common/transforms.h>
#include    <pcl/filters/crop_box.h>
#include	<sensor_msgs/PointCloud2.h>	
#include	<pcl_conversions/pcl_conversions.h>	


// MAIN
main(int	argc,	char**	argv) {	

    ros::init(argc,	argv,	"pcl_framesaver");	
    std::cout << "PCL Framesaver - Save PointCloud2 topics into pcd and bt files" << std::endl;

    // Variables
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_2_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);


    // Save 1 Pointcloud Message into Message object
    sensor_msgs::PointCloud2ConstPtr cloud_1_msg =   ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/cam_1/depth/color/points", ros::Duration(10));
    sensor_msgs::PointCloud2ConstPtr cloud_2_msg =   ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/cam_2/depth/color/points", ros::Duration(10));


    // Convert from ROS Topic to point cloud object
    pcl::fromROSMsg(*cloud_1_msg, *cloud_1_ptr);
    pcl::fromROSMsg(*cloud_2_msg, *cloud_2_ptr);


    // Rotate the Point Cloud
    Eigen::Matrix4f transform_x = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f transform_z = Eigen::Matrix4f::Identity();

    // 45 Degrees
    int angle = -30;

    // Rotate fist pointcloud by 45° in X axis
    transform_x(0, 0) = 1;
    transform_x(1, 1) = cos(angle * M_PI / 180);
    transform_x(1, 2) = -sin(angle * M_PI / 180);
    transform_x(2, 1) = sin(angle * M_PI / 180);
    transform_x(2, 2) = cos(angle * M_PI / 180);
    transform_x(3, 3) = 1;

    // 180 Degrees
    angle = 180;

    // Rotate fist pointcloud by 180 in Z axis
    transform_z(0, 0) = cos(angle * M_PI / 180);
    transform_z(0, 1) = -sin(angle * M_PI / 180);
    transform_z(1, 0) = sin(angle * M_PI / 180);
    transform_z(1, 1) = cos(angle * M_PI / 180);
    transform_z(2, 2) = 1;
    transform_z(3, 3) = 1;
    

    // Apply the Transformation
    pcl::transformPointCloud (*cloud_1_ptr, *transformed_cloud, transform_x);
    pcl::transformPointCloud (*transformed_cloud, *transformed_cloud, transform_z);

    // Passthrough / Cropbox filtering
    double minX, minY, minZ, maxX, maxY, maxZ;
    minX = 0.1;
    minY = -1.8;
    minZ = 1.5;
    maxX = 0.9;
    maxY = -0.7;
    maxZ = 1.63;

    // Values for the measure field on the desk (ground truth vs octomap)
    // minX = -0.0;
    // minY = -1.5;
    // minZ = 0.5;
    // maxX = 0.5;
    // maxY = -0.8;
    // maxZ = 1.51;

    // Values for noise measure of the desk
    // minX = 0.1;
    // minY = -1.8;
    // minZ = 1.5;
    // maxX = 0.9;
    // maxY = -0.7;
    // maxZ = 1.63;


    // Configure the Passthrough Filter - X - Axis
    pcl::CropBox<pcl::PointXYZRGB> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    boxFilter.setInputCloud(transformed_cloud);
    boxFilter.filter(*transformed_cloud);

    // Store all Z-Values of the points into a vector
    float min_noise_z, max_noise_z;
    std::vector<float> z_vals;
    for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = transformed_cloud->begin(); it != transformed_cloud->end(); it++){
        z_vals.push_back(it->z);
    }

    // The octree
    double resolution = 0.05;
    octomap::OcTree tree( resolution );

    // Convert PointCloud to Octomap
    for (auto p:transformed_cloud->points){
        tree.updateNode( octomap::point3d(p.x, p.y, p.z), true );
    }

    tree.updateInnerOccupancy();
    uint occupied_nodes = 0;
    double size;
    double volume;


        // Points for bounding box, minimum and maximum
    octomap::point3d min, max;

    
    // define bounding box 
    double min_x, min_y, min_z;
    double max_x, max_y, max_z;

    tree.getMetricMax(max_x, max_y, max_z);
    tree.getMetricMin(min_x, min_y, min_z);

    min.x() = min_x;
    min.y() = min_y;
    min.z() = min_z;
    max.x() = max_x;
    max.y() = max_y;
    max.z() = max_z;

    // Volumes
    double vol_occ = 0;
    double vol_free = 0;
    double vol_tot = (max_x-min_x)*(max_y-min_y)*(max_z-min_z);

    // Iterate trough octree leaf nodes
    for(octomap::OcTree::leaf_bbx_iterator it = tree.begin_leafs_bbx(min,max), end=tree.end_leafs_bbx(); it!= end; ++it){
        double side_length = it.getSize();
        if (it->getOccupancy() >= 0.7){ // occupied leaf node
            vol_occ += pow(side_length,3);
        }
        else {                          // free leaf node 
            vol_free += pow(side_length,3);
        }
    }
    
    // Calculate the volume of the unknown volume (inside the box)
    double vol_unkn = vol_tot - vol_occ - vol_free;

    // Save the PointClouds and OctoMaps into a file
    pcl::io::savePCDFileASCII("./cloud.pcd", *transformed_cloud);
    tree.writeBinary("./cloud.bt");
    /*
    // Output    
    std::cout << "z_values: "                                       << std::endl;
    for(auto z_val : z_vals){
        std::cout << z_val << "," << std::endl;
    }
    */

    std::cout << "vol_occ: "        << vol_occ              << "m³" << std::endl;
    std::cout << "vol_free: "       << vol_free             << "m³" << std::endl;
    std::cout << "vol_unknown: "    << vol_unkn             << "m³" << std::endl;
    std::cout << "vol_tot: "        << vol_tot              << "m³" << std::endl;
    std::cout << "vol_box: "        << (vol_occ + vol_unkn) << "m³" << std::endl;


    return	0;	
}	