#ifndef OCCLUSION_H_
#define OCCLUSION_H_

#include "ros/ros.h"
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Pose.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
//PCL
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <frustum_culling.h>
// #include <pcl/visualization/cloud_viewer.h>
// #include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image.h>
#include <voxel_grid_occlusion_estimation.h>

class OcclusionCulling
{
  public:
    //attributes
    ros::NodeHandle  nh;
    std::string model;
//     ros::Publisher original_pub;
//     ros::Publisher visible_pub;
//     ros::Publisher lines_pub1;
//     ros::Publisher lines_pub2;
//     ros::Publisher lines_pub3;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr occlusionFreeCloud;//I can add it to accumulate cloud if I want to extract visible surface from multiple locations
    pcl::PointCloud<pcl::PointXYZ> FreeCloud;
    float voxelRes, OriginalVoxelsSize;
    pcl::VoxelGridOcclusionEstimationT voxelFilterOriginal;
    Eigen::Vector3i  max_b1, min_b1;
//     visualization_msgs::Marker linesList1,linesList2,linesList3;
//     pcl::FrustumCullingTT fc;

    //methods
    OcclusionCulling(ros::NodeHandle & n, std::string modelName);
    OcclusionCulling(std::string modelName);
    OcclusionCulling();
    ~OcclusionCulling();
    pcl::PointCloud<pcl::PointXYZ> extractVisibleSurface(geometry_msgs::Pose location);
//    float calcCoveragePercent(geometry_msgs::Pose location);
    float calcCoveragePercent(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);
//     void visualizeFOV(pcl::FrustumCullingTT& fc);
    visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links, int id, int c_color);

  
};

#endif 
