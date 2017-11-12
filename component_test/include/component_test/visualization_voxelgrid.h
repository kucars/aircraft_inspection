#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_

#include "ros/ros.h"
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//PCL
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <voxel_grid.h>
#include <CGAL/intersections.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Segment_3.h>

typedef CGAL::Simple_cartesian<double> K;
typedef K::FT FT;
typedef K::Ray_3 Ray;
typedef K::Line_3 Line1;
typedef K::Point_3 Point;
typedef K::Triangle_3 Triangle;
typedef K::Segment_3 Segment;
typedef K::Plane_3 Plane;

class VoxelGridVisualization
{
public:
    //attributes
    ros::NodeHandle  nh;
    std::string model;
    ros::Publisher originalCloudPub;
    ros::Publisher voxelPointsPub;
//    ros::Publisher voxelPub;
    ros::Publisher intersectionPointPub;
    ros::Publisher gridBBPub;
    ros::Publisher linesPub;

    std::string frameid;
    pcl::VoxelGridT voxelFilter;
    std::vector<Eigen::Vector3i> bb_points;//the bounding box points (ijk)
    std::vector<Point> bb_centroids;//the bounding box points (coordinate)
    std::vector<Point> i_points;//the bounding box points (coordinate)
    std::vector<Point> fov_points;
    Point sensor_o;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxelPointsPtr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxelPtr;

    //methods
    VoxelGridVisualization(ros::NodeHandle &n, std::string modelName, pcl::VoxelGridT &grid, std::string frame_id);
    VoxelGridVisualization(pcl::VoxelGridT &grid);
    ~VoxelGridVisualization();
    visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links, int c_color[], double scale, std::string frame_id);
    visualization_msgs::Marker drawPoints(std::vector<geometry_msgs::Point> points, int c_color[], double scale, std::string frame_id);
    void getBBCentroids();
    void intersectionFOVBB(Point origin, std::vector<Point> fov_pts);
    void visualizeBB();
    void visualizeFOVBBIntersection(Point origin, std::vector<Point> fov_pts);
    void selectedVoxelPoints(Eigen::Vector3i ijk, pcl::PointCloud<pcl::PointXYZ>::Ptr &voxelPoints);



};

#endif
