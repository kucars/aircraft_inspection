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
#include <geometry_msgs/PoseArray.h>
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <deque>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <cmath>
//PCL
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image.h>
#include <voxel_grid_occlusion_estimation.h>
#include "fcl_utility.h"
#include <pcl/filters/voxel_grid.h>


int main(int argc, char **argv)
{

    ros::init(argc, argv, "coverge_quantification");
    ros::NodeHandle n;
    ros::Publisher pub1 = n.advertise<sensor_msgs::PointCloud2>("original_point_cloud", 100);
    ros::Publisher pub2 = n.advertise<sensor_msgs::PointCloud2>("occlusion_free_cloud", 100);

    pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloudGrid(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr occlusionFreeCloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr occlusionFreeGrid(new pcl::PointCloud<pcl::PointXYZ>);



    std::string path = ros::package::getPath("component_test");
    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/scaled_desktop.pcd", *originalCloud);
    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/occlusionFreeCloud_1m.pcd", *occlusionFreeCloud);

    // *******************original cloud Grid***************************
    //used VoxelGridOcclusionEstimationT since the voxelGrid does not include getcentroid function
        pcl::VoxelGridOcclusionEstimationT voxelFilterOriginal;
        voxelFilterOriginal.setInputCloud (originalCloud);
        float res = 0.5;
        voxelFilterOriginal.setLeafSize (res, res, res);
        voxelFilterOriginal.initializeVoxelGrid();
//        voxelFilterOriginal.filter(*originalCloudGrid);

     //*******************Occupied Cloud Grid***************************
        pcl::VoxelGridOcclusionEstimationT voxelFilterOccupied;
        voxelFilterOccupied.setInputCloud (occlusionFreeCloud);
        voxelFilterOccupied.setLeafSize (res, res, res);
        voxelFilterOccupied.initializeVoxelGrid();
//        voxelFilterFree.filter(*occlusionFreeGrid);



     //*****************************************************************

        Eigen::Vector3i  min_b = voxelFilterOccupied.getMinBoxCoordinates ();
        Eigen::Vector3i  max_b = voxelFilterOccupied.getMaxBoxCoordinates ();
        Eigen::Vector3i  min_b1 = voxelFilterOriginal.getMinBoxCoordinates ();
        Eigen::Vector3i  max_b1 = voxelFilterOriginal.getMaxBoxCoordinates ();

        float OriginalVoxelsSize=0, MatchedVoxels=0;

        // iterate over the entire original voxel grid to get the size of the grid
        ros::Time coverage_begin = ros::Time::now();
        for (int kk = min_b1.z (); kk <= max_b1.z (); ++kk)
        {
            for (int jj = min_b1.y (); jj <= max_b1.y (); ++jj)
            {
                for (int ii = min_b1.x (); ii <= max_b1.x (); ++ii)
                {
                    Eigen::Vector3i ijk1 (ii, jj, kk);
                    int index1 = voxelFilterOriginal.getCentroidIndexAt (ijk1);
                    if(index1!=-1)
                    {
                        OriginalVoxelsSize++;
                    }

                }
            }
        }

        //iterate through the entire coverage grid to check the number of matched voxel between the original and the covered ones
        for (int kk = min_b.z (); kk <= max_b.z (); ++kk)
        {
            for (int jj = min_b.y (); jj <= max_b.y (); ++jj)
            {
                for (int ii = min_b.x (); ii <= max_b.x (); ++ii)
                {

                    Eigen::Vector3i ijk (ii, jj, kk);
                    Eigen::Vector4f centroid = voxelFilterOccupied.getCentroidCoordinate (ijk);
                    Eigen::Vector3i ijk_in_Original= voxelFilterOriginal.getGridCoordinates(centroid[0],centroid[1],centroid[2]) ;

                    int index = voxelFilterOriginal.getCentroidIndexAt (ijk_in_Original);
//                    int index = voxelFilterFree.getCentroidIndexAt (ijk);

                    if(index!=-1)
                    {
                        MatchedVoxels++;
                    }

                }
            }
        }

        ros::Time coverage_end = ros::Time::now();
        double elapsed =  coverage_end.toSec() - coverage_begin.toSec();
        std::cout<<"discretization duration (s) = "<<elapsed<<"\n";

        //calculating the coverage percentage
        float coverage_ratio= MatchedVoxels/OriginalVoxelsSize;
        float coverage_percentage= coverage_ratio*100;

        std::cout<<" the coverage ratio is = "<<coverage_ratio<<"\n";
        std::cout<<" the number of covered voxels = "<<MatchedVoxels<<" voxel is covered"<<"\n";
        std::cout<<" the number of original voxels = "<<OriginalVoxelsSize<<" voxel"<<"\n\n\n";
        std::cout<<" the coverage percentage is = "<<coverage_percentage<<" %"<<"\n";


        // *****************Rviz Visualization ************

        ros::Rate loop_rate(10);
        while (ros::ok())
        {


            //        //***original cloud & frustum cull & occlusion cull publish***
            sensor_msgs::PointCloud2 cloud1;
            sensor_msgs::PointCloud2 cloud2;


            pcl::toROSMsg(*originalCloud, cloud1); //cloud of original (white) using original cloud
            pcl::toROSMsg(*occlusionFreeCloud, cloud2); //cloud of the not occluded voxels (blue) using occlusion culling

            cloud1.header.frame_id = "base_point_cloud";
            cloud2.header.frame_id = "base_point_cloud";

            cloud1.header.stamp = ros::Time::now();
            cloud2.header.stamp = ros::Time::now();

            pub1.publish(cloud1);
            pub2.publish(cloud2);

            ros::spinOnce();
            loop_rate.sleep();
        }

    return 0;
}

