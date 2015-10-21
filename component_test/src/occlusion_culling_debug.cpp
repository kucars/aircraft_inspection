#include "ros/ros.h"
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Pose.h>
#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/Marker.h>
//PCL
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image.h>
#include <voxel_grid_occlusion_estimation.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "occlusion_culling_test");
    ros::NodeHandle n;

    ros::Publisher pub1 = n.advertise<sensor_msgs::PointCloud2>("point_cloud", 100);
    ros::Publisher pub2 = n.advertise<sensor_msgs::PointCloud2>("occlusion_free_cloud", 100);
    ros::Publisher pub3 = n.advertise<sensor_msgs::PointCloud2>("ray_points", 100);
    
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr occlusionFreeCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rayCloud(new pcl::PointCloud<pcl::PointXYZ>);

    std::string path = ros::package::getPath("component_test");
    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/sphere_densed.pcd", *cloud);

    Eigen::Vector3d a(4,0,0);
    Eigen::Affine3d pose;
    pose.translation() = a;
    geometry_msgs::Pose output_vector;
    tf::poseEigenToMsg(pose, output_vector);
    
    tf::Quaternion orientation = tf::createQuaternionFromRPY(0,0,0);;
    Eigen::Quaterniond q;
    geometry_msgs::Quaternion quet;
    tf::quaternionTFToEigen(orientation, q);
    tf::quaternionTFToMsg(orientation,quet);

    pose.translation() = a;
    visualization_msgs::Marker marker;

    //*****************voxel grid occlusion estimation *****************
    Eigen::Quaternionf quat(q.w(),q.x(),q.y(),q.z());
    cloud->sensor_origin_  = Eigen::Vector4f(a[0],a[1],a[2],0);
    cloud->sensor_orientation_= quat;
    pcl::VoxelGridOcclusionEstimationT voxelFilter;
    voxelFilter.setInputCloud (cloud);
    voxelFilter.setLeafSize (0.03279f, 0.03279f, 0.03279f);
    //voxelFilter.filter(*occlusion_cloud);
    voxelFilter.initializeVoxelGrid(); 
    
    int state,ret;
    Eigen::Vector3i t;
    pcl::PointXYZ pt;
    std::vector< Eigen::Vector3i > & out_ray;
    for ( int i = 0; i < (int)cloud->points.size(); i ++ )
    {
        pt = cloud->points[i];
        t = voxelFilter.getGridCoordinates( pt.x, pt.y, pt.z);        
        ret = voxelFilter.occlusionEstimation( state,out_ray, t);
        if ( state != 1 )
        {
          occlusionFreeCloud->points.push_back(pt);
          for(uint j=0; j< out_ray.size(); j++)
          {
              Eigen::Vector4f centroid = voxelFilter.getCentroidCoordinate (out_ray[j]);
              rayCloud->points.push_back(pcl::PointXYZ(centroid[0], centroid[1], centroid[2]));
          }
        }
    }

    //*****************Rviz Visualization ************
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        //***marker publishing***
        uint32_t shape = visualization_msgs::Marker::ARROW;
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;
        //visulaization using the markers
        marker.scale.x = 0.5;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.ns = "basic_shapes";
        marker.id = 2;
        ROS_INFO("Publishing Marker");
        // Set the frame ID and timestamp. See the TF tutorials for information on these.
        marker.pose =  output_vector;
        marker.pose.orientation  = quet;//output_vector.orientation;
        marker.header.frame_id = "base_point_cloud";
        marker.header.stamp = ros::Time::now();
        marker.lifetime = ros::Duration(10);
        // Publish the marker
        marker_pub.publish(marker);

        //***frustum cull and occlusion cull publish***
        sensor_msgs::PointCloud2 cloud1;
        sensor_msgs::PointCloud2 cloud2;
        sensor_msgs::PointCloud2 cloud3;
        pcl::toROSMsg(*cloud, cloud1);
        pcl::toROSMsg(*occlusionFreeCloud, cloud2);
        pcl::toROSMsg(*occlusionFreeCloud, cloud2);
        
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
