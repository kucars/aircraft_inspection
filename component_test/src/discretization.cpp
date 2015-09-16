#include "ros/ros.h"
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
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

int main(int argc, char **argv)
{

    ros::init(argc, argv, "discretization");
    ros::NodeHandle n;
    ros::Publisher model_pub = n.advertise<sensor_msgs::PointCloud2>("point_cloud", 1);
    ros::Publisher point_pub = n.advertise<geometry_msgs::PoseArray>("voxel", 1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    std::string path = ros::package::getPath("component_test");
    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/scaled_desktop.pcd", *cloud);
    //    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/plane_desktop.pcd", *cloud);
    //      pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/bun000_Structured.pcd", *cloud);

    geometry_msgs::PoseArray points;
    geometry_msgs::Pose pose;

    //discretization
    int x_space=16;//put half the length here
    int y_space=11;//put half the length here
    int z_space=37;
    double res=1;
    for (int z=(-1*z_space) ; z < 2; z+=res)//the length of the aircraft
    {

        for (int y=-1*(y_space-4) ; y< y_space; y+=res)//the hight of the aircraft
        {

            for (int x=-1*x_space ; x< x_space; x+=res)//the width of the aircraft
            {
                pose.position.z=z;
                pose.position.y=y;
                pose.position.x=x;
                pose.orientation.x=0;pose.orientation.y=0;pose.orientation.z=0;pose.orientation.w=1;
                points.poses.push_back(pose);

            }
        }
    }

    visualization_msgs::Marker marker;

    while(ros::ok())
    {
        //mesh points publish
        sensor_msgs::PointCloud2 cloud1;
        pcl::toROSMsg(*cloud, cloud1);
        cloud1.header.frame_id = "base_point_cloud";
        cloud1.header.stamp = ros::Time::now();
        model_pub.publish(cloud1);


        //visualize the points
        //        marker.type = visualization_msgs::Marker::POINTS;
        //        marker.action = visualization_msgs::Marker::ADD;
        //        //visulaization using the markers
        //        marker.scale.x = 0.1;
        //        marker.scale.y = 0.1;
        ////        marker.scale.z = 1;
        //        // Set the color -- be sure to set alpha to something non-zero!
        //        marker.color.r = 0.0f;
        //        marker.color.g = 1.0f;
        //        marker.color.b = 0.0f;
        //        marker.color.a = 1.0;
        //        marker.ns = "basic_shapes";
        //        marker.id = 1;
        //        ROS_INFO("Publishing Marker");
        //        // Set the frame ID and timestamp. See the TF tutorials for information on these.
        ////        if(i<points.poses.size())
        ////        marker.pose =  points.poses[i];
        ////        marker.pose.orientation  = quet;//output_vector.orientation;
        //        marker.header.frame_id = "base_point_cloud";
        //        marker.header.stamp = ros::Time::now();
        //        points.poses.resize(100);
        //        marker.lifetime = ros::Duration(10);
        //        for(int i=0; i<points.poses.size(); i++)
        //        {
        //            geometry_msgs::Point p;
        //            p.x = points.poses[i].position.x;
        //            p.y = points.poses[i].position.y;
        //            p.z = points.poses[i].position.z;

        //            marker.points.push_back(p);
        //        }
        //        // Publish the marker
        //        marker_pub.publish(marker);


        points.header.frame_id= "base_point_cloud";
        points.header.stamp = ros::Time::now();
        point_pub.publish(points);

        ros::spinOnce();
    }


    return 0;
}
