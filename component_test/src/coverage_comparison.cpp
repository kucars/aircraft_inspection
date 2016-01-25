#include "ros/ros.h"
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
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
#include "fcl_utility.h"
#include <pcl/filters/voxel_grid.h>
#include <component_test/occlusion_culling.h>
pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>);

void callback(const sensor_msgs::PointCloud2::ConstPtr& recent_cloud)
{

    std::cout<<"I'm in callback"<<std::endl;
    OcclusionCulling obj("etihad.pcd");
    pcl::PointCloud<pcl::PointXYZ> temp_cloud;
    pcl::fromROSMsg(*recent_cloud,temp_cloud);
    tempCloud->points = temp_cloud.points;
    float cov;
    cov = obj.calcCoveragePercent(tempCloud);
    std::cout<<"******* coverage percentage: "<<cov<<" % *******" <<std::endl;

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "coverge_quantification");
    ros::NodeHandle n;
    ros::Publisher pub1 = n.advertise<sensor_msgs::PointCloud2>("originalPointCloud", 100);
    ros::Publisher pub2 = n.advertise<sensor_msgs::PointCloud2>("CoverageCloud", 100);
    std::string topic = n.resolveName("/iris/cloud_map");

    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(topic, 1, callback);

    pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud(new pcl::PointCloud<pcl::PointXYZ>);

    std::string path = ros::package::getPath("component_test");
    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/etihad.pcd", *originalCloud);


    // *****************Rviz Visualization ************

    ros::Rate loop_rate(10);
    while (ros::ok())
    {


        //***original cloud & frustum cull & occlusion cull publish***
        sensor_msgs::PointCloud2 cloud1;
        sensor_msgs::PointCloud2 cloud2;


        pcl::toROSMsg(*originalCloud, cloud1); //cloud of original (white) using original cloud
        pcl::toROSMsg(*tempCloud, cloud2); //cloud of the not occluded voxels (blue) using occlusion culling

        cloud1.header.frame_id = "map";
        cloud2.header.frame_id = "map";

        cloud1.header.stamp = ros::Time::now();
        cloud2.header.stamp = ros::Time::now();

        pub1.publish(cloud1);
        pub2.publish(cloud2);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

