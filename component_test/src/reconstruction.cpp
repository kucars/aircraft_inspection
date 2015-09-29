#include "ros/ros.h"
#include <ros/package.h>
#include <iostream>
#include <sstream>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <fstream>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/Marker.h>
//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/file_io.h>
#include <pcl/io/boost.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/io/vtk_lib_io.h>

#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <deque>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>

#include <stdlib.h>
#include <boost/foreach.hpp>
#include <Eigen/Eigen>

#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"


int main(int argc, char **argv)
{

    ros::init(argc, argv, "reconstruction");
    ros::NodeHandle n;

    pcl::visualization::PCLVisualizer visualizer;
    pcl::visualization::CloudViewer viewer ("viewer");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    std::string path = ros::package::getPath("component_test");
    
    //******************PCL Visualizer*******************
        for(int i=1; i<3;i=i++) //visualizing 3 scans
        {
            std::ostringstream ss;
            ss << i;
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_source (cloud, 255, 0, 0);
            std::cout << ss.str();
            pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/scan_blendoyne/scans"+ss.str()+".pcd", *cloud);
            visualizer.addPointCloud<pcl::PointXYZ> (cloud, red_source,"cloud"+i);

        }

        while (!visualizer.wasStopped ())
        {
            visualizer.spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }

    //*****************Cloud manager********************
//    for(int i=0; i<=1000;i=i+6)
//    {
//        std::ostringstream ss;
//        ss << i;
//        std::cout << ss.str();
//        pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/scan_blendoyne/scans"+ss.str()+"00000.pcd", *cloud);
//        viewer.showCloud (cloud);
//    }

//    while (!viewer.wasStopped ())
//    {
//        boost::this_thread::sleep (boost::posix_time::microseconds (100));
//    }

//    while(ros::ok())
//    {
//        // points publish
//        sensor_msgs::PointCloud2 cloud1;
//        pcl::toROSMsg(*cloud, cloud1);
//        cloud1.header.frame_id = "base_point_cloud";
//        cloud1.header.stamp = ros::Time::now();
//        model_pub.publish(cloud1);

//        ros::spinOnce();
//    }


    return 0;
}

