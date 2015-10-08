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

int main(int argc, char **argv)
{

    ros::init(argc, argv, "frustum_test");
    ros::NodeHandle n;

    ros::Publisher pub1 = n.advertise<sensor_msgs::PointCloud2>("point_cloud1", 100);
    ros::Publisher pub2 = n.advertise<sensor_msgs::PointCloud2>("point_cloud2", 100);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    std::string path = ros::package::getPath("component_test");
    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/apple.pcd", *cloud);

    pcl::FrustumCulling<pcl::PointXYZ> fc (true);
    fc.setInputCloud (cloud);
    fc.setVerticalFOV (45);
    fc.setHorizontalFOV (58);
    fc.setNearPlaneDistance (0.8);
    fc.setFarPlaneDistance (5.8);

    Eigen::Matrix4f camera_pose;
    camera_pose.setZero ();
    Eigen::Matrix3f R;
    R = Eigen::AngleAxisf (0 * M_PI / 180, Eigen::Vector3f::UnitX ()) *
            Eigen::AngleAxisf (0 * M_PI / 180, Eigen::Vector3f::UnitY ()) *
            Eigen::AngleAxisf (0 * M_PI / 180, Eigen::Vector3f::UnitZ ());
    camera_pose.block (0, 0, 3, 3) = R;
    Eigen::Vector3f T;
    T (0) = -2; T (1) = 0; T (2) = 0;
    camera_pose.block (0, 3, 3, 1) = T;
    camera_pose (3, 3) = 1;
    fc.setCameraPose (camera_pose);
    pcl::PointCloud <pcl::PointXYZ>::Ptr output (new pcl::PointCloud <pcl::PointXYZ>);
    fc.filter (*output);
    //    pcl::PCDWriter writer;
    //    writer.write<pcl::PointXYZRGB> (path+"/src/pcd/frustum_bun.pcd", *output, false);

    //*****************Camera View Vector (frustum culling tool camera) *****************
    tf::Matrix3x3 rotation;
    Eigen::Matrix3d D;
    D= R.cast<double>();
    tf::matrixEigenToTF(D,rotation);
    rotation = rotation.transpose();
    tf::Quaternion orientation;
    rotation.getRotation(orientation);

    //    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    Eigen::Vector3d axis_vector;
    geometry_msgs::Pose output_vector;
    Eigen::Quaterniond q;
    axis_vector[0] = T[0];
    axis_vector[1] = T[1];
    axis_vector[2] = T[2];
    geometry_msgs::Quaternion quet;
    tf::quaternionTFToEigen(orientation, q);
    tf::quaternionTFToMsg(orientation,quet);
    Eigen::Affine3d pose;
    Eigen::Vector3d a;
    a[0]= T[0];
    a[1]= T[1];
    a[2]= T[2];
    pose.translation() = a;
    tf::poseEigenToMsg(pose, output_vector);
    visualization_msgs::Marker marker;

    //*****************Z buffering test (range_image tool) *****************

    boost::shared_ptr<pcl::RangeImage> cull_ptr(new pcl::RangeImage);
    pcl::RangeImage& visible = *cull_ptr;

    Eigen::Matrix3f R1;
    R1 = Eigen::AngleAxisf (0 * M_PI / 180, Eigen::Vector3f::UnitX ()) *
            Eigen::AngleAxisf (90 * M_PI / 180, Eigen::Vector3f::UnitY ()) *
            Eigen::AngleAxisf (0 * M_PI / 180, Eigen::Vector3f::UnitZ ());
    Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(T[0],T[1],T[2]));
            sensorPose.rotate (R1);
    float angularResolutionX = (float)(50.0f / 640.0f * (M_PI / 180.0f));
    float angularResolutionY = (float)(40.0f / 480.0f * (M_PI / 180.0f));
    float maxAngleX = (float)(50.0f * (M_PI / 180.0f));
    float maxAngleY = (float)(40.0f * (M_PI / 180.0f));
    float noise_level=0.0;
    float min_range=0.0;
    float borderSize=1.0;
    visible.createFromPointCloud(*output, angularResolutionX, angularResolutionY,
                                    maxAngleX, maxAngleY, sensorPose, pcl::RangeImage::CAMERA_FRAME,
                                    noise_level, min_range, borderSize);
//    uint32_t width  = static_cast<uint32_t> (pcl_lrint (floor (maxAngleX*(1/angularResolutionX))));
//    uint32_t height = static_cast<uint32_t> (pcl_lrint (floor (maxAngleY*(1/angularResolutionY))));
//    int top=height, right=-1, bottom=-1, left=width;
//    visible.doZBuffer(*output, noise_level, min_range, top, right, bottom, left );
//    visible.recalculate3DPointPositions();
    //*****************PCL Visualer *****************
        pcl::visualization::PCLVisualizer visualizer;
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_source (output, 255, 0, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange>green_target (cull_ptr, 0, 255, 0);
        visualizer.addPointCloud<pcl::PointXYZ> (output, red_source,"cloud");
        visualizer.addPointCloud<pcl::PointWithRange> (cull_ptr, green_target,"output");

        while (!visualizer.wasStopped ())
        {
            visualizer.spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
    //*****************Cloud Visualer *****************
    //  pcl::visualization::CloudViewer viewer ("viewer");
    //        viewer.showCloud (cloud);
    //        viewer.showCloud (output);
    //        while (!viewer.wasStopped ())
    //        {
    //          boost::this_thread::sleep (boost::posix_time::microseconds (100));
    //        }

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

        //***mesh and intersection publish***
        sensor_msgs::PointCloud2 cloud1;
        sensor_msgs::PointCloud2 cloud2;
        pcl::toROSMsg(*output, cloud1);
        pcl::toROSMsg(*cull_ptr, cloud2);
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
