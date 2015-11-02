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
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image.h>
#include <frustum_culling.h>
visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links, int id, int c_color);

int main(int argc, char **argv)
{

    ros::init(argc, argv, "frustum_test");
    ros::NodeHandle n;

    ros::Publisher pub1 = n.advertise<sensor_msgs::PointCloud2>("original_point_cloud", 100);
    ros::Publisher pub2 = n.advertise<sensor_msgs::PointCloud2>("frustum_point_cloud", 100);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("sensor_origin", 1);
    ros::Publisher lines_pub = n.advertise<visualization_msgs::Marker>("field_of_view", 10);

    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    std::string path = ros::package::getPath("component_test");
    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/sphere_densed.pcd", *cloud);

    pcl::FrustumCullingTT fc (true);
    fc.setInputCloud (cloud);
    fc.setVerticalFOV (45);
    fc.setHorizontalFOV (58);
    fc.setNearPlaneDistance (0.8);
    fc.setFarPlaneDistance (5.8);

    Eigen::Matrix4f camera_pose;
    camera_pose.setZero ();
    Eigen::Matrix3f R;
    Eigen::Vector3f theta(0.0,45.0,0.0);
    R = Eigen::AngleAxisf (theta[0] * M_PI / 180, Eigen::Vector3f::UnitX ()) *
            Eigen::AngleAxisf (theta[1] * M_PI / 180, Eigen::Vector3f::UnitY ()) *
            Eigen::AngleAxisf (theta[2] * M_PI / 180, Eigen::Vector3f::UnitZ ());
    camera_pose.block (0, 0, 3, 3) = R;
    Eigen::Vector3f T;
    T (0) = -4; T (1) = 0; T (2) = 0;
    camera_pose.block (0, 3, 3, 1) = T;
    camera_pose (3, 3) = 1;
    fc.setCameraPose (camera_pose);
    pcl::PointCloud <pcl::PointXYZ>::Ptr output (new pcl::PointCloud <pcl::PointXYZ>);
    fc.filter (*output);

    std::vector<geometry_msgs::Point> fov_points;
    geometry_msgs::Point point;
    point.x=fc.fp_bl[0];point.y=fc.fp_bl[1];point.z=fc.fp_bl[2]; fov_points.push_back(point);
    point.x=fc.fp_br[0];point.y=fc.fp_br[1];point.z=fc.fp_br[2]; fov_points.push_back(point);
    point.x=fc.fp_tr[0];point.y=fc.fp_tr[1];point.z=fc.fp_tr[2]; fov_points.push_back(point);
    point.x=fc.fp_tl[0];point.y=fc.fp_tl[1];point.z=fc.fp_tl[2]; fov_points.push_back(point);
    point.x=fc.np_bl[0];point.y=fc.np_bl[1];point.z=fc.np_bl[2]; fov_points.push_back(point);
    point.x=fc.np_br[0];point.y=fc.np_br[1];point.z=fc.np_br[2]; fov_points.push_back(point);
    point.x=fc.np_tr[0];point.y=fc.np_tr[1];point.z=fc.np_tr[2]; fov_points.push_back(point);
    point.x=fc.np_tl[0];point.y=fc.np_tl[1];point.z=fc.np_tl[2]; fov_points.push_back(point);

    std::vector<geometry_msgs::Point> fov_lines;
    fov_lines.push_back(fov_points[0]);fov_lines.push_back(fov_points[1]);
    fov_lines.push_back(fov_points[1]);fov_lines.push_back(fov_points[2]);
    fov_lines.push_back(fov_points[2]);fov_lines.push_back(fov_points[3]);
    fov_lines.push_back(fov_points[3]);fov_lines.push_back(fov_points[0]);

    fov_lines.push_back(fov_points[4]);fov_lines.push_back(fov_points[5]);
    fov_lines.push_back(fov_points[5]);fov_lines.push_back(fov_points[6]);
    fov_lines.push_back(fov_points[6]);fov_lines.push_back(fov_points[7]);
    fov_lines.push_back(fov_points[7]);fov_lines.push_back(fov_points[4]);

    fov_lines.push_back(fov_points[7]);fov_lines.push_back(fov_points[3]);
    fov_lines.push_back(fov_points[6]);fov_lines.push_back(fov_points[2]);
    fov_lines.push_back(fov_points[5]);fov_lines.push_back(fov_points[1]);
    fov_lines.push_back(fov_points[4]);fov_lines.push_back(fov_points[0]);
    visualization_msgs::Marker linesList = drawLines(fov_lines,3333,1);


    //    pcl::PCDWriter writer;
    //    writer.write<pcl::PointXYZRGB> (path+"/src/pcd/frustum_bun.pcd", *output, false);

    //*****************Visualization Camera View Vector (frustum culling tool camera) *****************
    // the rviz axis is different from the frustum camera axis and range image axis
//        R = Eigen::AngleAxisf (theta[0] * M_PI / 180, Eigen::Vector3f::UnitX ()) *
//                Eigen::AngleAxisf (-theta[1] * M_PI / 180, Eigen::Vector3f::UnitY ()) *
//                Eigen::AngleAxisf (-theta[2] * M_PI / 180, Eigen::Vector3f::UnitZ ());
    tf::Matrix3x3 rotation;
    Eigen::Matrix3d D;
    D= R.cast<double>();
    tf::matrixEigenToTF(D,rotation);
    rotation = rotation.transpose();
    tf::Quaternion orientation;
    rotation.getRotation(orientation);

    //    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    geometry_msgs::Pose output_vector;
    Eigen::Quaterniond q;
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
        pcl::toROSMsg(*cloud, cloud1); //cloud of frustum cull (white) using pcl::frustumcull
        pcl::toROSMsg(*output, cloud2); //cloud of the Occlusion cull (blue) using pcl::rangeImage
        cloud1.header.frame_id = "base_point_cloud";
        cloud2.header.frame_id = "base_point_cloud";

        cloud1.header.stamp = ros::Time::now();
        cloud2.header.stamp = ros::Time::now();
        pub1.publish(cloud1);
        pub2.publish(cloud2);

        lines_pub.publish(linesList);


        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links, int id, int c_color)
{
    visualization_msgs::Marker linksMarkerMsg;
    linksMarkerMsg.header.frame_id="/base_point_cloud";
    linksMarkerMsg.header.stamp=ros::Time::now();
    linksMarkerMsg.ns="link_marker";
    linksMarkerMsg.id = id;
    linksMarkerMsg.type = visualization_msgs::Marker::LINE_LIST;
    linksMarkerMsg.scale.x = 0.006;
    linksMarkerMsg.action  = visualization_msgs::Marker::ADD;
    linksMarkerMsg.lifetime  = ros::Duration(1000);
    std_msgs::ColorRGBA color;
//    color.r = 1.0f; color.g=.0f; color.b=.0f, color.a=1.0f;
    if(c_color == 1)
    {
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
        color.a = 1.0;
    }
    else if(c_color == 2)
    {
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
        color.a = 1.0;
    }
    else
    {
        color.r = 0.0;
        color.g = 0.0;
        color.b = 1.0;
        color.a = 1.0;
    }
    std::vector<geometry_msgs::Point>::iterator linksIterator;
    for(linksIterator = links.begin();linksIterator != links.end();linksIterator++)
    {
        linksMarkerMsg.points.push_back(*linksIterator);
        linksMarkerMsg.colors.push_back(color);
    }
   return linksMarkerMsg;
}
