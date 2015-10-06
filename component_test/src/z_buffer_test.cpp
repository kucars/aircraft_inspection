#include <iostream>
#include "ros/ros.h"
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <boost/thread/thread.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>

#include <pcl/io/pcd_io.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

void setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
    Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
    Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f(0, 0, 1) + pos_vector;
    Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f(0, -1, 0);
    viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
            look_at_vector[0], look_at_vector[1], look_at_vector[2],
            up_vector[0], up_vector[1], up_vector[2]);
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "culling");
    ros::NodeHandle n;

    //    ros::Publisher pub1 = n.advertise<sensor_msgs::PointCloud2>("point_cloud1", 100);
    //    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Object for storing the point cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::string path = ros::package::getPath("component_test");
    //    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/scaled_desktop.pcd", *cloud);
    // Read a PCD file from disk.
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path+"/src/pcd/scaled_desktop.pcd", *cloud) != 0)
    {
        return -1;
    }

    // Parameters needed by the planar range image object:

    // Image size. Both Kinect and Xtion work at 640x480.
    int imageSizeX = 640;
    int imageSizeY = 480;
    // Center of projection. here, we choose the middle of the image.
    float centerX = 640.0f / 2.0f;
    float centerY = 480.0f / 2.0f;
    // Focal length. The value seen here has been taken from the original depth images.
    // It is safe to use the same value vertically and horizontally.
    float focalLengthX = 525.0f, focalLengthY = focalLengthX;
    // Sensor pose. Thankfully, the cloud includes the data.
    // 	Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(cloud->sensor_origin_[0],
    // 								 cloud->sensor_origin_[1],
    // 								 cloud->sensor_origin_[2])) *
    // 								 Eigen::Affine3f(cloud->sensor_orientation_);

    Eigen::Matrix3f R;
    Eigen::Vector3f T;
    T (0) = 0; T (1) = 0; T (2) = 0;
    R = Eigen::AngleAxisf (0 * M_PI / 180, Eigen::Vector3f::UnitX ()) *
            Eigen::AngleAxisf (180 * M_PI / 180, Eigen::Vector3f::UnitY ()) *
            Eigen::AngleAxisf (0 * M_PI / 180, Eigen::Vector3f::UnitZ ());

    Eigen::Affine3f sensorPose = Eigen::Affine3f (Eigen::Translation3f (0,0,3));
    sensorPose.rotate (R);
    // Noise level. If greater than 0, values of neighboring points will be averaged.
    // This would set the search radius (e.g., 0.03 == 3cm).
    float noiseLevel = 0.0f;
    // Minimum range. If set, any point closer to the sensor than this will be ignored.
    float minimumRange = 0.0f;

    // Planar range image object.
    boost::shared_ptr<pcl::RangeImagePlanar> range_image_ptr(new pcl::RangeImagePlanar);
    pcl::RangeImagePlanar& rangeImagePlanar=*range_image_ptr;
    rangeImagePlanar.createFromPointCloudWithFixedSize(*cloud, imageSizeX, imageSizeY,
                                                       centerX, centerY, focalLengthX, focalLengthY,
                                                       sensorPose, pcl::RangeImage::CAMERA_FRAME,
                                                       noiseLevel, minimumRange);

    //visualize the pointcloud of the range image
    pcl::visualization::PCLVisualizer viewer ("3D Viewer");
    viewer.setBackgroundColor (1, 1, 1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 255, 0, 0);
    viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
    // 	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");

    //transform the pcl visualizer tool view to the depth camera position so it visualizes the camera POV
    viewer.initCameraParameters ();
    setViewerPose(viewer, rangeImagePlanar.getTransformationToWorldSystem ());

    //view the global coordinate system in the visualizer
    viewer.addCoordinateSystem(1.0,0);

    //visualizing the original cloud
    //      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> point_cloud_color_handler (cloud, 0, 0, 0);
    //      viewer.addPointCloud (cloud, point_cloud_color_handler, "original point cloud");

    // Visualize the range image.
    pcl::visualization::RangeImageVisualizer viewer1("Planar range image");
    viewer1.showRangeImage(rangeImagePlanar);
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
        viewer1.spinOnce();
        // Sleep 100ms to go easy on the CPU.
        pcl_sleep(0.1);
    }



    //    //*****************Camera View Vector  (Not working since the camera frame is different than the global frame)*****************
    //    tf::Matrix3x3 rotation;
    //    Eigen::Matrix3d D;
    //    D= R.cast<double>();
    //    tf::matrixEigenToTF(D,rotation);
    //    rotation = rotation.transpose();
    //    tf::Quaternion orientation;
    //    rotation.getRotation(orientation);

    //    geometry_msgs::Pose output_vector;
    //    Eigen::Quaterniond q;
    //    geometry_msgs::Quaternion quet;
    //    tf::quaternionTFToEigen(orientation, q);
    //    tf::quaternionTFToMsg(orientation,quet);
    //    Eigen::Affine3d pose;
    //    Eigen::Vector3d a;
    //    a[0]= T[0];
    //    a[1]= T[1];
    //    a[2]= T[2];
    //    pose.translation() = a;
    //    tf::poseEigenToMsg(pose, output_vector);

    //    visualization_msgs::Marker marker;

    //    ros::Rate loop_rate(10);
    //    while (ros::ok())
    //    {
    //        //***marker publishing***
    //        uint32_t shape = visualization_msgs::Marker::ARROW;
    //        marker.type = shape;
    //        marker.action = visualization_msgs::Marker::ADD;
    //        //visulaization using the markers
    //        marker.scale.x = 0.5;
    //        marker.scale.y = 0.1;
    //        marker.scale.z = 0.1;
    //        // Set the color -- be sure to set alpha to something non-zero!
    //        marker.color.r = 0.0f;
    //        marker.color.g = 1.0f;
    //        marker.color.b = 0.0f;
    //        marker.color.a = 1.0;
    //        marker.ns = "basic_shapes";
    //        marker.id = 2;
    //        //        ROS_INFO("Publishing Marker");
    //        // Set the frame ID and timestamp. See the TF tutorials for information on these.
    //        marker.pose= output_vector;
    //        marker.pose.orientation=quet;
    //        marker.header.frame_id = "base_point_cloud";
    //        marker.header.stamp = ros::Time::now();
    //        marker.lifetime = ros::Duration(10);
    //        // Publish the marker
    //        marker_pub.publish(marker);

    //        //***intersection publish***
    //        sensor_msgs::PointCloud2 cloud1;
    //        pcl::toROSMsg(*range_image_ptr, cloud1);
    //        cloud1.header.frame_id = "base_point_cloud";
    //        cloud1.header.stamp = ros::Time::now();
    //        pub1.publish(cloud1);

    //        ros::spinOnce();
    //        loop_rate.sleep();
    //    }

    return 0;
}
