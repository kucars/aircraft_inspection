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
#include <pcl/io/pcd_io.h>
#include "pcl_ros/transforms.h"



int main(int argc, char **argv)
{

    ros::init(argc, argv, "sensor_data_combination");
    ros::NodeHandle n;
    std::string topic1 = n.resolveName("/firefly/xtion_sensor_up/firefly/xtion_sensor_up_camera/depth/points");
    std::string topic2 = n.resolveName("/firefly/xtion_sensor_down/firefly/xtion_sensor_down_camera/depth/points");

    ros::Publisher pub2 = n.advertise<sensor_msgs::PointCloud2>("sensors", 1);

    //using callback function is not working
//    ros::Subscriber sub1 = n.subscribe<sensor_msgs::PointCloud2>(topic1, 1, callback1);
//    ros::Subscriber sub2 = n.subscribe<sensor_msgs::PointCloud2>(topic2, 1, callback2);




    // *****************Rviz Visualization ************
    tf::TransformListener listener;
    sensor_msgs::PointCloud2 cloud, old_cloud1, old_cloud2;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_pcl, cloud1_pcl, cloud2_pcl;

    ros::Rate loop_rate(10);
    while (ros::ok())
    {

        sensor_msgs::PointCloud2::ConstPtr recent_cloud1 = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic1);
        sensor_msgs::PointCloud2::ConstPtr recent_cloud2 = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic2);

        int current_try=0, max_tries = 3;
        bool canTrans;
        while (1)
        {
            bool transform_success = true;
            try
            {
                std::cout<<"before transformation frame id:"<<(*recent_cloud1).header.frame_id<<std::endl;
                std::cout<<"before transformation frame id:"<<(*recent_cloud2).header.frame_id<<std::endl;

                //upward sensor
                listener.waitForTransform( "/firefly/base_link","/firefly/xtion_sensor_up/camera_depth_optical_frame",  ros::Time::now(),ros::Duration(1));
                canTrans = listener.canTransform("/firefly/base_link","/firefly/xtion_sensor_up/camera_depth_optical_frame",ros::Time::now());
                pcl_ros::transformPointCloud("/firefly/base_link", *recent_cloud1, old_cloud1, listener);
                std::cout<<" can transform?  "<<canTrans<<std::endl;

                //downward sensor
                listener.waitForTransform( "/firefly/base_link","/firefly/xtion_sensor_down/camera_depth_optical_frame",  ros::Time::now(),ros::Duration(1));
                canTrans = listener.canTransform("/firefly/base_link","/firefly/xtion_sensor_down/camera_depth_optical_frame",ros::Time::now());
                pcl_ros::transformPointCloud("/firefly/base_link", *recent_cloud2, old_cloud2, listener);
                std::cout<<" can transform? "<<canTrans<<std::endl;

                std::cout<<"after transformation frame id:"<<old_cloud1.header.frame_id<<std::endl;
                std::cout<<"after transformation frame id:"<<old_cloud2.header.frame_id<<std::endl;


                pcl::fromROSMsg(old_cloud1, cloud1_pcl);
                pcl::fromROSMsg(old_cloud2, cloud2_pcl);

                cloud_pcl = cloud1_pcl;
                cloud_pcl += cloud2_pcl;
                pcl::toROSMsg(cloud_pcl, cloud);

            }
            catch (tf::TransformException ex)
            {
                transform_success = false;
                if (++current_try >= max_tries)
                {
                    ROS_ERROR("Failed to transform cloud");
                }
                ROS_DEBUG("Failed to transform point cloud, attempt %d out of %d, exception: %s", current_try, max_tries, ex.what());

                //sleep to give the listener a chance to get a new transform
                ros::Duration(0.1).sleep();
            }
            if (transform_success)
                break;

        }
        cloud.header.frame_id = "/firefly/base_link";
        cloud.header.stamp = ros::Time::now();
        pub2.publish(cloud);
        std::cout<<"transform succeeded"<<std::endl;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
