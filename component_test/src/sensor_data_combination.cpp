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

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

sensor_msgs::PointCloud2::ConstPtr cloud1g, cloud2g;
int flag1=0, flag2=0;
ros::Time toc3, toc4;
void callback1(const sensor_msgs::PointCloud2ConstPtr& cloud1)
{
    flag1 = 1;
    toc3 = ros::Time::now();
    cloud1g = cloud1;

}

void callback2(const sensor_msgs::PointCloud2ConstPtr& cloud2)
{
    flag2 = 1;
    toc4 = ros::Time::now();
    cloud2g = cloud2;

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "sensor_data_combination_new");
    ros::NodeHandle n;
    std::string topic1 = n.resolveName("/firefly/xtion_sensor_up/firefly/xtion_sensor_up_camera/depth/points");
    std::string topic2 = n.resolveName("/firefly/xtion_sensor_down/firefly/xtion_sensor_down_camera/depth/points");

    ros::Publisher pub2 = n.advertise<sensor_msgs::PointCloud2>("sensors", 1000);

    //using callback function is working now
    ros::Subscriber sub1 = n.subscribe<sensor_msgs::PointCloud2>(topic1, 60, callback1);
    ros::Subscriber sub2 = n.subscribe<sensor_msgs::PointCloud2>(topic2, 60, callback2);

    // using message filters did not work
    //    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud1_sub(n, "/firefly/xtion_sensor_up/firefly/xtion_sensor_up_camera/depth/points", 60);
    //    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud2_sub(n, "/firefly/xtion_sensor_down/firefly/xtion_sensor_down_camera/depth/points", 60);
    //    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    //    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    //    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1000), cloud1_sub, cloud2_sub);
    //    sync.registerCallback(boost::bind(&callback, _1, _2));


    // *****************Rviz Visualization ************
    tf::TransformListener listener;
    sensor_msgs::PointCloud2 cloud, old_cloud1, old_cloud2;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_pcl, cloud1_pcl, cloud2_pcl;

    ros::Rate loop_rate(20);

    while (ros::ok())
    {

        if(flag1 == 1)
        {
            flag1=0;
            if(flag2 == 1)
            {
                double elapsed4 =  toc4.toSec() - toc3.toSec();
                //std::cout<<" elapsed time diff between two msgs: "<< elapsed4 <<std::endl;
                flag2=0;

                int current_try=0, max_tries = 3;
                bool canTrans;

                bool transform_success = true;
                try
                {
                    std::cout<<"before transformation frame id:"<<(*cloud1g).header.frame_id<<std::endl;
                    std::cout<<"before transformation frame id:"<<(*cloud2g).header.frame_id<<std::endl;

                    //upward sensor
                    ros::Time tic2 = ros::Time::now();
                    listener.waitForTransform( "/firefly/base_link","/firefly/xtion_sensor_up/camera_depth_optical_frame",  ros::Time::now(),ros::Duration(10));
                    canTrans = listener.canTransform("/firefly/base_link","/firefly/xtion_sensor_up/camera_depth_optical_frame",ros::Time::now());
                    pcl_ros::transformPointCloud("/firefly/base_link", *cloud1g, old_cloud1, listener);
                    std::cout<<" can transform?  "<<canTrans<<std::endl;
                    ros::Time toc2 = ros::Time::now();
                    double elapsed2 =  toc2.toSec() - tic2.toSec();
                    //std::cout<<" elapsed time listener1: "<< elapsed2 <<std::endl;

                    //downward sensor
                    ros::Time tic3 = ros::Time::now();
                    listener.waitForTransform( "/firefly/base_link","/firefly/xtion_sensor_down/camera_depth_optical_frame",  ros::Time::now(),ros::Duration(10));
                    canTrans = listener.canTransform("/firefly/base_link","/firefly/xtion_sensor_down/camera_depth_optical_frame",ros::Time::now());
                    pcl_ros::transformPointCloud("/firefly/base_link", *cloud2g, old_cloud2, listener);
                    std::cout<<" can transform? "<<canTrans<<std::endl;
                    ros::Time toc3 = ros::Time::now();
                    double elapsed3 =  toc3.toSec() - tic3.toSec();
                    //std::cout<<" elapsed time listener2: "<< elapsed3 <<std::endl;

                    std::cout<<"after transformation frame id:"<<old_cloud1.header.frame_id<<std::endl;
                    std::cout<<"after transformation frame id:"<<old_cloud2.header.frame_id<<std::endl;

                    ros::Time tic4 = ros::Time::now();
                    pcl::fromROSMsg(old_cloud1, cloud1_pcl);
                    pcl::fromROSMsg(old_cloud2, cloud2_pcl);
                    ros::Time toc4 = ros::Time::now();
                    double elapsed4 =  toc4.toSec() - tic4.toSec();
                    //std::cout<<" elapsed time fromRosmsg: "<< elapsed4 <<std::endl;

                    ros::Time tic5 = ros::Time::now();
                    cloud_pcl = cloud1_pcl;
                    cloud_pcl += cloud2_pcl;
                    pcl::toROSMsg(cloud_pcl, cloud);
                    ros::Time toc5 = ros::Time::now();
                    double elapsed5 =  toc5.toSec() - tic5.toSec();
                    //std::cout<<" elapsed time adding cloud: "<< elapsed5 <<std::endl;

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

                cloud.header.frame_id = "/firefly/base_link";
                cloud.header.stamp = ros::Time::now();
                pub2.publish(cloud);
                std::cout<<"transform succeeded"<<std::endl;


            }
        }


        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}
