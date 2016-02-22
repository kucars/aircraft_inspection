#include "component_test/occlusion_culling.h"
#include <geometry_msgs/PoseArray.h>



int main(int argc, char **argv)
{

    ros::init(argc, argv, "test");
    ros::NodeHandle n;
    ros::Publisher visible_pub = n.advertise<sensor_msgs::PointCloud2>("occlusion_free_cloud", 100);
    ros::Publisher original_pub = n.advertise<sensor_msgs::PointCloud2>("original_point_cloud", 10);
    ros::Publisher pub1 = n.advertise<sensor_msgs::PointCloud2>("publish1", 10);
    ros::Publisher pub2 = n.advertise<sensor_msgs::PointCloud2>("publish2", 10);

    ros::Publisher vector_pub = n.advertise<geometry_msgs::PoseArray>("pose", 10);

    pcl::PointCloud<pcl::PointXYZ>::Ptr test (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr test1 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr test2 (new pcl::PointCloud<pcl::PointXYZ>);

     pcl::PointCloud<pcl::PointXYZ> test3, test4, combined;
     geometry_msgs::PoseArray vec;
     OcclusionCulling obj(n,"etihad_nowheels_densed.pcd");

     geometry_msgs::Pose location;
    //example 1 from searchspace file
     double yaw = 2.3562;
     tf::Quaternion tf_q ;
     tf_q= tf::createQuaternionFromYaw(yaw);
     location.position.x=4.0; location.position.y=-30.0; location.position.z=10.5; location.orientation.x=tf_q.getX(); location.orientation.y=tf_q.getY();location.orientation.z=tf_q.getZ();location.orientation.w=tf_q.getW();
     test3 = obj.extractVisibleSurface(location);
     test1->points = test3.points;
     vec.poses.push_back(location);
//     float covpercent1 = obj.calcCoveragePercent(test);

     //example 2 from searchspace file
//     OcclusionCulling obj1(n,"scaled_desktop.pcd");
     location.position.x=4.0; location.position.y=29.0; location.position.z=10.5; location.orientation.x=tf_q.getX(); location.orientation.y=tf_q.getY();location.orientation.z=tf_q.getZ();location.orientation.w=tf_q.getW();
     test4 = obj.extractVisibleSurface(location);
     test2->points = test4.points;
     vec.poses.push_back(location);

     combined=test3;
     combined+=test4;
     test->points = combined.points;

//     float covpercent = obj.calcCoveragePercent(location);

//     float combinedcoverage = covpercent + covpercent1;

//     std::cout<<"coverage percentage: "<<combinedcoverage<<"%\n";
//     vec.poses.push_back(location);
     ros::Rate loop_rate(10);
     while (ros::ok())
     {
         std::cout<<"filtered original point cloud: "<<obj.filtered_cloud->size()<<"\n";
         sensor_msgs::PointCloud2 cloud1;
         pcl::toROSMsg(*obj.cloud, cloud1);
         cloud1.header.frame_id = "base_point_cloud";
         cloud1.header.stamp = ros::Time::now();
         original_pub.publish(cloud1);

         sensor_msgs::PointCloud2 cloud2;
         pcl::toROSMsg(*test, cloud2);
         cloud2.header.frame_id = "base_point_cloud";
         cloud2.header.stamp = ros::Time::now();
         visible_pub.publish(cloud2);

         sensor_msgs::PointCloud2 cloud3;
         pcl::toROSMsg(*test1, cloud3);
         cloud3.header.frame_id = "base_point_cloud";
         cloud3.header.stamp = ros::Time::now();
         pub1.publish(cloud3);
         sensor_msgs::PointCloud2 cloud4;
         pcl::toROSMsg(*test2, cloud4);
         cloud4.header.frame_id = "base_point_cloud";
         cloud4.header.stamp = ros::Time::now();
         pub2.publish(cloud4);

         vec.header.frame_id= "base_point_cloud";
         vec.header.stamp = ros::Time::now();
         vector_pub.publish(vec);

         ROS_INFO("Publishing Marker");

         ros::spinOnce();
         loop_rate.sleep();
     }


    return 0;
}
