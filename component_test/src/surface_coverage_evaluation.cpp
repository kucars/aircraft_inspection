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
#include "fcl_utility.h"
 using namespace fcl;
geometry_msgs::Pose uav2camTransformation(geometry_msgs::Pose pose, Vec3f rpy, Vec3f xyz);

int main(int argc, char **argv)
{

    ros::init(argc, argv, "accuracy_check");
    ros::NodeHandle n;
    ros::Publisher pub1 = n.advertise<sensor_msgs::PointCloud2>("originalPointCloud", 100);
    ros::Publisher pub2 = n.advertise<sensor_msgs::PointCloud2>("accuracyCloud", 100);
    ros::Publisher pub3 = n.advertise<geometry_msgs::PoseArray>("viewpoints", 100);

    pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr accuracyCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    OcclusionCulling obj(n,"etihad_nowheels_densed.pcd");
    double locationx,locationy,locationz,yaw,max=0, EMax,min=1,EMin;

    //reading the original cloud
    std::string path = ros::package::getPath("component_test");
    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/etihad_nowheels_densed.pcd", *originalCloud);
    geometry_msgs::PoseArray viewpoints;
    geometry_msgs::Pose pt;

    ros::Time accuracy_begin = ros::Time::now();


    //1: reading the path from a file
    std::string str1 = path+"/src/txt/3_90path_new.txt";
    const char * filename1 = str1.c_str();
    assert(filename1 != NULL);
    filename1 = strdup(filename1);
    FILE *file1 = fopen(filename1, "r");
    if (!file1)
    {
        std::cout<<"\nCan not open the File";
        fclose(file1);
    }
    pcl::PointCloud<pcl::PointXYZ> global;
    Vec3f rpy(0,0.093,0);
    Vec3f xyz(0,0.0,-0.055);
    geometry_msgs::Pose loc;

    while (!feof(file1))
    {
        fscanf(file1,"%lf %lf %lf %lf\n",&locationx,&locationy,&locationz,&yaw);
        pt.position.x=locationx; pt.position.y=locationy; pt.position.z=locationz;

        tf::Quaternion tf_q ;
        tf_q= tf::createQuaternionFromYaw(yaw);
        pt.orientation.x=tf_q.getX();pt.orientation.y=tf_q.getY();pt.orientation.z=tf_q.getZ();pt.orientation.w=tf_q.getW();
        loc= uav2camTransformation(pt,rpy,xyz);

        viewpoints.poses.push_back(loc);
        pcl::PointCloud<pcl::PointXYZ> visible;
        visible = obj.extractVisibleSurface(loc);
//        obj.visualizeFOV(pt);
        global +=visible;
//        cout<<"depth value of position 0 : "<<visible.at(0).z<<std::endl;
        std::cout<<"There are :  "<<visible.size()<<"points\n";
        for(int i=0; i<visible.size();i++){
            double temp = visible.at(i).z;//depth
            if(max<temp)
               max=temp;
            if(min>temp)
               min=temp;
        }
    }

    //2: calculate the Maximum Error
    EMax = 0.0000285 * max*max;
    EMin = 0.0000285 * min*min;

    std::cout<<"Maximum error: "<<EMax<<" for the depth of: "<<max<<"\n";
    std::cout<<"Minimum error: "<<EMin<<" for the depth of: "<<min<<"\n";
    std::cout<<"Size of the global cloud: "<<global.size()<<" points \n";

    //3: calculate the Error
    std::vector<double> ErrCal;
    geometry_msgs::PoseArray cloudPoses;
    double err;
    for (int j=0; j<global.size(); j++)
    {
        double val = global.at(j).z;//depth
        double valx = global.at(j).x;
        double valy = global.at(j).y;
//        std::cout<<"global cloud "<<j<<": x "<< valx<<" y "<<valy<<" z "<<val<<"\n";

        err= 0.0000285 * val * val;
        ErrCal.push_back(err);
    }

    //4: color coded model
    double color;
    int flag;
    pcl::PointCloud<pcl::PointXYZRGB> globalColored;
    for (int j=0; j<global.size(); j++)
    {
//        pcl::PointXYZRGB colored_point;
//        colored_point.data[0]=global.at(j).x;
//        colored_point.data[1]=global.at(j).y;
//        colored_point.data[2]=global.at(j).z;
        flag=0;
        color = (255 * (ErrCal.at(j)/EMax));
        pcl::PointXYZRGB p = pcl::PointXYZRGB(0,color,0);
        p.data[0] = global.at(j).x; p.data[1] = global.at(j).y; p.data[2] = global.at(j).z;
        for (int k=0; k<globalColored.size(); k++)
        {
            if (globalColored.at(k).x==global.at(j).x && globalColored.at(k).y==global.at(j).y)
            {
//                std::cout<<"found a match!!"<<std::endl;
                if(color<globalColored.at(k).g)
                {
//                    globalColored.at(k).r = color;
                    globalColored.at(k).g = color;
                }
                flag=1;
            }
        }
        if (flag==0)
        {
            globalColored.points.push_back(p);
        }

    }


    //use ptr cloud for visualization
    accuracyCloud->points=globalColored.points;

    std::cout<<"Size of the colored cloud: "<<globalColored.size()<<" points \n";

    ros::Time accuracy_end = ros::Time::now();
    double elapsed =  accuracy_end.toSec() - accuracy_begin.toSec();
    std::cout<<"accuracy check duration (s) = "<<elapsed<<"\n";

    // *****************Rviz Visualization ************

    ros::Rate loop_rate(10);
    while (ros::ok())
    {


        //***original cloud & frustum cull & occlusion cull publish***
        sensor_msgs::PointCloud2 cloud1;
        sensor_msgs::PointCloud2 cloud2;


        pcl::toROSMsg(*originalCloud, cloud1); //cloud of original (white) using original cloud
        pcl::toROSMsg(*accuracyCloud, cloud2); //cloud of the not occluded voxels (blue) using occlusion culling

        cloud1.header.frame_id = "map";
        cloud2.header.frame_id = "map";

        cloud1.header.stamp = ros::Time::now();
        cloud2.header.stamp = ros::Time::now();

        pub1.publish(cloud1);
        pub2.publish(cloud2);

        viewpoints.header.frame_id= "map";
        viewpoints.header.stamp = ros::Time::now();
        pub3.publish(viewpoints);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
geometry_msgs::Pose uav2camTransformation(geometry_msgs::Pose pose, Vec3f rpy, Vec3f xyz)
{


    Eigen::Matrix4d uav_pose, uav2cam, cam_pose;
    //UAV matrix pose
    Eigen::Matrix3d R; Eigen::Vector3d T1(pose.position.x,pose.position.y,pose.position.z);
    tf::Quaternion qt(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
    tf::Matrix3x3 R1(qt);
    tf::matrixTFToEigen(R1,R);
    uav_pose.setZero ();
    uav_pose.block (0, 0, 3, 3) = R;
    uav_pose.block (0, 3, 3, 1) = T1;
    uav_pose (3, 3) = 1;

    //transformation matrix
    qt = tf::createQuaternionFromRPY(rpy[0],rpy[1],rpy[2]);
    tf::Matrix3x3 R2(qt);Eigen::Vector3d T2(xyz[0],xyz[1],xyz[2]);
    tf::matrixTFToEigen(R2,R);
    uav2cam.setZero ();
    uav2cam.block (0, 0, 3, 3) = R;
    uav2cam.block (0, 3, 3, 1) = T2;
    uav2cam (3, 3) = 1;

    //preform the transformation
    cam_pose = uav_pose * uav2cam;

    Eigen::Matrix4d cam2cam;
    //the transofrmation is rotation by +90 around x axis of the camera
    cam2cam <<   1, 0, 0, 0,
                 0, 0,-1, 0,
                 0, 1, 0, 0,
                 0, 0, 0, 1;
    Eigen::Matrix4d cam_pose_new = cam_pose * cam2cam;
    geometry_msgs::Pose p;
    Eigen::Vector3d T3;Eigen::Matrix3d Rd; tf::Matrix3x3 R3;
    Rd = cam_pose_new.block (0, 0, 3, 3);
    tf::matrixEigenToTF(Rd,R3);
    T3 = cam_pose_new.block (0, 3, 3, 1);
    p.position.x=T3[0];p.position.y=T3[1];p.position.z=T3[2];
    R3.getRotation(qt);
    p.orientation.x = qt.getX(); p.orientation.y = qt.getY();p.orientation.z = qt.getZ();p.orientation.w = qt.getW();

    return p;

}

