#include "ros/ros.h"
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
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
//#include <voxel_grid_occlusion_estimation.h>
#include "fcl_utility.h"
#include <pcl/filters/voxel_grid.h>
#include <component_test/occlusion_culling.h>
using namespace fcl;
visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links, int c_color, float scale);
geometry_msgs::Pose uav2camTransformation(geometry_msgs::Pose pose, Vec3f rpy, Vec3f xyz);

int main(int argc, char **argv)
{

    ros::init(argc, argv, "coverge_quantification");
    ros::NodeHandle n;
    ros::Publisher pub1 = n.advertise<sensor_msgs::PointCloud2>("original_point_cloud", 100);
    ros::Publisher pub2 = n.advertise<sensor_msgs::PointCloud2>("occlusion_free_cloud", 100);
    ros::Publisher pub3 = n.advertise<sensor_msgs::PointCloud2>("original_filtered", 100);
    ros::Publisher pub4 = n.advertise<sensor_msgs::PointCloud2>("occlusion_filtered", 100);
    ros::Publisher pub5 = n.advertise<sensor_msgs::PointCloud2>("matched", 100);


    pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr originalFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr matchedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr occlusionFreeCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr occlusionFiltered(new pcl::PointCloud<pcl::PointXYZ>);


    std::string path = ros::package::getPath("component_test");
    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/etihad_nowheels_densed.pcd", *originalCloud);
    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/trial2.pcd", *occlusionFreeCloud);

    //******************************needed for debugging the predicted vs the constructed
    ros::Publisher predicted_pub = n.advertise<sensor_msgs::PointCloud2>("predicted_coverage", 100);
    ros::Publisher path_pub = n.advertise<visualization_msgs::Marker>("generated_path", 10);
    ros::Publisher sen_vector_pub = n.advertise<geometry_msgs::PoseArray>("sensor_pose", 10);

    pcl::PointCloud<pcl::PointXYZ>::Ptr predictedCloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/3_5percent.pcd", *predictedCloud);//option 1:loading the predicted from a file

    //Option 2: extract the visible surface at each viewpoint
    OcclusionCulling obj(n,"etihad_nowheels_densed.pcd");
    pcl::PointCloud<pcl::PointXYZ> predicted_cloud;

    double locationx,locationy,locationz,yaw;
    geometry_msgs::PoseArray viewpoints;
    geometry_msgs::Pose pt,loc;

    std::string str1 = path+"/src/txt/3_5path.txt";
    const char * filename1 = str1.c_str();
    assert(filename1 != NULL);
    filename1 = strdup(filename1);
    FILE *file1 = fopen(filename1, "r");
    if (!file1)
    {
        std::cout<<"\nCan not open the File";
        fclose(file1);
    }
    Vec3f rpy(0,0.093,0);
    Vec3f xyz(0,0.0,-0.055);
    while (!feof(file1))
    {
        fscanf(file1,"%lf %lf %lf %lf\n",&locationx,&locationy,&locationz,&yaw);
        pt.position.x=locationx; pt.position.y=locationy; pt.position.z=locationz;
        tf::Quaternion tf_q ;
        tf_q= tf::createQuaternionFromYaw(yaw);
        pt.orientation.x=tf_q.getX();pt.orientation.y=tf_q.getY();pt.orientation.z=tf_q.getZ();pt.orientation.w=tf_q.getW();
        loc= uav2camTransformation(pt,rpy,xyz);
        viewpoints.poses.push_back(loc);
        pcl::PointCloud<pcl::PointXYZ> temp_cloud;
        temp_cloud=obj.extractVisibleSurface(loc);
        predicted_cloud += temp_cloud;
    }
    predictedCloud->points = predicted_cloud.points;
    std::vector<geometry_msgs::Point> lineSegments;
    geometry_msgs::Point p;
    for (int i =0; i<viewpoints.poses.size(); i++)
    {
        if(i+1< viewpoints.poses.size())
        {
            p.x = viewpoints.poses[i].position.x;
            p.y =  viewpoints.poses.at(i).position.y;
            p.z =  viewpoints.poses.at(i).position.z;
            lineSegments.push_back(p);

            p.x = viewpoints.poses.at(i+1).position.x;
            p.y =  viewpoints.poses.at(i+1).position.y;
            p.z =  viewpoints.poses.at(i+1).position.z;
            lineSegments.push_back(p);
        }
    }
    visualization_msgs::Marker linesList = drawLines(lineSegments,1,0.15);

    //*******************************************************



    // *******************original cloud Grid***************************
    //used VoxelGridOcclusionEstimationT since the voxelGrid does not include getcentroid function
        pcl::VoxelGridOcclusionEstimationT voxelFilterOriginal;
        voxelFilterOriginal.setInputCloud (originalCloud);
        float res = 0.5;
        voxelFilterOriginal.setLeafSize (res, res, res);
        voxelFilterOriginal.initializeVoxelGrid();
        voxelFilterOriginal.filter(*originalFiltered);
        std::cout<<"original filtered "<<originalFiltered->points.size()<<"\n";

     //*******************Occupied Cloud Grid***************************
        pcl::VoxelGridOcclusionEstimationT voxelFilterOccupied;
        voxelFilterOccupied.setInputCloud (predictedCloud);
        voxelFilterOccupied.setLeafSize (res, res, res);
        voxelFilterOccupied.initializeVoxelGrid();
        voxelFilterOccupied.filter(*occlusionFiltered);
        std::cout<<"covered filtered "<<occlusionFiltered->points.size()<<"\n";

        float test = (float)occlusionFiltered->points.size()/(float)originalFiltered->points.size() *100;
        std::cout<<" TEST coverage percentage : "<<test<<"\n";

     //*****************************************************************

        Eigen::Vector3i  min_b = voxelFilterOccupied.getMinBoxCoordinates ();
        Eigen::Vector3i  max_b = voxelFilterOccupied.getMaxBoxCoordinates ();
        Eigen::Vector3i  min_b1 = voxelFilterOriginal.getMinBoxCoordinates ();
        Eigen::Vector3i  max_b1 = voxelFilterOriginal.getMaxBoxCoordinates ();

        float OriginalVoxelsSize=0, MatchedVoxels=0;

        // iterate over the entire original voxel grid to get the size of the grid
        ros::Time coverage_begin = ros::Time::now();
        for (int kk = min_b1.z (); kk <= max_b1.z (); ++kk)
        {
            for (int jj = min_b1.y (); jj <= max_b1.y (); ++jj)
            {
                for (int ii = min_b1.x (); ii <= max_b1.x (); ++ii)
                {
                    Eigen::Vector3i ijk1 (ii, jj, kk);
                    int index1 = voxelFilterOriginal.getCentroidIndexAt (ijk1);
                    if(index1!=-1)
                    {
                        OriginalVoxelsSize++;
                    }

                }
            }
        }

        //iterate through the entire coverage grid to check the number of matched voxel between the original and the covered ones
        for (int kk = min_b.z (); kk <= max_b.z (); ++kk)
        {
            for (int jj = min_b.y (); jj <= max_b.y (); ++jj)
            {
                for (int ii = min_b.x (); ii <= max_b.x (); ++ii)
                {

                    Eigen::Vector3i ijk (ii, jj, kk);
                    int index1 = voxelFilterOccupied.getCentroidIndexAt (ijk);
                    if(index1!=-1)
                    {
                        Eigen::Vector4f centroid = voxelFilterOccupied.getCentroidCoordinate (ijk);
                        Eigen::Vector3i ijk_in_Original= voxelFilterOriginal.getGridCoordinates(centroid[0],centroid[1],centroid[2]) ;

                        int index = voxelFilterOriginal.getCentroidIndexAt (ijk_in_Original);

                        if(index!=-1)
                        {
                            pcl::PointXYZRGB point = pcl::PointXYZRGB(0,244,0);
                            point.x = centroid[0];
                            point.y = centroid[1];
                            point.z = centroid[2];
                            matchedCloud->points.push_back(point);

                            MatchedVoxels++;
                        }
                    }

                }
            }
        }

        ros::Time coverage_end = ros::Time::now();
        double elapsed =  coverage_end.toSec() - coverage_begin.toSec();
        std::cout<<"coverage percentage duration (s) = "<<elapsed<<"\n";

        //calculating the coverage percentage
        float coverage_ratio= MatchedVoxels/OriginalVoxelsSize;
        float coverage_percentage= coverage_ratio*100;

        std::cout<<" the coverage ratio is = "<<coverage_ratio<<"\n";
        std::cout<<" the number of covered voxels = "<<MatchedVoxels<<" voxel is covered"<<"\n";
        std::cout<<" the number of original voxels = "<<OriginalVoxelsSize<<" voxel"<<"\n\n\n";
        std::cout<<" the coverage percentage is = "<<coverage_percentage<<" %"<<"\n";


        // *****************Rviz Visualization ************

        ros::Rate loop_rate(10);
        while (ros::ok())
        {


            //***original cloud & frustum cull & occlusion cull publish***
            sensor_msgs::PointCloud2 cloud1;
            sensor_msgs::PointCloud2 cloud2;
            sensor_msgs::PointCloud2 cloud3;
            sensor_msgs::PointCloud2 cloud4;
            sensor_msgs::PointCloud2 cloud5;
            sensor_msgs::PointCloud2 cloud6;

            pcl::toROSMsg(*originalCloud, cloud1); //cloud of original (white) using original cloud
            pcl::toROSMsg(*occlusionFreeCloud, cloud2); //cloud of the not occluded voxels (blue) using occlusion culling
            pcl::toROSMsg(*originalFiltered, cloud3); //cloud of original (white) using original cloud
            pcl::toROSMsg(*occlusionFiltered, cloud4); //cloud of the not occluded voxels (blue) using occlusion culling
            pcl::toROSMsg(*matchedCloud, cloud5); //cloud of the not occluded voxels (blue) using occlusion culling
            pcl::toROSMsg(*predictedCloud, cloud6); //cloud of the not occluded voxels (blue) using occlusion culling

            cloud1.header.frame_id = "base_point_cloud";
            cloud2.header.frame_id = "base_point_cloud";
            cloud3.header.frame_id = "base_point_cloud";
            cloud4.header.frame_id = "base_point_cloud";
            cloud5.header.frame_id = "base_point_cloud";
            cloud6.header.frame_id = "base_point_cloud";

            cloud1.header.stamp = ros::Time::now();
            cloud2.header.stamp = ros::Time::now();
            cloud3.header.stamp = ros::Time::now();
            cloud4.header.stamp = ros::Time::now();
            cloud5.header.stamp = ros::Time::now();
            cloud6.header.stamp = ros::Time::now();

            pub1.publish(cloud1);
            pub2.publish(cloud2);
            pub3.publish(cloud3);
            pub4.publish(cloud4);
            pub5.publish(cloud5);
            predicted_pub.publish(cloud6);

            viewpoints.header.frame_id= "base_point_cloud";
            viewpoints.header.stamp = ros::Time::now();
            sen_vector_pub.publish(viewpoints);

            path_pub.publish(linesList);


            ros::spinOnce();
            loop_rate.sleep();
        }

    return 0;
}

visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links, int c_color, float scale)
{
    visualization_msgs::Marker linksMarkerMsg;
    linksMarkerMsg.header.frame_id="base_point_cloud";
    linksMarkerMsg.header.stamp=ros::Time::now();
    linksMarkerMsg.ns="link_marker";
    linksMarkerMsg.id = 0;
    linksMarkerMsg.type = visualization_msgs::Marker::LINE_LIST;
    linksMarkerMsg.scale.x = scale;
    linksMarkerMsg.action  = visualization_msgs::Marker::ADD;
    linksMarkerMsg.lifetime  = ros::Duration(10000.0);
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
