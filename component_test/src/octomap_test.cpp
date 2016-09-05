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
#include <fstream>
#include <tf/transform_datatypes.h>
//PCL
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/filters/frustum_culling.h>
//#include <frustum_culling.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image.h>
//#include <voxel_grid_occlusion_estimation.h>
#include <pcl/filters/voxel_grid.h>

#include "fcl_utility.h"
//CGAL
#include <list>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>

#include <stdlib.h>
#include <boost/foreach.hpp>
#include <Eigen/Eigen>
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>

#include <tf/transform_broadcaster.h>
#include <component_test/occlusion_culling.h>
#include <component_test/occlusion_culling_gpu.h>
#include <component_test/visualization_voxelgrid.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>

#include <octomap_msgs/Octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include "nav_msgs/Odometry.h"

#include <iostream>
#include <octomap_msgs/conversions.h>
using namespace std;
using namespace octomap;


visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links, int id, int c_color);
visualization_msgs::Marker drawCUBE(fcl::Vec3f vec , int id , int c_color, double size);
geometry_msgs::Pose uav2camTransformation(geometry_msgs::Pose pose, fcl::Vec3f rpy, fcl::Vec3f xyz);

void print_query_info(point3d query, OcTreeNode* node) {

    if (node != NULL) {
        cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() <<" "<<node->getValue() << endl;
    }
    else
        cout << "occupancy probability at " << query << ":\t is unknown" << endl;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "octomap");
    ros::NodeHandle n;

    ros::Publisher pub1 = n.advertise<sensor_msgs::PointCloud2>("original_point_cloud", 100);
    ros::Publisher octomapPub = n.advertise<octomap_msgs::Octomap>("octomap", 1);
    ros::Publisher viewpointsPub    = n.advertise<geometry_msgs::PoseArray>("viewpoints", 100);
    ros::Publisher waypointsPub     = n.advertise<geometry_msgs::PoseArray>("waypoints", 100);
    ros::Publisher markerPub        = n.advertise<visualization_msgs::MarkerArray>("markers", 100);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    std::string path = ros::package::getPath("component_test");
    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/etihad_nowheels_nointernal_scaled_newdensed.pcd", *cloudPtr);

    cout << "generating octomap" << endl;
    OcTree* tree = new OcTree(0.25);  // create empty tree with resolution 0.1
    OcclusionCullingGPU obj(n,"etihad_nowheels_nointernal_scaled_newdensed.pcd");

    //1: reading the path from a file
    std::string str1 = path+"/src/txt/2_5_10pathV.txt";//3_90path_new
    const char * filename1 = str1.c_str();
    assert(filename1 != NULL);
    filename1 = strdup(filename1);
    FILE *file1 = fopen(filename1, "r");
    if (!file1)
    {
        std::cout<<"\nCan not open the File";
        fclose(file1);
    }
    pcl::PointCloud<pcl::PointXYZ> modelVoxels;
    pcl::VoxelGridOcclusionEstimationGPU voxelgrid;
    voxelgrid.setInputCloud (cloudPtr);
    voxelgrid.setLeafSize (0.25, 0.25, 0.25);
    voxelgrid.initializeVoxelGrid();
    voxelgrid.filter(modelVoxels);

    geometry_msgs::PoseArray viewpoints,points;
    geometry_msgs::Pose pt;
    geometry_msgs::Pose loc;
    visualization_msgs::MarkerArray marker_array ;
    visualization_msgs::Marker marker ;
    double locationx,locationy,locationz,yaw;
    fcl::Vec3f rpy(0,0.349,0);
    fcl::Vec3f xyz(0,0.022,-0.065);
    tree->setProbHit(0.99999);
    tree->setProbMiss(0.5);
    tree->setClampingThresMax(0.99999);
    tree->setClampingThresMin(0.5);

    //full model coverage
    octomap::Pointcloud octPointCloud;
    for(int i = 0;i<obj.cloud->points.size();i++)
    {
        octomap::point3d endpoint((float) obj.cloud->points[i].x,(float) obj.cloud->points[i].y,(float) obj.cloud->points[i].z);
        octPointCloud.push_back(endpoint);
    }
    octomap::OcTree fullModelTree(0.25);
    octomap::KeySet freeKeys,occupiedKeys;
    fullModelTree.computeUpdate(octPointCloud,octomap::point3d(0,0,0),freeKeys,occupiedKeys,-1);
    double modelVolume = 0;
    int i=0;
    for (KeySet::iterator it = occupiedKeys.begin(); it != occupiedKeys.end(); ++it) {
        modelVolume += (0.25*0.25*0.25);
        fullModelTree.updateNode(*it, true);
        i++;
    }
    std::cout<<"volume of the tree1: "<<modelVolume<<std::endl;
    std::cout<<"number of occupied voxels after: "<<i<<std::endl;

    //going through path
    double coveredVolume=0;
    int j=0;
    int num=0;
    while (!feof(file1))
    {
        fscanf(file1,"%lf %lf %lf %lf\n",&locationx,&locationy,&locationz,&yaw);
        pt.position.x=locationx; pt.position.y=locationy; pt.position.z=locationz;
        tf::Quaternion tf_q ;
        tf_q= tf::createQuaternionFromYaw(yaw);
        pt.orientation.x=tf_q.getX();pt.orientation.y=tf_q.getY();pt.orientation.z=tf_q.getZ();pt.orientation.w=tf_q.getW();
        points.poses.push_back(pt);
        loc= uav2camTransformation(pt,rpy,xyz);
        viewpoints.poses.push_back(loc);

        pcl::PointCloud<pcl::PointXYZ> visible;
        octomap::Pointcloud octPointCloud;

        visible = obj.extractVisibleSurface(loc);

        //put the visible cloud in the octomap
        for(int i = 0;i<visible.points.size();i++)
        {
            octomap::point3d endpoint((float) visible.points[i].x,(float) visible.points[i].y,(float) visible.points[i].z);
            octPointCloud.push_back(endpoint);
        }
        //        std::cout<<"visible cloud size: "<<visible.points.size()<<std::endl;
        //        std::cout<<"number of nodes before: "<<tree.calcNumNodes()<<std::endl;

        octomap::point3d origin(loc.position.x,loc.position.y,loc.position.z);
        KeySet freeKeys,occupiedKeys;
        tree->computeUpdate(octPointCloud,origin,freeKeys,occupiedKeys,-1);

        for (KeySet::iterator it = occupiedKeys.begin(); it != occupiedKeys.end(); ++it) {
            octomap::point3d center = tree->keyToCoord(*it);
            fcl::Vec3f vec2(center.x(), center.y(), center.z());
            double dist =  std::sqrt(( vec2[0] - origin.x())*(vec2[0] - origin.x()) + (vec2[1] - origin.y())*(vec2[1] - origin.y()) + (vec2[2] -origin.z())*(vec2[2] -origin.z()));
            double normAcc = (obj.maxAccuracyError - 0.0000285 * dist * dist*0.5)/obj.maxAccuracyError;
            float lg = octomap::logodds(normAcc);

            octomap::OcTreeNode* result = tree->search(*it);
            if(result!=NULL)//it is occupied
            {
//                std::cout<<"prob: "<<result->getOccupancy()<<std::endl;
            }
            else //it is NULL (unknown)
            {
                coveredVolume += (0.25*0.25*0.25);
                tree->setNodeValue(*it, lg);
                num++;
            }
        }

//        tree.insertPointCloud(octPointCloud,origin);
//        tree.updateInnerOccupancy();

    }


    std::cout<<"clamping threshhold max: "<<tree->getClampingThresMax()<<std::endl;
    std::cout<<"clamping threshhold min: "<<tree->getClampingThresMin()<<std::endl;
    std::cout<<"clamping threshhold logodd max: "<<tree->getClampingThresMaxLog()<<std::endl;
    std::cout<<"clamping threshhold logodd min: "<<tree->getClampingThresMinLog()<<std::endl;
    std::cout<<"Prob hit: "<<tree->getProbHit()<<std::endl;
    std::cout<<"Prob miss: "<<tree->getProbMiss()<<std::endl;
    std::cout<<"Prob hit logodd: "<<tree->getProbHitLog()<<std::endl;
    std::cout<<"Prob miss logodd: "<<tree->getProbMissLog()<<std::endl;
    std::cout<<"number of voxels: " <<num <<" covered volume percent: "<<(coveredVolume/modelVolume)*100<<std::endl;

    int r =0 ;
    //another way of looping through octomap (entire tree)
    //visualizing the occupied voxels
    for(OcTree::iterator it = tree->begin(); it!= tree->end(); ++it)
    {
        if(it->getValue()>0.0)
        {
            fcl::Vec3f vec2(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z());
            marker = drawCUBE(vec2,r,1,0.25) ;
            marker_array.markers.push_back(marker);
        }
        r++;
    }


    cout << endl;
    cout << "performing some queries:" << endl;

    //    point3d query1 (-0.125, -35.375, 4.625);
    point3d query1 (-1.125, -32.125, 6.625);
    OcTreeNode* result1 = tree->search (query1);
    print_query_info(query1, result1);

    point3d query (1.375, -31.375, 4.625);
    OcTreeNode* result = tree->search (query);
    print_query_info(query, result);

    point3d query2 (9.125, -6.625, 4.125);
    OcTreeNode* result2 = tree->search (query2);
    print_query_info(query2, result2);

    point3d query3 (1.625, -30.375, 6.875);
    OcTreeNode* result3 = tree->search (query3);
    print_query_info(query3, result3);


    ros::Rate loop_rate(10);//it was 10
    while (ros::ok())
    {

        octomap_msgs::Octomap octomap ;
        octomap.binary = 1 ;
        octomap.id = 1 ;
        octomap.resolution =0.1;
        octomap.header.frame_id = "/base_point_cloud";
        octomap.header.stamp = ros::Time::now();
        bool res = octomap_msgs::fullMapToMsg(*tree, octomap);
        if(res)
        {
            octomapPub.publish(octomap);
        }
        else
        {
            ROS_WARN("OCT Map serialization failed!");
        }

        //***viewpoints & waypoints  publish***
        viewpoints.header.frame_id= "base_point_cloud";
        viewpoints.header.stamp = ros::Time::now();
        viewpointsPub.publish(viewpoints);

        points.header.frame_id= "base_point_cloud";
        points.header.stamp = ros::Time::now();
        waypointsPub.publish(points);

        markerPub.publish(marker_array);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links, int id, int c_color)
{
    visualization_msgs::Marker linksMarkerMsg;
    linksMarkerMsg.header.frame_id="base_point_cloud";
    linksMarkerMsg.header.stamp=ros::Time::now();
    linksMarkerMsg.ns="link_marker";
    linksMarkerMsg.id = id;
    linksMarkerMsg.type = visualization_msgs::Marker::LINE_LIST;
    linksMarkerMsg.scale.x = 0.04;
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

visualization_msgs::Marker drawCUBE(fcl::Vec3f vec , int id , int c_color, double size)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_point_cloud";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = vec[0];
    marker.pose.position.y = vec[1];
    marker.pose.position.z = vec[2];
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;//poseQ[1];
    marker.pose.orientation.z = 0;//poseQ[2];
    marker.pose.orientation.w = 1;//poseQ[3];

    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = size;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    if(c_color == 1)
    {
        marker.color.r = 0.0;
        marker.color.b = 1.0;
        marker.color.g = 0.0;
    }
    else if(c_color == 2)
    {
        marker.color.r = 1.0;
        marker.color.b = 0.0;
        marker.color.g = 0.0;
    }
    else
    {
        marker.color.r = 0.0;
        marker.color.b = 0.0;
        marker.color.g = 1.0;
    }
    marker.lifetime = ros::Duration(120);
    return marker ;
}

geometry_msgs::Pose uav2camTransformation(geometry_msgs::Pose pose, fcl::Vec3f rpy, fcl::Vec3f xyz)
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
