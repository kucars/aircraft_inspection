#include "component_test/occlusion_culling.h"
#include <geometry_msgs/PoseArray.h>
#include <math.h>
#include <sstream>
#include <string>
#include <vector>
#include <limits>
#include <cassert>
#include <iomanip>
#include "fcl_utility.h"
//#include "voxel_grid.h"
#include"component_test/visualization_voxelgrid.h"
using namespace fcl;
visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links, int c_color, float scale);
visualization_msgs::Marker drawPoints(std::vector<geometry_msgs::Point> points, int c_color, int duration);
geometry_msgs::Pose uav2camTransformation(geometry_msgs::Pose pose, Vec3f rpy, Vec3f xyz);
visualization_msgs::Marker linesList;
geometry_msgs::PoseArray viewpoints, points;

void distance_viewpoints()
{
    double locationx,locationy,locationz,yaw;

    double total =0.0 ;
    //    total = sqrt(pow(a.position.x - b.position.x,2) + pow(a.position.y - b.position.y,2) + pow(a.position.z - b.position.z,2) );
    geometry_msgs::Pose pt,loc;
    geometry_msgs::Point pt1,pt2;
    std::string path = ros::package::getPath("component_test");


    std::string str1 = path+"/src/txt/3_10path.txt";
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

        fscanf(file1,"%lf %lf %lf %lf\n",&locationx,&locationy,&locationz, &yaw);
        pt.position.x=locationx; pt.position.y=locationy; pt.position.z=locationz;
        tf::Quaternion tf_q ;
        tf_q = tf::createQuaternionFromYaw(yaw);
        pt.orientation.x = tf_q.getX();
        pt.orientation.y = tf_q.getY();
        pt.orientation.z = tf_q.getZ();
        pt.orientation.w = tf_q.getW();
        points.poses.push_back(pt);
        loc= uav2camTransformation(pt,rpy,xyz);

        viewpoints.poses.push_back(loc);
    }
    std::vector<geometry_msgs::Point> lineSegments;

    for (int i =0; i<points.poses.size(); i++)
    {
        if(i+1< points.poses.size())
        {
            pt1.x = points.poses[i].position.x;
            pt1.y =  points.poses.at(i).position.y;
            pt1.z =  points.poses.at(i).position.z;
            lineSegments.push_back(pt1);

            pt2.x = points.poses.at(i+1).position.x;
            pt2.y =  points.poses.at(i+1).position.y;
            pt2.z =  points.poses.at(i+1).position.z;
            lineSegments.push_back(pt2);

            double temp =sqrt(pow(pt1.x - pt2.x,2) + pow(pt1.y - pt2.y,2) + pow(pt1.z - pt2.z,2) );
            total = total+temp;

        }
    }
    linesList = drawLines(lineSegments,1,0.15);

    std::cout<<"total length for points of size :"<<points.poses.size()<<" is : "<<total<<" m\n\n";
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "test");
    ros::NodeHandle n;
    ros::Publisher originalCloudPub          = n.advertise<sensor_msgs::PointCloud2>("original_cloud", 100);
    ros::Publisher voxelsPub                 = n.advertise<sensor_msgs::PointCloud2>("voxels", 100);
    ros::Publisher voxelPointsPub            = n.advertise<sensor_msgs::PointCloud2>("voxel_points", 100);
    ros::Publisher frustumPointsPub          = n.advertise<sensor_msgs::PointCloud2>("frustum_points", 100);
    ros::Publisher visiblePointsPub          = n.advertise<sensor_msgs::PointCloud2>("visible_points", 100);
    ros::Publisher intersectionPointPub      = n.advertise<visualization_msgs::Marker>("intersection_points", 10);
    ros::Publisher voxelBoxPub               = n.advertise<visualization_msgs::Marker>("voxels_box", 10);
    ros::Publisher linesPub                     = n.advertise<visualization_msgs::Marker>("lines", 10);

//    ros::Publisher fov                       = n.advertise<visualization_msgs::MarkerArray>("voxels_box", 10);

    ros::Publisher sensorPosePub             = n.advertise<geometry_msgs::PoseArray>("sensor_pose", 10);
    ros::Publisher PosePub                   = n.advertise<geometry_msgs::PoseArray>("pose", 10);
    ros::Publisher generatedPathPub          = n.advertise<visualization_msgs::Marker>("generated_path", 10);
    std::string path = ros::package::getPath("component_test");
    pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxelsPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxelsPointsPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr visiblePointsPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr frustumPointsPtr(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/etihad_nowheels_densed.pcd", *originalCloud);

    //**********************viewing the generated path*********
    //    distance_viewpoints();

    //**********************testing visulization voxelgrid***********
    geometry_msgs::Pose pose;
    geometry_msgs::PoseArray poses;
    pcl::VoxelGridT voxelgrid;
    voxelgrid.setInputCloud (originalCloud);
    voxelgrid.setLeafSize (0.5f, 0.5f, 0.5f);
    voxelgrid.filter(*voxelsPtr);
    //    std::cout<<"finished filtering peacefully\n";

    pose.position.x = 5;pose.position.y = -25;pose.position.z = 5.8;pose.orientation.x = -0.0328686;pose.orientation.y = 0.706342;pose.orientation.z = 0.706342;pose.orientation.w = 0.0328686;
    poses.poses.push_back(pose);
    pcl::PointCloud<pcl::PointXYZ> pointcloud;
    OcclusionCulling occ(n,"etihad_nowheels_densed.pcd");
    pointcloud = occ.extractVisibleSurface(pose);
    Point o(5,-25,5.8);
    std::vector<Point> far_points;
    Point b1(occ.fc.fp_tl[0], occ.fc.fp_tl[1], occ.fc.fp_tl[2]);far_points.push_back(b1);//top left (far)
    Point b2(occ.fc.fp_tr[0], occ.fc.fp_tr[1], occ.fc.fp_tr[2]);far_points.push_back(b2);//top right (far)
    Point b3(occ.fc.fp_bl[0], occ.fc.fp_bl[1], occ.fc.fp_bl[2]);far_points.push_back(b3);//bottom left (far)
    Point b4(occ.fc.fp_br[0], occ.fc.fp_br[1], occ.fc.fp_br[2]);far_points.push_back(b4);//bottom right (far)
    //testing the new class (use visualization_voxelgrid.rviz)
    VoxelGridVisualization obj(n,"etihad_nowheels_densed.pcd",voxelgrid,"world");
    obj.visualizeBB();
    obj.intersectionFOVBB(o, far_points);
    obj.visualizeFOVBBIntersection(o, far_points);
    Eigen::Vector3i ijk(59,137,28); //59,131,22
    obj.selectedVoxelPoints(ijk,visiblePointsPtr);
//    std::cout<<"finished the selectedVoxelsPoints peacefully\n";
//    Eigen::Vector4f centroid = voxelgrid.getCentroidCoordinate (ijk);
//    geometry_msgs::Pose pos;
//    pos.position.x = centroid[0];pos.position.y = centroid[1];pos.position.z = centroid[2];
//    poses.poses.push_back(pos);
//    std::cout<<"centroid: x="<<centroid[0]<<" y="<<centroid[1]<<" z="<<centroid[2]<<"\n";

    //**********************testing frustum culling (the voxel grid was used inside the frustum culling)******************
//    std::cout<<"size of original cloud : "<<originalCloud->points.size()<<"\n\n";
//    geometry_msgs::Pose pose;
//    geometry_msgs::PoseArray poses;
//    pcl::PointCloud<pcl::PointXYZ> pointcloud;
//    pose.position.x = 5;pose.position.y = -25;pose.position.z = 5.8;pose.orientation.x = -0.0328686;pose.orientation.y = 0.706342;pose.orientation.z = 0.706342;pose.orientation.w = 0.0328686;
//    poses.poses.push_back(pose);
//    OcclusionCulling obj(n,"etihad_nowheels_densed.pcd");
//    pointcloud = obj.extractVisibleSurface(pose);
//    frustumPointsPtr->points= obj.FrustumCloud->points;
//    visiblePointsPtr->points= obj.occlusionFreeCloud->points;
//    geometry_msgs::Point linePoint;
//    std::vector<geometry_msgs::Point> pointSegments;
//    for(int i =0 ; i<obj.fc.i_point_g.size(); i++){
//        linePoint.x = obj.fc.i_point_g.at(i)[0];
//        linePoint.y = obj.fc.i_point_g.at(i)[1];
//        linePoint.z = obj.fc.i_point_g.at(i)[2];
//        pointSegments.push_back(linePoint);
//     }
//    visualization_msgs::Marker pointsList = drawPoints(pointSegments,2,100000000);
//    std::vector<geometry_msgs::Point> lineSegments;
//    for(int i =0 ; i<obj.fc.bb_centroids_g.size(); i++){

//        if(i+1 != obj.fc.bb_centroids_g.size()){
//            linePoint.x = obj.fc.bb_centroids_g.at(i)[0];
//            linePoint.y = obj.fc.bb_centroids_g.at(i)[1];
//            linePoint.z = obj.fc.bb_centroids_g.at(i)[2];
//            lineSegments.push_back(linePoint);
//            linePoint.x = obj.fc.bb_centroids_g.at(i+1)[0];
//            linePoint.y = obj.fc.bb_centroids_g.at(i+1)[1];
//            linePoint.z = obj.fc.bb_centroids_g.at(i+1)[2];
//            lineSegments.push_back(linePoint);
//        }
//     }
//    linePoint.x = obj.fc.bb_centroids_g.at(0)[0];linePoint.y = obj.fc.bb_centroids_g.at(0)[1];linePoint.z = obj.fc.bb_centroids_g.at(0)[2];lineSegments.push_back(linePoint);
//    linePoint.x = obj.fc.bb_centroids_g.at(3)[0];linePoint.y = obj.fc.bb_centroids_g.at(3)[1];linePoint.z = obj.fc.bb_centroids_g.at(3)[2];lineSegments.push_back(linePoint);
//    linePoint.x = obj.fc.bb_centroids_g.at(0)[0];linePoint.y = obj.fc.bb_centroids_g.at(0)[1];linePoint.z = obj.fc.bb_centroids_g.at(0)[2];lineSegments.push_back(linePoint);
//    linePoint.x = obj.fc.bb_centroids_g.at(5)[0];linePoint.y = obj.fc.bb_centroids_g.at(5)[1];linePoint.z = obj.fc.bb_centroids_g.at(5)[2];lineSegments.push_back(linePoint);
//    linePoint.x = obj.fc.bb_centroids_g.at(1)[0];linePoint.y = obj.fc.bb_centroids_g.at(1)[1];linePoint.z = obj.fc.bb_centroids_g.at(1)[2];lineSegments.push_back(linePoint);
//    linePoint.x = obj.fc.bb_centroids_g.at(6)[0];linePoint.y = obj.fc.bb_centroids_g.at(6)[1];linePoint.z = obj.fc.bb_centroids_g.at(6)[2];lineSegments.push_back(linePoint);
//    linePoint.x = obj.fc.bb_centroids_g.at(2)[0];linePoint.y = obj.fc.bb_centroids_g.at(2)[1];linePoint.z = obj.fc.bb_centroids_g.at(2)[2];lineSegments.push_back(linePoint);
//    linePoint.x = obj.fc.bb_centroids_g.at(7)[0];linePoint.y = obj.fc.bb_centroids_g.at(7)[1];linePoint.z = obj.fc.bb_centroids_g.at(7)[2];lineSegments.push_back(linePoint);
//    linePoint.x = obj.fc.bb_centroids_g.at(7)[0];linePoint.y = obj.fc.bb_centroids_g.at(7)[1];linePoint.z = obj.fc.bb_centroids_g.at(7)[2];lineSegments.push_back(linePoint);
//    linePoint.x = obj.fc.bb_centroids_g.at(4)[0];linePoint.y = obj.fc.bb_centroids_g.at(4)[1];linePoint.z = obj.fc.bb_centroids_g.at(4)[2];lineSegments.push_back(linePoint);
//    visualization_msgs::Marker linesList = drawLines(lineSegments,1,0.1);
//    obj.visualizeFOV(pose);
//    std::vector<geometry_msgs::Point> lineSegments2;
//    for (int i =0 ; i<4; i++)
//    {
//        linePoint.x = obj.fc.origin[0];
//        linePoint.y = obj.fc.origin[1];
//        linePoint.z = obj.fc.origin[2];
//        lineSegments2.push_back(linePoint);
//        linePoint.x = obj.fc.far_pts_g[i][0];
//        linePoint.y = obj.fc.far_pts_g[i][1];
//        linePoint.z = obj.fc.far_pts_g[i][2];
//        lineSegments2.push_back(linePoint);

//    }
//    visualization_msgs::Marker linesList2 = drawLines(lineSegments2,3,0.2);

    //**********************testing voxelgrid*******************

    //    pcl::VoxelGridT voxelgrid;
    //    //    pcl::VoxelGridOcclusionEstimationT voxelgrid1;
    //    //    voxelgrid1.setInputCloud (originalCloud);
    //    //    voxelgrid1.setLeafSize (0.5f, 0.5f, 0.5f);
    //    //    voxelgrid1.initializeVoxelGrid();
    //    //    voxelgrid1.filter (*test);
    //    voxelgrid.setInputCloud (originalCloud);
    //    voxelgrid.setLeafSize (0.5f, 0.5f, 0.5f);
    //    voxelgrid.filter (*voxelsPtr);
    //    std::cout<<"voxel is created\n";
    //    std::cout<<"size: "<<voxelsPtr->points.size();
    //    std::cout<<"3";
    //    int s = voxelgrid.voxelSet[2][0][4].size();
    //    std::cout<<"point size : "<<s<<std::endl;
    //    for (int i =0; i<s; i++)
    //        voxelsPointsPtr->points.push_back(voxelgrid.voxelSet[2][0][4][i]);
    //    Eigen::Vector3i ijk(2,0,4);
    //    Eigen::Vector4f centroid = voxelgrid.getCentroidCoordinate(ijk);
    //    std::cout<<"centroid :"<<centroid[0]<<" "<<centroid[1]<<" "<<centroid[2]<<"\n";
    //    pose.position.x = centroid[0];pose.position.y = centroid[1];pose.position.z = centroid[2];pose.orientation.x = 0;pose.orientation.y = 0;pose.orientation.z = 0;pose.orientation.w = 1;
    //    poses.poses.push_back(pose);

    //**********************testing****************************
    //    ros::Publisher visible_pub = n.advertise<sensor_msgs::PointCloud2>("occlusion_free_cloud", 100);
    //    ros::Publisher original_pub = n.advertise<sensor_msgs::PointCloud2>("original_point_cloud", 10);
    //    ros::Publisher frustum_pub = n.advertise<sensor_msgs::PointCloud2>("frustum_point_cloud", 10);

    //    ros::Publisher pub1 = n.advertise<sensor_msgs::PointCloud2>("publish1", 10);
    //    ros::Publisher pub2 = n.advertise<sensor_msgs::PointCloud2>("publish2", 10);

    //    ros::Publisher vector_pub = n.advertise<geometry_msgs::PoseArray>("pose", 10);

    //    pcl::PointCloud<pcl::PointXYZ>::Ptr test (new pcl::PointCloud<pcl::PointXYZ>);
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr test1 (new pcl::PointCloud<pcl::PointXYZ>);
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr test2 (new pcl::PointCloud<pcl::PointXYZ>);

    //     pcl::PointCloud<pcl::PointXYZ> test3, test4, combined;
    //     geometry_msgs::PoseArray vec;
    //     OcclusionCulling obj(n,"etihad_nowheels_densed.pcd");

    //     geometry_msgs::Pose location;
    //    //example 1 from searchspace file
    //     double yaw = 2.3562;
    //     tf::Quaternion tf_q ;
    //     tf_q= tf::createQuaternionFromYaw(yaw);
    ////     4.5 -34.5 6 0 0 0.707107 0.707107
    ////     4.5 -31.5 7.445 0.476218 0.522701 0.476218 0.522701
    ////     4.5 -31.5 7.445 0.239939 0.665154 0.639997 0.300672
    ////     4.5 -31.5 8.945 0.239939 0.665154 0.639997 0.300672
    ////     4.5 -30 8.945 -0.0328686 0.706342 0.706342 0.0328686

    //     location.position.x=5.0; location.position.y=-28.0; location.position.z=5.8; location.orientation.x=-0.0328686; location.orientation.y=0.706342;location.orientation.z=0.706342;location.orientation.w=0.0328686;
    //     test3 = obj.extractVisibleSurface(location);
    //     test1->points = test3.points;
    //     vec.poses.push_back(location);
    ////     float covpercent1 = obj.calcCoveragePercent(test);
    ////     4.5 -34.5 5.945 0.476218 0.522701 0.476218 0.522701

    //     obj.visualizeFOV(location);
    //     //example 2 from searchspace file
    ////     OcclusionCulling obj1(n,"scaled_desktop.pcd");
    ////     location.position.x=4.0; location.position.y=29.0; location.position.z=10.5; location.orientation.x=tf_q.getX(); location.orientation.y=tf_q.getY();location.orientation.z=tf_q.getZ();location.orientation.w=tf_q.getW();
    ////     test4 = obj.extractVisibleSurface(location);
    ////     test2->points = test4.points;
    //////     vec.poses.push_back(location);

    ////     combined=test3;
    ////     combined+=test4;
    ////     test->points = combined.points;

    ////     float covpercent = obj.calcCoveragePercent(location);

    ////     float combinedcoverage = covpercent + covpercent1;

    ////     std::cout<<"coverage percentage: "<<combinedcoverage<<"%\n";
    ////     vec.poses.push_back(location);
    ros::Rate loop_rate(10);
    sensor_msgs::PointCloud2 cloud1,cloud2, cloud3,cloud4,cloud5;


    while (ros::ok())
    {
        //         std::cout<<"publishing\n";

//        pcl::toROSMsg(*originalCloud, cloud1); //cloud of original (white) using original cloud
//        cloud1.header.stamp = ros::Time::now();
//        cloud1.header.frame_id = "world"; //change according to the global frame please!!
//        originalCloudPub.publish(cloud1);

        //**********************testing frustum_culling (trial_testing.rviz)******************
        //        pcl::toROSMsg(*frustumPointsPtr, cloud4); //blue
        //        cloud4.header.stamp = ros::Time::now();
        //        cloud4.header.frame_id = "world";
        //        frustumPointsPub.publish(cloud4);

        //        pcl::toROSMsg(*visiblePointsPtr, cloud5); //red
        //        cloud5.header.stamp = ros::Time::now();
        //        cloud5.header.frame_id = "world";
        //        visiblePointsPub.publish(cloud5);

        //        poses.header.frame_id= "world";
        //        poses.header.stamp = ros::Time::now();
        //        PosePub.publish(poses);

        //        intersectionPointPub.publish(pointsList);
        //        voxelBoxPub.publish(linesList);
        //        linesPub.publish(linesList2);
        //**********************testing voxelgrid (trial_testing.rviz)******************
        //        pcl::toROSMsg(*voxelsPtr, cloud2); //blue
        //        cloud2.header.stamp = ros::Time::now();
        //        cloud2.header.frame_id = "world";
        //        voxelsPub.publish(cloud2);

        //        pcl::toROSMsg(*voxelsPointsPtr, cloud3); //red
        //        cloud3.header.stamp = ros::Time::now();
        //        cloud3.header.frame_id = "world";
        //        voxelPointsPub.publish(cloud3);

        //        poses.header.frame_id= "world";
        //        poses.header.stamp = ros::Time::now();
        //        PosePub.publish(poses);

        //**********************viewing the generated path(octomap_mapping.rviz)*********
        //         viewpoints.header.frame_id= "world";
        //         viewpoints.header.stamp = ros::Time::now();
        //         sensorPosePub.publish(viewpoints);
        //         points.header.frame_id= "world";
        //         points.header.stamp = ros::Time::now();
        //         PosePub.publish(points);
        //         generatedPathPub.publish(linesList);

        //**********************testing****************************
        //         std::cout<<"filtered original point cloud: "<<obj.filtered_cloud->size()<<"\n";
        //         sensor_msgs::PointCloud2 cloud1;
        //         pcl::toROSMsg(*obj.cloud, cloud1);
        //         cloud1.header.frame_id = "base_point_cloud";
        //         cloud1.header.stamp = ros::Time::now();
        //         original_pub.publish(cloud1);

        //         sensor_msgs::PointCloud2 cloud2;
        //         pcl::toROSMsg(*obj.occlusionFreeCloud, cloud2);
        //         cloud2.header.frame_id = "base_point_cloud";
        //         cloud2.header.stamp = ros::Time::now();
        //         visible_pub.publish(cloud2);

        //         sensor_msgs::PointCloud2 cloud3;
        //         pcl::toROSMsg(*test1, cloud3);
        //         cloud3.header.frame_id = "base_point_cloud";
        //         cloud3.header.stamp = ros::Time::now();
        //         pub1.publish(cloud3);
        //         sensor_msgs::PointCloud2 cloud4;
        //         pcl::toROSMsg(*test2, cloud4);
        //         cloud4.header.frame_id = "base_point_cloud";
        //         cloud4.header.stamp = ros::Time::now();
        //         pub2.publish(cloud4);

        //         sensor_msgs::PointCloud2 cloud5;
        //         pcl::toROSMsg(*obj.FrustumCloud, cloud5);
        //         cloud5.header.frame_id = "base_point_cloud";
        //         cloud5.header.stamp = ros::Time::now();
        //         frustum_pub.publish(cloud5);

        //         vec.header.frame_id= "base_point_cloud";
        //         vec.header.stamp = ros::Time::now();
        //         vector_pub.publish(vec);

        //         ROS_INFO("Publishing Marker");

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
visualization_msgs::Marker drawPoints(std::vector<geometry_msgs::Point> points, int c_color, int duration)
{
    visualization_msgs::Marker pointMarkerMsg;
    pointMarkerMsg.header.frame_id="world";
    pointMarkerMsg.header.stamp=ros::Time::now();
    pointMarkerMsg.ns="point_marker";
    pointMarkerMsg.id = 444444;
    pointMarkerMsg.type = visualization_msgs::Marker::POINTS;
    pointMarkerMsg.scale.x = 0.11;
    pointMarkerMsg.scale.y = 0.11;
    pointMarkerMsg.action  = visualization_msgs::Marker::ADD;
    pointMarkerMsg.lifetime  = ros::Duration(duration);
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
    std::vector<geometry_msgs::Point>::iterator pointsIterator;
    for(pointsIterator = points.begin();pointsIterator != points.end();pointsIterator++)
    {
        pointMarkerMsg.points.push_back(*pointsIterator);
        pointMarkerMsg.colors.push_back(color);
    }
   return pointMarkerMsg;
}
visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links, int c_color, float scale)
{
    visualization_msgs::Marker linksMarkerMsg;
    linksMarkerMsg.header.frame_id="world";
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
