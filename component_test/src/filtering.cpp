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
#include <frustum_culling.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image.h>
#include <voxel_grid_occlusion_estimation.h>
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

using namespace fcl;


typedef CGAL::Simple_cartesian<double> K;
typedef K::FT FT;
typedef K::Ray_3 Ray;
typedef K::Line_3 Line;
typedef K::Point_3 Point;
typedef K::Triangle_3 CGALTriangle;
typedef std::list<CGALTriangle>::iterator Iterator;
typedef CGAL::AABB_triangle_primitive<K, Iterator> Primitive;
typedef CGAL::AABB_traits<K, Primitive> AABB_triangle_traits;
typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;


void loadOBJFile(const char* filename, std::vector<Vec3f>& points, std::list<CGALTriangle>& triangles);
//geometry_msgs::Pose calcOrientation(Vec3f sensorP,Vec3f nearestP,geometry_msgs::Vector3& rpy);//previous option
visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links, int id, int c_color);
visualization_msgs::Marker drawCUBE(Vec3f vec , int id , int c_color, double size);
void computeOrientations(Vec3f pos,double deg, geometry_msgs::PoseArray& filtered_vectors);

int main(int argc, char **argv)
{

    ros::init(argc, argv, "filtering");
    ros::NodeHandle n;

    ros::Publisher pub1 = n.advertise<sensor_msgs::PointCloud2>("original_point_cloud", 100);
    ros::Publisher pub2 = n.advertise<sensor_msgs::PointCloud2>("frustum_point_cloud", 100);
    ros::Publisher oriented_point_pub = n.advertise<geometry_msgs::PoseArray>("oriented_poses", 1);
    ros::Publisher poses_pub = n.advertise<visualization_msgs::MarkerArray>("filtered_poses", 1);
    ros::Publisher cam_posespub = n.advertise<geometry_msgs::PoseArray>("cam_poses", 1);

    ros::Publisher lines_pub1 = n.advertise<visualization_msgs::Marker>("fov_far_near", 10);
    ros::Publisher lines_pub2 = n.advertise<visualization_msgs::Marker>("fov_top", 10);
    ros::Publisher lines_pub3 = n.advertise<visualization_msgs::Marker>("fov_bottom", 10);


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    std::string path = ros::package::getPath("component_test");
    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/etihad.pcd", *cloud);


    // 1: *******************discretization ***************************
    geometry_msgs::PoseArray points;
    geometry_msgs::Pose pose;

    int x_space=32;//half wingspan
    int y_space=43;//half the length
    int z_space=21;// the height
    float res=1.0;
    ros::Time disretization_begin = ros::Time::now();
    for (float z=(1*z_space) ; z > -2; z-=res)//the length of the aircraft
    {

        for (float y=-1*(y_space) ; y< y_space; y+=res)//the hight of the aircraft
        {

            for (float x=-1*x_space ; x< x_space; x+=res)//the width of the aircraft
            {
                pose.position.z=z;
                pose.position.y=y;
                pose.position.x=x;
                pose.orientation.x=0;pose.orientation.y=0;pose.orientation.z=0;pose.orientation.w=1;
                points.poses.push_back(pose);

            }
        }
    }

    ros::Time disretization_end = ros::Time::now();
    double elapsed =  disretization_end.toSec() - disretization_begin.toSec();
    std::cout<<"discretization duration (s) = "<<elapsed<<"\n";
    std::cout<<"discretized points size: "<<points.poses.size()<<"\n";



    // 2: *******************Filtering ***************************
    ofstream pointFile,cameraPointFile;
    std::string file_loc = path+"/src/txt/SearchSpaceUAV.txt";
    std::string file_loc1 = path+"/src/txt/SearchSpaceCam.txt";
    pointFile.open (file_loc.c_str());
    cameraPointFile.open (file_loc1.c_str());
    geometry_msgs::PoseArray filtered_vectors;
    visualization_msgs::MarkerArray marker_array ;
    visualization_msgs::Marker marker2 ;
    std::vector<Vec3f> pt1;
    std::list<CGALTriangle> triangles;
    int intersectionsCount=0;
    std::vector<geometry_msgs::Vector3> points_rpy;
    std::string str = path+"/src/mesh/etihad.obj";

    ros::Time filtering_begin = ros::Time::now();
    loadOBJFile(str.c_str(), pt1, triangles);
    // constructs AABB tree
    Tree tree(triangles.begin(),triangles.end());
    std::cout<<"traingles size:"<<triangles.size()<<"\n";

    for (int j=0; j<points.poses.size();j++)
        //    for (int j=100000; j<115000;j++) //working on part of the viewpoints
    {
        Point a(points.poses[j].position.x , points.poses[j].position.y  ,points.poses[j].position.z);
        // Some Random point in arbitrary orientation
        Point b(100.0, 10.0, 56.0);
        Ray ray_query(a,b);
        // counts #intersections
        intersectionsCount = tree.number_of_intersected_primitives(ray_query);
//        std::cout << "intersections: "<<intersectionsCount<< " intersections(s) with ray query" << std::endl;

        // Inside if the number of intersections is odd
        if(intersectionsCount%2 != 1)
        {
            // compute closest point and squared distance
//            Point closest_point = tree.closest_point(a);
//            std::cerr << "closest point is: " << closest_point << std::endl;
            FT sqd = tree.squared_distance(a);
//            std::cout << "squared distance: " << sqd << std::endl;
            if (sqd >=1 && sqd <= 4)
            {
                Vec3f vec2(points.poses[j].position.x , points.poses[j].position.y  ,points.poses[j].position.z);
                marker2 = drawCUBE(vec2, j, 3, 0.2);
                marker_array.markers.push_back(marker2);
//                Vec3f nearestP;
//                nearestP[0]= closest_point.x(); nearestP[1]= closest_point.y(); nearestP[2]= closest_point.z();
                Vec3f position = vec2;
//                geometry_msgs::Pose out_vector = calcOrientation(position,nearestP,rpy) ;//the previous way!
                computeOrientations(position,45,filtered_vectors);
//                filtered_vectors.poses.push_back(out_vector);
//                points_rpy.push_back(rpy);
            }
        }
    }
    ros::Time filtering_end = ros::Time::now();
    elapsed =  filtering_end.toSec() - filtering_begin.toSec();
    std::cout<<"filtering duration (s) = "<<elapsed<<"\n";
    std::cout<<"filtered points size = "<<filtered_vectors.poses.size()<<"\n";


    //3: *************transformation of uav point to cam point***********************

    // used frustum culling to visualize the FOV
    //    pcl::FrustumCullingTT fc (true);
    //    fc.setInputCloud (cloud);
    //    fc.setVerticalFOV (45);
    //    fc.setHorizontalFOV (58);
    //    fc.setNearPlaneDistance (0.5);
    //    fc.setFarPlaneDistance (6.8);
    //    visualization_msgs::Marker linesList1;
    //    visualization_msgs::Marker linesList2;
    //    visualization_msgs::Marker linesList3;
    //    pcl::PointCloud <pcl::PointXYZ>::Ptr output (new pcl::PointCloud <pcl::PointXYZ>);

    geometry_msgs::PoseArray camPoses;

    //    for(int j=10; j< 11; j++)// used to test small set of points
    for (int j=0; j< filtered_vectors.poses.size(); j++)
    {
        Eigen::Matrix4d uav_pose, uav2cam, cam_pose;
        //UAV matrix pose
        Eigen::Matrix3d R; Eigen::Vector3d T1(filtered_vectors.poses[j].position.x,filtered_vectors.poses[j].position.y,filtered_vectors.poses[j].position.z);
        tf::Quaternion qt(filtered_vectors.poses[j].orientation.x,filtered_vectors.poses[j].orientation.y,filtered_vectors.poses[j].orientation.z,filtered_vectors.poses[j].orientation.w);
        //write to file uav locations
        pointFile << filtered_vectors.poses[j].position.x<<" "<<filtered_vectors.poses[j].position.y<<" "<<filtered_vectors.poses[j].position.z<<" "<<filtered_vectors.poses[j].orientation.x<<" "<<filtered_vectors.poses[j].orientation.y<<" "<<filtered_vectors.poses[j].orientation.z<<" "<<filtered_vectors.poses[j].orientation.w<<"\n";
        tf::Matrix3x3 R1(qt);
        tf::matrixTFToEigen(R1,R);
        uav_pose.setZero ();
        uav_pose.block (0, 0, 3, 3) = R;
        uav_pose.block (0, 3, 3, 1) = T1;
        uav_pose (3, 3) = 1;

        //transformation matrix
        qt = tf::createQuaternionFromRPY(0,0.2,0);
        tf::Matrix3x3 R2(qt);Eigen::Vector3d T2(0,0,-0.1);
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
        camPoses.poses.push_back(p);
        //write to file camera locations
        cameraPointFile << camPoses.poses[j].position.x<<" "<<camPoses.poses[j].position.y<<" "<<camPoses.poses[j].position.z<<" "<<camPoses.poses[j].orientation.x<<" "<<camPoses.poses[j].orientation.y<<" "<<camPoses.poses[j].orientation.z<<" "<<camPoses.poses[j].orientation.w<<"\n";

        //        Eigen::Matrix4f cam_pose_newf = cam_pose_new.cast<float>();
        //        fc.setCameraPose (cam_pose_newf);
        //        fc.filter (*output);

        //*** visualization the FOV *****

        //        std::vector<geometry_msgs::Point> fov_points;
        //        geometry_msgs::Point point1;
        //        point1.x=fc.fp_bl[0];point1.y=fc.fp_bl[1];point1.z=fc.fp_bl[2]; fov_points.push_back(point1);//0
        //        point1.x=fc.fp_br[0];point1.y=fc.fp_br[1];point1.z=fc.fp_br[2]; fov_points.push_back(point1);//1
        //        point1.x=fc.fp_tr[0];point1.y=fc.fp_tr[1];point1.z=fc.fp_tr[2]; fov_points.push_back(point1);//2
        //        point1.x=fc.fp_tl[0];point1.y=fc.fp_tl[1];point1.z=fc.fp_tl[2]; fov_points.push_back(point1);//3
        //        point1.x=fc.np_bl[0];point1.y=fc.np_bl[1];point1.z=fc.np_bl[2]; fov_points.push_back(point1);//4
        //        point1.x=fc.np_br[0];point1.y=fc.np_br[1];point1.z=fc.np_br[2]; fov_points.push_back(point1);//5
        //        point1.x=fc.np_tr[0];point1.y=fc.np_tr[1];point1.z=fc.np_tr[2]; fov_points.push_back(point1);//6
        //        point1.x=fc.np_tl[0];point1.y=fc.np_tl[1];point1.z=fc.np_tl[2]; fov_points.push_back(point1);//7

        //        std::vector<geometry_msgs::Point> fov_linesNearFar;
        //        fov_linesNearFar.push_back(fov_points[0]);fov_linesNearFar.push_back(fov_points[1]);
        //        fov_linesNearFar.push_back(fov_points[1]);fov_linesNearFar.push_back(fov_points[2]);
        //        fov_linesNearFar.push_back(fov_points[2]);fov_linesNearFar.push_back(fov_points[3]);
        //        fov_linesNearFar.push_back(fov_points[3]);fov_linesNearFar.push_back(fov_points[0]);

        //        fov_linesNearFar.push_back(fov_points[4]);fov_linesNearFar.push_back(fov_points[5]);
        //        fov_linesNearFar.push_back(fov_points[5]);fov_linesNearFar.push_back(fov_points[6]);
        //        fov_linesNearFar.push_back(fov_points[6]);fov_linesNearFar.push_back(fov_points[7]);
        //        fov_linesNearFar.push_back(fov_points[7]);fov_linesNearFar.push_back(fov_points[4]);
        //        linesList1 = drawLines(fov_linesNearFar,3333,1);//red

        //        std::vector<geometry_msgs::Point> fov_linestop;
        //        fov_linestop.push_back(fov_points[7]);fov_linestop.push_back(fov_points[3]);//top
        //        fov_linestop.push_back(fov_points[6]);fov_linestop.push_back(fov_points[2]);//top
        //        linesList2 = drawLines(fov_linestop,4444,2);//green

        //        std::vector<geometry_msgs::Point> fov_linesbottom;
        //        fov_linesbottom.push_back(fov_points[5]);fov_linesbottom.push_back(fov_points[1]);//bottom
        //        fov_linesbottom.push_back(fov_points[4]);fov_linesbottom.push_back(fov_points[0]);//bottom
        //        linesList3 = drawLines(fov_linesbottom,5555,3);//blue

    }

    pointFile.close();
    cameraPointFile.close();
    ros::Rate loop_rate(100);//it was 10

    while (ros::ok())
    {


        //        //***original cloud & frustum cull & occlusion cull publish***
        sensor_msgs::PointCloud2 cloud1;
        //        sensor_msgs::PointCloud2 cloud2;

        pcl::toROSMsg(*cloud, cloud1); //cloud of original (white) using original cloud
        //        pcl::toROSMsg(*output, cloud2); geometry_msgs::PoseArray out =//cloud of frustum cull (red) using pcl::frustum cull

        cloud1.header.frame_id = "base_point_cloud";
        //        cloud2.header.frame_id = "base_point_cloud";

        cloud1.header.stamp = ros::Time::now();
        //        cloud2.header.stamp = ros::Time::now();

        pub1.publish(cloud1);
        //        pub2.publish(cloud2);

        //visualize the filtered samples and their orientation
        filtered_vectors.header.frame_id= "base_point_cloud";
        filtered_vectors.header.stamp = ros::Time::now();
        oriented_point_pub.publish(filtered_vectors);

        camPoses.header.frame_id= "base_point_cloud";
        camPoses.header.stamp = ros::Time::now();
        cam_posespub.publish(camPoses);

        poses_pub.publish(marker_array);

        //visualize the FOV
        //        lines_pub1.publish(linesList1);
        //        lines_pub2.publish(linesList2);
        //        lines_pub3.publish(linesList3);
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

visualization_msgs::Marker drawCUBE(Vec3f vec , int id , int c_color, double size)
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

void computeOrientations(Vec3f pos,double deg, geometry_msgs::PoseArray& filtered_vectors)
{

    int orientationsNum= 360/deg;
    double yaw=0.0;//radians
    geometry_msgs::PoseArray output_vector;
    geometry_msgs::Pose vec;
    tf::Quaternion tf_q ;

    vec.position.x = pos[0];vec.position.y = pos[1];vec.position.z = pos[2];
    for(int i=0; i<orientationsNum;i++)
    {
        tf_q= tf::createQuaternionFromRPY(0.0, 0.0, yaw);
        vec.orientation.x = tf_q.getX();
        vec.orientation.y = tf_q.getY();
        vec.orientation.z = tf_q.getZ();
        vec.orientation.w = tf_q.getW();
        std::cout<<"orientation: "<<vec.orientation.x<<" "<<vec.orientation.y<<" "<<vec.orientation.z<<" "<<vec.orientation.w<<"\n";
        filtered_vectors.poses.push_back(vec);
        yaw = yaw+(deg*M_PI/180);
    }

    //    return output_vector;

}



//Orientation between the viewpoint and the closest point of the mesh
//geometry_msgs::Pose calcOrientation(Vec3f sensorP,Vec3f nearestP, geometry_msgs::Vector3& rpy)
//{
//    geometry_msgs::Pose output_vector;
//    Eigen::Quaterniond q;

//    Eigen::Vector3d axis_vector;
//    axis_vector[0]=nearestP[0]-sensorP[0];
//    axis_vector[1]=nearestP[1]-sensorP[1];
//    axis_vector[2]=nearestP[2]-sensorP[2];
//    axis_vector.normalize();
//    Eigen::Vector3d up_vector(0.0, 0.0, 1.0);
//    Eigen::Vector3d right_axis_vector = axis_vector.cross(up_vector);
//    right_axis_vector.normalized();
//    double theta = axis_vector.dot(up_vector);
//    double angle_rotation = -1.0*acos(theta);
//    //-------------------------------------------
//    // Method 1 - TF - works
//    //Convert to TF
//    tf::Vector3 tf_right_axis_vector;
//    tf::vectorEigenToTF(right_axis_vector, tf_right_axis_vector);
//    // Create quaternion
//    tf::Quaternion tf_q(tf_right_axis_vector, angle_rotation);
//    // Convert back to Eigen
//    tf::quaternionTFToEigen(tf_q, q);

//    // Rotate so that vector points along line
//    Eigen::Affine3d pose1;
//    q.normalize();
//    Eigen::Vector3d b;
//    b[0]=sensorP[0]; b[1]=sensorP[1]; b[2]=sensorP[2];
//    pose1 = q * Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitY());
//    pose1.translation() = b;
//    tf::poseEigenToMsg(pose1, output_vector);

//    tf::Quaternion qt;
//    qt.setX(output_vector.orientation.x);
//    qt.setY(output_vector.orientation.y);
//    qt.setZ(output_vector.orientation.z);
//    qt.setW(output_vector.orientation.w);
//    double roll, pitch, yaw;
////    tf::Matrix3x3(qt).getRPY(roll, pitch, yaw);//gets the Roll Pitch Yaw in terms a fixed axis (can't be used by the AngleAxis function of Eigen)
//    tf::Matrix3x3(qt).getEulerZYX(yaw, pitch, roll);//gets the yaw around z then pitch around new y then roll around new x
//    rpy.x = roll;
//    rpy.y = pitch;
//    rpy.z = yaw;



////    std::cout<<"roll: "<<rpy.x<<" pitch: "<<rpy.y<<" yaw: "<<rpy.z<<std::endl;

//    return output_vector;

//}

void loadOBJFile(const char* filename, std::vector<Vec3f>& points, std::list<CGALTriangle>& triangles)
{

    FILE* file = fopen(filename, "rb");
    if(!file)
    {
        std::cerr << "file not exist" << std::endl;
        return;
    }

    bool has_normal = false;
    bool has_texture = false;
    char line_buffer[2000];
    while(fgets(line_buffer, 2000, file))
    {
        char* first_token = strtok(line_buffer, "\r\n\t ");
        if(!first_token || first_token[0] == '#' || first_token[0] == 0)
            continue;

        switch(first_token[0])
        {
        case 'v':
        {
            if(first_token[1] == 'n')
            {
                strtok(NULL, "\t ");
                strtok(NULL, "\t ");
                strtok(NULL, "\t ");
                has_normal = true;
            }
            else if(first_token[1] == 't')
            {
                strtok(NULL, "\t ");
                strtok(NULL, "\t ");
                has_texture = true;
            }
            else
            {
                FCL_REAL x = (FCL_REAL)atof(strtok(NULL, "\t "));
                FCL_REAL y = (FCL_REAL)atof(strtok(NULL, "\t "));
                FCL_REAL z = (FCL_REAL)atof(strtok(NULL, "\t "));
                Vec3f p(x, y, z);
                points.push_back(p);
            }
        }
            break;
        case 'f':
        {
            CGALTriangle tri;
            char* data[30];
            int n = 0;
            while((data[n] = strtok(NULL, "\t \r\n")) != NULL)
            {
                if(strlen(data[n]))
                    n++;
            }

            for(int t = 0; t < (n - 2); ++t)
            {
                if((!has_texture) && (!has_normal))
                {
                    Point p1(points[atoi(data[0]) - 1][0],points[atoi(data[0]) - 1][1],points[atoi(data[0]) - 1][2]);
                    Point p2(points[atoi(data[1]) - 1][0],points[atoi(data[1]) - 1][1],points[atoi(data[1]) - 1][2]);
                    Point p3(points[atoi(data[2]) - 1][0],points[atoi(data[2]) - 1][1],points[atoi(data[2]) - 1][2]);
                    tri = CGALTriangle(p1,p2,p3);
                    //std::cout<<"1: Yep, I get here p1:"<<atoi(data[0]) - 1<<" p2:"<<atoi(data[1]) - 1<<" p2:"<<atoi(data[2]) - 1;
                    if(!CGAL::collinear(p1,p2,p3))
                    {
                        triangles.push_back(tri);
                    }
                }
                else
                {
                    const char *v1;
                    uint indxs[3];
                    for(int i = 0; i < 3; i++)
                    {
                        // vertex ID
                        if(i == 0)
                            v1 = data[0];
                        else
                            v1 = data[t + i];

                        indxs[i] = atoi(v1) - 1;
                    }
                    Point p1(points[indxs[0]][0],points[indxs[0]][1],points[indxs[0]][2]);
                    Point p2(points[indxs[1]][0],points[indxs[1]][1],points[indxs[1]][2]);
                    Point p3(points[indxs[2]][0],points[indxs[2]][1],points[indxs[2]][2]);
                    tri = CGALTriangle(p1,p2,p3);
                    if(!CGAL::collinear(p1,p2,p3))
                    {
                        triangles.push_back(tri);
                    }
                }
            }
        }
        }
    }
}
