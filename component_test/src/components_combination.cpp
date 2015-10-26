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
//PCL
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/frustum_culling.h>
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
//FCL
#include "fcl/shape/geometric_shapes.h"
#include "fcl/narrowphase/narrowphase.h"
#include "fcl/collision.h"
#include "fcl/ccd/motion.h"
#include <stdlib.h>
#include <boost/foreach.hpp>
#include <Eigen/Eigen>
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/shape/geometric_shape_to_BVH_model.h"
#include "fcl/math/transform.h"
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>
#include "fcl/BV/AABB.h"
#include "fcl/collision_object.h"

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
geometry_msgs::Pose calcOrienation(Vec3f sensorP,Vec3f nearestP,geometry_msgs::Vector3 rpy);
visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links, int id, int c_color);
visualization_msgs::Marker drawCUBE(Vec3f vec , int id , int c_color, double size);

int main(int argc, char **argv)
{

    ros::init(argc, argv, "occlusion_culling_test");
    ros::NodeHandle n;

    ros::Publisher pub1 = n.advertise<sensor_msgs::PointCloud2>("original_point_cloud", 100);
    ros::Publisher pub2 = n.advertise<sensor_msgs::PointCloud2>("frustum_point_cloud", 100);
    ros::Publisher pub3 = n.advertise<sensor_msgs::PointCloud2>("occlusion_free_cloud", 100);
    ros::Publisher oriented_point_pub = n.advertise<geometry_msgs::PoseArray>("oriented_poses", 1);
    ros::Publisher poses_pub = n.advertise<visualization_msgs::MarkerArray>("filtered_poses", 1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("sensor_origin", 1);


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr occlusionFreeCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rayCloud(new pcl::PointCloud<pcl::PointXYZRGB>);



    std::string path = ros::package::getPath("component_test");
    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/scaled_desktop.pcd", *cloud);


    // 1: *******************discretization ***************************
    geometry_msgs::PoseArray points;
    geometry_msgs::Pose pose;

    int x_space=16;//put half the length here (32)
    int y_space=11;//put half the length here (18)
    int z_space=37;
    float res=1;
    ros::Time disretization_begin = ros::Time::now();
    for (float z=(-1*z_space) ; z < 2; z+=res)//the length of the aircraft
    {

        for (float y=-1*(y_space-4) ; y< y_space; y+=res)//the hight of the aircraft
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



    // 2: *******************Filtering ***************************
    geometry_msgs::PoseArray filtered_vectors;
    visualization_msgs::MarkerArray marker_array ;
    visualization_msgs::Marker marker2 ;
    std::vector<Vec3f> pt1;
    std::list<CGALTriangle> triangles;
    int intersectionsCount=0;
    std::vector<geometry_msgs::Vector3> points_rpy;
    std::string str = path+"/src/mesh/desktop_scaleddown.obj";

    ros::Time filtering_begin = ros::Time::now();
    loadOBJFile(str.c_str(), pt1, triangles);
    // constructs AABB tree
    Tree tree(triangles.begin(),triangles.end());
    std::cout<<"traingles size:"<<triangles.size()<<"\n";

//    for (int j=0; j<points.poses.size();j++)
    for (int j=0; j<5000;j++) //working on part of the viewpoints
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
            Point closest_point = tree.closest_point(a);
//            std::cerr << "closest point is: " << closest_point << std::endl;
            FT sqd = tree.squared_distance(a);
//            std::cout << "squared distance: " << sqd << std::endl;
            if (sqd >=1 && sqd <= 4)
            {
                Vec3f vec2(points.poses[j].position.x , points.poses[j].position.y  ,points.poses[j].position.z);
                marker2 = drawCUBE(vec2, j, 1, 0.3);
                marker_array.markers.push_back(marker2);
                Vec3f nearestP;
                nearestP[0]= closest_point.x(); nearestP[1]= closest_point.y(); nearestP[2]= closest_point.z();
                Vec3f position = vec2;
                geometry_msgs::Vector3 rpy;
                filtered_vectors.poses.push_back(calcOrienation(position,nearestP,rpy)); //rpy are in radians already!
                points_rpy.push_back(rpy);
            }
        }
    }
    ros::Time filtering_end = ros::Time::now();
    elapsed =  filtering_end.toSec() - filtering_begin.toSec();
    std::cout<<"filtering duration (s) = "<<elapsed<<"\n";



    // 3: *******************Occlusion Culling ***************************
    pcl::FrustumCulling<pcl::PointXYZ> fc (true);

    for (int num=0; num < filtered_vectors.poses.size(); num++)
    {
        // 3.1: *****Frustum Culling*******
        pcl::PointCloud <pcl::PointXYZ>::Ptr output (new pcl::PointCloud <pcl::PointXYZ>);

        fc.setInputCloud (cloud);
        fc.setVerticalFOV (45);
        fc.setHorizontalFOV (58);
        fc.setNearPlaneDistance (0.8);
        fc.setFarPlaneDistance (5.8);

        Eigen::Matrix4f camera_pose;
        Eigen::Matrix3f R;
        camera_pose.setZero ();
        Eigen::Vector3f theta_rad(points_rpy[num].x,points_rpy[num].y,points_rpy[num].z);
        R = Eigen::AngleAxisf (theta_rad[0], Eigen::Vector3f::UnitX ()) *
                Eigen::AngleAxisf (theta_rad[1], Eigen::Vector3f::UnitY ()) *
                Eigen::AngleAxisf (theta_rad[2], Eigen::Vector3f::UnitZ ());

        camera_pose.block (0, 0, 3, 3) = R;
        Eigen::Vector3f T;
        T (0) = filtered_vectors.poses[num].position.x; T (1) = filtered_vectors.poses[num].position.y; T (2) = filtered_vectors.poses[num].position.z;
        camera_pose.block (0, 3, 3, 1) = T;
        camera_pose (3, 3) = 1;
        fc.setCameraPose (camera_pose);
        fc.filter (*output);

        //*** Visualization Camera View Vector ****
        // the rviz axis is different from the frustum camera axis
        R = Eigen::AngleAxisf (theta_rad[0] , Eigen::Vector3f::UnitX ()) *
                Eigen::AngleAxisf (-theta_rad[1] , Eigen::Vector3f::UnitY ()) *
                Eigen::AngleAxisf (-theta_rad[2] , Eigen::Vector3f::UnitZ ());
        tf::Matrix3x3 rotation;
        Eigen::Matrix3d D;
        D= R.cast<double>();
        tf::matrixEigenToTF(D,rotation);
        rotation = rotation.transpose();
        tf::Quaternion orientation;
        rotation.getRotation(orientation);

        geometry_msgs::Pose output_vector;
        Eigen::Quaterniond q;
        geometry_msgs::Quaternion quet;
        tf::quaternionTFToEigen(orientation, q);
        tf::quaternionTFToMsg(orientation,quet);
        Eigen::Affine3d pose1;
        Eigen::Vector3d a;
        a[0]= T[0];
        a[1]= T[1];
        a[2]= T[2];
        pose1.translation() = a;
        tf::poseEigenToMsg(pose1, output_vector);
//        visualization_msgs::Marker marker;


        //3.2:****voxel grid occlusion estimation *****
        Eigen::Quaternionf quat(q.w(),q.x(),q.y(),q.z());
        output->sensor_origin_  = Eigen::Vector4f(a[0],a[1],a[2],0);
        output->sensor_orientation_= quat;
        pcl::VoxelGridOcclusionEstimationT voxelFilter;
        voxelFilter.setInputCloud (output);
        //voxelFilter.setLeafSize (0.03279f, 0.03279f, 0.03279f);
        voxelFilter.setLeafSize (0.04f, 0.04f, 0.04f);
        //voxelFilter.filter(*cloud); // This filter doesn't really work, it introduces points inside the sphere !
        voxelFilter.initializeVoxelGrid();

        int state,ret;

        pcl::PointXYZ pt,p1,p2;
        pcl::PointXYZRGB point;
        std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > out_ray;
        std::vector<geometry_msgs::Point> lineSegments;
        geometry_msgs::Point linePoint;
        Eigen::Vector3i  min_b = voxelFilter.getMinBoxCoordinates ();
        Eigen::Vector3i  max_b = voxelFilter.getMaxBoxCoordinates ();

        bool foundBug = false;
        // iterate over the entire voxel grid
        for (int kk = min_b.z (); kk <= max_b.z () && !foundBug; ++kk)
        {
            for (int jj = min_b.y (); jj <= max_b.y () && !foundBug; ++jj)
            {
                for (int ii = min_b.x (); ii <= max_b.x () && !foundBug; ++ii)
                {
                    Eigen::Vector3i ijk (ii, jj, kk);
                    // process all free voxels
                    int index = voxelFilter.getCentroidIndexAt (ijk);
                    Eigen::Vector4f centroid = voxelFilter.getCentroidCoordinate (ijk);
                    point = pcl::PointXYZRGB(0,244,0);
                    point.x = centroid[0];
                    point.y = centroid[1];
                    point.z = centroid[2];

                    if(index!=-1 )
                    {
                        out_ray.clear();
                        ret = voxelFilter.occlusionEstimation( state,out_ray, ijk);
//                        std::cout<<"State is:"<<state<<"\n";

                        if(state != 1)
                        {
                            // estimate direction to target voxel
                            Eigen::Vector4f direction = centroid - cloud->sensor_origin_;
                            direction.normalize ();
                            // estimate entry point into the voxel grid
                            float tmin = voxelFilter.rayBoxIntersection (cloud->sensor_origin_, direction,p1,p2);
                            if(tmin!=-1)
                            {
                                // coordinate of the boundary of the voxel grid
                                Eigen::Vector4f start = cloud->sensor_origin_ + tmin * direction;
                                linePoint.x = cloud->sensor_origin_[0]; linePoint.y = cloud->sensor_origin_[1]; linePoint.z = cloud->sensor_origin_[2];
                                //std::cout<<"Box Min X:"<<linePoint.x<<" y:"<< linePoint.y<<" z:"<< linePoint.z<<"\n";
                                lineSegments.push_back(linePoint);

                                linePoint.x = start[0]; linePoint.y = start[1]; linePoint.z = start[2];
                                //std::cout<<"Box Max X:"<<linePoint.x<<" y:"<< linePoint.y<<" z:"<< linePoint.z<<"\n";
                                lineSegments.push_back(linePoint);

                                linePoint.x = start[0]; linePoint.y = start[1]; linePoint.z = start[2];
                                //std::cout<<"Box Max X:"<<linePoint.x<<" y:"<< linePoint.y<<" z:"<< linePoint.z<<"\n";
                                lineSegments.push_back(linePoint);

                                linePoint.x = centroid[0]; linePoint.y = centroid[1]; linePoint.z = centroid[2];
                                //std::cout<<"Box Max X:"<<linePoint.x<<" y:"<< linePoint.y<<" z:"<< linePoint.z<<"\n";
                                lineSegments.push_back(linePoint);

                                rayCloud->points.push_back(point);
                                pt.x = centroid[0];
                                pt.y = centroid[1];
                                pt.z = centroid[2];
                                occlusionFreeCloud->points.push_back(pt);
                            }
                        }
                    }
                }
            }
        }
    }






    //4: *****************Rviz Visualization ************
    int publishCounter = 0;
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        //***marker publishing***
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
//        // ROS_INFO("Publishing Marker");
//        // Set the frame ID and timestamp. See the TF tutorials for information on these.
//        marker.pose =  output_vector;
//        marker.pose.orientation  = quet;//output_vector.orientation;
//        marker.header.frame_id = "base_point_cloud";
//        marker.header.stamp = ros::Time::now();
//        marker.lifetime = ros::Duration(10);
//        // Publish the marker
//        marker_pub.publish(marker);

//        //***frustum cull and occlusion cull publish***
        sensor_msgs::PointCloud2 cloud1;
//        sensor_msgs::PointCloud2 cloud2;
        sensor_msgs::PointCloud2 cloud3;

        pcl::toROSMsg(*cloud, cloud1); //cloud of original (white) using pcl::frustumcull
//        pcl::toROSMsg(*output, cloud2); //cloud of frustum cull (red) using pcl::frustumcull
        pcl::toROSMsg(*occlusionFreeCloud, cloud3); //cloud of the Occlusion cull (blue) using pcl::rangeImage
        cloud1.header.frame_id = "base_point_cloud";
//        cloud2.header.frame_id = "base_point_cloud";
        cloud3.header.frame_id = "base_point_cloud";

        cloud1.header.stamp = ros::Time::now();
//        cloud2.header.stamp = ros::Time::now();
        cloud3.header.stamp = ros::Time::now();

        pub1.publish(cloud1);
//        pub2.publish(cloud2);
        pub3.publish(cloud3);

        //visualize the filtered samples and their orientation
        filtered_vectors.header.frame_id= "base_point_cloud";
        filtered_vectors.header.stamp = ros::Time::now();
        oriented_point_pub.publish(filtered_vectors);

        if(publishCounter++<10)
        {
            poses_pub.publish(marker_array);
        }
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
    linksMarkerMsg.scale.x = 0.001;
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



geometry_msgs::Pose calcOrienation(Vec3f sensorP,Vec3f nearestP, geometry_msgs::Vector3 rpy)
{
    geometry_msgs::Pose output_vector;
    Eigen::Quaterniond q;

    Eigen::Vector3d axis_vector;
    axis_vector[0]=sensorP[0]-nearestP[0];
    axis_vector[1]=sensorP[1]-nearestP[1];
    axis_vector[2]=sensorP[2]-nearestP[2];
//    axis_vector[0]=nearestP[0]-sensorP[0];
//    axis_vector[1]=nearestP[1]-sensorP[1];
//    axis_vector[2]=nearestP[2]-sensorP[2];
    axis_vector.normalize();
    Eigen::Vector3d up_vector(0.0, 0.0, -1.0);
    Eigen::Vector3d right_axis_vector = axis_vector.cross(up_vector);
    right_axis_vector.normalized();
    double theta = axis_vector.dot(up_vector);
    double angle_rotation = -1.0*acos(theta);
    //-------------------------------------------
    // Method 1 - TF - works
    //Convert to TF
    tf::Vector3 tf_right_axis_vector;
    tf::vectorEigenToTF(right_axis_vector, tf_right_axis_vector);
    // Create quaternion
    tf::Quaternion tf_q(tf_right_axis_vector, angle_rotation);
    // Convert back to Eigen
    tf::quaternionTFToEigen(tf_q, q);

    // Rotate so that vector points along line
    Eigen::Affine3d pose1;
    q.normalize();
    Eigen::Vector3d b;
    b[0]=sensorP[0]; b[1]=sensorP[1]; b[2]=sensorP[2];
    pose1 = q * Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitY());
    pose1.translation() = b;
    tf::poseEigenToMsg(pose1, output_vector);

    double roll, pitch, yaw;
    tf::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;

    return output_vector;

}

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
