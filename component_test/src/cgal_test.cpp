#include <iostream>
#include <list>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>

#include "ros/ros.h"
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/Marker.h>
//PCL
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/file_io.h>
#include <pcl/io/boost.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/io/vtk_lib_io.h>

#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <deque>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>

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

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include "fcl/config.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/shape/geometric_shape_to_BVH_model.h"
#include "fcl/math/transform.h"
#include "fcl_utility.h"
#include "fcl/BVH/BV_fitter.h"
#include "fcl/BV/BV.h"
#include "fcl/distance.h"

//VTK
//VTK
#include <vtkVersion.h>
#include <vtkSphereSource.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkHardwareSelector.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkObjectFactory.h>
#include <vtkRendererCollection.h>
#include <vtkDataSetMapper.h>
#include <vtkExtractSelection.h>
#include <vtkSelection.h>
#include <vtkProperty.h>
#include <vtkPLYReader.h>
#include <vtkPLYWriter.h>
#include <vtkSelectVisiblePoints.h>
#include <vtkPLYWriter.h>
#include "vtkCamera.h"
#include <vtkDelaunay3D.h>
#include <vtkUnstructuredGrid.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkXMLUnstructuredGridWriter.h>
#include <vtkCellArray.h>
#include <vtkDataSetSurfaceFilter.h>
#include <vtkXMLPolyDataReader.h>

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
using namespace fcl;
visualization_msgs::Marker drawCUBE(Vec3f vec , int id , int c_color);
void loadOBJFile(const char* filename, std::vector<Vec3f>& points, std::list<CGALTriangle>& triangles);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cgal_discretization");
    ros::NodeHandle n;
    ros::Publisher model_pub = n.advertise<sensor_msgs::PointCloud2>("point_cloud", 1);
    ros::Publisher point_pub = n.advertise<geometry_msgs::PoseArray>("voxel", 1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Publisher OctmapPub = n.advertise<octomap_msgs::Octomap>("LaserOctmap", 1);
    ros::Publisher visCubePub = n.advertise<visualization_msgs::MarkerArray>("filtered_poses", 1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    std::string path = ros::package::getPath("component_test");
    std::cout<<"PATH is:"<<path;
    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/sphere_densed.pcd", *cloud);
    geometry_msgs::PoseArray points;
    geometry_msgs::Pose pose;

    int x_space=4.0;//put half the length here (32)
    int y_space=4.0;//put half the length here (18)
    int z_space=4.0;
    double res=1.0;
    for (int z=(-1*z_space) ; z <=z_space; z+=res)//the length of the aircraft//2
    {

        for (int y=-1*(y_space) ; y<=y_space; y+=res)//the hight of the aircraft//-4
        {

            for (int x=-1*x_space ; x<=x_space; x+=res)//the width of the aircraft
            {
                pose.position.z=z;
                pose.position.y=y;
                pose.position.x=x;
                pose.orientation.x=0;pose.orientation.y=0;pose.orientation.z=0;pose.orientation.w=1;
                points.poses.push_back(pose);

            }
        }
    }
    visualization_msgs::MarkerArray marker_array ;
    visualization_msgs::Marker marker2 ;
    std::vector<Vec3f> p1;
    std::list<CGALTriangle> triangles;
    std::string str = path+"/src/mesh/sphere_densed.obj";
    loadOBJFile(str.c_str(), p1, triangles);
    // constructs AABB tree
    Tree tree(triangles.begin(),triangles.end());
    std::cout<<"traingles size:"<<triangles.size()<<"\n";
    int intersectionsCount=0;
    for (int j=0; j<points.poses.size();j++)
    {
        Point a(points.poses[j].position.x , points.poses[j].position.y  ,points.poses[j].position.z);
        // Some Random point in arbitrary orientation
        Point b(100.0, 10.0, 56.0);
        Ray ray_query(a,b);
        // counts #intersections
        intersectionsCount = tree.number_of_intersected_primitives(ray_query);
        std::cout << "intersections: "<<intersectionsCount<< " intersections(s) with ray query" << std::endl;
        
        // Inside if the number of intersections is odd
        if(intersectionsCount%2 != 1)
	{
	    // compute closest point and squared distance
	    Point closest_point = tree.closest_point(a);
	    std::cerr << "closest point is: " << closest_point << std::endl;
	    FT sqd = tree.squared_distance(a);
	    std::cout << "squared distance: " << sqd << std::endl;
	    if (sqd >=1 && sqd <= 4)
	    {
		Vec3f vec2(points.poses[j].position.x , points.poses[j].position.y  ,points.poses[j].position.z);
		marker2 = drawCUBE(vec2, j, 1) ;
		marker_array.markers.push_back(marker2);
	    }
        }
    }
    visCubePub.publish(marker_array);

    //    int publishCounter = 0;
    ros::Rate loop_rate(1);
    while(ros::ok())
    {
        //mesh points publish
        sensor_msgs::PointCloud2 cloud1;
        pcl::toROSMsg(*cloud, cloud1);
        cloud1.header.frame_id = "base_point_cloud";
        cloud1.header.stamp = ros::Time::now();
        model_pub.publish(cloud1);

        //visualize the points
        points.header.frame_id= "base_point_cloud";
        points.header.stamp = ros::Time::now();
        point_pub.publish(points);
        //        if(publishCounter++<2)
        //        {
        //            visCubePub.publish(marker_array);
        //        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return EXIT_SUCCESS;
}

visualization_msgs::Marker drawCUBE(Vec3f vec , int id , int c_color)
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

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
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
