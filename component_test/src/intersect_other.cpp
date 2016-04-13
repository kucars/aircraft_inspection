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
#include <Eigen/Eigen>

//PCL
#include <iostream>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/io/pcd_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/common/io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
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

// Triangle triangle intersection
#include <CGAL/intersections.h>
// THIS CANNOT BE INCLUDED IN THE SAME FILE AS <CGAL/intersections.h>
// #include <CGAL/Boolean_set_operations_2.h>

// Constrained Delaunay Triangulation types
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_plus_2.h>

// Axis-align boxes for all-pairs self-intersection detection
#include <CGAL/point_generators_3.h>
#include <CGAL/Bbox_3.h>
#include <CGAL/box_intersection_d.h>
#include <CGAL/function_objects.h>
#include <CGAL/Join_input_iterator.h>
#include <CGAL/algorithm.h>
#include <vector>

// Axis-aligned bounding box tree for tet tri intersection
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>

// Boolean operations
#include <CGAL/Polyhedron_3.h>
// Is this actually used?
//#include <CGAL/Nef_polyhedron_3.h>

// Delaunay Triangulation in 3D
#include <CGAL/Delaunay_triangulation_3.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Cartesian_converter.h>
using namespace fcl;

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel; //CGAL::Exact_predicates_exact_constructions_kernel//it could be also CGAL::Exact_predicates_inexact_constructions_kernel CHECK LATER
typedef CGAL::Simple_cartesian<double> K;
typedef CGAL::Point_3<Kernel>    Point_3;
typedef CGAL::Segment_3<Kernel>  Segment_3;
typedef CGAL::Triangle_3<Kernel> Triangle_3;
typedef CGAL::Plane_3<Kernel>    Plane_3;
typedef CGAL::Tetrahedron_3<Kernel> Tetrahedron_3;
typedef K::Point_3   Point3;
typedef CGAL::Cartesian_converter<Kernel,K > converter;

// Axis-align boxes for all-pairs self-intersection detection
typedef std::vector<Triangle_3> Triangles;
typedef typename Triangles::iterator TrianglesIterator;
typedef typename Triangles::const_iterator TrianglesConstIterator;
typedef CGAL::Box_intersection_d::Box_with_handle_d<double,3,TrianglesIterator> Box;

void loadOBJFile(const char* filename, std::vector<Vec3f>& points, std::vector<Triangle_3>& triangles);
void box_up(Triangles & T, std::vector<Box> & boxes);
void cb(const Box &a, const Box &b);
double calcMeshSurfaceArea(std::vector<Triangle_3> mesh);

Triangles TA,TB;
pcl::PolygonMesh m;
std::vector<int> lIF;
int counter=0;
int main(int argc, char **argv)
{

    ros::init(argc, argv, "interesect_other");
    ros::NodeHandle n;
    std::string path = ros::package::getPath("component_test");
    std::vector<Vec3f> pt1, pt2;

    std::string str1 = path+"/src/mesh/p11.obj";
    std::string str2 = path+"/src/mesh/p22.obj";

    loadOBJFile(str1.c_str(), pt1, TA);
    loadOBJFile(str2.c_str(), pt2, TB);

    std::vector<Box> A_boxes,B_boxes;
    box_up(TA,A_boxes);
    box_up(TB,B_boxes);

    CGAL::box_intersection_d(
      A_boxes.begin(), A_boxes.end(),
      B_boxes.begin(), B_boxes.end(),
      cb);

    std::set<int> s;
    std::cout<<"number of instersections before sorting: "<<lIF.size()<<"\n";
    for(int i=0; i<lIF.size();i++)
        s.insert(lIF[i]);
    lIF.assign( s.begin(), s.end() );
    std::sort( lIF.begin(), lIF.end() );
    std::cout<<"number of instersections after sorting: "<<lIF.size()<<"\n";
    std::vector<Triangle_3> inter;
    for(int j=0; j<lIF.size();j++)
    {
        inter.push_back(TB[lIF[j]]);
    }
    double areaIntB = calcMeshSurfaceArea(inter);
    double areaAllB = calcMeshSurfaceArea(TB);
    std::cout<<"the total area of B= "<<areaAllB<<" , and interesected triangles area= "<<areaIntB<<"\n";
    std::cout<<"number of triangles in B="<<TB.size()<<"\n";
    std::cout<<"counter "<<counter<<"\n";

    while (ros::ok())
    {


        ros::spinOnce();
//         loop_rate.sleep();
    }


    return 0;
}
void box_up(Triangles & T, std::vector<Box> & boxes)
{
    boxes.reserve(T.size());
    for (
         TrianglesIterator tit = T.begin();
         tit != T.end();
         ++tit)
    {
        boxes.push_back(Box(tit->bbox(), tit));
    }
}

void cb(const Box &a, const Box &b)
{
    using namespace std;
    // index in F and T
    int fa = a.handle()-TA.begin();
    int fb = b.handle()-TB.begin();
    const Triangle_3 & A = *a.handle();
    const Triangle_3 & B = *b.handle();
    if(CGAL::do_intersect(A,B))
    {
        counter++;

        // There was an intersection
        //        lIF.push_back(fa);
                lIF.push_back(fb);
    }

}
double calcMeshSurfaceArea(std::vector<Triangle_3> mesh)
{
    int s = mesh.size();
    double totalArea=0.0;
    converter to_simple;
    K::Point_3 convertedP;
    for (int i =0; i<s; i++){
        pcl::PointCloud<pcl::PointXYZ> temp_cloud;
        pcl::PointXYZ pt;
        convertedP = to_simple( mesh.at(i)[0] );
        pt.data[0]= convertedP.x();pt.data[1]=  convertedP.y();pt.data[2]=  convertedP.z();
        temp_cloud.points.push_back( pt );
        convertedP = to_simple( mesh.at(i)[1] );
        pt.data[0]= convertedP.x();pt.data[1]= convertedP.y();pt.data[2]=  convertedP.z();
        temp_cloud.points.push_back( pt );
        convertedP = to_simple( mesh.at(i)[2] );
        pt.data[0]= convertedP.x();pt.data[1]=  convertedP.y();pt.data[2]= convertedP.z();
        temp_cloud.points.push_back( pt );
        totalArea += pcl::calculatePolygonArea(temp_cloud);
    }
//    std::cout<<"total area "<<totalArea<<"\n";
    return totalArea;
}
void loadOBJFile(const char* filename, std::vector<Vec3f>& points, std::vector<Triangle_3>& triangles)
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
            Triangle_3 tri;
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
                    Point_3 p1(points[atoi(data[0]) - 1][0],points[atoi(data[0]) - 1][1],points[atoi(data[0]) - 1][2]);
                    Point_3 p2(points[atoi(data[1]) - 1][0],points[atoi(data[1]) - 1][1],points[atoi(data[1]) - 1][2]);
                    Point_3 p3(points[atoi(data[2]) - 1][0],points[atoi(data[2]) - 1][1],points[atoi(data[2]) - 1][2]);
                    tri = Triangle_3(p1,p2,p3);
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
                    Point_3 p1(points[indxs[0]][0],points[indxs[0]][1],points[indxs[0]][2]);
                    Point_3 p2(points[indxs[1]][0],points[indxs[1]][1],points[indxs[1]][2]);
                    Point_3 p3(points[indxs[2]][0],points[indxs[2]][1],points[indxs[2]][2]);
                    tri = Triangle_3(p1,p2,p3);
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



