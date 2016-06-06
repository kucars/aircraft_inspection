#define CGAL_EIGEN3_ENABLED


#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_3.h>
#include <CGAL/point_generators_d.h>
#include <CGAL/algorithm.h>
#include <CGAL/assertions.h>
#include "ros/ros.h"
#include <ros/package.h>
#include <iostream>
#include <fstream>
#include <list>
#include <set>
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
#include "fcl/config.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/shape/geometric_shape_to_BVH_model.h"
#include "fcl/math/transform.h"
#include "fcl_utility.h"
#include "fcl/BVH/BV_fitter.h"
#include "fcl/BV/BV.h"
#include "fcl/distance.h"

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
#include <CGAL/Cartesian_converter.h>
#include <iostream>
#include <fstream>
#include <cassert>
#include <list>
#include <vector>
#include <utility> // defines std::pair
#include <cstdlib>

#include <CGAL/trace.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/Surface_mesh_default_triangulation_3.h>
#include <CGAL/make_surface_mesh.h>
#include <CGAL/Implicit_surface_3.h>
#include <CGAL/IO/output_surface_facets_to_polyhedron.h>
#include <CGAL/Poisson_reconstruction_function.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/compute_average_spacing.h>

#include <CGAL/Memory_sizer.h>
#include <CGAL/Timer.h>


#include <CGAL/pca_estimate_normals.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_off_points.h>
#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/IO/write_xyz_points.h>
#include <CGAL/Eigen_solver_traits.h>
#include <CGAL/Eigen_matrix.h>
#include <CGAL/Sphere_3.h>
//using namespace fcl;
#include <algorithm>
#include <CGAL/Scale_space_surface_reconstruction_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/read_off_points.h>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/passthrough.h>

//poisson reconstruction
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::FT FT;
typedef Kernel::Triangle_3                 exactTriangle;
typedef Kernel::Point_3                    exactPoint;
typedef Kernel::Point_3 Point;
typedef CGAL::Point_with_normal_3<Kernel> Point_with_normal;
typedef Kernel::Sphere_3 Sphere;
typedef std::vector<Point_with_normal> PointList;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef CGAL::Poisson_reconstruction_function<Kernel> Poisson_reconstruction_function;
typedef CGAL::Surface_mesh_default_triangulation_3 STr;
typedef CGAL::Surface_mesh_complex_2_in_triangulation_3<STr> C2t3;
typedef CGAL::Implicit_surface_3<Kernel, Poisson_reconstruction_function> Surface_3;

//normal estimation
typedef Kernel::Vector_3 Vector;
// Point with normal vector stored in a std::pair.
typedef std::pair<Point, Vector> PointVectorPair;
typedef std::vector<PointVectorPair> PointListVec;

//scale factor reconstruciton
typedef CGAL::Scale_space_surface_reconstruction_3< Kernel >    Reconstruction;
typedef Reconstruction::Point                                   RPoint;
typedef std::vector< RPoint >                                    Point_collection;
typedef Reconstruction::Triple_const_iterator                   Triple_iterator;

//cgal other needed definition
typedef CGAL::Simple_cartesian<double> KSimple;
typedef KSimple::Ray_3 Ray;
typedef KSimple::Line_3 Line;
typedef KSimple::Point_3 Pointt;
typedef KSimple::Triangle_3 CGALTriangle;
typedef CGAL::Cartesian_converter<KSimple,Kernel > converter;
typedef CGAL::Cartesian_converter<Kernel,KSimple > converterS;


void loadOBJFile(const char* filename, std::vector<Pointt>& points, std::list<CGALTriangle>& triangles);
visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links, int id, int c_color[]);

void dump_reconstruction(const Reconstruction& reconstruct, std::string name)
{
  std::ofstream output(name.c_str());
  output << "OFF " << reconstruct.number_of_points() << " "
         << reconstruct.number_of_triangles() << " 0\n";
  std::copy(reconstruct.points_begin(),
            reconstruct.points_end(),
            std::ostream_iterator<RPoint>(output,"\n"));
  for( Triple_iterator it = reconstruct.surface_begin(); it != reconstruct.surface_end(); ++it )
      output << "3 " << *it << std::endl;
}

int main(int argc,char **argv)
{

    ros::init(argc, argv, "cgal_reconstruction");

    ros::NodeHandle nh;

    // construction from a list of points :
    std::vector<Pointt> pts;
    std::list<CGALTriangle> tri;
    std::string path = ros::package::getPath("component_test");
    //    std::string str = path+"/src/mesh/sphere_densed.obj";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/sphere.pcd", *cloud);

    ros::Time tic1 = ros::Time::now();
    int methodNumber;
    std::cout<<" Reconstruction method: \n"<<"1- cgal Poisson \n2- cgal scale space \nenter the number ";
    std::cin>>methodNumber;

    if(methodNumber == 1)
    {
        //*********************************************************************************************************************************************
        //poisson surface reconstrucion
        PointListVec points;
        //convert the cloud to cgal
        for(int i=0; i<cloud->points.size();i++)
        {
            PointVectorPair pt;
            pt.first = Point(cloud->points.at(i).x, cloud->points.at(i).y, cloud->points.at(i).z);
            points.push_back(pt);
        }

        //**************normal estimation using CGAL*********
        unsigned int nb_neighbors_pca_normals = 18; // K-nearest neighbors = 3 rings (estimate normals by PCA)


        //Computes normals direction by Principal Component Analysis
        // Estimates normals direction.
        // Note: pca_estimate_normals() requires an iterator over points
        // as well as property maps to access each point's position and normal.
        CGAL::pca_estimate_normals(points.begin(), points.end(),
                                   CGAL::First_of_pair_property_map<PointVectorPair>(),
                                   CGAL::Second_of_pair_property_map<PointVectorPair>(),
                                   nb_neighbors_pca_normals);

        // *********Poisson options*********
        FT sm_angle = 20.0; // Min triangle angle in degrees.
        FT sm_radius = 30; // Max triangle size w.r.t. point set average spacing.
        FT sm_distance = 0.375; // Surface Approximation error w.r.t. point set average spacing.

        // Reads the point set file in points[].
        // Note: read_xyz_points_and_normals() requires an iterator over points
        // + property maps to access each point's position and normal.
        // The position property map can be omitted here as we use iterators over Point_3 elements.
        //  PointList points;
        Poisson_reconstruction_function function(points.begin(), points.end(),
                                                 CGAL::First_of_pair_property_map<PointVectorPair>(), //remeber: this access the first term of std::pair (point)
                                                 CGAL::Second_of_pair_property_map<PointVectorPair>()); //remeber: this access the second term of std::pair (normal)

        // Computes the Poisson indicator function f()
        // at each vertex of the triangulation.

        if ( ! function.compute_implicit_function() )
            return EXIT_FAILURE;
        // Computes average spacing
        FT average_spacing = CGAL::compute_average_spacing(points.begin(), points.end(),CGAL::First_of_pair_property_map<PointVectorPair>(),6 /* knn = 1 ring */);
        // Gets one point inside the implicit surface
        // and computes implicit function bounding sphere radius.
        Point inner_point = function.get_inner_point();
        Sphere bsphere = function.bounding_sphere();
        FT radius = std::sqrt(bsphere.squared_radius());


        // Defines the implicit surface: requires defining a
        // conservative bounding sphere centered at inner point.
        FT sm_sphere_radius = 5.0 * radius;
        FT sm_dichotomy_error = sm_distance*average_spacing/1000.0; // Dichotomy error must be << sm_distance
        Surface_3 surface(function,
                          Sphere(inner_point,sm_sphere_radius*sm_sphere_radius),
                          sm_dichotomy_error/sm_sphere_radius);

        // Defines surface mesh generation criteria
        CGAL::Surface_mesh_default_criteria_3<STr> criteria(sm_angle,  // Min triangle angle (degrees)
                                                            sm_radius*average_spacing,  // Max triangle size
                                                            sm_distance*average_spacing); // Approximation error


        // Generates surface mesh with manifold option
        STr tr; // 3D Delaunay triangulation for surface mesh generation
        C2t3 c2t3(tr); // 2D complex in 3D Delaunay triangulation
        CGAL::make_surface_mesh(c2t3,                                 // reconstructed mesh
                                surface,                              // implicit surface
                                criteria,                             // meshing criteria
                                CGAL::Manifold_with_boundary_tag());  // require manifold mesh

        ros::Time toc1 = ros::Time::now();
        std::cout<<"\nPoisson took:"<< toc1.toSec() - tic1.toSec()<<std::endl;

        if(tr.number_of_vertices() == 0)
            return EXIT_FAILURE;

        else std::cout<<"number of vertices: "<<tr.number_of_vertices()<<std::endl;
        std::cout<<"number of triangles: "<<c2t3.number_of_facets()<<std::endl;

        std::ofstream out("poisson-20-30-0.375.off");
        Polyhedron output_mesh;
        CGAL::output_surface_facets_to_polyhedron(c2t3, output_mesh);
        out << output_mesh;
    }
    else if ( methodNumber == 2 )
    {
        //*********************************************************************************************************************************************
        //scale space surface reconstruction
        ros::Time tic2 = ros::Time::now();

        Point_collection points;
        //convert the cloud to cgal
        for(int i=0; i<cloud->points.size();i++)
        {
            RPoint pt;
            pt = Point(cloud->points.at(i).x, cloud->points.at(i).y, cloud->points.at(i).z);
            points.push_back(pt);
        }

        // Construct the reconstruction with parameters for
        // the neighborhood squared radius estimation.
        Reconstruction reconstruct( 10, 100 );
        // Add the points.
        reconstruct.insert( points.begin(), points.end() );
        // Advance the scale-space several steps.
        // This automatically estimates the scale-space.
        reconstruct.increase_scale( 2 );
        // Reconstruct the surface from the current scale-space.
        std::cout << "Neighborhood squared radius is "
                  << reconstruct.neighborhood_squared_radius() << std::endl;
        reconstruct.reconstruct_surface();
        std::cout << "First reconstruction done." << std::endl;
        // Write the reconstruction.
        dump_reconstruction(reconstruct, "reconstruction1.off");
        // Advancing the scale-space further and visually compare the reconstruction result
        reconstruct.increase_scale( 2 );
        // Reconstruct the surface from the current scale-space.
        std::cout << "Neighborhood squared radius is "
                  << reconstruct.neighborhood_squared_radius() << std::endl;
        reconstruct.reconstruct_surface();
        std::cout << "Second reconstruction done." << std::endl;
        // Write the reconstruction.

        ros::Time toc1 = ros::Time::now();
        std::cout<<"\nscale factor took:"<< toc1.toSec() - tic2.toSec()<<std::endl;

        dump_reconstruction(reconstruct, "reconstruction2.off");
        std::cout <<"number of triangles :"<<reconstruct.number_of_triangles()<<std::endl;
        //*********************************************************************************************************************************************

    }
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
      ros::spinOnce();
      loop_rate.sleep();
  }
  return 0;
}
visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links, int id, int c_color[])
{
    visualization_msgs::Marker linksMarkerMsg;
    linksMarkerMsg.header.frame_id="map"; //change to "base_point_cloud" if it is used in component test package
    linksMarkerMsg.header.stamp=ros::Time::now();
    linksMarkerMsg.ns="link_marker";
    linksMarkerMsg.id = id;
    linksMarkerMsg.type = visualization_msgs::Marker::LINE_LIST;
    linksMarkerMsg.scale.x = 0.01;//0.03
    linksMarkerMsg.action  = visualization_msgs::Marker::ADD;
    linksMarkerMsg.lifetime  = ros::Duration(1000);
    std_msgs::ColorRGBA color;
    color.r = (float)c_color[0]; color.g=(float)c_color[1]; color.b=(float)c_color[2], color.a=1.0f;

    std::vector<geometry_msgs::Point>::iterator linksIterator;
    for(linksIterator = links.begin();linksIterator != links.end();linksIterator++)
    {
        linksMarkerMsg.points.push_back(*linksIterator);
        linksMarkerMsg.colors.push_back(color);
    }
   return linksMarkerMsg;
}
void loadOBJFile(const char* filename, std::vector<Pointt>& points, std::list<CGALTriangle>& triangles)
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
                fcl::FCL_REAL x = (fcl::FCL_REAL)atof(strtok(NULL, "\t "));
                fcl::FCL_REAL y = (fcl::FCL_REAL)atof(strtok(NULL, "\t "));
                fcl::FCL_REAL z = (fcl::FCL_REAL)atof(strtok(NULL, "\t "));
                double xd = x; double yd = y; double zd=z;
                Pointt p(xd, yd, zd);
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
                    Pointt p1(points[atoi(data[0]) - 1][0],points[atoi(data[0]) - 1][1],points[atoi(data[0]) - 1][2]);
                    Pointt p2(points[atoi(data[1]) - 1][0],points[atoi(data[1]) - 1][1],points[atoi(data[1]) - 1][2]);
                    Pointt p3(points[atoi(data[2]) - 1][0],points[atoi(data[2]) - 1][1],points[atoi(data[2]) - 1][2]);
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
                    Pointt p1(points[indxs[0]][0],points[indxs[0]][1],points[indxs[0]][2]);
                    Pointt p2(points[indxs[1]][0],points[indxs[1]][1],points[indxs[1]][2]);
                    Pointt p3(points[indxs[2]][0],points[indxs[2]][1],points[indxs[2]][2]);
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
