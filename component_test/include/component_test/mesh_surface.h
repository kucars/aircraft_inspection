#ifndef MESHSURFACE_H_
#define MESHSURFACE_H_

#define CGAL_EIGEN3_ENABLED

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
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <deque>
#include <std_msgs/Bool.h>
#include <cmath>
#include <math.h>
#include <sstream>
#include <string>
#include <vector>
#include <limits>
#include <cassert>
#include <iomanip>
#include "fcl_utility.h"
#include <iostream>

//PCL
#include <iostream>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
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
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/passthrough.h>
//CGAL
// Triangle triangle intersection
#include <CGAL/intersections.h>
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
// Delaunay Triangulation in 3D
#include <CGAL/Delaunay_triangulation_3.h>
// kernels
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Cartesian_converter.h>
//scale factor reconstruction
#include <algorithm>
#include <CGAL/Scale_space_surface_reconstruction_3.h>
#include <CGAL/IO/read_off_points.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel       exactKernel; //CGAL::Exact_predicates_exact_constructions_kernel//it could be also CGAL::Exact_predicates_inexact_constructions_kernel CHECK LATER
typedef CGAL::Simple_cartesian<double>                          simpleKernel;
typedef CGAL::Point_3<exactKernel>                              Point_3;
typedef CGAL::Segment_3<exactKernel>                            Segment_3;
typedef CGAL::Triangle_3<exactKernel>                           Triangle_3;
typedef CGAL::Plane_3<exactKernel>                              Plane_3;
typedef CGAL::Tetrahedron_3<exactKernel>                        Tetrahedron_3;
typedef simpleKernel::Point_3                                   Point_3_S;
typedef CGAL::Cartesian_converter<exactKernel,simpleKernel >    converter;

// Axis-align boxes for all-pairs self-intersection detection
typedef std::vector<Triangle_3>                                 Triangles;
typedef typename Triangles::iterator                            TrianglesIterator;
typedef typename Triangles::const_iterator                      TrianglesConstIterator;
typedef CGAL::Box_intersection_d::Box_with_handle_d<double,3,TrianglesIterator> Box;

//scale space surface reconstruciton
typedef CGAL::Exact_predicates_inexact_constructions_kernel     inexactKernel;
typedef inexactKernel::Point_3                                  inexactPoint;
typedef CGAL::Scale_space_surface_reconstruction_3< inexactKernel >    Reconstruction;
typedef Reconstruction::Point                                   RPoint;
typedef std::vector< RPoint >                                   Point_collection;
typedef Reconstruction::Triple_const_iterator                   Triple_iterator;
using namespace fcl;

class MeshSurface
{
public:
    //attributes
    ros::NodeHandle  nh;
    static Triangles meshTA,meshTB;
    static std::vector<int> facesIndicesA,facesIndicesB;
    //    pcl::PolygonMeshPtr& pclMesh;
    int count;

    //methods
    MeshSurface(ros::NodeHandle & n, Triangles TA, Triangles TB);
    MeshSurface(ros::NodeHandle &n);
    MeshSurface();
    ~MeshSurface();
    void setCGALMeshA(Triangles TA);
    void setCGALMeshB(Triangles TB);
    void clear();
    void meshingPCL(pcl::PointCloud<pcl::PointXYZ> pointCloud, Triangles& cgalMeshT, bool saveMeshFlag=false);
    void meshingPoissonPCL(pcl::PointCloud<pcl::PointXYZ> pointCloud, Triangles& cgalMeshT, bool saveMeshFlag=false);
    void meshingScaleSpaceCGAL(pcl::PointCloud<pcl::PointXYZ> pointCloud, Triangles& cgalMeshT, bool saveMeshFlag=false);
    pcl::PointCloud<pcl::PointXYZ> pointsDifference(pcl::PointCloud<pcl::PointXYZ> cloud1, pcl::PointCloud<pcl::PointXYZ> cloud2);
    bool containsCheck(pcl::PointCloud<pcl::PointXYZ> cloud, pcl::PointXYZ point);

    double getIntersectionArea(Triangles& intersectionFaces);
    double getExtraArea(Triangles& extraAreaFaces);
    void box_up(Triangles & T, std::vector<Box> & boxes);
    double calcCGALMeshSurfaceArea(std::vector<Triangle_3> mesh);
    double calcPCLMeshSurfaceArea(pcl::PolygonMesh::Ptr& mesh);
    void loadOBJFile(const char* filename, std::vector<Vec3f>& points, Triangles& triangles);

};

#endif 
