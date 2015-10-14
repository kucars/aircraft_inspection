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
#include <boost/shared_ptr.hpp>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/create_straight_skeleton_2.h>
//#include <print.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K ;
typedef K::Point_2                   Point ;
typedef CGAL::Polygon_2<K>           Polygon_2 ;
typedef CGAL::Straight_skeleton_2<K> Ss ;
typedef boost::shared_ptr<Ss> SsPtr ;

using namespace fcl;
visualization_msgs::Marker drawCUBE(Vec3f vec , int id , int c_color);
visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links);

template<class K>
void print_point ( CGAL::Point_2<K> const& p )
{
  std::cout << "(" << p.x() << "," << p.y() << ")" ;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cgal_inner_skeleton");
    ros::NodeHandle n;
    ros::Publisher contourCubePub  = n.advertise<visualization_msgs::MarkerArray>("contour_vertex_array", 10);
    ros::Publisher skeletonCubePub = n.advertise<visualization_msgs::MarkerArray>("skeleton_vertex_array", 10);
    ros::Publisher skeletonLinePub = n.advertise<visualization_msgs::Marker>("skeleton_Lines", 10);
     
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    std::string path = ros::package::getPath("component_test");
    std::cout<<"PATH is:"<<path;
    geometry_msgs::PoseArray points;
    geometry_msgs::Pose pose;

    Polygon_2 poly ;
  
    poly.push_back( Point(-1,-1) ) ;
    poly.push_back( Point(0,-5) ) ;
    poly.push_back( Point(1,-1) ) ;
    poly.push_back( Point(5,0) ) ;
    poly.push_back( Point(1,1) ) ;
    poly.push_back( Point(0,5) ) ;
    poly.push_back( Point(-1,1) ) ;
    poly.push_back( Point(-5,0) ) ;
     
    // You can pass the polygon via an iterator pair
    SsPtr iss = CGAL::create_interior_straight_skeleton_2(poly.vertices_begin(), poly.vertices_end());
    // Or you can pass the polygon directly, as below.   
    // To create an exterior straight skeleton you need to specify a maximum offset.
    double lMaxOffset = 5 ; 
    SsPtr oss = CGAL::create_exterior_straight_skeleton_2(lMaxOffset, poly);
    
    //print_straight_skeleton(*iss);
    //print_straight_skeleton(*oss);
    
    typedef CGAL::Straight_skeleton_2<K> Ss ;
  
    typedef typename Ss::Vertex_const_handle     Vertex_const_handle;
    typedef typename Ss::Halfedge_const_handle   Halfedge_const_handle;
    typedef typename Ss::Halfedge_const_iterator Halfedge_const_iterator;
  
    Halfedge_const_handle null_halfedge;
    Vertex_const_handle   null_vertex;

    std::cout << "Straight skeleton with " << (*iss).size_of_vertices() 
            << " vertices, " << (*iss).size_of_halfedges()
            << " halfedges and " << (*iss).size_of_faces()
            << " faces" << std::endl ;
            
 
    visualization_msgs::MarkerArray contour_marker_array,bisector_marker_array;
    visualization_msgs::Marker marker;
    int id=0;
    std::vector<geometry_msgs::Point> lineSegments;
    geometry_msgs::Point point;
    for ( Halfedge_const_iterator i = (*iss).halfedges_begin(); i != (*iss).halfedges_end(); ++i )
    {
        //print_point(i->opposite()->vertex()->point()) ;
        Vec3f vec2(i->opposite()->vertex()->point().x() , i->opposite()->vertex()->point().y(), 1);
        std::cout << "->" ;
        //print_point(i->vertex()->point());
        std::cout << " " << ( i->is_bisector() ? "bisector" : "contour" ) << std::endl;
        Vec3f vec1(i->vertex()->point().x(), i->vertex()->point().y(), 1);
        if(i->is_bisector())
        {
            point.x = i->vertex()->point().x(); point.y = i->vertex()->point().y(); point.z = 1;
            lineSegments.push_back(point);
            point.x = i->opposite()->vertex()->point().x(); point.y = i->opposite()->vertex()->point().y(); point.z = 1;
            lineSegments.push_back(point);
            /*
            marker = drawLines(vec1, id, 1);
            bisector_marker_array.markers.push_back(marker);
            marker = drawLines(vec2, id, 1);
            bisector_marker_array.markers.push_back(marker);            
            */
        }
        else
        {
            /*
            marker = drawLines(vec1, id, 2);
            contour_marker_array.markers.push_back(marker);            
            marker = drawLines(vec2, id, 2);
            contour_marker_array.markers.push_back(marker);                        
            */
        }
        id++;
    }    
    visualization_msgs::Marker linesList = drawLines(lineSegments);
    /*
    contourCubePub.publish(contour_marker_array);
    skeletonCubePub.publish(bisector_marker_array);
    */
    ros::Rate loop_rate(0.5);
    while(ros::ok())
    {
        ROS_INFO("Looping till eternity");
        contourCubePub.publish(contour_marker_array);
        skeletonCubePub.publish(bisector_marker_array);
        skeletonLinePub.publish(linesList);
        ros::spinOnce();
        loop_rate.sleep(); 
    }
    return EXIT_SUCCESS;
}

visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links)
{
    visualization_msgs::Marker linksMarkerMsg;
    linksMarkerMsg.header.frame_id="/base_point_cloud";
    linksMarkerMsg.header.stamp=ros::Time::now();
    linksMarkerMsg.ns="link_marker";
    linksMarkerMsg.id = 0;
    linksMarkerMsg.type = visualization_msgs::Marker::LINE_LIST;
    linksMarkerMsg.scale.x = 0.005;
    linksMarkerMsg.action  = visualization_msgs::Marker::ADD;
    linksMarkerMsg.lifetime  = ros::Duration(0.1);
    std_msgs::ColorRGBA color;
    color.r = 1.0f; color.g=.0f; color.b=.0f, color.a=1.0f;
    std::vector<geometry_msgs::Point>::iterator linksIterator;
    for(linksIterator = links.begin();linksIterator != links.end();linksIterator++)
    {
        linksMarkerMsg.points.push_back(*linksIterator);
        linksMarkerMsg.colors.push_back(color);
    }
   return linksMarkerMsg;
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
    marker.lifetime = ros::Duration(1);
    return marker ;
}
