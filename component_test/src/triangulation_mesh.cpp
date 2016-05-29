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
using namespace fcl;
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Triangle_3                 exactTriangle;
typedef K::Point_3                    exactPoint;
typedef CGAL::Triangulation_3<K>      Triangulation;
typedef Triangulation::Finite_vertices_iterator Finite_vertices_iterator;
typedef Triangulation::Finite_edges_iterator Finite_edges_iterator;
typedef Triangulation::Facet_iterator facets_iterator;
typedef Triangulation::Finite_cells_iterator Finite_cells_iterator;
typedef Triangulation::Simplex        Simplex;
typedef Triangulation::Locate_type    Locate_type;
typedef Triangulation::Point          Point;
typedef CGAL::Simple_cartesian<double> KSimple;
typedef KSimple::Ray_3 Ray;
typedef KSimple::Line_3 Line;
typedef KSimple::Point_3 Pointt;
typedef KSimple::Triangle_3 CGALTriangle;
typedef CGAL::Cartesian_converter<KSimple,K > converter;
typedef CGAL::Cartesian_converter<K,KSimple > converterS;


void loadOBJFile(const char* filename, std::vector<Pointt>& points, std::list<CGALTriangle>& triangles);
visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links, int id, int c_color[]);

int main(int argc,char **argv)
{

    ros::init(argc, argv, "triangulation");

    ros::NodeHandle nh;
    ros::Publisher reconstruction_pub = nh.advertise<visualization_msgs::MarkerArray>("reconstruction", 10);
    ros::Publisher reconstruction_pub2 = nh.advertise<visualization_msgs::Marker>("gradual_reconstruction", 10);

  // construction from a list of points :
    std::vector<Pointt> pts;
    std::list<CGALTriangle> tri;
    std::string path = ros::package::getPath("component_test");
//    std::string str = path+"/src/mesh/sphere_densed.obj";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/sphere.pcd", *cloud);

  std::vector<Pointt> L;
  for(int i=0; i<cloud->points.size();i++)
  {
      Pointt pt(cloud->points.at(i).x, cloud->points.at(i).y, cloud->points.at(i).z);
      L.push_back(pt);
  }
  converter to_exact;// to convert from Exact cgal to exact kernel
  Point convertedPt;// to convert from Exact cgal to simple cartesian kernel
  std::list<Point> LExact;

  for(int i=0; i<L.size(); i++)
  {
      convertedPt = to_exact( L[i] );
      LExact.push_back(convertedPt);

  }
  Triangulation T(LExact.begin(), LExact.end());
  Triangulation::size_type n = T.number_of_vertices();
  Triangulation::size_type n1 = T.number_of_facets();

  std::cout<<"number of vertices :"<<n<<std::endl;
  std::cout<<"number of faces :"<<n1<<std::endl;

  Triangulation::Finite_facets_iterator it;
  visualization_msgs::MarkerArray markerArray;
  visualization_msgs::Marker marker;
  std::list<CGALTriangle> triangles;
  converterS to_simple;// to convert from Exact cgal to simple cartesian kernel
  Pointt convertedPtt;


  Triangulation::Cell_handle c=T.infinite_cell();
  std::vector<geometry_msgs::Point> ptsTri;

  int i=0;
  int id=0;
  for (it = T.finite_facets_begin(); it != T.finite_facets_end(); it++)
  {

      exactPoint pt1 = T.triangle(*it).vertex(0) ;
      convertedPtt = to_simple(pt1);
      Pointt p1(convertedPtt[0],convertedPtt[1],convertedPtt[2]);
      exactPoint pt2 = T.triangle(*it).vertex(1) ;
      convertedPtt = to_simple(pt2);
      Pointt p2(convertedPtt[0],convertedPtt[1],convertedPtt[2]);
      exactPoint pt3 = T.triangle(*it).vertex(2) ;
      convertedPtt = to_simple(pt3);
      Pointt p3(convertedPtt[0],convertedPtt[1],convertedPtt[2]);
      exactTriangle tri(pt1,pt2,pt3);


      CGALTriangle simpleTri(p1,p2,p3);
//      tri = exactTriangle(ep1,ep2,ep3);

//      if(!CGAL::collinear(p1,p2,p3))
//      {
          triangles.push_back(simpleTri);
          std::cout<<"triangles size: "<<triangles.size()<<std::endl;
//          int c_color[3];
          geometry_msgs::Point point1,point2,point3;
          point1.x = p1[0];point1.y = p1[1]; point1.z= p1[2];//ptsTri.push_back(point1);
          point2.x = p2[0];point2.y = p2[1]; point2.z= p2[2];//ptsTri.push_back(point2);
          point3.x = p3[0];point3.y = p3[1]; point3.z= p3[2];//ptsTri.push_back(point3);
          ptsTri.push_back(point1);
          ptsTri.push_back(point2);
          ptsTri.push_back(point1);
          ptsTri.push_back(point3);
          ptsTri.push_back(point2);
          ptsTri.push_back(point3);

//      }
      i++;

  }
  int c_color[3];
  c_color[0]=1; c_color[1]=0; c_color[2]=0;
  marker = drawLines(ptsTri,id++,c_color);//purple


  ros::Rate loop_rate(10);

  while (ros::ok())
  {
      reconstruction_pub2.publish(marker);

//      reconstruction_pub.publish(markerArray);
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
                FCL_REAL x = (FCL_REAL)atof(strtok(NULL, "\t "));
                FCL_REAL y = (FCL_REAL)atof(strtok(NULL, "\t "));
                FCL_REAL z = (FCL_REAL)atof(strtok(NULL, "\t "));
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
