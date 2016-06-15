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
#include "fcl_utility.h"
#include <pcl/filters/voxel_grid.h>
#include <component_test/occlusion_culling.h>
#include <component_test/occlusion_culling_gpu.h>
#include <pcl/io/pcd_io.h>
#include "fcl_utility.h"
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
//VTK
#include <vtkVersion.h>
#include <vtkSmartPointer.h>
#include <vtkSurfaceReconstructionFilter.h>
#include <vtkProgrammableSource.h>
#include <vtkContourFilter.h>
#include <vtkReverseSense.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkPolyData.h>
#include <vtkCamera.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSphereSource.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkPolyDataReader.h>
#include <vtkPLYWriter.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <cmath>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

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

#include <rviz_visual_tools/rviz_visual_tools.h>
//#include <igl/copyleft/cgal/intersect_other.h>
#include <component_test/mesh_surface.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel exactKernel; //CGAL::Exact_predicates_exact_constructions_kernel//it could be also CGAL::Exact_predicates_inexact_constructions_kernel CHECK LATER
typedef CGAL::Simple_cartesian<double> simpleKernel;
typedef CGAL::Point_3<exactKernel>    Point_3;
typedef CGAL::Segment_3<exactKernel>  Segment_3;
typedef CGAL::Triangle_3<exactKernel> Triangle_3;
typedef CGAL::Plane_3<exactKernel>    Plane_3;
typedef CGAL::Tetrahedron_3<exactKernel> Tetrahedron_3;
typedef simpleKernel::Point_3   Point_3_S;
typedef CGAL::Cartesian_converter<exactKernel,simpleKernel > converter;

using namespace fcl;
geometry_msgs::Pose uav2camTransformation(geometry_msgs::Pose pose, Vec3f rpy, Vec3f xyz);
visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links, int c_color[], double scale, std::string frame_id);

int main(int argc, char **argv)
{

    ros::init(argc, argv, "surface_coverage_evaluation");
    ros::NodeHandle n;
    ros::Publisher originalCloudPub = n.advertise<sensor_msgs::PointCloud2>("originalPointCloud", 100);
    ros::Publisher visibleCloudPub  = n.advertise<sensor_msgs::PointCloud2>("accuracyCloud", 100);
    ros::Publisher extraCloudPub    = n.advertise<sensor_msgs::PointCloud2>("extraCloud", 100);
    ros::Publisher viewpointsPub    = n.advertise<geometry_msgs::PoseArray>("viewpoints", 100);
    ros::Publisher waypointsPub     = n.advertise<geometry_msgs::PoseArray>("waypoints", 100);
    ros::Publisher pathPub          = n.advertise<visualization_msgs::Marker>("path", 10);
    ros::Publisher reconstructionPub = n.advertise<visualization_msgs::Marker>("reconstruction", 10);


    pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr visibleCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr extraPtr(new pcl::PointCloud<pcl::PointXYZ>);

    OcclusionCullingGPU obj(n,"sphere_densed_new.pcd");
    double locationx,locationy,locationz,yaw,qx,qy,qz,qw;

    //reading the original cloud
    std::string path = ros::package::getPath("component_test");
    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/sphere_densed_new.pcd", *originalCloud);


    //option 1: reading the path from a fileto quantify the coverage accross the path
    std::string str = path+"/src/txt/2.5_50pathAreaNew_10.txt";//2.5_90path_new

    //option2: reading the viewpoints in order to get maximum coverage
    //    std::string str1 = path+"/src/txt/SearchSpaceCam_1.5m_1to4_etihad_nowheels_nointernal_newdensed.pcd_0.txt";//3_90path_new
    //    std::string str2 = path+"/src/txt/SearchSpaceCam_1.5m_1to4_etihad_nowheels_nointernal_newdensed.pcd_1.txt";//3_90path_new

    const char * filename1 = str.c_str();
    assert(filename1 != NULL);
    filename1 = strdup(filename1);
    FILE *file1 = fopen(filename1, "r");
    if (!file1)
    {
        std::cout<<"\nCan not open the File";
        fclose(file1);
    }

    //    const char * filename2 = str2.c_str();
    //    assert(filename2 != NULL);
    //    filename2 = strdup(filename2);
    //    FILE *file2 = fopen(filename2, "r");
    //    if (!file2)
    //    {
    //        std::cout<<"\nCan not open the File";
    //        fclose(file2);
    //    }

    geometry_msgs::Pose pt;
    std::string str3 = path+"/src/mesh/sphere_scaled_translated.obj";

    MeshSurface ms(n);

    std::vector<Vec3f> pt2;
    Triangles modelCGALT , coveredPartT;
    ms.loadOBJFile(str3.c_str(), pt2, modelCGALT);
    double modelArea = ms.calcCGALMeshSurfaceArea(modelCGALT);
    std::cout<<"Total model area: "<<modelArea<<std::endl;

    pcl::PointCloud<pcl::PointXYZ> pointCloud;
    pointCloud.points = visibleCloudPtr->points;


    geometry_msgs::PoseArray viewpoints;
    pcl::PointCloud<pcl::PointXYZ> global;
    int i=0;
    while (!feof(file1))
    {
        fscanf(file1,"%lf %lf %lf %lf\n",&locationx,&locationy,&locationz,&qw);
        pt.position.x=locationx; pt.position.y=locationy; pt.position.z=locationz;

        tf::Quaternion tf = tf::createQuaternionFromYaw(qw);
        pt.orientation.x = tf.getX();
        pt.orientation.y = tf.getY();
        pt.orientation.z = tf.getZ();
        pt.orientation.w = tf.getW();

        //getting the sensor points
        Vec3f xyz1(0, 0.022, 0.065); Vec3f rpy1(0, -0.349, 0.0);
        geometry_msgs::Pose sensor1 = uav2camTransformation(pt,rpy1,xyz1);
        pcl::PointCloud<pcl::PointXYZ> visible;
        visible = obj.extractVisibleSurface(sensor1);
        viewpoints.poses.push_back(sensor1);

        Vec3f xyz2(0, 0.022, -0.065); Vec3f rpy2(0, 0.349, 0.0);
        geometry_msgs::Pose sensor2 = uav2camTransformation(pt,rpy2,xyz2);
        visible += obj.extractVisibleSurface(sensor2);
        viewpoints.poses.push_back(sensor2);
        //        std::cout<<"visible"<<visible.points.size()<<std::endl;

        Triangles temp,intersectedT,extraTri;

        ms.meshingScaleSpaceCGAL(visible,temp,false);

        if(i!=0){//if it is not the first point
            ms.setCGALMeshA(coveredPartT);
            ms.setCGALMeshB(temp);
            double value = ms.getExtraArea(extraTri);
            converter to_simple;
            for(int i=0; i<extraTri.size(); i++)
            {
                pcl::PointXYZ pt;
                Triangle_3 tri = extraTri[i];
                for(int j=0; j<3; j++)
                {
                    Point_3 ptCGAL = tri.vertex(j) ;
                    Point_3_S psimple  = to_simple(ptCGAL);
                    pt.data[0] = psimple[0];pt.data[1] = psimple[1]; pt.data[2]= psimple[2];
                    global.points.push_back(pt);

                }
            }
            coveredPartT.erase(coveredPartT.begin(),coveredPartT.end());
            ms.meshingScaleSpaceCGAL(global,coveredPartT,false);

            std::cout<<"extra area: "<<value<<" extra triangles #: "<<temp.size()-intersectedT.size()<<std::endl;
            //        std::cout<<"Intersection area: "<<value2<<" intersected triangles #: "<<intersectedT.size()<<std::endl;
            std::cout<<"triangles: "<<coveredPartT.size()<<std::endl<<std::endl;
            double covered = ms.calcCGALMeshSurfaceArea(coveredPartT);
            double percent = (covered/modelArea)*100;
            std::cout<<"Total covered area % : "<<percent<<std::endl;
        }else {
            coveredPartT = temp;
            global.points = visible.points;
            i++;

        }

    }

    Triangles reconstructed;
    ms.meshingScaleSpaceCGAL(global,reconstructed,true);

    //visualizing the triangles
    converter to_simple;
    std::vector<geometry_msgs::Point> ptsTri;
    for(int i =0 ; i< coveredPartT.size(); i++)
    {
        Triangle_3 t = coveredPartT[i] ;
        geometry_msgs::Point point1,point2,point3;
        Point_3 pt1 = t.vertex(0) ;
        Point_3_S psimple1  = to_simple(pt1);
        point1.x = psimple1[0];point1.y = psimple1[1]; point1.z= psimple1[2];

        Point_3 pt2 = t.vertex(1) ;
        Point_3_S psimple2  = to_simple(pt2);
        point2.x = psimple2[0];point2.y = psimple2[1]; point2.z= psimple2[2];

        Point_3 pt3 = t.vertex(2) ;
        Point_3_S psimple3  = to_simple(pt3);
        point3.x = psimple3[0];point3.y = psimple3[1]; point3.z= psimple3[2];

        ptsTri.push_back(point1);
        ptsTri.push_back(point2);
        ptsTri.push_back(point1);
        ptsTri.push_back(point3);
        ptsTri.push_back(point2);
        ptsTri.push_back(point3);
    }

    int c_color[3];
    c_color[0]=1; c_color[1]=0; c_color[2]=0;
    visualization_msgs::Marker marker = drawLines(ptsTri,c_color,0.009,"map");//red

    double covered = ms.calcCGALMeshSurfaceArea(reconstructed);
    std::cout<<"Total covered area: "<<covered<<std::endl;

    double percent = (covered/modelArea)*100;
    std::cout<<"Coverage Area Percent"<<percent<<std::endl;


    ros::Rate loop_rate(10);
    sensor_msgs::PointCloud2 cloud1;
    while (ros::ok())
    {

        //***original cloud ***
        pcl::toROSMsg(*obj.cloud, cloud1); //cloud of original (white) using original cloud


        cloud1.header.frame_id = "map";
        cloud1.header.stamp = ros::Time::now();
        originalCloudPub.publish(cloud1);


        //***viewpoints publish***
        viewpoints.header.frame_id= "map";
        viewpoints.header.stamp = ros::Time::now();
        viewpointsPub.publish(viewpoints);

        reconstructionPub.publish(marker);


        //        //***path publish***
        //        pathPub.publish(linesList);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
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
visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links, int c_color[], double scale, std::string frame_id)
{
    visualization_msgs::Marker linksMarkerMsg;
    linksMarkerMsg.header.frame_id=frame_id; //change to "base_point_cloud" if it is used in component test package
    linksMarkerMsg.header.stamp=ros::Time::now();
    linksMarkerMsg.ns="link_marker";
    linksMarkerMsg.id = 2342342;
    linksMarkerMsg.type = visualization_msgs::Marker::LINE_LIST;
    linksMarkerMsg.scale.x = scale;//0.08
    linksMarkerMsg.action  = visualization_msgs::Marker::ADD;
    linksMarkerMsg.lifetime  = ros::Duration(10000000);
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
