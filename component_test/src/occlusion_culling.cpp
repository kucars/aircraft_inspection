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
#include <pcl/filters/voxel_grid_occlusion_estimation.h>

//CGAL
#include <list>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_3.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>

typedef CGAL::Simple_cartesian<double> K;
typedef K::FT FT;
typedef K::Ray_3 Ray;
typedef K::Line_3 Line;
typedef K::Vector_3 Vector;
typedef K::Point_3 Point;
typedef K::Triangle_3 CGALTriangle;
typedef std::list<CGALTriangle>::iterator Iterator;
typedef CGAL::AABB_triangle_primitive<K, Iterator> Primitive;
typedef CGAL::AABB_traits<K, Primitive> AABB_triangle_traits;
typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;


//For triangulation (Later)
//typedef CGAL::Exact_predicates_inexact_constructions_kernel K1;
//typedef CGAL::Triangulation_3<K1>      Triangulation;
//typedef Triangulation::Cell_handle    Cell_handle;
//typedef Triangulation::Vertex_handle  Vertex_handle;
//typedef Triangulation::Locate_type    Locate_type;
//typedef Triangulation::Point          Point1;

//typedef Triangulation::Triangle        Tr;
//typedef K1::Triangle_3 CGALTriangle;
//typedef std::list<CGALTriangle>::iterator Iterator;
//typedef CGAL::AABB_triangle_primitive<K1, Iterator> Primitive;
//typedef CGAL::AABB_traits<K1, Primitive> AABB_triangle_traits;
//typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;

//typedef CGAL::Triangulation_vertex_base_with_info_3<CGAL::Color, K1> Vb;
//typedef CGAL::Triangulation_data_structure_3<Vb>                    Tds;
//typedef CGAL::Delaunay_triangulation_3<K1, Tds>                      Delaunay;

void from_voxel(Eigen::Vector4f t, float res, std::list<CGALTriangle>& triangles)
{
    std::vector<Eigen::Vector3f> vertices;
    Eigen::Vector3f p1,p2,p3,p4,p5,p6,p7,p8;
    p1[0]=t[0]+(res/2);p1[1]=t[1]+(res/2);p1[2]=t[2]+(res/2);vertices.push_back(p1);
    p2[0]=t[0]-(res/2);p2[1]=t[1]+(res/2);p2[2]=t[2]+(res/2);vertices.push_back(p2);
    p3[0]=t[0]-(res/2);p3[1]=t[1]-(res/2);p3[2]=t[2]+(res/2);vertices.push_back(p3);
    p4[0]=t[0]+(res/2);p4[1]=t[1]-(res/2);p4[2]=t[2]+(res/2);vertices.push_back(p4);

    p5[0]=t[0]+(res/2);p5[1]=t[1]+(res/2);p5[2]=t[2]-(res/2);vertices.push_back(p5);
    p6[0]=t[0]-(res/2);p6[1]=t[1]+(res/2);p6[2]=t[2]-(res/2);vertices.push_back(p6);
    p7[0]=t[0]-(res/2);p7[1]=t[1]-(res/2);p7[2]=t[2]-(res/2);vertices.push_back(p7);
    p8[0]=t[0]+(res/2);p7[1]=t[1]-(res/2);p7[2]=t[2]-(res/2);vertices.push_back(p8);

    //    std::cout<<"vertice 1x: "<<t[0]<<"vertice 1y: "<<t[1]<<"vertice 1z: "<<t[2]<<std::endl;
    CGALTriangle tri;
    //0
    tri = CGALTriangle(Point(vertices.at(0)[0],vertices.at(0)[1],vertices.at(0)[2]),Point(vertices.at(1)[0],vertices.at(1)[1],vertices.at(1)[2]),Point(vertices.at(2)[0],vertices.at(2)[1],vertices.at(2)[2]));
    triangles.push_back(tri);
    tri = CGALTriangle(Point(vertices.at(0)[0],vertices.at(0)[1],vertices.at(0)[2]),Point(vertices.at(3)[0],vertices.at(3)[1],vertices.at(3)[2]),Point(vertices.at(2)[0],vertices.at(2)[1],vertices.at(2)[2]));
    triangles.push_back(tri);
    tri = CGALTriangle(Point(vertices.at(0)[0],vertices.at(0)[1],vertices.at(0)[2]),Point(vertices.at(3)[0],vertices.at(3)[1],vertices.at(3)[2]),Point(vertices.at(7)[0],vertices.at(7)[1],vertices.at(7)[2]));
    triangles.push_back(tri);
    tri = CGALTriangle(Point(vertices.at(0)[0],vertices.at(0)[1],vertices.at(0)[2]),Point(vertices.at(4)[0],vertices.at(4)[1],vertices.at(4)[2]),Point(vertices.at(7)[0],vertices.at(7)[1],vertices.at(7)[2]));
    triangles.push_back(tri);
    tri = CGALTriangle(Point(vertices.at(0)[0],vertices.at(0)[1],vertices.at(0)[2]),Point(vertices.at(4)[0],vertices.at(4)[1],vertices.at(4)[2]),Point(vertices.at(5)[0],vertices.at(5)[1],vertices.at(5)[2]));
    triangles.push_back(tri);
    tri = CGALTriangle(Point(vertices.at(0)[0],vertices.at(0)[1],vertices.at(0)[2]),Point(vertices.at(1)[0],vertices.at(1)[1],vertices.at(1)[2]),Point(vertices.at(5)[0],vertices.at(5)[1],vertices.at(5)[2]));
    triangles.push_back(tri);
    tri = CGALTriangle(Point(vertices.at(0)[0],vertices.at(0)[1],vertices.at(0)[2]),Point(vertices.at(4)[0],vertices.at(4)[1],vertices.at(4)[2]),Point(vertices.at(6)[0],vertices.at(6)[1],vertices.at(6)[2]));
    triangles.push_back(tri);
    tri = CGALTriangle(Point(vertices.at(0)[0],vertices.at(0)[1],vertices.at(0)[2]),Point(vertices.at(2)[0],vertices.at(2)[1],vertices.at(2)[2]),Point(vertices.at(6)[0],vertices.at(6)[1],vertices.at(6)[2]));
    triangles.push_back(tri);

    //7
    tri = CGALTriangle(Point(vertices.at(6)[0],vertices.at(6)[1],vertices.at(6)[2]),Point(vertices.at(2)[0],vertices.at(2)[1],vertices.at(2)[2]),Point(vertices.at(1)[0],vertices.at(1)[1],vertices.at(1)[2]));
    triangles.push_back(tri);
    tri = CGALTriangle(Point(vertices.at(6)[0],vertices.at(6)[1],vertices.at(6)[2]),Point(vertices.at(5)[0],vertices.at(5)[1],vertices.at(5)[2]),Point(vertices.at(1)[0],vertices.at(1)[1],vertices.at(1)[2]));
    triangles.push_back(tri);
    tri = CGALTriangle(Point(vertices.at(6)[0],vertices.at(6)[1],vertices.at(6)[2]),Point(vertices.at(5)[0],vertices.at(5)[1],vertices.at(5)[2]),Point(vertices.at(4)[0],vertices.at(4)[1],vertices.at(4)[2]));
    triangles.push_back(tri);
    tri = CGALTriangle(Point(vertices.at(6)[0],vertices.at(6)[1],vertices.at(6)[2]),Point(vertices.at(7)[0],vertices.at(7)[1],vertices.at(7)[2]),Point(vertices.at(4)[0],vertices.at(4)[1],vertices.at(4)[2]));
    triangles.push_back(tri);
    tri = CGALTriangle(Point(vertices.at(6)[0],vertices.at(6)[1],vertices.at(6)[2]),Point(vertices.at(7)[0],vertices.at(7)[1],vertices.at(7)[2]),Point(vertices.at(3)[0],vertices.at(3)[1],vertices.at(3)[2]));
    triangles.push_back(tri);
    tri = CGALTriangle(Point(vertices.at(6)[0],vertices.at(6)[1],vertices.at(6)[2]),Point(vertices.at(2)[0],vertices.at(2)[1],vertices.at(2)[2]),Point(vertices.at(3)[0],vertices.at(3)[1],vertices.at(3)[2]));
    triangles.push_back(tri);
    tri = CGALTriangle(Point(vertices.at(5)[0],vertices.at(5)[1],vertices.at(5)[2]),Point(vertices.at(1)[0],vertices.at(1)[1],vertices.at(1)[2]),Point(vertices.at(3)[0],vertices.at(3)[1],vertices.at(3)[2]));
    triangles.push_back(tri);
    tri = CGALTriangle(Point(vertices.at(5)[0],vertices.at(5)[1],vertices.at(5)[2]),Point(vertices.at(7)[0],vertices.at(7)[1],vertices.at(7)[2]),Point(vertices.at(3)[0],vertices.at(3)[1],vertices.at(3)[2]));
    triangles.push_back(tri);

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "occlusion_culling_test");
    ros::NodeHandle n;

    ros::Publisher pub1 = n.advertise<sensor_msgs::PointCloud2>("point_cloud1", 100);
    ros::Publisher pub2 = n.advertise<sensor_msgs::PointCloud2>("point_cloud2", 100);
    ros::Publisher pub3 = n.advertise<sensor_msgs::PointCloud2>("occlusion_free_cloud", 100);

    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr occlusionFreeCloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::string path = ros::package::getPath("component_test");
    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/sphere_densed.pcd", *cloud);

    pcl::FrustumCulling<pcl::PointXYZ> fc (true);
    fc.setInputCloud (cloud);
    fc.setVerticalFOV (45);
    fc.setHorizontalFOV (58);
    fc.setNearPlaneDistance (0.8);
    fc.setFarPlaneDistance (5.8);

    Eigen::Matrix4f camera_pose;
    camera_pose.setZero ();
    Eigen::Matrix3f R;
    Eigen::Vector3f theta(0.0,180.0,0.0);
    R = Eigen::AngleAxisf (theta[0] * M_PI / 180, Eigen::Vector3f::UnitX ()) *
            Eigen::AngleAxisf (theta[1] * M_PI / 180, Eigen::Vector3f::UnitY ()) *
            Eigen::AngleAxisf (theta[2] * M_PI / 180, Eigen::Vector3f::UnitZ ());
    camera_pose.block (0, 0, 3, 3) = R;
    Eigen::Vector3f T;
    T (0) = 4; T (1) = 0; T (2) = 0;
    camera_pose.block (0, 3, 3, 1) = T;
    camera_pose (3, 3) = 1;
    fc.setCameraPose (camera_pose);
    pcl::PointCloud <pcl::PointXYZ>::Ptr output (new pcl::PointCloud <pcl::PointXYZ>);
    pcl::PointCloud <pcl::PointXYZ>::Ptr occlusion_cloud (new pcl::PointCloud <pcl::PointXYZ>);

    fc.filter (*output);
    //    pcl::PCDWriter writer;
    //    writer.write<pcl::PointXYZRGB> (path+"/src/pcd/frustum_bun.pcd", *output, false);

    //*****************Visualization Camera View Vector (frustum culling tool camera) *****************
    // the rviz axis is different from the frustum camera axis and range image axis
    R = Eigen::AngleAxisf (theta[0] * M_PI / 180, Eigen::Vector3f::UnitX ()) *
            Eigen::AngleAxisf (-theta[1] * M_PI / 180, Eigen::Vector3f::UnitY ()) *
            Eigen::AngleAxisf (-theta[2] * M_PI / 180, Eigen::Vector3f::UnitZ ());
    tf::Matrix3x3 rotation;
    Eigen::Matrix3d D;
    D= R.cast<double>();
    tf::matrixEigenToTF(D,rotation);
    rotation = rotation.transpose();
    tf::Quaternion orientation;
    rotation.getRotation(orientation);

    //    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    geometry_msgs::Pose output_vector;
    Eigen::Quaterniond q;
    geometry_msgs::Quaternion quet;
    tf::quaternionTFToEigen(orientation, q);
    tf::quaternionTFToMsg(orientation,quet);
    Eigen::Affine3d pose;
    Eigen::Vector3d a;
    a[0]= T[0];
    a[1]= T[1];
    a[2]= T[2];
    pose.translation() = a;
    tf::poseEigenToMsg(pose, output_vector);
    visualization_msgs::Marker marker;

    //*****************voxel grid occlusion estimation *****************
    Eigen::Quaternionf quat(q.w(),q.x(),q.y(),q.z());
    //    Eigen::Quaternionf quat(-0.7071067811865476,0,0,0.7071067811865475);
    output->sensor_origin_  = Eigen::Vector4f(T[0],T[1],T[2],0);
    output->sensor_orientation_= quat;
    std::cout<<"output origin= x: "<<output->sensor_origin_[0]<<" y: "<<output->sensor_origin_[1]<<" z: "<<output->sensor_origin_[2]<<"\n";
    std::cout<<"output orientatio= x: "<<output->sensor_orientation_.x()<<" y: "<<output->sensor_orientation_.y()<<" z: "<<output->sensor_orientation_.z()<<" w: "<<output->sensor_orientation_.w()<<"\n";
    pcl::VoxelGridOcclusionEstimation<pcl::PointXYZ> voxelFilter;
    voxelFilter.setInputCloud (output);
    float res = 0.1;
    std::list<CGALTriangle> triangles;
    voxelFilter.setLeafSize (res, res, res);
    voxelFilter.filter(*occlusion_cloud);
    voxelFilter.initializeVoxelGrid();


    Eigen::Vector3i max_b = voxelFilter.getMaxBoxCoordinates();
    Eigen::Vector3i min_b = voxelFilter.getMinBoxCoordinates();
    std::cout<<"minimum box  x: "<<min_b[0]<<" y: "<<min_b[1]<<" z: "<<min_b[2]<<"\n";
    std::cout<<"maximum box  x: "<<max_b[0]<<" y: "<<max_b[1]<<" z: "<<max_b[2]<<"\n";

    for (int kk = min_b[2]; kk <= max_b[2]; ++kk)
        for (int jj = min_b[1]; jj <= max_b[1]; ++jj)
            for (int ii = min_b[0]; ii <= max_b[0]; ++ii)
            {
                int intersectionsCount=0;
                Point a(T[0] , T[1]  ,T[2]);//sensor origin
                Eigen::Vector3i ijk (ii, jj, kk);
                int index = voxelFilter.getCentroidIndexAt (ijk);

                if(index!=-1){ // get the occupied voxels
                    Eigen::Vector4f centroid = voxelFilter.getCentroidCoordinate (ijk);
                    from_voxel(centroid,res, triangles);
                    Tree tree(triangles.begin(),triangles.end());
                    std::cout<<"traingles size:"<<triangles.size()<<"\n";
                    Point b(centroid[0], centroid[1], centroid[2]);
//                    Line l(a,b);

                    //calculate the voxel entry point
                    //using calculations
//                    float distance = std::sqrt(((b[0]-a[0])*(b[0]-a[0]))+((b[1]-a[1])*(b[1]-a[1]))+((b[2]-a[2])*(b[2]-a[2])));
//                    float dist_p2p3= (res/2)+((res/2)*(res/2));//distance from point 2
//                    float ratio = dist_partial/distance;
//                    float x3= (ratio * b[0]) + ((1-ratio)* a[0]);
//                    float y3= (ratio * b[1]) + ((1-ratio)* a[1]);
//                    float z3= (ratio * b[2]) + ((1-ratio)* a[2]);
//                    Point c(x3,y3,z3);


//                    //using CGAL
//                    float dist_p0p3= distance - (res/2)+((res/2)*(res/2)); // distance from a (sensor origin)
//                    Vector v = l.to_vector();
//                    Point c = a + (dist_p0p3 * dist_p0p3 / v.squared_length()) * v;


//                    Ray ray_query(a,b);
//                    Line l2(a,c);
//                    intersectionsCount = tree.number_of_intersected_primitives(l2);
//                    std::cout << "intersections: "<<intersectionsCount<< " intersections(s) with line query" << std::endl;

//                    if(intersectionsCount <= 7)
//                    {
//                        std::cout<<"NOT OCCLUDED POINTS"<<"\n";
                          pcl::PointXYZ pt(centroid[0],centroid[1],centroid[2]);
                          occlusionFreeCloud->points.push_back(pt);
//                     }
                    for (int r=0; r<16; r++)
                        triangles.pop_back();
                }
            }


    //*****************Z buffering test (range_image tool) *****************

    boost::shared_ptr<pcl::RangeImage> cull_ptr(new pcl::RangeImage);
    pcl::RangeImage& visible = *cull_ptr;
    // range image camera z axis orientation is different from the frustum cull camera
    Eigen::Matrix3f R1;
    R1 = Eigen::AngleAxisf (0 * M_PI / 180, Eigen::Vector3f::UnitX ()) *
            Eigen::AngleAxisf (90 * M_PI / 180, Eigen::Vector3f::UnitY ()) *
            Eigen::AngleAxisf (0 * M_PI / 180, Eigen::Vector3f::UnitZ ());
    Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(T[0],T[1],T[2]));
    sensorPose.rotate (R1);
    float angularResolutionX = (float)(50.0f / 640.0f * (M_PI / 180.0f));
    float angularResolutionY = (float)(40.0f / 480.0f * (M_PI / 180.0f));
    float maxAngleX = (float)(50.0f * (M_PI / 180.0f));
    float maxAngleY = (float)(40.0f * (M_PI / 180.0f));
    float noise_level=0.0;
    float min_range=0.0;
    float borderSize=1.0;
    visible.createFromPointCloud(*output, angularResolutionX, angularResolutionY,
                                 maxAngleX, maxAngleY, sensorPose, pcl::RangeImage::CAMERA_FRAME,
                                 noise_level, min_range, borderSize);
    //    uint32_t width  = static_cast<uint32_t> (pcl_lrint (floor (maxAngleX*(1/angularResolutionX))));
    //    uint32_t height = static_cast<uint32_t> (pcl_lrint (floor (maxAngleY*(1/angularResolutionY))));
    //    int top=height, right=-1, bottom=-1, left=width;
    //    visible.doZBuffer(*output, noise_level, min_range, top, right, bottom, left );
    //    visible.recalculate3DPointPositions();

    //*****************Rviz Visualization ************
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        //***marker publishing***
        uint32_t shape = visualization_msgs::Marker::ARROW;
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;
        //visulaization using the markers
        marker.scale.x = 0.5;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.ns = "basic_shapes";
        marker.id = 2;
        // ROS_INFO("Publishing Marker");
        // Set the frame ID and timestamp. See the TF tutorials for information on these.
        marker.pose =  output_vector;
        marker.pose.orientation  = quet;//output_vector.orientation;
        marker.header.frame_id = "base_point_cloud";
        marker.header.stamp = ros::Time::now();
        marker.lifetime = ros::Duration(10);
        // Publish the marker
        marker_pub.publish(marker);

        //***frustum cull and occlusion cull publish***
        sensor_msgs::PointCloud2 cloud1;
        sensor_msgs::PointCloud2 cloud2;
        sensor_msgs::PointCloud2 cloud3;
        pcl::toROSMsg(*output, cloud1); //cloud of frustum cull (white) using pcl::frustumcull
        pcl::toROSMsg(*cull_ptr, cloud2); //cloud of the Occlusion cull (blue) using pcl::rangeImage
        pcl::toROSMsg(*occlusionFreeCloud, cloud3); //cloud of the Occlusion cull (blue) using pcl::rangeImage
        cloud1.header.frame_id = "base_point_cloud";
        cloud2.header.frame_id = "base_point_cloud";
        cloud3.header.frame_id = "base_point_cloud";

        cloud1.header.stamp = ros::Time::now();
        cloud2.header.stamp = ros::Time::now();
        cloud3.header.stamp = ros::Time::now();
        pub1.publish(cloud1);
        pub2.publish(cloud2);
        pub3.publish(cloud3);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
