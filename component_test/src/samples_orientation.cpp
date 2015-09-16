#include "ros/ros.h"
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <fstream>
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
//#include "fcl/traversal/traversal_node_bvhs.h"
//#include "fcl/traversal/traversal_node_setup.h"
//#include "fcl/collision_node.h"
//#include <boost/timer.hpp>
//#include "fcl_resources/config.h"
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

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

//inline double dist(pcl::PointXYZ &p1, geometry_msgs::Pose &p2)
//{
//    return sqrt((p2.position.x - p1.x)*(p2.position.x - p1.x) + (p2.position.y - p1.y)*(p2.position.y - p1.y) + (p2.position.z - p1.z)*(p2.position.z - p1.z));
//}
using namespace fcl;
visualization_msgs::Marker drawCUBE(Vec3f vec , int id , int c_color, double size);
geometry_msgs::Pose calcOrienation(Vec3f position,Vec3f nearestP);
geometry_msgs::Vector3 rpy;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "discretization");
    ros::NodeHandle n;
    ros::Publisher model_pub = n.advertise<sensor_msgs::PointCloud2>("point_cloud", 1);
    ros::Publisher point_pub = n.advertise<geometry_msgs::PoseArray>("voxel", 1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Publisher OctmapPub = n.advertise<octomap_msgs::Octomap>("LaserOctmap", 1);
    ros::Publisher visCubePub = n.advertise<visualization_msgs::MarkerArray>("Cube", 1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    std::string path = ros::package::getPath("component_test");
    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/scaled_desktop.pcd", *cloud);
    //    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/plane_desktop.pcd", *cloud);
    //      pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/bun000_Structured.pcd", *cloud);

    geometry_msgs::PoseArray points,vectors;
    geometry_msgs::Pose pose;

    int x_space=16;//put half the length here (32)
    int y_space=11;//put half the length here (18)
    int z_space=37;
    double res=1;
    for (int z=(-1*z_space) ; z < 2; z+=res)//the length of the aircraft
    {

        for (int y=-1*(y_space-4) ; y< y_space; y+=res)//the hight of the aircraft
        {

            for (int x=-1*x_space ; x< x_space; x+=res)//the width of the aircraft
            {
                pose.position.z=z;
                pose.position.y=y;
                pose.position.x=x;
                pose.orientation.x=0;pose.orientation.y=0;pose.orientation.z=0;pose.orientation.w=1;
                points.poses.push_back(pose);

            }
        }
    }

    //filtering
    //testing the mesh example
    std::vector<Vec3f> p1;
    std::vector<Triangle> t1;
    std::string str = path+"/src/mesh/desktop_scaleddown.obj";
    loadOBJFile(str.c_str(), p1, t1);

    BVHModel<OBBRSS>* m1 = new BVHModel<OBBRSS>();
    boost::shared_ptr<CollisionGeometry> m1_ptr(m1);

    m1->beginModel();
    m1->addSubModel(p1, t1);
    m1->endModel();

    Transform3f tf1;
    tf1.setIdentity();
    tf1.setTranslation(Vec3f(0,0,0));
    CollisionObject* obj = new CollisionObject(boost::shared_ptr<CollisionGeometry>(m1), tf1);
    Transform3f tf0;

    visualization_msgs::MarkerArray marker_array ;
    visualization_msgs::Marker marker2 ;

    //filtered_points for creating a mesh of points
    vtkSmartPointer<vtkPoints> filtered_points = vtkSmartPointer< vtkPoints >::New();

    //openning the file for writing the points positions and the orientation
    ofstream pointfile;
    std::string file_loc = path+"/src/poses.txt";
    pointfile.open (file_loc.c_str());
    for (int j=0; j<points.poses.size();j++)
    {
        boost::shared_ptr<Box> Sample(new Box(1));
        tf0.setIdentity();
        tf0.setTranslation(Vec3f(points.poses[j].position.x , points.poses[j].position.y  ,points.poses[j].position.z ));
        CollisionObject co0(Sample, tf0);
        static const int num_max_contacts = std::numeric_limits<int>::max();
        static const bool enable_contact = true ;
        fcl::CollisionResult result;
        fcl::CollisionRequest request(num_max_contacts, enable_contact);
        fcl::collide(&co0, obj, request, result);
        AABB a    = co0.getAABB() ;
        Vec3f vec2 =  a.center() ;

        //        if (result.isCollision() == true )
        //        {
        //////            marker = drawCUBE(vec2,i,2) ;
        //////            marker_array.markers.push_back(marker);
        //////            collisionFlag.data = true ;
        //////            collisionFlagPub.publish(collisionFlag) ;
        //////            collisionDetected = true;
        //        }
        //        else
        //        {
        //            marker2 = drawCUBE(vec2, j, 1) ;
        //            marker_array.markers.push_back(marker2);
        //        }

        DistanceRequest request2;
        DistanceResult localResult;
        distance(&co0, obj, request2, localResult);
        FCL_REAL min_dist = localResult.min_distance;
        //        std::cout<<"minimum distance: "<<min_dist<<std::endl;
        if(min_dist <= 2 && min_dist >= 1.0)
        {
            marker2 = drawCUBE(vec2, j, 1, 0.3) ;
            marker_array.markers.push_back(marker2);
            //filtered_points for creating a mesh of points
            filtered_points->InsertNextPoint(points.poses[j].position.x, points.poses[j].position.y, points.poses[j].position.z);
            Vec3f nearestP = localResult.nearest_points[0];
            Vec3f position = vec2;
            vectors.poses.push_back(calcOrienation(position,nearestP));
            pointfile << points.poses[j].position.x<<" "<<points.poses[j].position.y<<" "<<points.poses[j].position.z<<" "<<rpy.x<<" "<<rpy.y<<" "<<rpy.z<<"\n";
            //            if(min_dist > 1 && min_dist < 1.3)
            //            {
            //                std::cout<<"minimum distance: "<<min_dist<<std::endl;
            //                std::cout<<"points position x: "<<points.poses[j].position.x<<" points position y: "<<points.poses[j].position.y<<" points position z: "<<points.poses[j].position.z<<std::endl;
            //            }
        }

    }
    //file close
    pointfile.close();

    //***************making a mesh (Optional)*******************
    //    vtkSmartPointer< vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    //    polydata->SetPoints(filtered_points);

    //    vtkSmartPointer<vtkXMLPolyDataWriter> pointsWriter = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
    //    pointsWriter->SetFileName("points.vtp");
    //    pointsWriter->SetInput(polydata);
    //    pointsWriter->Write();

    //    //convert vtp to ply mesh
    //    std::string inputFileName = "points.vtp";
    //    std::string outputFileName = "samples.ply";

    //    vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
    //    reader->SetFileName(inputFileName.c_str());
    //    reader->Update();

    //    vtkSmartPointer<vtkPLYWriter> writer = vtkSmartPointer<vtkPLYWriter>::New();
    //    writer->SetFileName(outputFileName.c_str());
    //    writer->SetInputConnection(reader->GetOutputPort());
    //    writer->Update();
    //    ROS_INFO("DONE :) samples mesh is generated");
    //***********************************************

    visCubePub.publish(marker_array);
    visualization_msgs::Marker marker;

    while(ros::ok())
    {
        //mesh points publish
        sensor_msgs::PointCloud2 cloud1;
        pcl::toROSMsg(*cloud, cloud1);
        cloud1.header.frame_id = "base_point_cloud";
        cloud1.header.stamp = ros::Time::now();
        model_pub.publish(cloud1);


        //visualize the points
        vectors.header.frame_id= "base_point_cloud";
        vectors.header.stamp = ros::Time::now();
        point_pub.publish(vectors);

        //        octomap_msgs::Octomap octomap ;
        //        octomap.binary = 1 ;
        //        octomap.id = 1 ;
        //        octomap.resolution =1.0;
        //        octomap.header.frame_id = "base_point_cloud";
        //        octomap.header.stamp = ros::Time::now();
        //        bool res = octomap_msgs::fullMapToMsg(*octTree, octomap);
        //        if(res)
        //        {
        //            OctmapPub.publish(octomap);
        //        }
        //        else
        //        {
        //            ROS_WARN("OCT Map serialization failed!");
        //        }

        ros::spinOnce();
    }


    return 0;
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

geometry_msgs::Pose calcOrienation(Vec3f position,Vec3f nearestP)
{
    geometry_msgs::Pose output_vector;
    Eigen::Quaterniond q;

    Eigen::Vector3d axis_vector;
//    axis_vector[0]=position[0]-nearestP[0];
//    axis_vector[1]=position[1]-nearestP[1];
//    axis_vector[2]=position[2]-nearestP[2];
    axis_vector[0]=nearestP[0]-position[0];
    axis_vector[1]=nearestP[1]-position[1];
    axis_vector[2]=nearestP[2]-position[2];
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
    b[0]=position[0]; b[1]=position[1]; b[2]=position[2];
    pose1 = q * Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitY());
    pose1.translation() = b;
    tf::poseEigenToMsg(pose1, output_vector);


    double roll, pitch, yaw;
    tf::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;

    std::cout<<"roll: "<<roll<<" pitch: "<<pitch<<" yaw: "<<yaw<<std::endl;
    return output_vector;

}
