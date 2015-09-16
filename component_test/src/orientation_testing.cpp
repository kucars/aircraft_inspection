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
#include "test_fcl_utility.h"
#include "fcl/BVH/BV_fitter.h"
#include "fcl/BV/BV.h"
#include "fcl/traversal/traversal_node_bvhs.h"
//#include "fcl/traversal/traversal_node_setup.h"
#include "fcl/collision_node.h"
//#include <boost/timer.hpp>
//#include "fcl_resources/config.h"
#include "fcl/distance.h"
#include "fcl/traversal/traversal_recurse.h"
#include "fcl/traversal/traversal_node_setup.h"


using namespace fcl;
visualization_msgs::Marker drawCUBE(Vec3f vec , int id , int c_color, double size);
visualization_msgs::Marker drawLine(Vec3f p1, Vec3f p2, int id , int c_color, double size);

int main(int argc, char **argv)
{

    ros::init(argc, argv, "collision_distance_test");
    ros::NodeHandle n;
    ros::Publisher model_pub = n.advertise<sensor_msgs::PointCloud2>("point_cloud", 1);
    ros::Publisher point_pub = n.advertise<geometry_msgs::PoseArray>("voxel", 1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Publisher visCubePub = n.advertise<visualization_msgs::MarkerArray>("Cube", 1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    std::string path = ros::package::getPath("component_test");
    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/scaled_desktop.pcd", *cloud);


    int points_num=0;
    geometry_msgs::PoseArray points,vec;
    geometry_msgs::Pose pose[points_num];
    //x is the width of the aircraft, y is the hight of the aircraft, z is the length of the aircraft
    pose[0].position.x=5.0; pose[0].position.y=0.0; pose[0].position.z=-10.0;
    //    pose[1].position.x=0.0; pose[1].position.y=0.0; pose[1].position.z=-5.0;
    //    pose[2].position.x=0.0; pose[2].position.y=0.0; pose[2].position.z=0.0;
    //    pose[3].position.x=0.0; pose[3].position.y=0.0; pose[3].position.z=0.0;

    for (int i =0; i<=points_num ; i++)
    {
        points.poses.push_back(pose[i]);
    }


    //testing the mesh example
    std::vector<Vec3f> p1;
    std::vector<Triangle> t1;
    loadOBJFile("/home/randa/workspace/catkin_ws/src/component_test/src/mesh/desktop_scaleddown.obj", p1, t1);

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
    visualization_msgs::Marker marker ;

    for(int j=0; j<=points_num; j++)
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

        if (result.isCollision() == true )
        {
            std::cout<<"Collision"<<endl;
            marker = drawCUBE(vec2,j,2,1) ;//red
            //            marker_array.markers.push_back(marker);
        }
        else
        {
std:cout<<"NO Collision"<<endl;
            marker = drawCUBE(vec2, j, 1,1) ;//blue
            //            marker_array.markers.push_back(marker);

        }

        //***********distance computation
        DistanceRequest request2;
        DistanceResult localResult;
        distance(&co0, obj, request2, localResult);
        FCL_REAL min_dist = localResult.min_distance;
        std::cout<<"minimum distance point"<<j<<" : "<<min_dist<<std::endl;
        Vec3f p1 = localResult.nearest_points[0];
        Vec3f p2 = localResult.nearest_points[1];
        //        Vec3f p3 = localResult.nearest_points[2];
        //        Vec3f p4 = localResult.nearest_points[3];
        //        Vec3f p5 = localResult.nearest_points[4];
        //        Vec3f p6 = localResult.nearest_points[5];

        std::cout<<"p1 point x: "<<p1[0]<<" p1 point y: "<<p1[1]<<" p1 point z: "<<p1[2]<<std::endl;
        std::cout<<"p2 point x: "<<p2[0]<<" p2 point y: "<<p2[1]<<" p2 point z: "<<p2[2]<<std::endl;
        //        std::cout<<"p3 point x: "<<p3[0]<<" p3 point y: "<<p3[1]<<" p3 point z: "<<p3[2]<<std::endl;
        //        std::cout<<"p4 point x: "<<p4[0]<<" p4 point y: "<<p4[1]<<" p4 point z: "<<p4[2]<<std::endl;
        //        std::cout<<"p5 point x: "<<p5[0]<<" p5 point y: "<<p5[1]<<" p5 point z: "<<p5[2]<<std::endl;
        //        std::cout<<"p6 point x: "<<p6[0]<<" p6 point y: "<<p6[1]<<" p6 point z: "<<p6[2]<<std::endl;

        marker = drawCUBE(p1, 22, 3,0.05) ;//green
        marker_array.markers.push_back(marker);
        //        marker = drawCUBE(p2, 23, 3,0.05) ;//green
        //        marker_array.markers.push_back(marker);

        //        marker = drawCUBE(p3, 24, 3,0.05) ;//green
        //        marker_array.markers.push_back(marker);
        //        marker = drawCUBE(p4, 28, 3,0.05) ;//green
        //        marker_array.markers.push_back(marker);
        //        marker = drawCUBE(p5, 29, 3,0.05) ;//green
        //        marker_array.markers.push_back(marker);
        //        marker = drawCUBE(p6, 30, 3,0.05) ;//green
        //        marker_array.markers.push_back(marker);



        //*******finding the orientation
        //        marker = drawLine(p1, vec2,1000, 3,0.1) ;//green

        // Declare goal output pose
        geometry_msgs::Pose output_vector;
        Eigen::Quaterniond q;

        Eigen::Vector3d axis_vector;
        axis_vector[0]=p1[0]-vec2[0];
        axis_vector[1]=p1[1]-vec2[1];
        axis_vector[2]=p1[2]-vec2[2];
        axis_vector.normalize();
        Eigen::Vector3d up_vector(0.0, 0.0, 1.0);
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
        b[0]=vec2[0]; b[1]=vec2[1]; b[2]=vec2[2];
        pose1 = q * Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitY());
        pose1.translation() = b;
        tf::poseEigenToMsg(pose1, output_vector);



        //trial
        vec.poses.push_back(output_vector);

    }

    visCubePub.publish(marker_array);
    //    marker_pub.publish(marker);


    while(ros::ok())
    {
        //mesh points publish
        sensor_msgs::PointCloud2 cloud1;
        pcl::toROSMsg(*cloud, cloud1);
        cloud1.header.frame_id = "base_point_cloud";
        cloud1.header.stamp = ros::Time::now();
        model_pub.publish(cloud1);

        //visualize the vector with orientation
        vec.header.frame_id= "base_point_cloud";
        vec.header.stamp = ros::Time::now();
        point_pub.publish(vec);

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


visualization_msgs::Marker drawLine(Vec3f p1, Vec3f p2, int id , int c_color, double size)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_point_cloud";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;//poseQ[1];
    marker.pose.orientation.z = 0;//poseQ[2];
    marker.pose.orientation.w = 1;//poseQ[3];

    marker.scale.x = size;
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
    geometry_msgs::Point point1,point2;
    point1.x=p1[0] ; point1.y=p1[1] ; point1.z=p1[2] ;
    point2.x=p2[0] ; point2.y=p2[1] ; point2.z= p2[2];
    marker.points.push_back(point1);
    marker.points.push_back(point2);
    marker.lifetime = ros::Duration(120);
    return marker ;
}
