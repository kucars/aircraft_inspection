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
#include "fcl/traversal/traversal_node_bvhs.h"
//#include "fcl/traversal/traversal_node_setup.h"
#include "fcl/collision_node.h"
//#include <boost/timer.hpp>
//#include "fcl_resources/config.h"
#include "fcl/distance.h"
#include "fcl/traversal/traversal_recurse.h"
#include "fcl/traversal/traversal_node_setup.h"



//inline double dist(pcl::PointXYZ &p1, geometry_msgs::Pose &p2)
//{
//    return sqrt((p2.position.x - p1.x)*(p2.position.x - p1.x) + (p2.position.y - p1.y)*(p2.position.y - p1.y) + (p2.position.z - p1.z)*(p2.position.z - p1.z));
//}
using namespace fcl;
visualization_msgs::Marker drawCUBE(Vec3f vec , int id , int c_color, double size);

int main(int argc, char **argv)
{

    ros::init(argc, argv, "collision_distance_test");
    ros::NodeHandle n;
    ros::Publisher model_pub = n.advertise<sensor_msgs::PointCloud2>("point_cloud", 1);
//    ros::Publisher point_pub = n.advertise<geometry_msgs::PoseArray>("voxel", 1);
//    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Publisher visCubePub = n.advertise<visualization_msgs::MarkerArray>("Cube", 1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    std::string path = ros::package::getPath("component_test");
    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/scaled_desktop.pcd", *cloud);


    int points_num=0;
    geometry_msgs::PoseArray points;
    geometry_msgs::Pose pose[points_num];
    //x is the width of the aircraft, y is the hight of the aircraft, z is the length of the aircraft
    pose[0].position.x=0.0; pose[0].position.y=0.0; pose[0].position.z=-1.0;
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
    std::string str = path+"/src/mesh/desktop_scaled_filled1.obj";
    loadOBJFile(str.c_str(), p1, t1);

    BVHModel<OBBRSS>* m1 = new BVHModel<OBBRSS>();
    m1->bv_splitter.reset(new BVSplitter<OBBRSS>(SPLIT_METHOD_MEAN));
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
        boost::shared_ptr<Box> Sample(new Box(0.2));
        tf0.setIdentity();
        tf0.setTranslation(Vec3f(points.poses[j].position.x , points.poses[j].position.y  ,points.poses[j].position.z ));
        CollisionObject co0(Sample, tf0);
        static const int num_max_contacts = std::numeric_limits<int>::max();
        static const bool enable_contact = true ;
        fcl::CollisionResult result;
        fcl::CollisionRequest request(num_max_contacts, enable_contact);
        fcl::collide(&co0, obj, request, result);
//        MeshCollisionTraversalNodeOBB node;
//        if(!initialize(node, (const BVHModel<OBB>&)co0, tf0, (const BVHModel<OBB>&)obj, tf1, CollisionRequest(), result))
//            std::cout << "initialize error" << std::endl;
//        bool verbose;
//        node.enable_statistics = verbose;
//        collide(&node);

        AABB a    = co0.getAABB() ;
        Vec3f vec2 =  a.center() ;

        if (result.isCollision() == true )
        {
            std::cout<<"Collision"<<endl;
            marker = drawCUBE(vec2,j,2,0.2) ;//red
            marker_array.markers.push_back(marker);
        }
        else
        {
            std:cout<<"NO Collision"<<endl;
            marker = drawCUBE(vec2, j, 1,0.2) ;//blue
            marker_array.markers.push_back(marker);

        }

        DistanceRequest request2;
        DistanceResult localResult;
        distance(&co0, obj, request2, localResult);
        FCL_REAL min_dist = localResult.min_distance;
//        Vec3f vec= localResult.nearest_points[0];
//        marker = drawCUBE(vec, 222, 3,0.005) ;//green
//        marker_array.markers.push_back(marker);
        std::cout<<"minimum distance point"<<j<<" : "<<min_dist<<std::endl;


    }

    visCubePub.publish(marker_array);

    while(ros::ok())
    {
        //mesh points publish
        sensor_msgs::PointCloud2 cloud1;
        pcl::toROSMsg(*cloud, cloud1);
        cloud1.header.frame_id = "base_point_cloud";
        cloud1.header.stamp = ros::Time::now();
        model_pub.publish(cloud1);

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

