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
#include <fstream>
#include <tf/transform_datatypes.h>
//PCL
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/filters/frustum_culling.h>
//#include <frustum_culling.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image.h>
//#include <voxel_grid_occlusion_estimation.h>
#include <pcl/filters/voxel_grid.h>

#include "fcl_utility.h"
//CGAL
#include <list>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>

#include <stdlib.h>
#include <boost/foreach.hpp>
#include <Eigen/Eigen>
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>

#include <tf/transform_broadcaster.h>
#include <component_test/occlusion_culling.h>
#include <component_test/occlusion_culling_gpu.h>
#include <component_test/visualization_voxelgrid.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
using namespace fcl;


typedef CGAL::Simple_cartesian<double> K;
typedef K::FT FT;
typedef K::Ray_3 Ray;
typedef K::Line_3 Line;
typedef K::Point_3 Point;
typedef K::Triangle_3 CGALTriangle;
typedef std::list<CGALTriangle>::iterator Iterator;
typedef CGAL::AABB_triangle_primitive<K, Iterator> Primitive;
typedef CGAL::AABB_traits<K, Primitive> AABB_triangle_traits;
typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;


void loadOBJFile(const char* filename, std::vector<Vec3f>& points, std::list<CGALTriangle>& triangles);
visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links, int id, int c_color);
visualization_msgs::Marker drawCUBE(Vec3f vec , int id , int c_color, double size);
void computeOrientations(geometry_msgs::Pose pos,double deg, geometry_msgs::PoseArray& vectors);//orintation discretization by angle
void coverageFiltering(geometry_msgs::PoseArray& invectors, geometry_msgs::PoseArray& uavVectors, std::vector<geometry_msgs::PoseArray> &camVectors, std::vector<geometry_msgs::Pose> camsPose, OcclusionCullingGPU &obj);
void distanceFiltering(double min,double max,double id, geometry_msgs::PoseArray& uavVectors, Tree& tree, Point& a, visualization_msgs::MarkerArray& marker_array);
geometry_msgs::Pose uav2camTransformation(geometry_msgs::Pose pose, Vec3f rpy, Vec3f xyz);


pcl::PointCloud<pcl::PointXYZ>::Ptr globalCloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ> globalCloud;
std::vector<pcl::PointCloud<pcl::PointXYZ> > accuracy_clusters; //vector to store clusters

int main(int argc, char **argv)
{

    ros::init(argc, argv, "filtering");
    ros::NodeHandle n;

    ros::Publisher pub1 = n.advertise<sensor_msgs::PointCloud2>("original_point_cloud", 100);
    ros::Publisher pub2 = n.advertise<sensor_msgs::PointCloud2>("covered_cloud", 100);
    ros::Publisher pub3 = n.advertise<sensor_msgs::PointCloud2>("extra_cloud", 100);
    ros::Publisher pub4 = n.advertise<sensor_msgs::PointCloud2>("colored_clusters", 100);

    ros::Publisher oriented_point_pub = n.advertise<geometry_msgs::PoseArray>("oriented_poses", 1);
    ros::Publisher poses_pub = n.advertise<visualization_msgs::MarkerArray>("filtered_poses", 1);
    ros::Publisher fov_pub = n.advertise<visualization_msgs::MarkerArray>("fov", 10);

    ros::Publisher lines_pub1 = n.advertise<visualization_msgs::Marker>("fov_far_near", 10);
    ros::Publisher lines_pub2 = n.advertise<visualization_msgs::Marker>("fov_top", 10);
    ros::Publisher lines_pub3 = n.advertise<visualization_msgs::Marker>("fov_bottom", 10);

    OcclusionCullingGPU obj(n,"etihad_nowheels_nointernal_scaled_newdensed.pcd");
    std::string path = ros::package::getPath("component_test");


    //////////////Initial discretization/////////////////
    geometry_msgs::PoseArray points;
    geometry_msgs::Pose pose;

    int x_space=18;//half wingspan
    int y_space=25;//half the length
    int z_space=11;// the height
    float res=4.5;
    ros::Time disretization_begin = ros::Time::now();
    for (float z=(1*z_space) ; z > 0; z-=res)//the length of the aircraft (it was >-2 because of the wheels)
    {

        for (float y=-1*(y_space) ; y< y_space; y+=res)//the hight of the aircraft
        {

            for (float x=-1*x_space ; x< x_space; x+=res)//the width of the aircraft
            {
                pose.position.z=z;
                pose.position.y=y;
                pose.position.x=x;
                pose.orientation.x=0;pose.orientation.y=0;pose.orientation.z=0;pose.orientation.w=1;
                points.poses.push_back(pose);

            }
        }
    }

    ros::Time disretization_end = ros::Time::now();
    double elapsed =  disretization_end.toSec() - disretization_begin.toSec();
    std::cout<<"discretization duration (s) = "<<elapsed<<"\n";
    std::cout<<"discretized points size: "<<points.poses.size()<<"\n";


    //////////////Sensor Registration/////////////////
    std::vector<ros::Publisher> cam_publishers;
    std::vector<geometry_msgs::PoseArray> camsVec;
    std::vector<geometry_msgs::Pose> camsPose;

    int sensorNum = 2;
    cout<<"number of onboard sensors is: "<<sensorNum<<std::endl;

    //TODO: change to read sensors positions from yaml file
    geometry_msgs::Pose cm;
    cm.position.x = 0; cm.position.y = 0.022; cm.position.z = 0.065;
    cm.orientation.x = 0; cm.orientation.y = -0.349; cm.orientation.z = 0;
    camsPose.push_back(cm);
    cm.position.x = 0; cm.position.y = 0.022; cm.position.z = -0.065;
    cm.orientation.x = 0; cm.orientation.y = 0.349; cm.orientation.z = 0;
    camsPose.push_back(cm);
    
    int num=0;
    while (sensorNum != 0)
    {
	// camsVec  -> array of camera position arrays (pose array)
        geometry_msgs::PoseArray cam;
        camsVec.push_back(cam);
	
	//creating publishers
        std::stringstream ss;
        ss << num;
        std::string sensorTopic = "cam_poses"+ss.str();
        ros::Publisher cam_posespub = n.advertise<geometry_msgs::PoseArray>(sensorTopic.c_str(), 10);
        cam_publishers.push_back(cam_posespub);
        sensorNum--;
        num++;
    }

    
    //////////////Initial Filtering/////////////////
    geometry_msgs::PoseArray filtered_vectors;
    visualization_msgs::MarkerArray marker_array ;
       
    std::vector<Vec3f> pt1;
    std::list<CGALTriangle> triangles;
    int intersectionsCount=0;
    std::string str = path+"/src/mesh/etihad_nowheels_nointernal_scaled_new.obj";
    ros::Time filtering_begin = ros::Time::now();
    loadOBJFile(str.c_str(), pt1, triangles);   
    Tree tree(triangles.begin(),triangles.end()); // constructs AABB tree
    std::cout<<"traingles size:"<<triangles.size()<<"\n";
    
    
    for (int j=0; j<points.poses.size();j++)
    {
        Point a(points.poses[j].position.x , points.poses[j].position.y  ,points.poses[j].position.z);
        // Some Random point in arbitrary orientation
        Point b(100.0, 10.0, 56.0);
        Ray ray_query(a,b);
        // counts #intersections
        intersectionsCount = tree.number_of_intersected_primitives(ray_query);
	
        // Inside if the number of intersections is odd
        geometry_msgs::PoseArray vectors;
        if(intersectionsCount%2 != 1)
        {
            distanceFiltering(1,16,j,vectors, tree, a, marker_array);
            coverageFiltering(vectors,filtered_vectors,camsVec,camsPose,obj);

//            std::cout << "filtered Vectors #"<<filtered_vectors.poses.size()<< std::endl;
        }
    }

    std::cout << "filtered Vectors #"<<filtered_vectors.poses.size()<< std::endl;

    ros::Time filtering_end = ros::Time::now();
    elapsed =  filtering_end.toSec() - filtering_begin.toSec();
    std::cout<<"filtering duration (s) = "<<elapsed<<"\n";
    std::cout<<"filtered points size = "<<filtered_vectors.poses.size()<<"\n";

    
    
    //////////////Voxel Filtering original cloud (used to find the uncovered part)////////
    float voxelRes = 0.1f;
    pcl::PointCloud<pcl::PointXYZ> modelVoxels;
    pcl::VoxelGridOcclusionEstimationGPU originalCloudFilteredVoxels;
    originalCloudFilteredVoxels.setInputCloud (obj.cloud);
    originalCloudFilteredVoxels.setLeafSize (voxelRes, voxelRes, voxelRes);
    originalCloudFilteredVoxels.initializeVoxelGrid();
    originalCloudFilteredVoxels.filter(modelVoxels);
    std::cout<<"original filtered "<<modelVoxels.points.size()<<"\n";



    //******************************//
    //****Dynamic Sampling Loop*****//
    //******************************//
    while (true)
    {
    ////////////////////////////find the Uncovered part////////////////////////////
    globalCloudPtr->points=globalCloud.points;
    pcl::PointCloud<pcl::PointXYZ> coveredVoxels;

    pcl::VoxelGridOcclusionEstimationGPU coveredCloudFilteredVoxels;
    coveredCloudFilteredVoxels.setInputCloud (globalCloudPtr);
    coveredCloudFilteredVoxels.setLeafSize (voxelRes, voxelRes, voxelRes);
    coveredCloudFilteredVoxels.initializeVoxelGrid();
    coveredCloudFilteredVoxels.filter(coveredVoxels);
    std::cout<<" ////// Coverage % : "<<((double)coveredVoxels.size()/(double)modelVoxels.size())*100<<" //////" <<std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr diffPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr diffPtrNoColor(new pcl::PointCloud<pcl::PointXYZ>);
    //iterate through the entire coverage grid to check the number of matched voxel between the original and the covered ones
    Eigen::Vector3i  min_b = originalCloudFilteredVoxels.getMinBoxCoordinates ();
    Eigen::Vector3i  max_b = originalCloudFilteredVoxels.getMaxBoxCoordinates ();
    int extraNum=0;
    for (int kk = min_b.z (); kk <= max_b.z (); ++kk)
    {
        for (int jj = min_b.y (); jj <= max_b.y (); ++jj)
        {
            for (int ii = min_b.x (); ii <= max_b.x (); ++ii)
            {

                Eigen::Vector3i ijk (ii, jj, kk);
                int index1 = originalCloudFilteredVoxels.getCentroidIndexAt (ijk);
                if(index1!=-1)
                {
                    Eigen::Vector4f centroid = originalCloudFilteredVoxels.getCentroidCoordinate (ijk);
                    Eigen::Vector3i ijk_in_Original= coveredCloudFilteredVoxels.getGridCoordinates(centroid[0],centroid[1],centroid[2]) ;

                    int index = coveredCloudFilteredVoxels.getCentroidIndexAt (ijk_in_Original);

                    if(index==-1)
                    {
                        pcl::PointXYZRGB point = pcl::PointXYZRGB(0,244,0);
                        pcl::PointXYZ pt ;
                        point.x = centroid[0];
                        pt.x = centroid[0];
                        point.y = centroid[1];
                        pt.y = centroid[1];
                        point.z = centroid[2];
                        pt.z = centroid[2];
                        diffPtr->points.push_back(point);
                        diffPtrNoColor->points.push_back(pt);
                        extraNum++;
                    }
                }

            }
        }
    }
    //    std::cout<<"number of diff cloud: "<<diffPtrNoColor->points.size()<<std::endl;
    //    std::cout<<"number of diff cloud: "<<diffPtr->points.size()<<std::endl;

    ///////////check the difference percentage///////////////
    double uncoveredPercent = ((double) diffPtr->size()/ (double) modelVoxels.size()) *100;
    std::cout<<" /////// Uncovered % : "<<uncoveredPercent <<" ////////"<<std::endl;
    if(uncoveredPercent<=0.01 && accuracy_clusters.size()>0) //termination condition
        break;

    
    ///////////cluster the uncovered part into regions///////////////
    
    //divide the un covered parts into subregions (using euclidean clustering)
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr Kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
    Kdtree->setInputCloud (diffPtrNoColor);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.3); // 30cm
    ec.setMinClusterSize (5);
    ec.setMaxClusterSize (10000);
    ec.setSearchMethod (Kdtree);
    ec.setInputCloud (diffPtrNoColor);
    ec.extract (cluster_indices);
    std::cout<<"number of clusters: "<<cluster_indices.size()<<std::endl;
    
    

    //////////converting the clusters to pointcloud/////////////////
    int j = 0;
    int r =0,g=0,b=0;
    std::vector<pcl::PointCloud<pcl::PointXYZ> > clusters_pointcloud; //vector to store clusters
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredClusters (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back (diffPtrNoColor->points[*pit]); //*
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      clusters_pointcloud.push_back(*cloud_cluster);
      r+=10; b+=30; g+=20; //coloring
      for(int i=0; i<cloud_cluster->points.size(); i++)
      {

          pcl::PointXYZRGB point = pcl::PointXYZRGB(r,g,b);
          point.x = cloud_cluster->points[i].x;
          point.y = cloud_cluster->points[i].y;
          point.z = cloud_cluster->points[i].z;
          coloredClusters->points.push_back(point);
      }
      std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

      j++;
    }
    std::cout<<" clusters_pointcloud clusters size : "<< clusters_pointcloud.size() <<std::endl;
    for(int i =0; i<accuracy_clusters.size(); i++)
    {
        clusters_pointcloud.push_back(accuracy_clusters[i]);
        //accuracy_clusters.pop_back();
    }
    std::cout<<" clusters_pointcloud clusters size after adding the accuracy: "<< clusters_pointcloud.size() <<std::endl;
    std::cout<<" accuracy clusters size : "<< accuracy_clusters.size() <<std::endl;
    accuracy_clusters.erase(accuracy_clusters.begin(),accuracy_clusters.end());

    std::cout<<" accuracy clusters size after free: "<< accuracy_clusters.size() <<std::endl;

    /////////////loop through the clusters and perform discretization and filtering//////////
    res -= 1.5;
    std::cout<<"resolution updated: "<<res;
    if(res>0)
    {
        for (int i =0 ; i<clusters_pointcloud.size(); i++)
        {

            pcl::PointCloud<pcl::PointXYZ>::Ptr clusterPtr(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ> cluster;

            //finding bounding box for sampling
            clusterPtr->points = clusters_pointcloud[i].points;
            pcl::VoxelGridT grid;
            grid.setInputCloud (clusterPtr);
            grid.setLeafSize (voxelRes, voxelRes, voxelRes);
            grid.filter(cluster);

            VoxelGridVisualization vgVis(grid);
            //VoxelGridVisualization vgVis(n,"etihad_nowheels_densed.pcd",grid,"base_point_cloud");
            vgVis.getBBCentroids();
            //vgVis.visualizeBB();

            geometry_msgs::Point gridSize;
            gridSize.x = std::abs(vgVis.bb_centroids[7].x()-vgVis.bb_centroids[0].x());
            gridSize.y = std::abs(vgVis.bb_centroids[7].y()-vgVis.bb_centroids[0].y());
            gridSize.z = std::abs(vgVis.bb_centroids[7].z()-vgVis.bb_centroids[0].z());

            //discretization
            geometry_msgs::PoseArray clusterSamplePoints;

            for(float x = vgVis.bb_centroids[0].x()-3; x<=(vgVis.bb_centroids[0].x() + gridSize.x+3); x+=(res))
                for(float y = vgVis.bb_centroids[0].y()-3; y<=(vgVis.bb_centroids[0].y() + gridSize.y+3); y+=(res))
                    for(float z = vgVis.bb_centroids[0].z()-3; z<=(vgVis.bb_centroids[0].z() + gridSize.z+3); z+=(res))
                    {
                        pose.position.z=z;
                        pose.position.y=y;
                        pose.position.x=x;
                        pose.orientation.x=0;pose.orientation.y=0;pose.orientation.z=0;pose.orientation.w=1;
                        clusterSamplePoints.poses.push_back(pose);
                    }

            //filtering
            for (int j=0; j<clusterSamplePoints.poses.size();j++)
            {
                Point a(clusterSamplePoints.poses[j].position.x , clusterSamplePoints.poses[j].position.y  ,clusterSamplePoints.poses[j].position.z);
                // Some Random point in arbitrary orientation
                Point b(100.0, 10.0, 56.0);
                Ray ray_query(a,b);
                // counts #intersections
                intersectionsCount = tree.number_of_intersected_primitives(ray_query);

                // Inside if the number of intersections is odd
                geometry_msgs::PoseArray vectors;
                if(intersectionsCount%2 != 1)
                {
                    distanceFiltering(1,16,j,vectors, tree, a, marker_array);
                    coverageFiltering(vectors,filtered_vectors,camsVec,camsPose,obj);
//                    std::cout << "filtered Vectors #"<<filtered_vectors.poses.size()<< std::endl;
                }               
            }//filtering
            std::cout << "filtered Vectors #"<<filtered_vectors.poses.size()<< std::endl;

        }//clusters loop
    }//if statement
    else break;
    }//big loop

    ///////////// Writing waypoints and viewpoints and covered cloud to files //////////
    std::string modelName = "etihad_nowheels_nointernal_newdensed.pcd";

    for(int i=0; i<camsPose.size();i++)
    {
        std::stringstream ss;
        ss << i;
        std::string file_loc = path+"/src/txt/SearchSpaceCam_1.5m_1to4_"+modelName+"_"+ss.str()+".txt";
        const char * filename = file_loc.c_str();
        std::ofstream cameraPointFile(filename);
        for(int j=0; j<camsVec[i].poses.size(); j++)
            cameraPointFile << camsVec[i].poses[j].position.x<<" "<<camsVec[i].poses[j].position.y<<" "<<camsVec[i].poses[j].position.z<<" "<<camsVec[i].poses[j].orientation.x<<" "<<camsVec[i].poses[j].orientation.y<<" "<<camsVec[i].poses[j].orientation.z<<" "<<camsVec[i].poses[j].orientation.w<<"\n";

        cameraPointFile.close();
    }
    ofstream pointFile;

    std::string file_loc = path+"/src/txt/SearchSpaceUAV_1.5m_1to4_"+modelName+"_dynamic.txt";
    pointFile.open (file_loc.c_str());
    for (int j=0; j<filtered_vectors.poses.size(); j++)
        pointFile << filtered_vectors.poses[j].position.x<<" "<<filtered_vectors.poses[j].position.y<<" "<<filtered_vectors.poses[j].position.z<<" "<<filtered_vectors.poses[j].orientation.x<<" "<<filtered_vectors.poses[j].orientation.y<<" "<<filtered_vectors.poses[j].orientation.z<<" "<<filtered_vectors.poses[j].orientation.w<<"\n";

    pointFile.close();

    //PCD file writing
    //write the occlusionfreecloud to pcd file used by the coverage_quantification to calculate the percentage
    globalCloudPtr->points=globalCloud.points;
    globalCloudPtr->width = globalCloudPtr->points.size();
    globalCloudPtr->height = 1;
    pcl::PCDWriter writer;
    std::stringstream ss;
    ss << res;
    writer.write<pcl::PointXYZ> (path+"/src/pcd/occlusionFreeCloud_"+ ss.str()+"m_1to4_etihadNoWheels_dynamic.pcd", *(globalCloudPtr), false);
    std::cout<<" DONE writing files"<<"\n";



    ros::Rate loop_rate(100);//it was 10
    sensor_msgs::PointCloud2 cloud1,cloud2,cloud3,cloud4;
    while (ros::ok())
    {


        pcl::toROSMsg(modelVoxels, cloud1); //cloud of original (white) using original cloud
//        pcl::toROSMsg(*diffPtr, cloud2);
//        pcl::toROSMsg(coveredVoxels, cloud3);
//        pcl::toROSMsg(*coloredClusters, cloud4);

        cloud1.header.frame_id = "base_point_cloud";
//        cloud2.header.frame_id = "base_point_cloud";
//        cloud3.header.frame_id = "base_point_cloud";
//        cloud4.header.frame_id = "base_point_cloud";

        cloud1.header.stamp = ros::Time::now();
//        cloud2.header.stamp = ros::Time::now();
//        cloud3.header.stamp = ros::Time::now();
//        cloud4.header.stamp = ros::Time::now();

        pub1.publish(cloud1);
//        pub3.publish(cloud2);
//        pub2.publish(cloud3);
//        pub4.publish(cloud4);

        //visualize the filtered samples and their orientation
        filtered_vectors.header.frame_id= "base_point_cloud";
        filtered_vectors.header.stamp = ros::Time::now();
        oriented_point_pub.publish(filtered_vectors);

        for(int i=0; i<cam_publishers.size(); i++)
        {
            camsVec[i].header.frame_id= "base_point_cloud";
            camsVec[i].header.stamp = ros::Time::now();
            cam_publishers[i].publish(camsVec[i]);
        }


        poses_pub.publish(marker_array);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links, int id, int c_color)
{
    visualization_msgs::Marker linksMarkerMsg;
    linksMarkerMsg.header.frame_id="base_point_cloud";
    linksMarkerMsg.header.stamp=ros::Time::now();
    linksMarkerMsg.ns="link_marker";
    linksMarkerMsg.id = id;
    linksMarkerMsg.type = visualization_msgs::Marker::LINE_LIST;
    linksMarkerMsg.scale.x = 0.04;
    linksMarkerMsg.action  = visualization_msgs::Marker::ADD;
    linksMarkerMsg.lifetime  = ros::Duration(1000);
    std_msgs::ColorRGBA color;
    //    color.r = 1.0f; color.g=.0f; color.b=.0f, color.a=1.0f;
    if(c_color == 1)
    {
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
        color.a = 1.0;
    }
    else if(c_color == 2)
    {
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
        color.a = 1.0;
    }
    else
    {
        color.r = 0.0;
        color.g = 0.0;
        color.b = 1.0;
        color.a = 1.0;
    }
    std::vector<geometry_msgs::Point>::iterator linksIterator;
    for(linksIterator = links.begin();linksIterator != links.end();linksIterator++)
    {
        linksMarkerMsg.points.push_back(*linksIterator);
        linksMarkerMsg.colors.push_back(color);
    }
    return linksMarkerMsg;
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

void computeOrientations(geometry_msgs::Pose pos,double deg, geometry_msgs::PoseArray& outvectors)
{

    int orientationsNum= 360/deg;
    double yaw=0.0;//radians
    tf::Quaternion tf_q ;

    for(int i=0; i<orientationsNum;i++)
    {
        tf_q= tf::createQuaternionFromYaw(yaw);
        pos.orientation.x = tf_q.getX();
        pos.orientation.y = tf_q.getY();
        pos.orientation.z = tf_q.getZ();
        pos.orientation.w = tf_q.getW();
//        std::cout<<"orientation: "<<pos.orientation.x<<" "<<pos.orientation.y<<" "<<pos.orientation.z<<" "<<pos.orientation.w<<"\n";
        outvectors.poses.push_back(pos);
        yaw = yaw+(deg*M_PI/180);
    }

}

void coverageFiltering(geometry_msgs::PoseArray& invectors, geometry_msgs::PoseArray& uavVectors, std::vector<geometry_msgs::PoseArray>& camVectors, std::vector<geometry_msgs::Pose> camsPose, OcclusionCullingGPU& obj)
{
    geometry_msgs::Pose loc;

    //going through the UAV samples
    for (int i=0; i<invectors.poses.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ> pts;
        std::vector<geometry_msgs::Pose> poses;
        //going through the sensors per the UAV samples
        double accuracy;
        for (int j =0; j<camsPose.size() ; j++)
        {
            Vec3f xyz(camsPose[j].position.x, camsPose[j].position.y, camsPose[j].position.z);
            Vec3f rpy(camsPose[j].orientation.x, camsPose[j].orientation.y, camsPose[j].orientation.z);
            loc= uav2camTransformation(invectors.poses[i],rpy,xyz);
            poses.push_back(loc);
            //        std::cout << "Before transformed x: "<<invectors.poses[i].orientation.x<<" y: "<<invectors.poses[i].orientation.y<<" z: "<<invectors.poses[i].orientation.z<<" w: "<<invectors.poses[i].orientation.w<< std::endl;
            //        std::cout << "After transformed x: "<<loc.orientation.x<<" y: "<<loc.orientation.y<<" z: "<<loc.orientation.z<<" w: "<<loc.orientation.w<< std::endl;

            pts += obj.extractVisibleSurface(loc);
        }
            if (pts.size()>5)
            {
    //            obj.visualizeFOV(loc);
                globalCloud += pts;
                uavVectors.poses.push_back(invectors.poses[i]);
                for (int j =0; j<camsPose.size() ; j++)
                {
                    camVectors[j].poses.push_back(poses[j]);
                }
                accuracy = obj.calcAvgAccuracy(pts);
                //std::cout<<"accuracy : "<<accuracy<<std::endl;
                if(accuracy >= 0.00150)//1.5mm , for aircraft scaled it is limited between 0.2mm to 1.9mm
                {
                    accuracy_clusters.push_back(pts);
                }
            }
    }

}
void distanceFiltering(double min,double max,double id, geometry_msgs::PoseArray& uavVectors, Tree& tree, Point& a, visualization_msgs::MarkerArray& marker_array)
{
    visualization_msgs::Marker marker;

    FT sqd = tree.squared_distance(a);
//    std::cout << "Point x: " << a[0] <<" y: "<<a[1]<<" z: "<<a[2] <<std::endl;
//    std::cout << "tree size: "<<tree.size() <<std::endl;
//    std::cout << "squared distance: " << sqd << std::endl;
    if (sqd >=min && sqd <= max)
    {
//        std::cout << " within distance"<<std::endl;

        Vec3f vec(a[0] , a[1]  ,a[2]);
        marker = drawCUBE(vec, id, 3, 0.2);
        marker_array.markers.push_back(marker);
        geometry_msgs::Pose pt;
        pt.position.x = a[0];pt.position.y = a[1];pt.position.z = a[2];
        computeOrientations(pt,45,uavVectors);
    }


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


void loadOBJFile(const char* filename, std::vector<Vec3f>& points, std::list<CGALTriangle>& triangles)
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
                    Point p1(points[atoi(data[0]) - 1][0],points[atoi(data[0]) - 1][1],points[atoi(data[0]) - 1][2]);
                    Point p2(points[atoi(data[1]) - 1][0],points[atoi(data[1]) - 1][1],points[atoi(data[1]) - 1][2]);
                    Point p3(points[atoi(data[2]) - 1][0],points[atoi(data[2]) - 1][1],points[atoi(data[2]) - 1][2]);
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
                    Point p1(points[indxs[0]][0],points[indxs[0]][1],points[indxs[0]][2]);
                    Point p2(points[indxs[1]][0],points[indxs[1]][1],points[indxs[1]][2]);
                    Point p3(points[indxs[2]][0],points[indxs[2]][1],points[indxs[2]][2]);
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

