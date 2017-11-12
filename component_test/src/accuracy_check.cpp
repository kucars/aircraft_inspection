/***************************************************************************
 *   Copyright (C) 2016 - 2017 by                                          *
 *      Tarek Taha, KURI  <tataha@tarektaha.com>                           *
 *      Randa Almadhoun   <randa.almadhoun@kustar.ac.ae>                   *
 *                                                                         *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          *
 ***************************************************************************/
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

using namespace fcl;
geometry_msgs::Pose uav2camTransformation(geometry_msgs::Pose pose, Vec3f rpy, Vec3f xyz);
pcl::PointCloud<pcl::PointXYZ> pointCloudViewportTransform(pcl::PointCloud<pcl::PointXYZ> pointCloud, geometry_msgs::Pose cameraPose);
void transformPointMatVec(tf::Vector3 translation, tf::Matrix3x3 rotation, geometry_msgs::Point32 in, geometry_msgs::Point32 &out);

int main(int argc, char **argv)
{

    ros::init(argc, argv, "accuracy_check");
    ros::NodeHandle n;
    ros::Publisher pub1 = n.advertise<sensor_msgs::PointCloud2>("originalPointCloud", 100);
    ros::Publisher pub2 = n.advertise<sensor_msgs::PointCloud2>("accuracyCloud", 100);
    ros::Publisher pub3 = n.advertise<geometry_msgs::PoseArray>("viewpoints", 100);

    pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud(new pcl::PointCloud<pcl::PointXYZ>);
    OcclusionCullingGPU obj(n,"hoa_translated_densed_scaled.pcd");
    double locationx,locationy,locationz,yaw,max=0, EMax,min=std::numeric_limits<double>::max(),EMin;

    //reading the original cloud
    std::string path = ros::package::getPath("component_test");
    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/hoa_translated_densed_scaled.pcd", *originalCloud);
    geometry_msgs::PoseArray viewpoints;
    geometry_msgs::Pose pt;

    ros::Time accuracy_begin = ros::Time::now();


    //1: reading the path from a file
    //*******************************
    std::string str1 = path+"/src/txt/2.5_98%path_newtests1to4_hoa_translated_densed_scaled.pcdscaledGPU_NewIG_Dynamic_dsscpp_s5.txt";
    const char * filename1 = str1.c_str();
    assert(filename1 != NULL);
    filename1 = strdup(filename1);
    FILE *file1 = fopen(filename1, "r");
    if (!file1)
    {
        std::cout<<"\nCan not open the File";
        fclose(file1);
    }


    //2: determine the number of the sensors
    //***********************************
    std::vector<geometry_msgs::Pose> camsPose;
    pcl::PointCloud<pcl::PointXYZ> global, globalVis;
    geometry_msgs::Pose loc;
    geometry_msgs::PoseArray poses;
    cout<<"\n\nnumber of onboard sensors is: ";
    int sensorNum;
    cin>> sensorNum;
    int num=0;
    while (sensorNum != 0)
    {

        std::cout<<"Sensor "<<num<<" pose xyz & rpy: ";
        geometry_msgs::Pose camPos;
        cin>>camPos.position.x;
        cin>>camPos.position.y;
        cin>>camPos.position.z;
        cin>>camPos.orientation.x;
        cin>>camPos.orientation.y;
        cin>>camPos.orientation.z;
        camsPose.push_back(camPos);

        sensorNum--;
        num++;
    }

    //3: exracting visible surfaces from the model using the viewpoints encapsulated in the generated path
    //****************************************************************************************************
    int r=0;//for debugging purposes
    while (!feof(file1))
    {
        fscanf(file1,"%lf %lf %lf %lf\n",&locationx,&locationy,&locationz,&yaw);
        pt.position.x=locationx; pt.position.y=locationy; pt.position.z=locationz;

        tf::Quaternion tf_q ;
        tf_q= tf::createQuaternionFromYaw(yaw);
        pt.orientation.x=tf_q.getX();pt.orientation.y=tf_q.getY();pt.orientation.z=tf_q.getZ();pt.orientation.w=tf_q.getW();

        for (int j =0; j<camsPose.size() ; j++)
        {
            Vec3f xyz(camsPose[j].position.x, camsPose[j].position.y, camsPose[j].position.z);
            Vec3f rpy(camsPose[j].orientation.x, camsPose[j].orientation.y, camsPose[j].orientation.z);
            loc= uav2camTransformation(pt,rpy,xyz);
            viewpoints.poses.push_back(loc);


            pcl::PointCloud<pcl::PointXYZ> visible;
            pcl::PointCloud<pcl::PointXYZ> transformedVisible;

            visible += obj.extractVisibleSurface(loc);
            //obj.visualizeFOV(pt); //uncomment if you want to visualize the FOV

            transformedVisible = pointCloudViewportTransform(visible, loc);
            global +=transformedVisible;
            globalVis += visible;
            //std::cout<<"num of visible points :  "<<visible.size()<<"points, num of transformed: "<<transformedVisible.points.size() <<"points \n";
            for(int i=0; i<transformedVisible.points.size();i++){
                double temp = transformedVisible.points[i].x;//depth (it is at x axis because of the frustum culling camera pose requirement)
                //std::cout<<"depth are :  "<<temp<<"points\n";
                if(max<temp)
                   max=temp;
                if(min>temp)
                   min=temp;

            }
        }


        poses.poses.push_back(pt);// the UAV waypoints


        //for debugging purposes (work on few viewpoints)
        if(r++==5)
            break;
    }

    //4: calculate the Maximum Error
    //***************************************
    EMax = 0.0000285 * max*max; // the standard deviation equation is taken from paper
    EMin = 0.0000285 * min*min;

    std::cout<<"Maximum error: "<<EMax<<" for the depth of: "<<max<<"\n";
    std::cout<<"Minimum error: "<<EMin<<" for the depth of: "<<min<<"\n";
    std::cout<<"Size of the global cloud: "<<global.size()<<" points \n";

    //5: calculate the Error
    //***************************************
    std::vector<double> ErrCal;
    geometry_msgs::PoseArray cloudPoses;
    double err;
    for (int j=0; j<global.size(); j++)
    {
        double val = global.at(j).x;//depth
        //double valx = global.at(j).x;
        //double valy = global.at(j).y;
//        std::cout<<"global cloud "<<j<<": x "<< valx<<" y "<<valy<<" z "<<val<<"\n";

        err= 0.0000285 * val * val;
        ErrCal.push_back(err);
    }

    //6: color coded model
    //***************************************
    double color;
    int flag;
    pcl::PointCloud<pcl::PointXYZRGB> globalColored, globalColoredOriginal;
    for (int j=0; j<global.size(); j++)
    {
        flag=0;
        color = 255- (255 * (ErrCal.at(j)/EMax)); //dark regions (low accuracy , high error)
        pcl::PointXYZRGB p = pcl::PointXYZRGB(color,color,0);
        pcl::PointXYZRGB pCopy = pcl::PointXYZRGB(color,color,0);

        if(global.at(j).x >= 6.0)
        {
            pCopy.r = 255;
            pCopy.g = 0;
            pCopy.b = 0;

        }

        p.data[0] = global.at(j).x; p.data[1] = global.at(j).y; p.data[2] = global.at(j).z;
        pCopy.data[0] = globalVis.at(j).x; pCopy.data[1] = globalVis.at(j).y; pCopy.data[2] = globalVis.at(j).z;

        for (int k=0; k<globalColored.size(); k++)
        {

            if (globalColored.at(k).x==global.at(j).x && globalColored.at(k).y==global.at(j).y)
            {
                //std::cout<<"found a match!!"<<std::endl;
                if(color<globalColored.at(k).g)
                {
                    globalColored.at(k).r = color;
                    globalColored.at(k).g = color;


                    globalColoredOriginal.at(k).r = color;
                    globalColoredOriginal.at(k).g = color;
                }
                flag=1;
            }
        }
        if (flag==0)
        {
            globalColored.points.push_back(p);
            globalColoredOriginal.points.push_back(pCopy);

        }

    }


    //accuracy and transformed accuracy color gradient model
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr accuracyCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr accuracyCopyCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    accuracyCloud->points=globalColored.points; //transformed
    accuracyCopyCloud->points=globalColoredOriginal.points;

    std::cout<<"Size of the colored cloud: "<<globalColored.size()<<" points \n";

    ros::Time accuracy_end = ros::Time::now();
    double elapsed =  accuracy_end.toSec() - accuracy_begin.toSec();
    std::cout<<"accuracy check duration (s) = "<<elapsed<<"\n";


    //*****************Rviz Visualization ************
    ros::Rate loop_rate(10);
    while (ros::ok())
    {


        //***original cloud & color gradient model***
        sensor_msgs::PointCloud2 cloud1;
        sensor_msgs::PointCloud2 cloud2;

        pcl::toROSMsg(*originalCloud, cloud1); //cloud of original (white) using original cloud
        pcl::toROSMsg(*accuracyCopyCloud, cloud2); //cloud of the accuracy color gradient model

        cloud1.header.frame_id = "map";
        cloud2.header.frame_id = "map";

        cloud1.header.stamp = ros::Time::now();
        cloud2.header.stamp = ros::Time::now();

        pub1.publish(cloud1);
        pub2.publish(cloud2);

        viewpoints.header.frame_id= "map";
        viewpoints.header.stamp = ros::Time::now();
        pub3.publish(viewpoints);

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

void transformPointMatVec(tf::Vector3 translation, tf::Matrix3x3 rotation, geometry_msgs::Point32 in, geometry_msgs::Point32& out)
{

    double x = rotation[0].x() * in.x + rotation[0].y() * in.y + rotation[0].z() * in.z + translation.x();
    double y = rotation[1].x() * in.x + rotation[1].y() * in.y + rotation[1].z() * in.z + translation.y();
    double z = rotation[2].x() * in.x + rotation[2].y() * in.y + rotation[2].z() * in.z + translation.z();

    out.x = x;
    out.y = y;
    out.z = z;
}


//translate the pcd viewport (0,0,0) to the camera viewport (viewpoints)
pcl::PointCloud<pcl::PointXYZ> pointCloudViewportTransform(pcl::PointCloud<pcl::PointXYZ> pointCloud, geometry_msgs::Pose cameraPose)
{

    pcl::PointCloud<pcl::PointXYZ> posArray;
    tf::Matrix3x3 rotZ, rotX, rotY, rotE;

    //traslation accroding to the camera position
    tf::Vector3 cameraPoseTrans(-1*cameraPose.position.x, -1* cameraPose.position.y, -1*cameraPose.position.z);
    //std::cout<<"camera position: "<< cameraPose.position.x<<" "<<cameraPose.position.y<<" "<< cameraPose.position.z<<std::endl;

    //vector and matrice to help in performing no translation or no rotation
    tf::Vector3 transE(0, 0, 0); //No translation
    //No Rotation
    rotE.setValue(1,0,0,
                  0,1,0,
                  0,0,1);

    // rotation of the uav interms of the previous viewport of the pointcloud
    tf::Quaternion qt(cameraPose.orientation.x, cameraPose.orientation.y, cameraPose.orientation.z, cameraPose.orientation.w) ;
    double r, p, y;
    tf::Matrix3x3(qt).getRPY(r, p, y);
    double yaw = -1 * y;
    //std::cout<<" yaw angle: "<<yaw<<std::endl;
    //std::cout<<" cos angle: "<<std::cos(yaw)<<std::endl;
    //std::cout<<" sin angle: "<<std::sin(yaw)<<std::endl;

    rotZ.setValue(std::cos(yaw),-1*std::sin(yaw),0,
                  std::sin(yaw),std::cos(yaw),0,
                  0, 0, 1);

    // rotation required for frustum culling (not required !)
    double roll = -1 * 90;
    rotX.setValue(1,0,0,
                  0, std::cos(roll),-1*std::sin(roll),
                  0, std::sin(roll),std::cos(roll)  );

    // rotation for the camera orientation
    double pitch = -1 * p;
    //std::cout<<" pitch angle: "<<pitch<<std::endl;

    rotY.setValue(std::cos(pitch),0,std::sin(pitch),
                  0,1,0,
                  -1*std::sin(pitch), 0, std::cos(pitch));

    for (int i = 0; i < pointCloud.size() ; i++) {

        geometry_msgs::Point32 ptIN, ptOUT, ptOUT1, ptOUT2, ptOUT3 ;
        ptIN.x = pointCloud.points[i].data[0];
        ptIN.y = pointCloud.points[i].data[1];
        ptIN.z = pointCloud.points[i].data[2];

        //std::cout<<"camera position: "<< cameraPose.position.x<<" "<<cameraPose.position.y<<" "<< cameraPose.position.z<<std::endl;
        //std::cout<<"point in : "<<ptIN.x<<" "<<ptIN.y << " "<<ptIN.z<<" "<<std::endl;

        //translation to camera position
        transformPointMatVec(cameraPoseTrans, rotE, ptIN, ptOUT);
        //std::cout<<"point out1 : "<<ptOUT.x<<" "<<ptOUT.y << " "<<ptOUT.z<<" "<<std::endl;

        //rotation around z (yaw) according to the camera orientation
        transformPointMatVec(transE, rotZ, ptOUT, ptOUT1);
        //std::cout<<"point out2 : "<<ptOUT1.x<<" "<<ptOUT1.y << " "<<ptOUT1.z<<" "<<std::endl;

        //rotation around y (pitch) according to the camera tilt
        transformPointMatVec(transE, rotY, ptOUT1, ptOUT2);
        //std::cout<<"point out3 : "<<ptOUT2.x<<" "<<ptOUT2.y << " "<<ptOUT2.z<<" "<<std::endl;

        //rotation around x (roll) (not necessary for now)
        //transformPointMatVec(transE, rotX, ptOUT2, ptOUT3);
        //std::cout<<"point out4 : "<<ptOUT3.x<<" "<<ptOUT3.y << " "<<ptOUT3.z<<" "<<std::endl;

        pcl::PointXYZ finalPt;
        finalPt.data[0] = ptOUT2.x;
        finalPt.data[1] = ptOUT2.y;
        finalPt.data[2] = ptOUT2.z;
        posArray.points.push_back(finalPt);

    }

    posArray.header = pointCloud.header;
    return posArray;
}
