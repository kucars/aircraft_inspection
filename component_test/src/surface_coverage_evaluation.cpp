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

#include <rviz_visual_tools/rviz_visual_tools.h>
//#include <igl/copyleft/cgal/intersect_other.h>

 using namespace fcl;
geometry_msgs::Pose uav2camTransformation(geometry_msgs::Pose pose, Vec3f rpy, Vec3f xyz);
visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links, int c_color[], double scale, std::string frame_id);
void meshingVTK(pcl::PointCloud<pcl::PointXYZ> pointcloud, pcl::PolygonMeshPtr& mesh);
void meshingPCL(pcl::PointCloud<pcl::PointXYZ> pointcloud, pcl::PolygonMeshPtr& mesh);
float calcMeshSurfaceArea(pcl::PolygonMesh::Ptr& mesh);
bool contains(pcl::PointCloud<pcl::PointXYZ> c, pcl::PointXYZ p) ;
pcl::PointCloud<pcl::PointXYZ> pointsDifference(pcl::PointCloud<pcl::PointXYZ> c1, pcl::PointCloud<pcl::PointXYZ> c2) ;
vtkSmartPointer<vtkPolyData> transform_back(vtkSmartPointer<vtkPoints> pt, vtkSmartPointer<vtkPolyData> pd);

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
    rviz_visual_tools::RvizVisualToolsPtr visual_tools_(new rviz_visual_tools::RvizVisualTools("map","/rviz_visual_markers"));

    geometry_msgs::Pose p;
    p.position.x=0;    p.position.y=0; p.position.z=0; p.orientation.x=0; p.orientation.y=0;p.orientation.z=0;p.orientation.w=1;

    geometry_msgs::Point p1;p1.x=0;p1.y=0;p1.z=0;    geometry_msgs::Point p2;p2.x=0;p2.y=0;p2.z=6;
    visual_tools_->publishLine(p1,p2,rviz_visual_tools::PINK, rviz_visual_tools::LARGE);

    pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr visibleCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr extraPtr(new pcl::PointCloud<pcl::PointXYZ>);

    OcclusionCullingGPU obj(n,"etihad_nowheels_densed.pcd");
//    OcclusionCullingGPU obj(n,"etihad_nowheels_nointernal_newdensed.pcd");
    double locationx,locationy,locationz,yaw;

    //reading the original cloud
    std::string path = ros::package::getPath("component_test");
    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/etihad_nowheels_densed.pcd", *originalCloud);
//    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/etihad_nowheels_nointernal_newdensed.pcd", *originalCloud);


    //1: reading the path from a file
    std::string str1 = path+"/src/txt/2_5_10path.txt";//3_90path_new
    const char * filename1 = str1.c_str();
    assert(filename1 != NULL);
    filename1 = strdup(filename1);
    FILE *file1 = fopen(filename1, "r");
    if (!file1)
    {
        std::cout<<"\nCan not open the File";
        fclose(file1);
    }
    pcl::PointCloud<pcl::PointXYZ> global;
    Vec3f rpy(0,0.093,0);
    Vec3f xyz(0,0.0,-0.055);
    geometry_msgs::Pose loc;
    geometry_msgs::PoseArray viewpoints,points;
    geometry_msgs::PoseArray extraPoints;
    std::vector<double> extraAreas,avgAccuracyErrors;
    geometry_msgs::Pose pt;

    int i=0;
    while (!feof(file1))
    {
        fscanf(file1,"%lf %lf %lf %lf\n",&locationx,&locationy,&locationz,&yaw);
        pt.position.x=locationx; pt.position.y=locationy; pt.position.z=locationz;
//        pt.position.x=4.0; pt.position.y=-25; pt.position.z=5;
        tf::Quaternion tf_q ;
        tf_q= tf::createQuaternionFromYaw(yaw);
//        yaw=3.14;
        pt.orientation.x=tf_q.getX();pt.orientation.y=tf_q.getY();pt.orientation.z=tf_q.getZ();pt.orientation.w=tf_q.getW();
        points.poses.push_back(pt);
        loc= uav2camTransformation(pt,rpy,xyz);
        viewpoints.poses.push_back(loc);

        pcl::PointCloud<pcl::PointXYZ> visible, extra_cloud;
        visible = obj.extractVisibleSurface(loc);
//        global +=visible;
//        extra_cloud = pointsDifference(visible,global);
//        extraPtr->points = extra_cloud.points;
//        global.points =visible.points;
        visibleCloudPtr->points = visible.points;
        std::cout<<"There are :  "<<visible.size()<<"points\n";
        pcl::PolygonMesh::Ptr m(new pcl::PolygonMesh());
        meshingPCL(visible, m);
        float area = calcMeshSurfaceArea(m);

//        pcl::Vertices v;

//        std::cout<<"polygons in main "<<m->polygons.size()<<"\n";
//        std::cout<<"polygons vertices "<<m->polygons[0].vertices.size()<<"\n";
//        pcl::PointCloud<pcl::PointXYZ> cloud;
//        pcl::fromPCLPointCloud2 (m->cloud, cloud);
//        cloud.points[ mesh.polygons[tri_i].vertices[vertex_i] ]
//        std::cout<<"polygon 0 vertices 0: x:"<<cloud.points[ m->polygons[0].vertices[0] ].x<<" y:"<<cloud.points[ m->polygons[0].vertices[0] ].y<<" z:"<< cloud.points[ m->polygons[0].vertices[0] ].z<<"\n";

          //3: calculate the accuracy error ratio average
          double accuracyAvg = obj.calcAvgAccuracy(visible);
          avgAccuracyErrors.push_back(accuracyAvg);
          std::cout<<"average accuracy at "<<i<<" is " <<accuracyAvg<<"\n";

          //just to handle one point
          if(i==1)
              break;
          i++;
    }


    // *****************Rviz Visualization ************
    geometry_msgs::Point pt1;
    std::vector<geometry_msgs::Point> lineSegments;
    for (int i =0; i<points.poses.size(); i++)
    {
        if(i+1< points.poses.size())
        {
            pt1.x = points.poses[i].position.x;
            pt1.y =  points.poses.at(i).position.y;
            pt1.z =  points.poses.at(i).position.z;
            lineSegments.push_back(pt1);

            pt1.x = points.poses.at(i+1).position.x;
            pt1.y =  points.poses.at(i+1).position.y;
            pt1.z =  points.poses.at(i+1).position.z;
            lineSegments.push_back(pt1);

        }
    }
    int c_color[3];
    c_color[0]=1; c_color[1]=0; c_color[2]=0;
    visualization_msgs::Marker linesList = drawLines(lineSegments,c_color,0.15, "map");

    ros::Rate loop_rate(10);
    sensor_msgs::PointCloud2 cloud1;
    sensor_msgs::PointCloud2 cloud2,cloud3;
    while (ros::ok())
    {

        //***original cloud & accuracy cloud publish***
        pcl::toROSMsg(*originalCloud, cloud1); //cloud of original (white) using original cloud
        pcl::toROSMsg(*visibleCloudPtr, cloud2); //cloud of the not occluded voxels (blue) using occlusion culling
        pcl::toROSMsg(*extraPtr, cloud3); //cloud of the not occluded voxels (blue) using occlusion culling

        cloud1.header.frame_id = "map";
        cloud2.header.frame_id = "map";
        cloud3.header.frame_id = "map";

        cloud1.header.stamp = ros::Time::now();
        cloud2.header.stamp = ros::Time::now();
        cloud3.header.stamp = ros::Time::now();

        originalCloudPub.publish(cloud1);
        visibleCloudPub.publish(cloud2);
        extraCloudPub.publish(cloud3);

        //***viewpoints & waypoints  publish***
        viewpoints.header.frame_id= "map";
        viewpoints.header.stamp = ros::Time::now();
        viewpointsPub.publish(viewpoints);

        points.header.frame_id= "map";
        points.header.stamp = ros::Time::now();
        waypointsPub.publish(points);

        //***path publish***
        pathPub.publish(linesList);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void meshingPCL(pcl::PointCloud<pcl::PointXYZ> pointcloud, pcl::PolygonMeshPtr& mesh)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 cloud_blob;

    cloud->points = pointcloud.points;
    //* the data should be available in cloud

    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*normals);
    //* normals should not contain the point normals + surface curvatures
    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (0.5);

    // Set typical values for the parameters
    gp3.setMu (3.0);
    gp3.setMaximumNearestNeighbors (100);
    gp3.setMaximumSurfaceAngle(M_PI); // 45 degrees
    gp3.setMinimumAngle(M_PI/4); // 10 degrees
    gp3.setMaximumAngle(M_PI/2); // 120 degrees
    gp3.setNormalConsistency(true);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);

    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();
  //  pcl::io::saveOBJFile ("bun01.obj", triangles);
    pcl::io::savePolygonFilePLY("plane.ply",triangles);
    pcl::io::saveVTKFile ("plane.vtk", triangles);
    mesh->polygons= triangles.polygons;
    mesh->cloud= triangles.cloud;
    std::cout<<"triangles: "<<mesh->polygons.size()<<"\n";
}

float calcMeshSurfaceArea(pcl::PolygonMesh::Ptr& mesh)
{
    int s = mesh->polygons.size();
    float totalArea=0.0;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2 (mesh->cloud, cloud);
    for (int i =0; i<s; i++){
        pcl::PointCloud<pcl::PointXYZ> temp_cloud;
        temp_cloud.points.push_back( cloud.points[ mesh->polygons[i].vertices[0] ]);
        temp_cloud.points.push_back( cloud.points[ mesh->polygons[i].vertices[1] ]);
        temp_cloud.points.push_back( cloud.points[ mesh->polygons[i].vertices[2] ]);
        totalArea += pcl::calculatePolygonArea(temp_cloud);
    }
    std::cout<<"total area "<<totalArea<<"\n";
    return totalArea;
}
bool contains(pcl::PointCloud<pcl::PointXYZ> c, pcl::PointXYZ p) {
    pcl::PointCloud<pcl::PointXYZ>::iterator it = c.begin();
    for (; it != c.end(); ++it) {
        if (it->x == p.x && it->y == p.y && it->z == p.z)
            return true;
    }
    return false;
}
pcl::PointCloud<pcl::PointXYZ> pointsDifference(pcl::PointCloud<pcl::PointXYZ> c1, pcl::PointCloud<pcl::PointXYZ> c2) { //c1 visible  //c2 global
    pcl::PointCloud<pcl::PointXYZ> inter;
    pcl::PointCloud<pcl::PointXYZ>::iterator it = c1.begin();
    for (; it != c1.end(); ++it) {
        if (!contains(c2, *it))
            inter.push_back(*it);
    }

    return inter;
}
void meshingVTK(pcl::PointCloud<pcl::PointXYZ> pointcloud, pcl::PolygonMeshPtr& mesh)
{
    //reading the cloud into vtkpolydata
    vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();

    for (int i = 0; i < pointcloud.points.size (); i++)
    {
        double p[3];
        p[0] = pointcloud.points[i].x;
        p[1] = pointcloud.points[i].y;
        p[2] = pointcloud.points[i].z;
        pts->InsertNextPoint (p);
    }

    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    polydata->SetPoints(pts);
    std::cout<<"after assinging the polydata\n";

    // Construct the surface and create isosurface.
    vtkSmartPointer<vtkSurfaceReconstructionFilter> surf =
            vtkSmartPointer<vtkSurfaceReconstructionFilter>::New();
#if VTK_MAJOR_VERSION <= 5
    surf->SetInput(polydata);
#else
    surf->SetInputData(polydata);
#endif

    vtkSmartPointer<vtkContourFilter> cf =
            vtkSmartPointer<vtkContourFilter>::New();
    cf->SetInputConnection(surf->GetOutputPort());
    cf->SetValue(0, 0.0);

    // Sometimes the contouring algorithm can create a volume whose gradient
    // vector and ordering of polygon (using the right hand rule) are
    // inconsistent. vtkReverseSense cures this problem.
    vtkSmartPointer<vtkReverseSense> reverse =
            vtkSmartPointer<vtkReverseSense>::New();
    reverse->SetInputConnection(cf->GetOutputPort());
    reverse->ReverseCellsOn();
    reverse->ReverseNormalsOn();

    vtkSmartPointer<vtkPolyData> newSurf = transform_back( pts, cf->GetOutput());

    vtkSmartPointer<vtkPolyData> polydatatest  = vtkSmartPointer<vtkPolyData>::New();
    polydatatest->Reset();
    polydatatest= reverse->GetOutput();
    polydatatest->Update();
    std::cout<<"polygons are "<<polydatatest->GetNumberOfPolys()<<"\n";
    //     pcl::PolygonMesh::Ptr result(new pcl::PolygonMesh());

    //     pcl::VTKUtils::convertToPCL (polydatatest, *mesh); //option 1
    pcl::VTKUtils::vtk2mesh(newSurf,*mesh); //option 2
    //     pcl::io::mesh2vtk(test,polydatatest);
    std::cout<<"polygons in pcl are "<<mesh->polygons.size()<<"\n";

     //write the mesh to file (optional)
     std::string filename = "trial.ply";//argv[1];
     vtkSmartPointer<vtkPLYWriter> plyWriter =
             vtkSmartPointer<vtkPLYWriter>::New();
     plyWriter->SetFileName(filename.c_str());
     plyWriter->SetInputConnection(reverse->GetOutputPort());
     plyWriter->Write();

}

vtkSmartPointer<vtkPolyData> transform_back(vtkSmartPointer<vtkPoints> pt, vtkSmartPointer<vtkPolyData> pd)
{
  // The reconstructed surface is transformed back to where the
  // original points are. (Hopefully) it is only a similarity
  // transformation.

  // 1. Get bounding box of pt, get its minimum corner (left, bottom, least-z), at c0, pt_bounds

  // 2. Get bounding box of surface pd, get its minimum corner (left, bottom, least-z), at c1, pd_bounds

  // 3. compute scale as:
  //       scale = (pt_bounds[1] - pt_bounds[0])/(pd_bounds[1] - pd_bounds[0]);

  // 4. transform the surface by T := T(pt_bounds[0], [2], [4]).S(scale).T(-pd_bounds[0], -[2], -[4])



  // 1.
  double pt_bounds[6];  // (xmin,xmax, ymin,ymax, zmin,zmax)
  pt->GetBounds(pt_bounds);


  // 2.
  double pd_bounds[6];  // (xmin,xmax, ymin,ymax, zmin,zmax)
  pd->GetBounds(pd_bounds);

//   // test, make sure it is isotropic
   std::cout<<std::abs((pd_bounds[1] - pd_bounds[0])/(pt_bounds[1] - pt_bounds[0]))<<std::endl;
   std::cout<<(pd_bounds[3] - pd_bounds[2])/(pt_bounds[3] - pt_bounds[2])<<std::endl;
   std::cout<<(pd_bounds[5] - pd_bounds[4])/(pt_bounds[5] - pt_bounds[4])<<std::endl;
//   // TEST


  // 3
  double scalex = std::abs ((pd_bounds[1] - pd_bounds[0])/(pt_bounds[1] - pt_bounds[0]));
  double scaley = std::abs ((pd_bounds[3] - pd_bounds[2])/(pt_bounds[3] - pt_bounds[2]));
//  double scalez = std::abs ((pd_bounds[5] - pd_bounds[4])/(pt_bounds[5] - pt_bounds[4]));


  // 4.
  vtkSmartPointer<vtkTransform> transp = vtkSmartPointer<vtkTransform>::New();
  transp->Translate(pt_bounds[0], pt_bounds[2], pt_bounds[4]);
  transp->Scale(scaley, scaley, scaley);
//  transp->RotateZ(180);
  transp->Translate( -pd_bounds[0],  -pd_bounds[2],  -pd_bounds[4]);

  vtkSmartPointer<vtkTransformPolyDataFilter> tpd = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
#if VTK_MAJOR_VERSION <= 5
  tpd->SetInput(pd);
#else
  tpd->SetInputData(pd);
#endif
  tpd->SetTransform(transp);
  tpd->Update();
  std::string filename = "new.ply";//argv[1];
  vtkSmartPointer<vtkPLYWriter> plyWriter =
          vtkSmartPointer<vtkPLYWriter>::New();
  plyWriter->SetFileName(filename.c_str());
  plyWriter->SetInputConnection(tpd->GetOutputPort());
  plyWriter->Write();

  return tpd->GetOutput();
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
