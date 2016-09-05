#include "component_test/occlusion_culling_gpu.h"


OcclusionCullingGPU::OcclusionCullingGPU(ros::NodeHandle &n, std::string modelName):
    nh(n),
    model(modelName),
    fc(true)
{

   fov_pub = n.advertise<visualization_msgs::MarkerArray>("fov", 10);
   cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
   cloudCopy = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
   filtered_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);

   occlusionFreeCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
   FrustumCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
   std::string path = ros::package::getPath("component_test");
   pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/"+model, *cloud);
   cloudCopy->points = cloud->points;
   voxelRes = 0.5f;
   OriginalVoxelsSize=0.0;
   id=0.0;
   viewEntropy=0.0;
   voxelFilterOriginal.setInputCloud (cloud);
   voxelFilterOriginal.setLeafSize (voxelRes, voxelRes, voxelRes);
   voxelFilterOriginal.initializeVoxelGrid();
   min_b1 = voxelFilterOriginal.getMinBoxCoordinates ();
   max_b1 = voxelFilterOriginal.getMaxBoxCoordinates ();
   for (int kk = min_b1.z (); kk <= max_b1.z (); ++kk)
   {
       for (int jj = min_b1.y (); jj <= max_b1.y (); ++jj)
       {
           for (int ii = min_b1.x (); ii <= max_b1.x (); ++ii)
           {
               Eigen::Vector3i ijk1 (ii, jj, kk);
               int index1 = voxelFilterOriginal.getCentroidIndexAt (ijk1);
               if(index1!=-1)
               {
                   OriginalVoxelsSize++;
               }

           }
       }
   }

   pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
   voxelgrid.setInputCloud (cloud);
   voxelgrid.setLeafSize(voxelRes, voxelRes,voxelRes);
   voxelgrid.filter (*filtered_cloud);

   fc.initializeGPUPointData(cloud);
   fc.setVerticalFOV (45);
   fc.setHorizontalFOV (58);
   fc.setNearPlaneDistance (0.7);
   fc.setFarPlaneDistance (6.0);

   //max accuracy calculation
   double max=0,min=std::numeric_limits<double>::max();
   for(int i=0; i<cloud->points.size();i++){
       double temp = cloud->at(i).z;//depth
       if(max<temp)
          max=temp;
       if(min>temp)
          min=temp;
   }
   maxAccuracyError = 0.0000285 * max*max;
   minAccuracyError = 0.0000285 * min*min;

}
OcclusionCullingGPU::OcclusionCullingGPU(std::string modelName):
    model(modelName),
    fc(true)
{
    cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
    filtered_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
    FrustumCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);

    std::string path = ros::package::getPath("component_test");
    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/"+model, *cloud);
    voxelRes = 0.5f;
    OriginalVoxelsSize=0.0;
    id=0.0;
    viewEntropy=0.0;
    voxelFilterOriginal.setInputCloud (cloud);
    voxelFilterOriginal.setLeafSize (voxelRes, voxelRes, voxelRes);
    voxelFilterOriginal.initializeVoxelGrid();
    min_b1 = voxelFilterOriginal.getMinBoxCoordinates ();
    max_b1 = voxelFilterOriginal.getMaxBoxCoordinates ();
    for (int kk = min_b1.z (); kk <= max_b1.z (); ++kk)
    {
        for (int jj = min_b1.y (); jj <= max_b1.y (); ++jj)
        {
            for (int ii = min_b1.x (); ii <= max_b1.x (); ++ii)
            {
                Eigen::Vector3i ijk1 (ii, jj, kk);
                int index1 = voxelFilterOriginal.getCentroidIndexAt (ijk1);
                if(index1!=-1)
                {
                    OriginalVoxelsSize++;
                }

            }
        }
    }
    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
    voxelgrid.setInputCloud (cloud);
    voxelgrid.setLeafSize(voxelRes, voxelRes,voxelRes);
    voxelgrid.filter (*filtered_cloud);

    fc.initializeGPUPointData(cloud);
    fc.setVerticalFOV (45);
    fc.setHorizontalFOV (58);
    fc.setNearPlaneDistance (0.7);
    fc.setFarPlaneDistance (6.0);

    //max accuracy calculation
    double max=0,min=std::numeric_limits<double>::max();
    for(int i=0; i<cloud->points.size();i++){
        double temp = cloud->at(i).z;//depth
        if(max<temp)
           max=temp;
        if(min>temp)
           min=temp;
    }
    maxAccuracyError = 0.0000285 * max*max;
    minAccuracyError = 0.0000285 * min*min;
}
OcclusionCullingGPU::OcclusionCullingGPU():
    model(NULL),
    fc(true)
{
    cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
    filtered_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
    FrustumCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);

    std::string path = ros::package::getPath("component_test");
    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/scaled_desktop.pcd", *cloud);
    voxelRes = 0.5f;
    OriginalVoxelsSize=0.0;
    id=0.0;
    viewEntropy=0.0;
    voxelFilterOriginal.setInputCloud (cloud);
    voxelFilterOriginal.setLeafSize (voxelRes, voxelRes, voxelRes);
    voxelFilterOriginal.initializeVoxelGrid();
    min_b1 = voxelFilterOriginal.getMinBoxCoordinates ();
    max_b1 = voxelFilterOriginal.getMaxBoxCoordinates ();
    for (int kk = min_b1.z (); kk <= max_b1.z (); ++kk)
    {
        for (int jj = min_b1.y (); jj <= max_b1.y (); ++jj)
        {
            for (int ii = min_b1.x (); ii <= max_b1.x (); ++ii)
            {
                Eigen::Vector3i ijk1 (ii, jj, kk);
                int index1 = voxelFilterOriginal.getCentroidIndexAt (ijk1);
                if(index1!=-1)
                {
                    OriginalVoxelsSize++;
                }

            }
        }
    }
    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
    voxelgrid.setInputCloud (cloud);
    voxelgrid.setLeafSize(voxelRes, voxelRes,voxelRes);
    voxelgrid.filter (*filtered_cloud);

    fc.initializeGPUPointData(cloud);
    fc.setVerticalFOV (45);
    fc.setHorizontalFOV (58);
    fc.setNearPlaneDistance (0.7);
    fc.setFarPlaneDistance (6.0);

    //max accuracy calculation
    double max=0,min=std::numeric_limits<double>::max();
    for(int i=0; i<cloud->points.size();i++){
        double temp = cloud->at(i).z;//depth
        if(max<temp)
           max=temp;
        if(min>temp)
           min=temp;
    }
    maxAccuracyError = 0.0000285 * max*max;
    minAccuracyError = 0.0000285 * min*min;
}
OcclusionCullingGPU::~OcclusionCullingGPU()
{
}

pcl::PointCloud<pcl::PointXYZ> OcclusionCullingGPU::extractVisibleSurface(geometry_msgs::Pose location)
{
    pcl::PointCloud <pcl::PointXYZ>::Ptr output (new pcl::PointCloud <pcl::PointXYZ>);
    Eigen::Matrix4f camera_pose;
    Eigen::Matrix3d Rd;
    Eigen::Matrix3f Rf;

    camera_pose.setZero();

    tf::Quaternion qt;
    qt.setX(location.orientation.x);
    qt.setY(location.orientation.y);
    qt.setZ(location.orientation.z);
    qt.setW(location.orientation.w);
    tf::Matrix3x3 R_tf(qt);
    tf::matrixTFToEigen(R_tf,Rd);
    Rf = Rd.cast<float>();
    camera_pose.block (0, 0, 3, 3) = Rf;
    Eigen::Vector3f T;
    T (0) = location.position.x; T (1) = location.position.y; T (2) = location.position.z;
    camera_pose.block (0, 3, 3, 1) = T;
    camera_pose (3, 3) = 1;
    fc.setCameraPose (camera_pose);

//    ros::Time tic = ros::Time::now();
    fc.filter(*output);
//    ros::Time toc = ros::Time::now();

//    std::cout<<"\nFrustum Filter took:"<< toc.toSec() - tic.toSec();

    FrustumCloud->points = output->points;

    //2:****voxel grid occlusion estimation *****
    Eigen::Quaternionf quat(qt.w(),qt.x(),qt.y(),qt.z());
    output->sensor_origin_  = Eigen::Vector4f(T[0],T[1],T[2],0);
    output->sensor_orientation_= quat;
    pcl::VoxelGridOcclusionEstimationGPU voxelOcclusionEstimationFilter;
    voxelOcclusionEstimationFilter.setInputCloud(output);
    voxelOcclusionEstimationFilter.setLeafSize (voxelRes, voxelRes, voxelRes);
    voxelOcclusionEstimationFilter.initializeVoxelGrid();
    double timeOnRayTracing=0;
    freeCloud.clear();
    ros::Time t1 = ros::Time::now();
    voxelOcclusionEstimationFilter.occlusionFreeEstimationAll(output,freeCloud);
    viewEntropy= voxelOcclusionEstimationFilter.entropyTot(0);
    occlusionFreeCloud->points = freeCloud.points;
    ros::Time t2 = ros::Time::now();
    timeOnRayTracing = (t2.toSec() - t1.toSec());

//    toc = ros::Time::now();
//    double diff = toc.toSec() - tic.toSec();
//    std::cout<<"\nProcessing Occlusuin culling on remaining points took:"<< diff <<" raytracing took:"<<timeOnRayTracing<<" percentage:"<<timeOnRayTracing*100.0f/diff;
    return freeCloud;
}

float OcclusionCullingGPU::calcCoveragePercent(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{

    // *******************original cloud Grid***************************
    //used VoxelGridOcclusionEstimationT since the voxelGrid does not include getcentroid function
//        pcl::VoxelGridOcclusionEstimationT voxelFilterOriginal;
//        voxelFilterOriginal.setInputCloud (cloud);
//        voxelFilterOriginal.setLeafSize (voxelRes, voxelRes, voxelRes);
//        voxelFilterOriginal.initializeVoxelGrid();

     //*******************Occupied Cloud Grid***************************
    ros::Time covpercent_begin = ros::Time::now();
    pcl::VoxelGridOcclusionEstimationGPU voxelFilterOccupied;
//        voxelFilterOccupied.setInputCloud (occlusionFreeCloud);
    voxelFilterOccupied.setInputCloud (cloud_filtered);
    voxelFilterOccupied.setLeafSize (voxelRes, voxelRes, voxelRes);
    voxelFilterOccupied.initializeVoxelGrid();



     //*****************************************************************
    Eigen::Vector3i  min_b = voxelFilterOccupied.getMinBoxCoordinates ();
    Eigen::Vector3i  max_b = voxelFilterOccupied.getMaxBoxCoordinates ();
//        Eigen::Vector3i  min_b1 = voxelFilterOriginal.getMinBoxCoordinates ();
//        Eigen::Vector3i  max_b1 = voxelFilterOriginal.getMaxBoxCoordinates ();

    float MatchedVoxels=0 ;//OriginalVoxelsSize=0, ;
     //iterate through the entire coverage grid to check the number of matched voxel between the original and the covered ones
    for (int kk = min_b.z (); kk <= max_b.z (); ++kk)
    {
        for (int jj = min_b.y (); jj <= max_b.y (); ++jj)
        {
            for (int ii = min_b.x (); ii <= max_b.x (); ++ii)
            {

                Eigen::Vector3i ijk (ii, jj, kk);
                int index1 = voxelFilterOccupied.getCentroidIndexAt (ijk);
                if(index1!=-1)
                {
                    Eigen::Vector4f centroid = voxelFilterOccupied.getCentroidCoordinate (ijk);
                    Eigen::Vector3i ijk_in_Original= voxelFilterOriginal.getGridCoordinates(centroid[0],centroid[1],centroid[2]) ;

                    int index = voxelFilterOriginal.getCentroidIndexAt (ijk_in_Original);

                    if(index!=-1)
                    {
                        MatchedVoxels++;
                    }
                }

            }
        }
    }

    //calculating the coverage percentage
    float coverage_ratio= MatchedVoxels/OriginalVoxelsSize;
    float coverage_percentage= coverage_ratio*100;

//    std::cout<<" the coverage ratio is = "<<coverage_ratio<<"\n";
//    std::cout<<" the number of covered voxels = "<<MatchedVoxels<<" voxel is covered"<<"\n";
//    std::cout<<" the number of original voxels = "<<OriginalVoxelsSize<<" voxel"<<"\n\n\n";
//    std::cout<<" the coverage percentage is = "<<coverage_percentage<<" %"<<"\n";

    ros::Time covpercent_end = ros::Time::now();
    double elapsed =  covpercent_end.toSec() - covpercent_begin.toSec();
//    std::cout<<"Coverage Percentage Calculation duration (s) = "<<elapsed<<"\n";

    return coverage_percentage;
}
double OcclusionCullingGPU::calcAvgAccuracy(pcl::PointCloud<pcl::PointXYZ> pointCloud)
{
    double avgAccuracy;
    double pointError,val,errorSum=0, errorRatio;
    for (int j=0; j<pointCloud.size(); j++)
    {
        val = pointCloud.at(j).z;//depth
        pointError= 0.0000285 * val * val;
//        errorRatio=pointError/maxAccuracyError;
        errorSum += pointError;
    }
    avgAccuracy=errorSum/pointCloud.size();
    return avgAccuracy;
}
void OcclusionCullingGPU::visualizeFOV(geometry_msgs::Pose location)
{

    pcl::PointCloud <pcl::PointXYZ>::Ptr output (new pcl::PointCloud <pcl::PointXYZ>);

    //    pcl::FrustumCullingGPU fc(true);
    //    fc.setInputCloud (cloud);
    //    fc.setVerticalFOV (45);
    //    fc.setHorizontalFOV (58);
    //    fc.setNearPlaneDistance (0.7);
    //    fc.setFarPlaneDistance (6.0);

    Eigen::Matrix4f camera_pose;
    Eigen::Matrix3d Rd;
    Eigen::Matrix3f Rf;

    camera_pose.setZero ();

    tf::Quaternion qt;
    qt.setX(location.orientation.x);
    qt.setY(location.orientation.y);
    qt.setZ(location.orientation.z);
    qt.setW(location.orientation.w);
    tf::Matrix3x3 R_tf(qt);
    tf::matrixTFToEigen(R_tf,Rd);
    Rf = Rd.cast<float>();
    camera_pose.block (0, 0, 3, 3) = Rf;
    Eigen::Vector3f T;
    T (0) = location.position.x; T (1) = location.position.y; T (2) = location.position.z;
    camera_pose.block (0, 3, 3, 1) = T;
    camera_pose (3, 3) = 1;
    fc.setCameraPose (camera_pose);
    fc.filter (*output);

    //*** visualization the FOV *****
    std::vector<geometry_msgs::Point> fov_points;
    int c_color[3];
    geometry_msgs::Point point1;
    point1.x=fc.fp_bl[0];point1.y=fc.fp_bl[1];point1.z=fc.fp_bl[2]; fov_points.push_back(point1);//0
    point1.x=fc.fp_br[0];point1.y=fc.fp_br[1];point1.z=fc.fp_br[2]; fov_points.push_back(point1);//1
    point1.x=fc.fp_tr[0];point1.y=fc.fp_tr[1];point1.z=fc.fp_tr[2]; fov_points.push_back(point1);//2
    point1.x=fc.fp_tl[0];point1.y=fc.fp_tl[1];point1.z=fc.fp_tl[2]; fov_points.push_back(point1);//3
    point1.x=fc.np_bl[0];point1.y=fc.np_bl[1];point1.z=fc.np_bl[2]; fov_points.push_back(point1);//4
    point1.x=fc.np_br[0];point1.y=fc.np_br[1];point1.z=fc.np_br[2]; fov_points.push_back(point1);//5
    point1.x=fc.np_tr[0];point1.y=fc.np_tr[1];point1.z=fc.np_tr[2]; fov_points.push_back(point1);//6
    point1.x=fc.np_tl[0];point1.y=fc.np_tl[1];point1.z=fc.np_tl[2]; fov_points.push_back(point1);//7

    std::vector<geometry_msgs::Point> fov_linesNear;
    fov_linesNear.push_back(fov_points[4]);fov_linesNear.push_back(fov_points[5]);
    fov_linesNear.push_back(fov_points[5]);fov_linesNear.push_back(fov_points[6]);
    fov_linesNear.push_back(fov_points[6]);fov_linesNear.push_back(fov_points[7]);
    fov_linesNear.push_back(fov_points[7]);fov_linesNear.push_back(fov_points[4]);
    c_color[0]=1; c_color[1]=0; c_color[2]=1;
    linesList1 = drawLines(fov_linesNear,id++,c_color);//purple

    std::vector<geometry_msgs::Point> fov_linesFar;
    fov_linesFar.push_back(fov_points[0]);fov_linesFar.push_back(fov_points[1]);
    fov_linesFar.push_back(fov_points[1]);fov_linesFar.push_back(fov_points[2]);
    fov_linesFar.push_back(fov_points[2]);fov_linesFar.push_back(fov_points[3]);
    fov_linesFar.push_back(fov_points[3]);fov_linesFar.push_back(fov_points[0]);
    c_color[0]=1; c_color[1]=1; c_color[2]=0;
    linesList2 = drawLines(fov_linesFar,id++,c_color);//yellow


    std::vector<geometry_msgs::Point> fov_linestop;
    fov_linestop.push_back(fov_points[7]);fov_linestop.push_back(fov_points[3]);//top
    fov_linestop.push_back(fov_points[6]);fov_linestop.push_back(fov_points[2]);//top
    c_color[0]=0; c_color[1]=1; c_color[2]=0;
    linesList3 = drawLines(fov_linestop,id++,c_color);//green

    std::vector<geometry_msgs::Point> fov_linesbottom;
    fov_linesbottom.push_back(fov_points[5]);fov_linesbottom.push_back(fov_points[1]);//bottom
    fov_linesbottom.push_back(fov_points[4]);fov_linesbottom.push_back(fov_points[0]);//bottom
    c_color[0]=0; c_color[1]=0; c_color[2]=1;
    linesList4 = drawLines(fov_linesbottom,id++,c_color);//blue

    marker_array.markers.push_back(linesList1);
    marker_array.markers.push_back(linesList2);
    marker_array.markers.push_back(linesList3);
    marker_array.markers.push_back(linesList4);
    fov_pub.publish(marker_array);
}
visualization_msgs::Marker OcclusionCullingGPU::drawLines(std::vector<geometry_msgs::Point> links, int id, int c_color[])
{
    visualization_msgs::Marker linksMarkerMsg;
    linksMarkerMsg.header.frame_id="map"; //change to "base_point_cloud" if it is used in component test package
    linksMarkerMsg.header.stamp=ros::Time::now();
    linksMarkerMsg.ns="link_marker";
    linksMarkerMsg.id = id;
    linksMarkerMsg.type = visualization_msgs::Marker::LINE_LIST;
    linksMarkerMsg.scale.x = 0.08;//0.03
    linksMarkerMsg.action  = visualization_msgs::Marker::ADD;
    linksMarkerMsg.lifetime  = ros::Duration(1000);
    std_msgs::ColorRGBA color;
    color.r = (float)c_color[0]; color.g=(float)c_color[1]; color.b=(float)c_color[2], color.a=1.0f;
//    if(c_color == 1)
//    {
//        color.r = 1.0;
//        color.g = 0.0;
//        color.b = 0.0;
//        color.a = 1.0;
//    }
//    else if(c_color == 2)
//    {
//        color.r = 0.0;
//        color.g = 1.0;
//        color.b = 0.0;
//        color.a = 1.0;
//    }
//    else
//    {
//        color.r = 0.0;
//        color.g = 0.0;
//        color.b = 1.0;
//        color.a = 1.0;
//    }
    std::vector<geometry_msgs::Point>::iterator linksIterator;
    for(linksIterator = links.begin();linksIterator != links.end();linksIterator++)
    {
        linksMarkerMsg.points.push_back(*linksIterator);
        linksMarkerMsg.colors.push_back(color);
    }
   return linksMarkerMsg;
}
