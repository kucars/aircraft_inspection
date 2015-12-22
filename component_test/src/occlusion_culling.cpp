#include "component_test/occlusion_culling.h"


OcclusionCulling::OcclusionCulling(ros::NodeHandle &n, std::string modelName):
    nh(n),
    model(modelName)
{
//   original_pub = nh.advertise<sensor_msgs::PointCloud2>("original_point_cloud", 10);
//   visible_pub = nh.advertise<sensor_msgs::PointCloud2>("occlusion_free_cloud", 100);
//    lines_pub1 = n.advertise<visualization_msgs::Marker>("fov_far_near", 100);
//    lines_pub2 = n.advertise<visualization_msgs::Marker>("fov_top", 100);
//    lines_pub3 = n.advertise<visualization_msgs::Marker>("fov_bottom", 100);
   cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
   filtered_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);

//   occlusionFreeCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
   std::string path = ros::package::getPath("component_test");
   pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/"+model, *cloud);
   voxelRes = 0.5;
   OriginalVoxelsSize=0.0;
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
   voxelgrid.setLeafSize (0.5f, 0.5f, 0.5f);
   voxelgrid.filter (*filtered_cloud);
}
OcclusionCulling::OcclusionCulling(std::string modelName):
    model(modelName)
{
    cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
//    occlusionFreeCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
    std::string path = ros::package::getPath("component_test");
    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/"+model, *cloud);
    voxelRes = 0.5;
    OriginalVoxelsSize=0.0;
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
}
OcclusionCulling::OcclusionCulling():
    model(NULL)
{
    cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
//    occlusionFreeCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
    std::string path = ros::package::getPath("component_test");
    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/scaled_desktop.pcd", *cloud);
    voxelRes = 0.5;
    OriginalVoxelsSize=0.0;
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

}
OcclusionCulling::~OcclusionCulling()
{
}
pcl::PointCloud<pcl::PointXYZ> OcclusionCulling::extractVisibleSurface(geometry_msgs::Pose location)
{
    // 1 *****Frustum Culling*******
    pcl::PointCloud <pcl::PointXYZ>::Ptr output (new pcl::PointCloud <pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr occlusionFreeCloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::FrustumCullingTT fc (true);
    fc.setInputCloud (cloud);
    fc.setVerticalFOV (45);
    fc.setHorizontalFOV (58);
    fc.setNearPlaneDistance (0.7);
    fc.setFarPlaneDistance (6.0);

    Eigen::Matrix4f camera_pose;
    Eigen::Matrix3f R;
    camera_pose.setZero ();

    //calculating the rpy out of the location quetrenion
    tf::Quaternion qt;
    qt.setX(location.orientation.x);
    qt.setY(location.orientation.y);
    qt.setZ(location.orientation.z);
    qt.setW(location.orientation.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(qt).getEulerZYX(yaw, pitch, roll);//gets the yaw around z then pitch around new y then roll around new x
    Eigen::Vector3f theta_rad(roll,pitch,yaw);
    Eigen::Vector3f xV(1,0,0) ;//unit vector of X
    Eigen::Vector3f yV(0,1,0) ;//unit vector of Y
    Eigen::Vector3f zV(0,0,1) ;//unit vector of Z

    //first rotation around z ,then around the new y, then around the new x
    // "AngleAxisf" starts the rotation around the defined axis then moves to the next angle and applies
    //   the rotation around the new axis and continues the same for the third angle
    R = Eigen::AngleAxisf (theta_rad[2], zV) *
            Eigen::AngleAxisf (theta_rad[1], yV) *
            Eigen::AngleAxisf (theta_rad[0], xV);
    //        std::cout<<"theta_rad r : "<<theta_rad[0]<<"theta_rad p: "<<theta_rad[1]<<"theta_rad y: "<<theta_rad[2]<<"\n";
    camera_pose.block (0, 0, 3, 3) = R;
    Eigen::Vector3f T;
    T (0) = location.position.x; T (1) = location.position.y; T (2) = location.position.z;
    //        std::cout<<"position x : "<<T[0]<<"position y: "<<T[1]<<"position z: "<<T[2]<<"\n";

    camera_pose.block (0, 3, 3, 1) = T;
    camera_pose (3, 3) = 1;
    //Transformation for the frustum camera ( in to be x forward, z right and y up)
    Eigen::Matrix4f pose_orig = camera_pose;
    Eigen::Matrix4f cam2robot;
    //the transofrmation is rotation by +90 around x axis
    cam2robot << 1, 0, 0, 0,
            0, 0,-1, 0,
            0, 1, 0, 0,
            0, 0, 0, 1;
    Eigen::Matrix4f pose_new = pose_orig * cam2robot;
    fc.setCameraPose (pose_new);
    fc.filter (*output);


    //*** Camera View Vector ****
    tf::Matrix3x3 rotation;
    Eigen::Matrix3d D;
    D= R.cast<double>();
    tf::matrixEigenToTF(D,rotation);
    rotation = rotation.transpose();
    tf::Quaternion orientation;
    rotation.getRotation(orientation);

    geometry_msgs::Pose output_vector;
    Eigen::Quaterniond q;
    geometry_msgs::Quaternion quet;
    tf::quaternionTFToEigen(orientation, q);
    tf::quaternionTFToMsg(orientation,quet);
    Eigen::Affine3d pose1;
    Eigen::Vector3d a;
    a[0]= T[0];
    a[1]= T[1];
    a[2]= T[2];
    pose1.translation() = a;
    tf::poseEigenToMsg(pose1, output_vector);

//    visualizeFOV(fc);

    //2:****voxel grid occlusion estimation *****
    Eigen::Quaternionf quat(q.w(),q.x(),q.y(),q.z());
    output->sensor_origin_  = Eigen::Vector4f(a[0],a[1],a[2],0);
    output->sensor_orientation_= quat;
    pcl::VoxelGridOcclusionEstimationT voxelFilter;
    voxelFilter.setInputCloud (output);
    //voxelFilter.setLeafSize (0.03279f, 0.03279f, 0.03279f);
    voxelFilter.setLeafSize (voxelRes, voxelRes, voxelRes);
    voxelFilter.initializeVoxelGrid();

    int state,ret;

    pcl::PointXYZ pt,p1,p2;
    pcl::PointXYZRGB point;
    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > out_ray;
    std::vector<geometry_msgs::Point> lineSegments;
    geometry_msgs::Point linePoint;

    // iterate over the entire frustum points
    for ( int i = 0; i < (int)output->points.size(); i ++ )
    {
        pcl::PointXYZ ptest = output->points[i];
        Eigen::Vector3i ijk = voxelFilter.getGridCoordinates( ptest.x, ptest.y, ptest.z);
        // process all free voxels
        int index = voxelFilter.getCentroidIndexAt (ijk);
        Eigen::Vector4f centroid = voxelFilter.getCentroidCoordinate (ijk);
        point = pcl::PointXYZRGB(0,244,0);
        point.x = centroid[0];
        point.y = centroid[1];
        point.z = centroid[2];

        if(index!=-1 )
        {
            out_ray.clear();
            ret = voxelFilter.occlusionEstimation( state,out_ray, ijk);
            //                        std::cout<<"State is:"<<state<<"\n";

            if(state != 1)
            {
                // estimate direction to target voxel
                Eigen::Vector4f direction = centroid - cloud->sensor_origin_;
                direction.normalize ();
                // estimate entry point into the voxel grid
                float tmin = voxelFilter.rayBoxIntersection (cloud->sensor_origin_, direction,p1,p2);
                if(tmin!=-1)
                {
                    // coordinate of the boundary of the voxel grid
                    Eigen::Vector4f start = cloud->sensor_origin_ + tmin * direction;
                    linePoint.x = cloud->sensor_origin_[0]; linePoint.y = cloud->sensor_origin_[1]; linePoint.z = cloud->sensor_origin_[2];
                    //std::cout<<"Box Min X:"<<linePoint.x<<" y:"<< linePoint.y<<" z:"<< linePoint.z<<"\n";
                    lineSegments.push_back(linePoint);

                    linePoint.x = start[0]; linePoint.y = start[1]; linePoint.z = start[2];
                    //std::cout<<"Box Max X:"<<linePoint.x<<" y:"<< linePoint.y<<" z:"<< linePoint.z<<"\n";
                    lineSegments.push_back(linePoint);

                    linePoint.x = start[0]; linePoint.y = start[1]; linePoint.z = start[2];
                    //std::cout<<"Box Max X:"<<linePoint.x<<" y:"<< linePoint.y<<" z:"<< linePoint.z<<"\n";
                    lineSegments.push_back(linePoint);

                    linePoint.x = centroid[0]; linePoint.y = centroid[1]; linePoint.z = centroid[2];
                    //std::cout<<"Box Max X:"<<linePoint.x<<" y:"<< linePoint.y<<" z:"<< linePoint.z<<"\n";
                    lineSegments.push_back(linePoint);

                    occlusionFreeCloud->points.push_back(ptest);
                }
            }
        }


    }
    FreeCloud.points = occlusionFreeCloud->points;
    return FreeCloud;
}
float OcclusionCulling::calcCoveragePercent(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{

    // *******************original cloud Grid***************************
    //used VoxelGridOcclusionEstimationT since the voxelGrid does not include getcentroid function
//        pcl::VoxelGridOcclusionEstimationT voxelFilterOriginal;
//        voxelFilterOriginal.setInputCloud (cloud);
//        voxelFilterOriginal.setLeafSize (voxelRes, voxelRes, voxelRes);
//        voxelFilterOriginal.initializeVoxelGrid();

     //*******************Occupied Cloud Grid***************************
    ros::Time covpercent_begin = ros::Time::now();
    pcl::VoxelGridOcclusionEstimationT voxelFilterOccupied;
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

        // iterate over the entire original voxel grid to get the size of the grid
//        for (int kk = min_b1.z (); kk <= max_b1.z (); ++kk)
//        {
//            for (int jj = min_b1.y (); jj <= max_b1.y (); ++jj)
//            {
//                for (int ii = min_b1.x (); ii <= max_b1.x (); ++ii)
//                {
//                    Eigen::Vector3i ijk1 (ii, jj, kk);
//                    int index1 = voxelFilterOriginal.getCentroidIndexAt (ijk1);
//                    if(index1!=-1)
//                    {
//                        OriginalVoxelsSize++;
//                    }

//                }
//            }
//        }

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

    std::cout<<" the coverage ratio is = "<<coverage_ratio<<"\n";
    std::cout<<" the number of covered voxels = "<<MatchedVoxels<<" voxel is covered"<<"\n";
    std::cout<<" the number of original voxels = "<<OriginalVoxelsSize<<" voxel"<<"\n\n\n";
    std::cout<<" the coverage percentage is = "<<coverage_percentage<<" %"<<"\n";

    ros::Time covpercent_end = ros::Time::now();
    double elapsed =  covpercent_end.toSec() - covpercent_begin.toSec();
    std::cout<<"Coverage Percentage Calculation duration (s) = "<<elapsed<<"\n";

    return coverage_percentage;
}
//float OcclusionCulling::calcCoveragePercent(geometry_msgs::Pose location)
//{

//    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ> temp;
//    ros::Time occlusion_begin = ros::Time::now();
//    temp= extractVisibleSurface(location);
//    tempCloud->points = temp.points;
//    ros::Time occlusion_end = ros::Time::now();
//    double elapsed =  occlusion_end.toSec() - occlusion_begin.toSec();
//    std::cout<<"Extract visible surface duration (s) = "<<elapsed<<"\n";
//    // *******************original cloud Grid***************************
//    //used VoxelGridOcclusionEstimationT since the voxelGrid does not include getcentroid function
////        pcl::VoxelGridOcclusionEstimationT voxelFilterOriginal;
////        voxelFilterOriginal.setInputCloud (cloud);
////        voxelFilterOriginal.setLeafSize (voxelRes, voxelRes, voxelRes);
////        voxelFilterOriginal.initializeVoxelGrid();

//     //*******************Occupied Cloud Grid***************************
//    ros::Time covpercent_begin = ros::Time::now();
//    pcl::VoxelGridOcclusionEstimationT voxelFilterOccupied;
////        voxelFilterOccupied.setInputCloud (occlusionFreeCloud);
//    voxelFilterOccupied.setInputCloud (tempCloud);
//    voxelFilterOccupied.setLeafSize (voxelRes, voxelRes, voxelRes);
//    voxelFilterOccupied.initializeVoxelGrid();



//     //*****************************************************************
//    Eigen::Vector3i  min_b = voxelFilterOccupied.getMinBoxCoordinates ();
//    Eigen::Vector3i  max_b = voxelFilterOccupied.getMaxBoxCoordinates ();
////        Eigen::Vector3i  min_b1 = voxelFilterOriginal.getMinBoxCoordinates ();
////        Eigen::Vector3i  max_b1 = voxelFilterOriginal.getMaxBoxCoordinates ();

//    float MatchedVoxels=0 ;//OriginalVoxelsSize=0, ;

//        // iterate over the entire original voxel grid to get the size of the grid
////        for (int kk = min_b1.z (); kk <= max_b1.z (); ++kk)
////        {
////            for (int jj = min_b1.y (); jj <= max_b1.y (); ++jj)
////            {
////                for (int ii = min_b1.x (); ii <= max_b1.x (); ++ii)
////                {
////                    Eigen::Vector3i ijk1 (ii, jj, kk);
////                    int index1 = voxelFilterOriginal.getCentroidIndexAt (ijk1);
////                    if(index1!=-1)
////                    {
////                        OriginalVoxelsSize++;
////                    }

////                }
////            }
////        }

//        //iterate through the entire coverage grid to check the number of matched voxel between the original and the covered ones
//    for (int kk = min_b.z (); kk <= max_b.z (); ++kk)
//    {
//        for (int jj = min_b.y (); jj <= max_b.y (); ++jj)
//        {
//            for (int ii = min_b.x (); ii <= max_b.x (); ++ii)
//            {

//                Eigen::Vector3i ijk (ii, jj, kk);
//                int index1 = voxelFilterOccupied.getCentroidIndexAt (ijk);
//                if(index1!=-1)
//                {
//                    Eigen::Vector4f centroid = voxelFilterOccupied.getCentroidCoordinate (ijk);
//                    Eigen::Vector3i ijk_in_Original= voxelFilterOriginal.getGridCoordinates(centroid[0],centroid[1],centroid[2]) ;

//                    int index = voxelFilterOriginal.getCentroidIndexAt (ijk_in_Original);

//                    if(index!=-1)
//                    {
//                        MatchedVoxels++;
//                    }
//                }

//            }
//        }
//    }

//    //calculating the coverage percentage
//    float coverage_ratio= MatchedVoxels/OriginalVoxelsSize;
//    float coverage_percentage= coverage_ratio*100;

//    std::cout<<" the coverage ratio is = "<<coverage_ratio<<"\n";
//    std::cout<<" the number of covered voxels = "<<MatchedVoxels<<" voxel is covered"<<"\n";
//    std::cout<<" the number of original voxels = "<<OriginalVoxelsSize<<" voxel"<<"\n\n\n";
//    std::cout<<" the coverage percentage is = "<<coverage_percentage<<" %"<<"\n";

//    ros::Time covpercent_end = ros::Time::now();
//    elapsed =  covpercent_end.toSec() - covpercent_begin.toSec();
//    std::cout<<"Coverage Percentage Calculation duration (s) = "<<elapsed<<"\n";

//    return coverage_percentage;
//}

//void OcclusionCulling::visualizeFOV(pcl::FrustumCullingTT& fc)
//{
////*** visualization the FOV *****
//    std::vector<geometry_msgs::Point> fov_points;
//    geometry_msgs::Point point1;
//    point1.x=fc.fp_bl[0];point1.y=fc.fp_bl[1];point1.z=fc.fp_bl[2]; fov_points.push_back(point1);//0
//    point1.x=fc.fp_br[0];point1.y=fc.fp_br[1];point1.z=fc.fp_br[2]; fov_points.push_back(point1);//1
//    point1.x=fc.fp_tr[0];point1.y=fc.fp_tr[1];point1.z=fc.fp_tr[2]; fov_points.push_back(point1);//2
//    point1.x=fc.fp_tl[0];point1.y=fc.fp_tl[1];point1.z=fc.fp_tl[2]; fov_points.push_back(point1);//3
//    point1.x=fc.np_bl[0];point1.y=fc.np_bl[1];point1.z=fc.np_bl[2]; fov_points.push_back(point1);//4
//    point1.x=fc.np_br[0];point1.y=fc.np_br[1];point1.z=fc.np_br[2]; fov_points.push_back(point1);//5
//    point1.x=fc.np_tr[0];point1.y=fc.np_tr[1];point1.z=fc.np_tr[2]; fov_points.push_back(point1);//6
//    point1.x=fc.np_tl[0];point1.y=fc.np_tl[1];point1.z=fc.np_tl[2]; fov_points.push_back(point1);//7

//    std::vector<geometry_msgs::Point> fov_linesNearFar;
//    fov_linesNearFar.push_back(fov_points[0]);fov_linesNearFar.push_back(fov_points[1]);
//    fov_linesNearFar.push_back(fov_points[1]);fov_linesNearFar.push_back(fov_points[2]);
//    fov_linesNearFar.push_back(fov_points[2]);fov_linesNearFar.push_back(fov_points[3]);
//    fov_linesNearFar.push_back(fov_points[3]);fov_linesNearFar.push_back(fov_points[0]);

//    fov_linesNearFar.push_back(fov_points[4]);fov_linesNearFar.push_back(fov_points[5]);
//    fov_linesNearFar.push_back(fov_points[5]);fov_linesNearFar.push_back(fov_points[6]);
//    fov_linesNearFar.push_back(fov_points[6]);fov_linesNearFar.push_back(fov_points[7]);
//    fov_linesNearFar.push_back(fov_points[7]);fov_linesNearFar.push_back(fov_points[4]);
//    linesList1 = drawLines(fov_linesNearFar,3333,1);//red

//    std::vector<geometry_msgs::Point> fov_linestop;
//    fov_linestop.push_back(fov_points[7]);fov_linestop.push_back(fov_points[3]);//top
//    fov_linestop.push_back(fov_points[6]);fov_linestop.push_back(fov_points[2]);//top
//    linesList2 = drawLines(fov_linestop,4444,2);//green
//    std::vector<geometry_msgs::Point> fov_linesbottom;
//    fov_linesbottom.push_back(fov_points[5]);fov_linesbottom.push_back(fov_points[1]);//bottom
//    fov_linesbottom.push_back(fov_points[4]);fov_linesbottom.push_back(fov_points[0]);//bottom
//    linesList3 = drawLines(fov_linesbottom,5555,3);//blue

//    lines_pub1.publish(linesList1);
//    lines_pub2.publish(linesList2);
//    lines_pub3.publish(linesList3);

//}
visualization_msgs::Marker OcclusionCulling::drawLines(std::vector<geometry_msgs::Point> links, int id, int c_color)
{
    visualization_msgs::Marker linksMarkerMsg;
    linksMarkerMsg.header.frame_id="/base_point_cloud";
    linksMarkerMsg.header.stamp=ros::Time::now();
    linksMarkerMsg.ns="link_marker";
    linksMarkerMsg.id = id;
    linksMarkerMsg.type = visualization_msgs::Marker::LINE_LIST;
    linksMarkerMsg.scale.x = 0.006;
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
