#include "component_test/visualization_voxelgrid.h"


VoxelGridVisualization::VoxelGridVisualization(ros::NodeHandle &n, std::string modelName, pcl::VoxelGridT &grid, std::string frame_id):
    nh(n),
    model(modelName),
    voxelFilter(grid),
    frameid(frame_id)
{
    originalCloudPub      = n.advertise<sensor_msgs::PointCloud2>("original_cloud", 100);
//    voxelPub              = n.advertise<geometry_msgs::PoseArray>("voxel", 100); (still being debugged)
    voxelPointsPub        = n.advertise<sensor_msgs::PointCloud2>("voxel_points", 100);
    intersectionPointPub  = n.advertise<visualization_msgs::Marker>("intersection_points", 10);
    gridBBPub             = n.advertise<visualization_msgs::Marker>("voxels_box", 10);
    linesPub              = n.advertise<visualization_msgs::Marker>("lines", 10);

    cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
    voxelPointsPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
    voxelPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);

    std::string path = ros::package::getPath("component_test");
    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/"+model, *cloud);

    //visualize the original cloud
    sensor_msgs::PointCloud2 cloud1;
    pcl::toROSMsg(*cloud, cloud1); //cloud of original (white) using original cloud
    cloud1.header.stamp = ros::Time::now();
    cloud1.header.frame_id = frameid; //change according to the global frame please!!
    originalCloudPub.publish(cloud1);
}

VoxelGridVisualization::VoxelGridVisualization():
     model(NULL)
{

}
VoxelGridVisualization::~VoxelGridVisualization()
{
}
void VoxelGridVisualization::getBBCentroids()
{
    Eigen::Vector3i  min_b = voxelFilter.getMinBoxCoordinates ();
    Eigen::Vector3i  max_b = voxelFilter.getMaxBoxCoordinates ();

    std::vector<Eigen::Vector3i> b_points;
    Eigen::Vector3i pt1(min_b.x(),min_b.y(),min_b.z());b_points.push_back(pt1);
    Eigen::Vector3i pt2(max_b.x(),min_b.y(),min_b.z());b_points.push_back(pt2);
    Eigen::Vector3i pt3(max_b.x(),min_b.y(),max_b.z());b_points.push_back(pt3);
    Eigen::Vector3i pt4(min_b.x(),min_b.y(),max_b.z());b_points.push_back(pt4);
    Eigen::Vector3i pt5(min_b.x(),max_b.y(),max_b.z());b_points.push_back(pt5);
    Eigen::Vector3i pt6(min_b.x(),max_b.y(),min_b.z());b_points.push_back(pt6);
    Eigen::Vector3i pt7(max_b.x(),max_b.y(),min_b.z());b_points.push_back(pt7);
    Eigen::Vector3i pt8(max_b.x(),max_b.y(),max_b.z());b_points.push_back(pt8);
    bb_points = b_points;

    std::vector<Point> b_centroids;
    Eigen::Vector4f centroid;
    for(int i=0; i<b_points.size() ;i++){
        centroid = voxelFilter.getCentroidCoordinate (b_points[i]);
        Point pt_test(centroid[0], centroid[1], centroid[2]);
        b_centroids.push_back(pt_test);
    }
    bb_centroids = b_centroids;
}
void VoxelGridVisualization::visualizeBB()
{
    getBBCentroids();
    geometry_msgs::Point linePoint;
    std::vector<geometry_msgs::Point> lineSegments;
    for(int i =0 ; i<bb_centroids.size(); i++){
        if(i+1 != bb_centroids.size()){
            linePoint.x = bb_centroids.at(i)[0];
            linePoint.y = bb_centroids.at(i)[1];
            linePoint.z = bb_centroids.at(i)[2];
            lineSegments.push_back(linePoint);
            linePoint.x = bb_centroids.at(i+1)[0];
            linePoint.y = bb_centroids.at(i+1)[1];
            linePoint.z = bb_centroids.at(i+1)[2];
            lineSegments.push_back(linePoint);
        }
    }
    linePoint.x = bb_centroids.at(0)[0];linePoint.y = bb_centroids.at(0)[1];linePoint.z = bb_centroids.at(0)[2];lineSegments.push_back(linePoint);
    linePoint.x = bb_centroids.at(3)[0];linePoint.y = bb_centroids.at(3)[1];linePoint.z = bb_centroids.at(3)[2];lineSegments.push_back(linePoint);
    linePoint.x = bb_centroids.at(0)[0];linePoint.y = bb_centroids.at(0)[1];linePoint.z = bb_centroids.at(0)[2];lineSegments.push_back(linePoint);
    linePoint.x = bb_centroids.at(5)[0];linePoint.y = bb_centroids.at(5)[1];linePoint.z = bb_centroids.at(5)[2];lineSegments.push_back(linePoint);
    linePoint.x = bb_centroids.at(1)[0];linePoint.y = bb_centroids.at(1)[1];linePoint.z = bb_centroids.at(1)[2];lineSegments.push_back(linePoint);
    linePoint.x = bb_centroids.at(6)[0];linePoint.y = bb_centroids.at(6)[1];linePoint.z = bb_centroids.at(6)[2];lineSegments.push_back(linePoint);
    linePoint.x = bb_centroids.at(2)[0];linePoint.y = bb_centroids.at(2)[1];linePoint.z = bb_centroids.at(2)[2];lineSegments.push_back(linePoint);
    linePoint.x = bb_centroids.at(7)[0];linePoint.y = bb_centroids.at(7)[1];linePoint.z = bb_centroids.at(7)[2];lineSegments.push_back(linePoint);
    linePoint.x = bb_centroids.at(7)[0];linePoint.y = bb_centroids.at(7)[1];linePoint.z = bb_centroids.at(7)[2];lineSegments.push_back(linePoint);
    linePoint.x = bb_centroids.at(4)[0];linePoint.y = bb_centroids.at(4)[1];linePoint.z = bb_centroids.at(4)[2];lineSegments.push_back(linePoint);
    int c_color[3];
    c_color[0]=1; c_color[1]=0; c_color[2]=0;
    visualization_msgs::Marker linesList = drawLines(lineSegments,c_color,0.1,frameid);
    gridBBPub.publish(linesList);

}
void VoxelGridVisualization::intersectionFOVBB(Point origin, std::vector<Point> fov_pts)
{
    sensor_o=origin;
    fov_points = fov_pts;
    //create rays
    std::vector<Ray> rays;
    for (int i=0; i<fov_pts.size(); i++){
        Ray rayX(origin,fov_pts[i]);
        rays.push_back(rayX);
    }

    //create the Triangles that forms the Bounding Box (not professional and will be fixed later)
    std::vector<Triangle> bb_triangles;
    Triangle Tr1(bb_centroids[0],bb_centroids[3],bb_centroids[1]);bb_triangles.push_back(Tr1);
    Triangle Tr2(bb_centroids[3],bb_centroids[2],bb_centroids[1]);bb_triangles.push_back(Tr2);
    Triangle Tr3(bb_centroids[3],bb_centroids[2],bb_centroids[7]);bb_triangles.push_back(Tr3);
    Triangle Tr4(bb_centroids[3],bb_centroids[4],bb_centroids[2]);bb_triangles.push_back(Tr4);
    Triangle Tr5(bb_centroids[7],bb_centroids[4],bb_centroids[5]);bb_triangles.push_back(Tr5);
    Triangle Tr6(bb_centroids[7],bb_centroids[6],bb_centroids[5]);bb_triangles.push_back(Tr6);

    Triangle Tr7(bb_centroids[4],bb_centroids[3],bb_centroids[0]);bb_triangles.push_back(Tr7);
    Triangle Tr8(bb_centroids[4],bb_centroids[5],bb_centroids[0]);bb_triangles.push_back(Tr8);
    Triangle Tr9(bb_centroids[5],bb_centroids[0],bb_centroids[1]);bb_triangles.push_back(Tr9);
    Triangle Tr10(bb_centroids[5],bb_centroids[6],bb_centroids[1]);bb_triangles.push_back(Tr10);
    Triangle Tr11(bb_centroids[7],bb_centroids[2],bb_centroids[1]);bb_triangles.push_back(Tr11);
    Triangle Tr12(bb_centroids[7],bb_centroids[6],bb_centroids[1]);bb_triangles.push_back(Tr12);

    //find the intersection between the rays and the BB
    std::vector<Point> i_pts;
    for (int i=0; i<bb_triangles.size(); i++){
        for (int j=0; j<rays.size(); j++){
            //          CGAL::Object result = CGAL::intersection(bb_planes[i], lines[j]);
            CGAL::Object result = CGAL::intersection(rays[j], bb_triangles[i]);
            if (const Point *ipoint = CGAL::object_cast<Point>(&result)) {
                Point t(ipoint->hx(), ipoint->hy(), ipoint->hz());
                i_pts.push_back(t);
            }
        }
    }
    i_points = i_pts;

}
void VoxelGridVisualization::visualizeFOVBBIntersection(Point origin, std::vector<Point> fov_pts)
{
    //visualizae Bounding Box
    visualizeBB();

    //visualize intersection points
    geometry_msgs::Point linePoint;
    std::vector<geometry_msgs::Point> pointSegments;
    for(int i =0 ; i<i_points.size(); i++){
        linePoint.x = i_points.at(i)[0];
        linePoint.y = i_points.at(i)[1];
        linePoint.z = i_points.at(i)[2];
        pointSegments.push_back(linePoint);
     }
    int c_color[3];
    c_color[0]=0; c_color[1]=1; c_color[2]=0;
    visualization_msgs::Marker pointsList = drawPoints(pointSegments,c_color,0.8,frameid);
    intersectionPointPub.publish(pointsList);

    //visualize the lines from sensor origin to the FOV points
    std::vector<geometry_msgs::Point> lineSegments2;
    for (int i =0 ; i<4; i++)
    {
        linePoint.x = origin[0];
        linePoint.y = origin[1];
        linePoint.z = origin[2];
        lineSegments2.push_back(linePoint);
        linePoint.x = fov_pts[i][0];
        linePoint.y = fov_pts[i][1];
        linePoint.z = fov_pts[i][2];
        lineSegments2.push_back(linePoint);

    }
    c_color[0]=0; c_color[1]=0; c_color[2]=1;
    visualization_msgs::Marker linesList2 = drawLines(lineSegments2,c_color,0.2,frameid);
    linesPub.publish(linesList2);
}
void VoxelGridVisualization::selectedVoxelPoints(Eigen::Vector3i ijk, pcl::PointCloud<pcl::PointXYZ>::Ptr &voxelPoints)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);

    int s = voxelFilter.voxelSet[ijk[0]][ijk[1]][ijk[2]].size();
    for (int i =0; i<s; i++)
        tempCloudPtr->points.push_back(voxelFilter.voxelSet[ijk[0]][ijk[1]][ijk[2]][i]);

//    std::cout<<"size of the points: "<<voxelPoints->points.size();
    voxelPointsPtr->points = tempCloudPtr->points;
    voxelPoints->points = tempCloudPtr->points;

    //visualization
    //visualize the selected voxel centroid (still being debugged)
//    std::vector<geometry_msgs::Point> pointSegments;
//    Eigen::Vector4f centroid = voxelFilter.getCentroidCoordinate (ijk);
//    geometry_msgs::Pose pos;
//    geometry_msgs::PoseArray poses;
//    pos.position.x = centroid[0];pos.position.y = centroid[1];pos.position.z = centroid[2];
//    poses.poses.push_back(pos);
//    poses.header.frame_id= "world";
//    poses.header.stamp = ros::Time::now();
//    voxelPub.publish(poses);
//    geometry_msgs::Point pt;
//    pt.x = centroid[0];pt.y = centroid[1];pt.z = centroid[2];
//    std::cout<<"centroid: x="<<centroid[0]<<" y="<<centroid[1]<<" z="<<centroid[2]<<"\n";
//    pointSegments.push_back(pt);
//    int c_color[3];
//    c_color[0]=0; c_color[1]=0; c_color[2]=1;
//    visualization_msgs::Marker pointsList = drawPoints(pointSegments,c_color,1.2,frameid);
//    voxelPub.publish(pointsList);

    //visualize the points
    sensor_msgs::PointCloud2 cloud1;
    pcl::toROSMsg(*voxelPointsPtr, cloud1); //red
    cloud1.header.stamp = ros::Time::now();
    cloud1.header.frame_id = frameid;
    voxelPointsPub.publish(cloud1);


}

visualization_msgs::Marker VoxelGridVisualization::drawLines(std::vector<geometry_msgs::Point> links, int c_color[], double scale, std::string frame_id)
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
visualization_msgs::Marker VoxelGridVisualization::drawPoints(std::vector<geometry_msgs::Point> points, int c_color[], double scale, std::string frame_id)
{
    visualization_msgs::Marker pointMarkerMsg;
    pointMarkerMsg.header.frame_id=frame_id;
    pointMarkerMsg.header.stamp=ros::Time::now();
    pointMarkerMsg.ns="point_marker";
    pointMarkerMsg.id = 444444;
    pointMarkerMsg.type = visualization_msgs::Marker::POINTS;
    pointMarkerMsg.scale.x = scale;
    pointMarkerMsg.scale.y = scale;
    pointMarkerMsg.action  = visualization_msgs::Marker::ADD;
    pointMarkerMsg.lifetime  = ros::Duration(1000000);
    std_msgs::ColorRGBA color;
    color.r = (float)c_color[0]; color.g=(float)c_color[1]; color.b=(float)c_color[2], color.a=1.0f;

    std::vector<geometry_msgs::Point>::iterator pointsIterator;
    for(pointsIterator = points.begin();pointsIterator != points.end();pointsIterator++)
    {
        pointMarkerMsg.points.push_back(*pointsIterator);
        pointMarkerMsg.colors.push_back(color);
    }
   return pointMarkerMsg;
}

