#include "component_test/mesh_surface.h"
#include "CGAL/exceptions.h"
//They are static member variables (this should be done otherwise you will get undefined reference to these variables error)
Triangles MeshSurface::meshTA,MeshSurface::meshTB;
std::vector<int> MeshSurface::facesIndicesA, MeshSurface::facesIndicesB;

MeshSurface::MeshSurface(ros::NodeHandle &n, Triangles TA, Triangles TB):
    nh(n)
{
    setCGALMeshA(TA);
    setCGALMeshB(TB);
    count=0;
}
MeshSurface::MeshSurface(ros::NodeHandle &n):
    nh(n)
{
    count=0;
}
MeshSurface::MeshSurface()
{
}
MeshSurface::~MeshSurface()
{
}
void MeshSurface::setCGALMeshA(Triangles TA)
{
    meshTA=TA;
}
void MeshSurface::setCGALMeshB(Triangles TB)
{
    meshTB=TB;
}

//function that erases the elements of the mesh triangles vectors
void MeshSurface::clear()
{
     meshTA.erase(meshTA.begin(),meshTA.end());
     meshTB.erase(meshTB.begin(),meshTB.end());
}

//function that preforms reconstruction (meshing) from unordered pcl point cloud (IMP for mesh surface area evaluation)
void MeshSurface::meshingPCL(pcl::PointCloud<pcl::PointXYZ> pointCloud, Triangles& cgalMeshT, bool saveMeshFlag)
{
    ros::Time tic1 = ros::Time::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr (new pcl::PointCloud<pcl::PointXYZ>);

    cloudPtr->points = pointCloud.points;
    //* the data should be available in cloud

    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloudPtr);
    n.setInputCloud (cloudPtr);
    n.setSearchMethod (tree);
    n.setKSearch (100);
//    n.setRadiusSearch(0.3);
    n.compute (*normals);

    // Normal Estimation OMP (speed up the algorithm of NE)
    //    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    //    int nr_cores = boost::thread::hardware_concurrency();
    //    std::cout<<"number of cores: "<<nr_cores<<std::endl;
    //    ne.setNumberOfThreads(nr_cores);
    //    ne.setInputCloud(cloudPtr);
    //    ne.setRadiusSearch(0.3);
    //    ne.setSearchMethod(tree);
    //    Eigen::Vector4f centroid;
    //    pcl::compute3DCentroid(*cloudPtr, centroid);
    //    ne.setViewPoint(centroid[0], centroid[1], centroid[2]);
    //    ne.compute(*normals);

    //* normals should not contain the point normals + surface curvatures
    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloudPtr, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (0.5);//0.5

    // Set typical values for the parameters
    gp3.setMu (3.0);
    gp3.setMaximumNearestNeighbors (5000);//100
    gp3.setMaximumSurfaceAngle(M_PI); //180 deg ... it is originally 45 degrees
    gp3.setMinimumAngle(M_PI/4); // 45 deg ...it is originally 10 degrees
    gp3.setMaximumAngle(M_PI/2); // 90 deg ... it is originally 120 degrees
    gp3.setNormalConsistency(true);// setting to true doesn't make differece, the mesh normals remains unconsistant, anyways it is not imp in our case :)

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);

    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();

    if(saveMeshFlag)
    {
        std::stringstream ss;
        ss << count++;
        std::string fileName=std::string("testpcl")+ ss.str() +".ply";
        pcl::io::savePolygonFilePLY(fileName,triangles);
    }

    //storing pcl mesh (OPTIONAL : if we want to use pcl polygon mesh)
    //    pclMesh->polygons= triangles.polygons;
    //    pclMesh->cloud= triangles.cloud;
    ros::Time tic2 = ros::Time::now();
    std::cout<<"\nMeshing PCL took:"<< tic2.toSec() - tic1.toSec();

    //storing cgal mesh (maybe should be optimized later)
    pcl::PointCloud<pcl::PointXYZ> verticesCloud;
    pcl::fromPCLPointCloud2 (triangles.cloud, verticesCloud);
    Triangle_3 tri;
    for(int i=0; i<triangles.polygons.size();i++)
    {
        pcl::PointXYZ pt;
        pt=verticesCloud.points[triangles.polygons[i].vertices[0]];
        Point_3 p1(pt.data[0], pt.data[1], pt.data[2]);
        pt=verticesCloud.points[triangles.polygons[i].vertices[1]];
        Point_3 p2(pt.data[0], pt.data[1], pt.data[2]);
        pt=verticesCloud.points[triangles.polygons[i].vertices[2]];
        Point_3 p3(pt.data[0], pt.data[1], pt.data[2]);
        tri = Triangle_3(p1,p2,p3);
        cgalMeshT.push_back(tri);
    }
    ros::Time tic3 = ros::Time::now();
//    std::cout<<"\n Coversion to CGAL :"<< tic3.toSec() - tic2.toSec();

}

//provides inaccurate reconstruction
void MeshSurface::meshingPoissonPCL(pcl::PointCloud<pcl::PointXYZ> pointCloud, Triangles& cgalMeshT, bool saveMeshFlag)
{
    ros::Time tic1 = ros::Time::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr (new pcl::PointCloud<pcl::PointXYZ>);

    cloudPtr->points = pointCloud.points;
    std::cout << "loaded" << std::endl;

    std::cout << "begin passthrough filter" << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PassThrough<pcl::PointXYZ> filter;
    filter.setInputCloud(cloudPtr);
    filter.filter(*filtered);
    std::cout << "passthrough filter complete" << std::endl;

    std::cout << "begin normal estimation" << std::endl;
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNumberOfThreads(8);
    ne.setInputCloud(filtered);
    ne.setRadiusSearch(0.2);
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*filtered, centroid);
    ne.setViewPoint(centroid[0], centroid[1], centroid[2]);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
    ne.compute(*cloud_normals);
    std::cout << "normal estimation complete" << endl;
    std::cout << "reverse normals' direction" << endl;

    for(size_t i = 0; i < cloud_normals->size(); ++i){
      cloud_normals->points[i].normal_x *= -1;
      cloud_normals->points[i].normal_y *= -1;
      cloud_normals->points[i].normal_z *= -1;
    }

    std::cout << "combine points and normals" << std::endl;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::concatenateFields(*filtered, *cloud_normals, *cloud_smoothed_normals);

    std::cout << "begin poisson reconstruction" << std::endl;
    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setDepth(8);
    poisson.setSolverDivide (8);
    poisson.setIsoDivide (8);
    poisson.setPointWeight(4.0f);
    poisson.setInputCloud(cloud_smoothed_normals);
    pcl::PolygonMesh mesh;
    poisson.reconstruct(mesh);

    if(saveMeshFlag)
    {
        std::stringstream ss;
        ss << count++;
        std::string fileName=std::string("test")+ ss.str() +".ply";
        pcl::io::savePLYFile(fileName,mesh);
    }

    //storing pcl mesh (OPTIONAL : if we want to use pcl polygon mesh)
    //    pclMesh->polygons= mesh.polygons;
    //    pclMesh->cloud= mesh.cloud;
    ros::Time tic2 = ros::Time::now();
    std::cout<<"\nMeshing PCL took:"<< tic2.toSec() - tic1.toSec();

    //storing cgal mesh (maybe should be optimized later)
    pcl::PointCloud<pcl::PointXYZ> verticesCloud;
    pcl::fromPCLPointCloud2 (mesh.cloud, verticesCloud);
    Triangle_3 tri;
    for(int i=0; i<mesh.polygons.size();i++)
    {
        pcl::PointXYZ pt;
        pt=verticesCloud.points[mesh.polygons[i].vertices[0]];
        Point_3 p1(pt.data[0], pt.data[1], pt.data[2]);
        pt=verticesCloud.points[mesh.polygons[i].vertices[1]];
        Point_3 p2(pt.data[0], pt.data[1], pt.data[2]);
        pt=verticesCloud.points[mesh.polygons[i].vertices[2]];
        Point_3 p3(pt.data[0], pt.data[1], pt.data[2]);
        tri = Triangle_3(p1,p2,p3);
        cgalMeshT.push_back(tri);
    }
    ros::Time tic3 = ros::Time::now();
    std::cout<<"\n Conversion to CGAL :"<< tic3.toSec() - tic2.toSec();

}
void writeOFFReconstructionMesh(const Reconstruction& reconstruct, std::string name)
{
  std::ofstream output(name.c_str());
  output << "OFF " << reconstruct.number_of_points() << " "
         << reconstruct.number_of_triangles() << " 0\n";
  std::copy(reconstruct.points_begin(),
            reconstruct.points_end(),
            std::ostream_iterator<RPoint>(output,"\n"));
  for( Triple_iterator it = reconstruct.surface_begin(); it != reconstruct.surface_end(); ++it )
      output << "3 " << *it << std::endl;
}
//provides good reconstruction
void MeshSurface::meshingScaleSpaceCGAL(pcl::PointCloud<pcl::PointXYZ> pointCloud, Triangles& cgalMeshT, bool saveMeshFlag)
{
    ros::Time tic2 = ros::Time::now();

    Point_collection points;
    //convert the cloud to cgal
    for(int i=0; i<pointCloud.points.size();i++)
    {
        RPoint pt;
        pt = inexactPoint(pointCloud.points.at(i).x, pointCloud.points.at(i).y, pointCloud.points.at(i).z);
        points.push_back(pt);
    }

    try{
        // Construct the reconstruction with parameters for
        // the neighborhood squared radius estimation.
        Reconstruction reconstruct( 20, 200 ); //was 10neigbours,100samples
        // Add the points.
        reconstruct.insert( points.begin(), points.end() );
        // Advance the scale-space several steps.
        // This automatically estimates the scale-space.

        reconstruct.increase_scale( 1 ); //was 2
        // Reconstruct the surface from the current scale-space.
        //     std::cout << "Neighborhood squared radius is "
        //               << reconstruct.neighborhood_squared_radius() << std::endl;
        reconstruct.reconstruct_surface();
        //     std::cout << "First reconstruction done." << std::endl;


        //     reconstruct.increase_scale( 2 );
        //     // Reconstruct the surface from the current scale-space.
        //     std::cout << "Neighborhood squared radius is "
        //               << reconstruct.neighborhood_squared_radius() << std::endl;
        //     reconstruct.reconstruct_surface();
        //     std::cout << "Second reconstruction done." << std::endl;
        // Write the reconstruction.


        ros::Time toc1 = ros::Time::now();
        //     std::cout<<"\nscale space took:"<< toc1.toSec() - tic2.toSec()<<std::endl;
        Triangle_3 tri;

        for( Triple_iterator it = reconstruct.surface_begin(); it != reconstruct.surface_end(); ++it )
        {
            Reconstruction::Triple t = *it;
            Point_3 p1(points.at(t.at(0))[0], points.at(t.at(0))[1], points.at(t.at(0))[2]);
            Point_3 p2(points.at(t.at(1))[0], points.at(t.at(1))[1], points.at(t.at(1))[2]);
            Point_3 p3(points.at(t.at(2))[0], points.at(t.at(2))[1], points.at(t.at(2))[2]);
            tri = Triangle_3(p1,p2,p3);
            cgalMeshT.push_back(tri);
            //         std::cout<<points.at(t.at(0))[0]<<" "<<points.at(t.at(0))[1]<<" "<<points.at(t.at(0))[2]<<std::endl;

        }


        if(saveMeshFlag)
        {
            std::stringstream ss;
            ss << count++;
            std::string fileName=std::string("reconstruction")+ ss.str() +".off";
            writeOFFReconstructionMesh(reconstruct, fileName);
            std::cout <<"number of triangles :"<<reconstruct.number_of_triangles()<<std::endl;
        }
    }catch(CGAL::Precondition_exception e){std::cout<<"*****CGAL scale space throwed an Exception****"<<std::endl;}
}

//creates boxes for each face of the cgal Triangle vector
void MeshSurface::box_up(Triangles & T, std::vector<Box> & boxes)
{
    boxes.reserve(T.size());
    for (
         TrianglesIterator tit = T.begin();
         tit != T.end();
         ++tit)
    {
        boxes.push_back(Box(tit->bbox(), tit));
    }
}
//callback function that is used by the cgal box intersection function to perform triangle to triangle intersection
static void cb(const Box &a, const Box &b)
{
    using namespace std;
    // index in Faces Matrix (libigl Matrices but it is not used here) and in CGAL Triangle vector
    int fa = a.handle()-MeshSurface::meshTA.begin();
    int fb = b.handle()-MeshSurface::meshTB.begin();
    const Triangle_3 & A = *a.handle();
    const Triangle_3 & B = *b.handle();
    try{
        if(CGAL::do_intersect(A,B))
        {
            // There was an intersection
            MeshSurface::facesIndicesA.push_back(fa); //the index of the intersected face from mesh 1
            MeshSurface::facesIndicesB.push_back(fb); // the index of the intersected face from mesh 2
        }
    }catch(CGAL::Precondition_exception e){std::cout<<"*****Exception3****"<<std::endl;}

}
// This function finds the intersection area between two meshes and return the area and fill the passed vector with the intersected faces from the second mesh
double MeshSurface::getIntersectionArea(Triangles& intersectionFaces)
{
    try{
        std::vector<Box> A_boxes,B_boxes;
        box_up(meshTA,A_boxes);//create boxes for cgal triangles mesh A
        box_up(meshTB,B_boxes);//create boxes for cgal triangles mesh B

        CGAL::box_intersection_d(
                    A_boxes.begin(), A_boxes.end(),
                    B_boxes.begin(), B_boxes.end(),
                    &cb);
    }catch(CGAL::Precondition_exception e){std::cout<<"*****Exception2****"<<std::endl;}

    //facesIndices contains the indices of the intersected triangles from mesh B
    // here we sort it and remove duplicates in order to use it in area calculation (maybe I should find a better optimized way later)
    std::set<int> s;
    for(int i=0; i<facesIndicesB.size();i++)
        s.insert(facesIndicesB[i]);
    facesIndicesB.assign( s.begin(), s.end() );
    std::sort( facesIndicesB.begin(), facesIndicesB.end() );

    double areaInt;
    //filling the passed parameter with the intersected faces from mesh B
    if(facesIndicesB.size() != 0){//important in the case of no intersection

        for(int j=0; j<facesIndicesB.size();j++)
        {
            intersectionFaces.push_back(meshTB[facesIndicesB[j]]);
        }

        //calculating the intersection estimated area based on the intersected triangles from mesh B
        areaInt = calcCGALMeshSurfaceArea(intersectionFaces);
        facesIndicesB.erase(facesIndicesB.begin(),facesIndicesB.end());//in order not to accumelate the intersected faces (it causes a major problem when iterative intersection checks are performed)

    }else areaInt=0.0;
    std::cout<<"stage3"<<std::endl;

    return areaInt;
}

double MeshSurface::getExtraArea(Triangles& extraAreaFaces)
{
    try{
        std::vector<Box> A_boxes,B_boxes;
        box_up(meshTA,A_boxes);//create boxes for cgal triangles mesh A
        box_up(meshTB,B_boxes);//create boxes for cgal triangles mesh B
        ros::Time tic1 = ros::Time::now();
        CGAL::box_intersection_d(
                    A_boxes.begin(), A_boxes.end(),
                    B_boxes.begin(), B_boxes.end(),
                    &cb);
        ros::Time tic2 = ros::Time::now();
//        std::cout<<"\nintersection Calc:"<< tic2.toSec() - tic1.toSec();
    }catch(CGAL::Precondition_exception e){std::cout<<"*****Exception1****"<<std::endl;}


    //facesIndices contains the indices of the intersected triangles from mesh B
    // here we sort it and remove duplicates in order to use it in area calculation (maybe I should find a better optimized way later)
    std::set<int> s;
    for(int i=0; i<facesIndicesB.size();i++)
        s.insert(facesIndicesB[i]);
    facesIndicesB.assign( s.begin(), s.end() );
    std::sort( facesIndicesB.begin(), facesIndicesB.end() );

    std::vector<Triangle_3> extraAreaFacesTemp;
    int i=0;

    double extraAreaB;
    ros::Time tic3 = ros::Time::now();
    if(facesIndicesB.size()!=0){//important in the case of no intersection
        for(int j=0; j<meshTB.size();j++)
        {
            if(j==facesIndicesB[i])
            {
                //I can also get the intersection here and calc the area at the end (but I did it in a seperate function)
                //later I'll merge them better
                i++;
            }else
            {
                extraAreaFaces.push_back(meshTB[j]);// it is the passed vector which could contain other faces (we could take advantage in the path planning)
                extraAreaFacesTemp.push_back(meshTB[j]);// just to make sure that we are going to calculate the extra area of the additional faces (maybe we should remove it later for optimization reasons)
            }

        }
        //calculating the Extra estimated area based on the extra triangles from mesh B
        //remeber: this the extra area from the second mesh which is mesh B)
        extraAreaB = calcCGALMeshSurfaceArea(extraAreaFacesTemp);
        facesIndicesB.erase(facesIndicesB.begin(),facesIndicesB.end());//in order not to accumelate the intersected faces (it causes a major problem when iterative intersection checks are performed)

    }else {
        for(int j=0; j<meshTB.size();j++)
        {
            extraAreaFaces.push_back(meshTB[j]);// it is the passed vector which could contain other faces (we could take advantage in the path planning)
            extraAreaFacesTemp.push_back(meshTB[j]);
        }
        //calculating the Extra estimated area based on the extra triangles from mesh B
        //remeber: this the extra area from the second mesh which is mesh B)
        extraAreaB = calcCGALMeshSurfaceArea(extraAreaFacesTemp);
        facesIndicesB.erase(facesIndicesB.begin(),facesIndicesB.end());//in order not to accumelate the intersected faces (it causes a major problem when iterative intersection checks are performed)

    }
    ros::Time tic4 = ros::Time::now();
//    std::cout<<"\nExtra Area LOOP Calc:"<< tic4.toSec() - tic3.toSec();

    return extraAreaB;
}

//function that calculates the mesh surface area, the mesh is represented by the cgal triangles
double MeshSurface::calcCGALMeshSurfaceArea(std::vector<Triangle_3> mesh)
{
    int s = mesh.size();
    double totalArea=0.0;
    // remember: Exact Kernel can't be used in calculation and comparisons, you should do coversion to simple cartesian kernel
    converter to_simple;// to convert from Exact cgal to simple cartesian kernel
    Point_3_S convertedPt;
    for (int i =0; i<s; i++){
        pcl::PointCloud<pcl::PointXYZ> temp_cloud;
        pcl::PointXYZ pt;
        convertedPt = to_simple( mesh.at(i)[0] );
        pt.data[0]= convertedPt.x();pt.data[1]=  convertedPt.y();pt.data[2]=  convertedPt.z();
        temp_cloud.points.push_back( pt );
        convertedPt = to_simple( mesh.at(i)[1] );
        pt.data[0]= convertedPt.x();pt.data[1]= convertedPt.y();pt.data[2]=  convertedPt.z();
        temp_cloud.points.push_back( pt );
        convertedPt = to_simple( mesh.at(i)[2] );
        pt.data[0]= convertedPt.x();pt.data[1]=  convertedPt.y();pt.data[2]= convertedPt.z();
        temp_cloud.points.push_back( pt );
        totalArea += pcl::calculatePolygonArea(temp_cloud);
    }
//    std::cout<<"total area "<<totalArea<<"\n";
    return totalArea;
}

//OPTIONAL: function that calculates the mesh surface area, the mesh is represented by the pcl polygons
//remember: the vertices member of each pcl mesh polygon represents the vertices index in the original cloud
double MeshSurface::calcPCLMeshSurfaceArea(pcl::PolygonMesh::Ptr& mesh)
{
    int s = mesh->polygons.size();
    double totalArea=0.0;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2 (mesh->cloud, cloud);
    for (int i =0; i<s; i++){
        pcl::PointCloud<pcl::PointXYZ> temp_cloud;
        temp_cloud.points.push_back( cloud.points[ mesh->polygons[i].vertices[0] ]);
        temp_cloud.points.push_back( cloud.points[ mesh->polygons[i].vertices[1] ]);
        temp_cloud.points.push_back( cloud.points[ mesh->polygons[i].vertices[2] ]);
        totalArea += pcl::calculatePolygonArea(temp_cloud);
    }
//    std::cout<<"total area "<<totalArea<<"\n";
    return totalArea;
}

//OPTIONAL:if you want to load obj file instead of doing meshing(reconstruction) from points
void MeshSurface::loadOBJFile(const char* filename, std::vector<Vec3f>& points, Triangles& triangles)
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
            Triangle_3 tri;
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
                    Point_3 p1(points[atoi(data[0]) - 1][0],points[atoi(data[0]) - 1][1],points[atoi(data[0]) - 1][2]);
                    Point_3 p2(points[atoi(data[1]) - 1][0],points[atoi(data[1]) - 1][1],points[atoi(data[1]) - 1][2]);
                    Point_3 p3(points[atoi(data[2]) - 1][0],points[atoi(data[2]) - 1][1],points[atoi(data[2]) - 1][2]);
                    tri = Triangle_3(p1,p2,p3);
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
                    Point_3 p1(points[indxs[0]][0],points[indxs[0]][1],points[indxs[0]][2]);
                    Point_3 p2(points[indxs[1]][0],points[indxs[1]][1],points[indxs[1]][2]);
                    Point_3 p3(points[indxs[2]][0],points[indxs[2]][1],points[indxs[2]][2]);
                    tri = Triangle_3(p1,p2,p3);
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
