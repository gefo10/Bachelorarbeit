#include <iostream>
//#include <Eigen/Dense>
//#include <pcl/point_types.h>

#include <pcl/filters/passthrough.h>
//#include <pcl/io/pcd_io.h>
//#include <thread>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
//#include <pcl/common/transforms.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include "pcl_helper.h"
#include <string>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/surface/gp3.h>
//#include <chrono>
#include <pcl/io/vtk_io.h>
//#include <pcl/io/obj_io.h>
#include <pcl/search/kdtree.h> // for KdTree
#include <pcl/surface/mls.h>
#include <pcl/console/time.h>
#include <pcl/console/print.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
//#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/principal_curvatures.h>
#include <vector>
#include <fstream>
//#include <stdlib.h>
#include <stdio.h>
//#include <unordered_set>
//#include <boost/thread/thread.hpp>
//#include <pcl/surface/gp3.h>
//#include <pcl/surface/poisson.h>

void pcl_helpers::testPrint()
{
    std::cout << "Hello World" << std::endl;
}

Cloud_simplePtr pcl_helpers::load_PLY(const std::string &file_name,Cloud_simplePtr cloud_ptr)
{
   if(cloud_ptr == nullptr){
       throw std::invalid_argument("No cloud ptr suplied!");
   }

   if(pcl::io::loadPLYFile<pcl::PointXYZ> (file_name, *cloud_ptr) == -1)
   {
     PCL_ERROR ("Could't read file\n");
     std::exit(EXIT_FAILURE);
   }

   return cloud_ptr;
}

Cloud_rgbPtr pcl_helpers::load_PLY(const std::string &file_name, Cloud_rgbPtr cloud_ptr)
{

    if(cloud_ptr == nullptr)
    {
        throw std::invalid_argument("No cloud ptr suplied!");
    }

    if(pcl::io::loadPLYFile<pcl::PointXYZRGB> (file_name, *cloud_ptr) == -1)
    {
      PCL_ERROR ("Could't read file\n");
      std::exit(EXIT_FAILURE);
    }

    return cloud_ptr;
}

pcl::PointCloud<pcl::Normal>::Ptr pcl_helpers::estimateNormals(Cloud_simplePtr& cloud)
{
    //create normal estimation class and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
    ne.setInputCloud(cloud);

    //create an empty Kdtree representation and pass it to the normal estimation object
    //its content will be filled inside the object, based on the given input dataset
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod(tree);

    //Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    //Use all neighbors in a sphere of radius 3cm)
    ne.setRadiusSearch(0.03);

    //compute the features
    ne.compute(*cloud_normals);

    //cloud_normals->size() should have the same size as cloud->size()
    return cloud_normals;
}


pcl::PointCloud<pcl::Normal>::Ptr pcl_helpers::estimateNormals(Cloud_rgbPtr& cloud)
{
 //create normal estimation class and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZRGB,pcl::Normal> ne;
    ne.setInputCloud(cloud);

    //create an empty Kdtree representation and pass it to the normal estimation object
    //its content will be filled inside the object, based on the given input dataset
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod(tree);

    //Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    //Use all neighbors in a sphere of radius 3cm)
    ne.setRadiusSearch(0.03);

    //compute the features
    ne.compute(*cloud_normals);

    //cloud_normals->size() should have the same size as cloud->size()
    return cloud_normals;

}


void pcl_helpers::filterX(Cloud_simplePtr& cloud,float min, float max)
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(min,max);
    pass.filter(*cloud);
}


void pcl_helpers::filterX(Cloud_rgbPtr& cloud,float min, float max)
{
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(min,max);
    pass.filter(*cloud);
}



void pcl_helpers::filterY(Cloud_simplePtr& cloud,float min, float max)
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(min,max);
    pass.filter(*cloud);
}


void pcl_helpers::filterY(Cloud_rgbPtr& cloud,float min, float max)
{
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(min,max);
    pass.filter(*cloud);
}



void pcl_helpers::filterZ(Cloud_simplePtr& cloud,float min, float max)
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min,max);
    pass.filter(*cloud);
}


void pcl_helpers::filterZ(Cloud_rgbPtr& cloud,float min, float max)
{
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min,max);
    pass.filter(*cloud);
}

void pcl_helpers::fastTriangulation(Cloud_simplePtr& cloud,pcl::PolygonMesh& mesh)
{
// Load input file into a PointCloud<T> with an appropriate type
  //* the data should be available in cloud

  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (10);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
 
  pcl::PointCloud<pcl::PointNormal>::Ptr  cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.05);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (10);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(true);


  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (mesh);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();

  //return mesh;
}


void pcl_helpers::fastTriangulation(const std::string &file_name,pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals)
{
// Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPLYFile (file_name, cloud_blob);
  pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
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
 // if(!cloud_with_normals)
    //cloud_with_normals =new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.015);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();

  pcl::io::saveOBJFile("mesh.obj",triangles);
}

void pcl_helpers::transform(Cloud_rgbPtr& cloud, float x, float y, float z,float rotation)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());


  float theta = rotation; // The angle of rotation in radians

  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

  transform_2.translation() << x, y, z;

  // The same rotation matrix as before; theta radians around Z axis
  transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

  // Print the transformation
  printf ("\nTransorfmation: using an Affine3f\n");
  std::cout << transform_2.matrix() << std::endl;

  // Executing the transformation
  pcl::transformPointCloud (*cloud, *transformed_cloud, transform_2);  
}

 void pcl_helpers::transform(Cloud_simplePtr& cloud, float x, float y, float z,float rotation)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());


  float theta = rotation; // The angle of rotation in radians

  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

  transform_2.translation() << x, y, z;

  // The same rotation matrix as before; theta radians around Z axis
  transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

  // Print the transformation
  printf ("\nTransorfmation: using an Affine3f\n");
  std::cout << transform_2.matrix() << std::endl;

  // Executing the transformation
  pcl::transformPointCloud (*cloud, *transformed_cloud, transform_2);  
}



void pcl_helpers::view(Cloud_simplePtr cloud_ptr,float red, float green, float blue, float alpha)
{
    pcl::visualization::PCLVisualizer viewer;
    Color background(red,green,blue,alpha);
    // Set background to a dark grey
    viewer.setBackgroundColor(background.x(),background.y(),background.z(),background.w());

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"View");


    viewer.addPointCloud(cloud_ptr,"view");
    //viewer.addCoordinateSystem(1.0);
   // viewer.initCameraParameters();

    while(!viewer.wasStopped())
    {
      viewer.spinOnce(100);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

}

void pcl_helpers::view(Cloud_rgbPtr cloud_ptr,float red, float green, float blue, float alpha)
{
    pcl::visualization::PCLVisualizer viewer;
    Color background(red,green,blue,alpha);
    // Set background to a dark grey
    viewer.setBackgroundColor(background.x(),background.y(),background.z(),background.w());

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"View");

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_ptr);


    viewer.addPointCloud(cloud_ptr,rgb,"view");
    //viewer.addCoordinateSystem(1.0);
    //viewer.initCameraParameters();
    while(!viewer.wasStopped())
    {
      viewer.spinOnce(100);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

}




void pcl_helpers::view(pcl::PolygonMesh poly,float red, float green, float blue, float alpha)
{
    pcl::visualization::PCLVisualizer viewer;
    Color background(red,green,blue,alpha);
    // Set background to a dark grey
    viewer.setBackgroundColor(background.x(),background.y(),background.z(),background.w());

    viewer.addPolygonMesh(poly,"view",0);
    //viewer.addCoordinateSystem(1.0);
    //viewer.initCameraParameters();

    while(!viewer.wasStopped())
    {
      viewer.spinOnce(100);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

}

void pcl_helpers::view(const std::string& filename,float red, float green, float blue, float alpha)
{
    pcl::PolygonMesh poly;
    pcl::io::loadPLYFile(filename,poly);
    pcl::visualization::PCLVisualizer viewer;
    Color background(red,green,blue,alpha);
    // Set background to a dark grey
    viewer.setBackgroundColor(background.x(),background.y(),background.z(),background.w());

    viewer.addPolygonMesh(poly,"view",0);
    //viewer.addCoordinateSystem(1.0);
    //viewer.initCameraParameters();

    while(!viewer.wasStopped())
    {
      viewer.spinOnce(100);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

}

pcl::PointCloud<pcl::PointNormal>::Ptr pcl_helpers::smooth_mls(Cloud_simplePtr& cloud) 
{
    // Load input file into a Pointhresholdoud<T> with an appropriate type
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  // Load bun0.pcd -- should be available with the PCL archive in test
  // pcl::io::loadPCDFile ("bun0.pcd", *cloud);

  // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

  mls.setComputeNormals (true);

  // Set parameters
  mls.setInputCloud (cloud);
  mls.setPolynomialOrder (2);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);  
  //mls.setInputNormals(normals);
  // Reconstruct
  mls.process (*mls_points);

  // Save output
  //pcl::io::savePCDFile ("bun0-mls.pcd", mls_points);
  return mls_points;
}

void pcl_helpers::poisson_reconstruction(pcl::PointCloud<pcl::PointNormal>::Ptr& input,pcl::PolygonMesh& output,int depth,int solver_divide,int iso_divide,int point_weight)
{
    using namespace pcl;
    using namespace pcl::io;
    using namespace pcl::console;

    //PointCloud<PointNormal>::Ptr xyz_cloud (new pcl::PointCloud<PointNormal> ());
    //fromPCLPointCloud2 (*input, *xyz_cloud);

  //print_info ("Using parameters: depth %d, solverDivide %d, isoDivide %d\n", depth, solver_divide, iso_divide);

    Poisson<PointNormal> poisson;
	poisson.setDepth (depth);
	poisson.setSolverDivide (solver_divide);
	poisson.setIsoDivide (iso_divide);
    poisson.setPointWeight (point_weight);
    poisson.setInputCloud (input);

    TicToc tt;
    tt.tic ();
    //print_highlight ("Computing ...");
    poisson.reconstruct(output);

    print_info("[Done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");
}

void pcl_helpers::statistical_removal(Cloud_rgbPtr& input,int meanK,float threshold)
{
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    //pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    //reader.read<pcl::PointXYZ> ("table_scene_lms400.pcd", *cloud);

    //std::cerr << "Cloud before filtering: " << std::endl;
    //std::cerr << *cloud << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (input);
    sor.setMeanK (meanK);
    sor.setStddevMulThresh (threshold);
    sor.filter (*input);

    //std::cerr << "Cloud after filtering: " << std::endl;
    //std::cerr << *cloud_filtered << std::endl;

    //pcl::PCDWriter writer;
    //writer.write<pcl::PointXYZ> ("table_scene_lms400_inliers.pcd", *cloud_filtered, false);

    //sor.setNegative (true);
    //sor.filter (*cloud_filtered);
    //writer.write<pcl::PointXYZ> ("table_scene_lms400_outliers.pcd", *cloud_filtered, false);
}


void pcl_helpers::statistical_removal(Cloud_simplePtr& input,int meanK, float threshold)
{
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    //pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    //reader.read<pcl::PointXYZ> ("table_scene_lms400.pcd", *cloud);

    //std::cerr << "Cloud before filtering: " << std::endl;
    //std::cerr << *cloud << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (input);
    sor.setMeanK (meanK);
    sor.setStddevMulThresh (threshold);
    sor.filter (*input);

    //std::cerr << "Cloud after filtering: " << std::endl;
    //std::cerr << *cloud_filtered << std::endl;

    //pcl::PCDWriter writer;
    //writer.write<pcl::PointXYZ> ("table_scene_lms400_inliers.pcd", *cloud_filtered, false);

    //sor.setNegative (true);
    //sor.filter (*cloud_filtered);
    //writer.write<pcl::PointXYZ> ("table_scene_lms400_outliers.pcd", *cloud_filtered, false);
}


void pcl_helpers::downsample(Cloud_simplePtr& input)
{
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::toPCLPointCloud2(*input,*cloud);
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter(*cloud);
  pcl::fromPCLPointCloud2(*cloud,*input); 

}



void pcl_helpers::poisson_gp3_reconstruction(std::vector<Point> cloud, int surface_mode,int normal_method,pcl::PolygonMesh& mesh)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

    for(int i = 0 ; i< cloud.size(); i++)
    {
        pcl::PointXYZRGB p;
        p.x = cloud[i].x;
        p.y = cloud[i].y;
        p.z = cloud[i].z;
        p.r = 255*cloud[i].r;
        p.g = 255*cloud[i].g;
        p.b = 255*cloud[i].b;
        
        cloud_ptr->points.push_back(p);
    }
 /* ****Translated point cloud to origin**** */
     Eigen::Vector4f centroid;
     pcl::compute3DCentroid(*cloud_ptr, centroid);

     Eigen::Affine3f transform = Eigen::Affine3f::Identity();
     transform.translation() << -centroid[0], -centroid[1], -centroid[2];

     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudTranslated(new pcl::PointCloud<pcl::PointXYZRGB>());
     pcl::transformPointCloud(*cloud_ptr, *cloudTranslated, transform);

     /* ****kdtree search and msl object**** */
     pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree_for_points (new pcl::search::KdTree<pcl::PointXYZRGB>);
     kdtree_for_points->setInputCloud(cloudTranslated);
     pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());

     bool mls_mode = false;
     bool normal_mode = false;

     if(normal_method == 1){
        normal_mode = true;
     }else if(normal_method == 2){
        mls_mode = true;
     }else{
        std::cout << "Select:\n '1' for normal estimation \n '2' for mls normal estimation " << std::endl;
        std::exit(-1);
     }

     bool gp3_mode = false;
     bool poisson_mode = false;

     if(surface_mode == 1){
        poisson_mode = true;
     }else if(surface_mode == 2){
        gp3_mode = true;
     }else{
        std::cout << "Select: \n'1' for surface poisson method \n '2' for surface gp3 method " << std::endl;
        std::exit(-1);
     }

     if(mls_mode){

       std::cout << "Using mls method estimation...";

       pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr mls_points (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
       pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> mls;
       //Set parameters
       mls.setComputeNormals(true);
       mls.setInputCloud(cloudTranslated);
      // mls.setDilationIterations(10);
       //mls.setDilationVoxelSize(0.5);
       //mls.setSqrGaussParam(2.0);
       //mls.setUpsamplingRadius(5);
       //mls.setPolynomialOrder (2);
       //mls.setPointDensity(30);
       mls.setSearchMethod(kdtree_for_points);
       mls.setSearchRadius(0.03);
       mls.process(*mls_points);

       pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>());

       for(int i = 0; i < mls_points->points.size(); i++) {

              pcl::PointXYZRGB pt;
              pt = cloud_ptr->points[i];

              temp->points.push_back(pt);
        }


       pcl::concatenateFields (*temp, *mls_points, *cloud_with_normals);
       std::cout << "[OK]" << std::endl;

     }else if(normal_mode){

       std::cout << "Using normal method estimation...";

       pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> n;
       pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

       n.setInputCloud(cloudTranslated);
       n.setSearchMethod(kdtree_for_points);
       n.setKSearch(20); //It was 20
       n.compute(*normals);//Normals are estimated using standard method.

       //pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal> ());
       pcl::concatenateFields(*cloud_ptr, *normals, *cloud_with_normals);

       std::cout << "[OK]" << std::endl;

     }else{
        std::cout << "Select: '1' for normal method estimation \n '2' for mls normal estimation " << std::endl;
        std::exit(-1);
     }

     // Create search tree*
     pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr kdtree_for_normals (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
     kdtree_for_normals->setInputCloud(cloud_with_normals);

     std::cout << "Applying surface meshing...";

     if(gp3_mode){

       std::cout << "Using surface method: gp3 ..." << std::endl;

       int searchK = 100; //was 100
       int search_radius = 10; //was 10
       int setMU = 5; //was 5
       int maxiNearestNeighbors = 100; //was 100
       bool normalConsistency = false;

       pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;

       gp3.setSearchRadius(search_radius);//It was 0.025
       gp3.setMu(setMU); //It was 2.5
       gp3.setMaximumNearestNeighbors(maxiNearestNeighbors);    //It was 100
       //gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees    //it was 4
       //gp3.setMinimumAngle(M_PI/18); // 10 degrees //It was 18
       //gp3.setMaximumAngle(M_PI/1.5); // 120 degrees        //it was 1.5
       gp3.setNormalConsistency(normalConsistency); //It was false
       gp3.setInputCloud(cloud_with_normals);
       gp3.setSearchMethod(kdtree_for_normals);
       gp3.reconstruct(mesh);

       //vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
       //pcl::PolygonMesh mesh_pcl;
       //pcl::VTKUtils::convertToVTK(triangles,polydata);
       //pcl::VTKUtils::convertToPCL(polydata,mesh_pcl);

       //pcl::io::savePolygonFilePLY("mesh.ply", mesh_pcl);

       std::cout << "[OK]" << std::endl;

     }else if(poisson_mode){

        std::cout << "Using surface method: poisson ..." << std::endl;

        int nThreads=8;
        int setKsearch=10;
        int depth=7;
        float pointWeight=2.0;
        float samplePNode=1.5;
        float scale=1.1;
        int isoDivide=8;
        bool confidence=true;
        bool outputPolygons=true;
        bool manifold=true;
        int solverDivide=8;

        pcl::Poisson<pcl::PointXYZRGBNormal> poisson;

        poisson.setDepth(depth);//9
        poisson.setInputCloud(cloud_with_normals);
        poisson.setPointWeight(pointWeight);//4
        poisson.setDegree(2);
        poisson.setSamplesPerNode(samplePNode);//1.5
        poisson.setScale(scale);//1.1
        poisson.setIsoDivide(isoDivide);//8
        poisson.setConfidence(confidence);
        poisson.setOutputPolygons(outputPolygons);
        poisson.setManifold(manifold);
        poisson.setSolverDivide(solverDivide);//8
        poisson.reconstruct(mesh);

        //pcl::PolygonMesh mesh2;
        //poisson.reconstruct(mesh2);
        //pcl::surface::SimplificationRemoveUnusedVertices rem;
        //rem.simplify(mesh2,triangles);

        std::cout << "[OK]" << std::endl;

     }else{
        std::cout << "Select: \n'1' for surface poisson method \n '2' for surface gp3 method " << std::endl;
        std::exit(-1);
     }
     pcl::io::saveOBJFile("poisson_test.obj",mesh);

}


Cloud_simple::Ptr pcl_helpers::pc_toPCL(const rs2::points& points)
{
    Cloud_simple::Ptr cloud(new Cloud_simple);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    return cloud;
}  

std::tuple<int,int,int> RGB_Texture(rs2::video_frame texture,rs2::texture_coordinate Texture_XY)
{
    using namespace std;
    //Get width and height coordinates of texture
    int width = texture.get_width();
    int height = texture.get_height();

    //Normals to texture coordinates conversion
    int x_value = min(max(int(Texture_XY.u * width + .5f),0),width-1);
    int y_value = min(max(int(Texture_XY.v * height + .5f),0),height-1);
    int bytes = x_value * texture.get_bytes_per_pixel(); //get # of bytes per pixel
    int strides = y_value * texture.get_stride_in_bytes(); //get line width in bytes
    int text_index = (bytes+strides);

    const auto new_texture = reinterpret_cast<const uint8_t*> (texture.get_data());

    //RGB components to save in a tuple
    int NT1 = new_texture[text_index];
    int NT2 = new_texture[text_index+1];
    int NT3 = new_texture[text_index+2];

    return std::tuple<int,int,int> (NT1,NT2,NT3);
}


Cloud_rgb::Ptr pcl_helpers::pcRGB_toPCL(const rs2::points& points, const rs2::video_frame& color)
{
    Cloud_rgb::Ptr cloud(new Cloud_rgb);
    

    //declare tuple for RGB value storage (<t0>, <t1>, <t2>)
    std::tuple<uint8_t,uint8_t,uint8_t> RGB_color;


    //================================
    // PCL Cloud Object Configuration
    // =============================

    //Convert data captured from realsense to point cloud
    auto sp = points.get_profile().as<rs2::video_stream_profile>();


    cloud->width = static_cast<uint32_t>( sp.width() );
    cloud->height = static_cast<uint32_t>( sp.height() );
    cloud-> is_dense = false;
    cloud-> points.resize( points.size() );

    auto text_coord = points.get_texture_coordinates();
    auto vertex = points.get_vertices();

    //Iterate through all points and set XYZ coord
    //and RGB values

    for (int i = 0; i < points.size(); i++)
    {
        //=============================
        //mapping depth coord
        //-depth data stored as XYZ cood
        //==============================
        cloud->points[i].x = vertex[i].x;
        cloud->points[i].y = vertex[i].y;
        cloud->points[i].z = vertex[i].z;


        //Obtain color textures for specific point 
        RGB_color = RGB_Texture(color, text_coord[i]);


        //Map color

        cloud->points[i].r = std::get<0>(RGB_color); //reference tuple<2>
        cloud->points[i].g = std::get<1>(RGB_color); //reference tuple<1>
        cloud->points[i].b = std::get<2>(RGB_color); //refence tuple<0>
    }

    return cloud;
    


}

//pcl::PolygonMesh pcl_helpers::gp3Normal_reconstruction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
//{
//    using namespace pcl;
//    PolygonMesh mesh;
//    poisson_gp3_reconstruction<PointXYZRGB,PointXYZRGBNormal>(cloud,2,1,mesh);
//    return mesh;
//}
//pcl::PolygonMesh pcl_helpers::gp3Normal_reconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
//{
//    using namespace pcl;
//    PolygonMesh mesh;
//    poisson_gp3_reconstruction<PointXYZ,PointXYZNormal>(cloud,2,1,mesh);
//    return mesh;
//
//}
//pcl::PolygonMesh pcl_helpers::gp3Normal_reconstruction(std::vector<Point> cloud)
//{
//    using namespace pcl;
//    PolygonMesh mesh;
//    poisson_gp3_reconstruction(cloud,2,1,mesh);
//    return mesh;
//
//}
//pcl::PolygonMesh pcl_helpers::gp3Mls_reconstruction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
//{
//    using namespace pcl;
//    PolygonMesh mesh;
//    poisson_gp3_reconstruction<PointXYZRGB,PointXYZRGBNormal>(cloud,2,2,mesh);
//    return mesh;
//
//}
//pcl::PolygonMesh pcl_helpers::gp3Mls_reconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
//{
//    using namespace pcl;
//    PolygonMesh mesh;
//    poisson_gp3_reconstruction<PointXYZ,PointXYZNormal>(cloud,2,2,mesh);
//    return mesh;
//
//}
//pcl::PolygonMesh pcl_helpers::gp3Mls_reconstruction(std::vector<Point> cloud)
//{
//    using namespace pcl;
//    PolygonMesh mesh;
//    poisson_gp3_reconstruction(cloud,2,2,mesh);
//    return mesh;
//
//}
//pcl::PolygonMesh pcl_helpers::poissonNormal_reconstruction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
//{
//    using namespace pcl;
//    PolygonMesh mesh;
//    poisson_gp3_reconstruction<PointXYZRGB,PointXYZRGBNormal>(cloud,1,1,mesh);
//    return mesh;
//
//}
//pcl::PolygonMesh pcl_helpers::poissonNormal_reconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
//{
//    using namespace pcl;
//    PolygonMesh mesh;
//    poisson_gp3_reconstruction<PointXYZ,PointXYZNormal>(cloud,1,1,mesh);
//    return mesh;
//
//}
//pcl::PolygonMesh pcl_helpers::poissonNormal_reconstruction(std::vector<Point> cloud)
//{
//    using namespace pcl;
//    PolygonMesh mesh;
//    poisson_gp3_reconstruction(cloud,1,1,mesh);
//    return mesh;
//
//}
//pcl::PolygonMesh pcl_helpers::poissonMls_reconstruction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
//{
//    using namespace pcl;
//    PolygonMesh mesh;
//    poisson_gp3_reconstruction<PointXYZRGB,PointXYZRGBNormal>(cloud,1,2,mesh);
//    return mesh;
//
//}
//
//pcl::PolygonMesh pcl_helpers::poissonMls_reconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
//{
//    using namespace pcl;
//    PolygonMesh mesh;
//    poisson_gp3_reconstruction<PointXYZ,PointXYZNormal>(cloud,1,2,mesh);
//    return mesh;
//
//}
//
//pcl::PolygonMesh pcl_helpers::poissonMls_reconstruction(std::vector<Point> cloud)
//{
//    using namespace pcl;
//    PolygonMesh mesh;
//    poisson_gp3_reconstruction(cloud,1,2,mesh);
//    return mesh;
//
//}


//template <typename PointT>
//std::pair<typename pcl::PointCloud<PointT>::Ptr,typename pcl::PointCloud<PointT>::Ptr> pcl_helpers::ransac_impl(typename pcl::PointCloud<PointT>::Ptr& cloud,int maxIterations,float distanceThreshold)
//{
//
//    std::unordered_set<int> inliersResult;
//    srand(time(NULL));
//
//    while(maxIterations--)
//    {
//        std::unordered_set<int> inliers;
//        while (inliers.size() < 3)
//            inliers.insert( rand() % cloud->points.size() );
//
//        float x1,y1,z1,x2,y2,z2,x3,y3,z3;
//        auto indx_ptr = inliers.begin();
//
//        x1 = cloud->points[*indx_ptr].x;
//        y1 = cloud->points[*indx_ptr].y;
//        z1 = cloud->points[*indx_ptr].z;
//
//        ++indx_ptr;
//       
//        x2 = cloud->points[*indx_ptr].x;
//        y2 = cloud->points[*indx_ptr].y;
//        z2 = cloud->points[*indx_ptr].z;
//     
//        ++indx_ptr;
//       
//        x3 = cloud->points[*indx_ptr].x;
//        y3 = cloud->points[*indx_ptr].y;
//        z3 = cloud->points[*indx_ptr].z;
//
//        float a = (y2 - y1) * (z3-z1) - (z2-z1) * (y3-y1);
//        float b = (z2-z1) * (x3-x1) - (x2-x1) * (z3-z1);
//        float c = (x2-x1) * (y3-y1) - (y2-y1) * (x3-x1);
//        float d = - (a * x1 + b * y1 + c * z1);
//
//        for(int i = 0; i < cloud->points.size(); i++) {
//            if (inliers.count(i) > 0)
//                continue;
//
//            PointT point = cloud->points[i];
//            float x4 = point.x;
//            float y4 = point.y;
//            float z4 = point.z;
//
//            float dis = fabs(a*x4 + b*y4 + c*z4 + d)/sqrt(a*a + b*b + c*c);
//
//            if (dis <= distanceThreshold)
//                inliers.insert(i);
//        }
//        
//        if (inliers.size() > inliersResult.size())
//            inliersResult = inliers;
//        
//    }
//
//    typename pcl::PointCloud<PointT>::Ptr cloudInliers (new pcl::PointCloud<PointT>());
//    typename pcl::PointCloud<PointT>::Ptr cloudOutliers (new pcl::PointCloud<PointT>());
//
//    for(int i = 0 ; i < cloud->points.size(); i++){
//
//        PointT point = cloud->points[i];
//        if (inliersResult.count(i))
//            cloudInliers->points.push_back( point );
//        else
//            cloudOutliers->points.push_back( point );
//    }
//
//    std::pair<typename pcl::PointCloud<PointT>::Ptr,typename pcl::PointCloud<PointT>::Ptr> segResult = std::make_pair(cloudInliers,cloudOutliers);
//
//    return segResult;
//
//
//}
