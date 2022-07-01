//#include <librealsense2/rs.hpp>
#include <iostream>
#include <AndreiUtils/utilsOpenCVRealsense.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>

//#include <pcl/filters/passthrough.h>
//#include <pcl/io/pcd_io.h>
#include <thread>
//#include <pcl/io/ply_io.h>
//#include <pcl/console/parse.h>
//#include <pcl/common/transforms.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/common/common_headers.h>
#include "pcl_helper.h"
#include <string>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/surface/gp3.h>
#include <chrono>
//#include <pcl/io/vtk_io.h>
//#include <pcl/io/obj_io.h>
//#include <pcl/search/kdtree.h> // for KdTree
//#include <pcl/surface/mls.h>
#include <thread>
#include "Scanner.h"
#include <GLFW/glfw3.h>
//#include <CGAL/IO/read_ply_points.h>
//#include <CGAL/Polyhedron_3.h>
#include "KdTree.h"
#include "Point.h"
int main2();

int main(int argc, char** argv) {
    
    //std::string model("../raptor-model/source/raptor.ply");
    
    
   std::string model("../elephant-model/elephant-skeleton/source/elephant/elephant.ply");

   // std::string model("../spartan_recap2.ply");
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);

    Scanner sc(1920,780,"PointCloud");
    sc.load<pcl::PointXYZRGB>("pc_captured10.pcd",cloud);


    sc.load<pcl::PointXYZRGB>("pc_captured10.pcd",cloud2);
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr = pcl_helpers::load_PLY(model, cloud);
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr2 = pcl_helpers::load_PLY(model2, cloud2);



// for(int i = 0 ;i < cloud->points.size(); i++)
//  {
//      cloud->points[i].r = 255;
//      cloud->points[i].g = 0;
//      cloud->points[i].b = 0;
//  }
//
//
//  //std::cout << "1ST DONE "<< std::endl << std::flush;
//  for(int i = 0 ;i < cloud2->points.size(); i++)
//  {
//      cloud2->points[i].r = 0;
//      cloud2->points[i].g = 0;
//      cloud2->points[i].b = 255;
//  }
//  //std::cout << "2ST DONE "<< std::endl << std::flush;
  // 
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr2 = pcl_helpers::load_PLY(model2, cloud2);
   // pcl::PolygonMesh poly;
    //sc.load_obj("mesh.obj");

//    sc.view(cloud);
//    sc.view(cloud2);
  auto res = sc.allign_ICP(cloud,cloud2);

  //pcl_helpers::poisson_reconstruction4<pcl::PointXYZRGB,pcl::PointXYZRGBNormal>(res,2,2,mesh); // 2 -> GreedyProjection triangulation, 2 -> mls works best
  //pcl_helpers::poisson_reconstruction4(res,2,2,mesh); // 2 -> GreedyProjection triangulation, 2 -> mls works best
  PolygonMesh mesh = sc.gp3Mls_reconstruction(res);
  // pcl_helpers::poisson_reconstruction5<pcl::PointXYZ>(cloud_ptr,mesh);
  //Mesh mesh_obj;
  //mesh_obj.loadOBJ("poisson_test.obj");
  //sc.load_obj("poisson_test.ply");

  pcl_helpers::view(mesh);

  //sc.load_obj("poisson_test.obj");
    //sc.view(res);
    

  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
    

 // if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("test_pcd.pcd", *cloud2) == -1) //* load the file
 // {
 //   PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
 //   return (-1);
 //}
  // sc.view(cloud2);
   Point p{0.121f, 0.03f, 0.f};
   Point p1{0.12f,0.02f,0.0f};
   Point p2{-0.12f, 0.0f, 0.0f};
   Point p3{-0.12f, -2.01f,0.0f};
   Point p4{ 0.12f,-2.01f, 0.0f};

//    auto frames = sc.capture_PcRGBData(true);
//   
//    std::string filename{"pc_captured"};
//
//   for(int i = 0 ; i< frames.size(); i++)
//    {
//        std::string filename_toUse = filename + std::to_string(i) + ".pcd";
//        std::cout << filename_toUse << std::endl;
//        sc.save<pcl::PointXYZRGB>(*frames[i],filename_toUse);
//    }
//
 // std::vector<Point> cloud{p,p1,p2,p3,p4};
 // KdTree tree(cloud);
   

   Point result {0.f,0.f,0.f};
   Point search_p {0.0f,2.0,0.0f};
 //  tree.search(search_p,result);


 //  std::cout << "x:" << result.x << " y:" << result.y << " z:" << result.z << std::endl;
    

 //  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_test = sc.captureOnce_PcRGBData(true);
 //  std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,pcl::PointCloud<pcl::PointXYZRGB>::Ptr> segmentation = pcl_helpers::ransac_impl<pcl::PointXYZRGB>(cloud2,400,0.5f);
 //  cloud2->points.clear();
 //   for(int i = 0; i < segmentation.first->points.size(); i++) {
 //      segmentation.first->points[i].r = 0;
 //      segmentation.first->points[i].g = 0;
 //      segmentation.first->points[i].b = 255;
 //      cloud2->points.push_back(segmentation.first->points[i]);
 //  }
 //  for(int i = 0; i < segmentation.second->points.size(); i++) {
 //      segmentation.second->points[i].r = 255;
 //      segmentation.second->points[i].g = 0;
 //      segmentation.second->points[i].b = 0;
 //      cloud2->points.push_back(segmentation.second->points[i]);
 //  }
 //  
    
    //sc.record_pointcloudFrames("record");

    // t.join();

    //pcl_helpers::fastTriangulation(cloud_ptr,normals);
    
   // sc.view(cloud2);
    
    return EXIT_SUCCESS;
}


