//#include <librealsense2/rs.hpp>
#include <iostream>
#include <AndreiUtils/utilsOpenCVRealsense.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <thread>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include "pcl_helper.h"
#include <string>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <chrono>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/search/kdtree.h> // for KdTree
#include <pcl/surface/mls.h>
#include <thread>
#include "Scanner.h"
#include <GLFW/glfw3.h>
//#include <CGAL/IO/read_ply_points.h>
//#include <CGAL/Polyhedron_3.h>
#include "KdTree.h"
#include "Point.h"
int main2();

int main(int argc, char** argv) {
    
    std::string model2("../raptor-model/source/raptor.ply");
    
    
    std::string model("../elephant-model/elephant-skeleton/source/elephant/elephant.ply");
  //  std::string model2("../spartan_recap2.ply");
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr = pcl_helpers::load_PLY(model, cloud);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr2 = pcl_helpers::load_PLY(model2, cloud2);
   // pcl::PolygonMesh poly;
    Scanner sc(1920,780,"PointCloud");
    //sc.load_obj("mesh.obj");

    auto res = sc.allign_ICP(cloud_ptr,cloud_ptr2);
    sc.view(res);
    

  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);


 // if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("test_pcd.pcd", *cloud2) == -1) //* load the file
 // {
 //   PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
 //   return (-1);
 //}
   //sc.load<pcl::PointXYZRGB>("test_pcd.pcd",cloud2);
  // sc.view(cloud2);
   Point p{0.121f, 0.03f, 0.f};
   Point p1{0.12f,0.02f,0.0f};
   Point p2{-0.12f, 0.0f, 0.0f};
   Point p3{-0.12f, -2.01f,0.0f};
   Point p4{ 0.12f,-2.01f, 0.0f};




 // std::vector<Point> cloud{p,p1,p2,p3,p4};
 // KdTree tree(cloud);
   

   Point result {0.f,0.f,0.f};
   Point search_p {0.0f,2.0,0.0f};
 //  tree.search(search_p,result);


 //  std::cout << "x:" << result.x << " y:" << result.y << " z:" << result.z << std::endl;
    

   //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_test = sc.captureOnce_PcRGBData(true);
  // std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,pcl::PointCloud<pcl::PointXYZRGB>::Ptr> segmentation = pcl_helpers::ransac_impl<pcl::PointXYZRGB>(cloud2,1000,0.5f);
  // cloud2->points.clear();
  //  for(int i = 0; i < segmentation.first->points.size(); i++) {
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
    
  //  sc.view<pcl::PointXYZRGB>(cloud2);
    
    return EXIT_SUCCESS;
}


