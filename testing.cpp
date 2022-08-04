//#include <librealsense2/rs.hpp>
#include <iostream>
#include <Eigen/Dense>
#include <pcl/point_types.h>

//#include <pcl/filters/passthrough.h>
//#include <pcl/io/pcd_io.h>
#include <thread>
//#include <pcl/io/ply_io.h>
//#include <pcl/console/parse.h>
//#include <pcl/common/transforms.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/common/common_headers.h>
#include <ScannerLib/pcl_helper.h>
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
#include <ScannerLib/Scanner.h>
#include <GLFW/glfw3.h>
//#include <CGAL/IO/read_ply_points.h>
//#include <CGAL/Polyhedron_3.h>
#include <ScannerLib/KdTree.h>
#include <ScannerLib/Point.h>



//void testICP() 
//{
// 	// std::string model("../spartan_recap2.ply");
//   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
//
//    Scanner sc;
//    
//    sc.load<pcl::PointXYZRGB>("rec2_plane.pcd",cloud);
//    sc.load<pcl::PointXYZRGB>("rec2_obj.pcd",cloud2);
// 
//}
void testRANSAC() 
{
    
    //std::string model("../raptor-model/source/raptor.ply");
    
    
   //std::string model("../elephant-model/elephant-skeleton/source/elephant/elephant.ply");

   // std::string model("../spartan_recap2.ply");
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);

    Scanner sc("../config/defaultConfig.json");
    
    sc.load<pcl::PointXYZRGB>("../data/rec2_plane.pcd",cloud);
    sc.load<pcl::PointXYZRGB>("../data/rec2_obj.pcd",cloud2);
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

    sc.view(cloud);
    sc.view(cloud2);
 
sc.view(cloud);
sc.ransac_SVD<pcl::PointXYZRGB>(cloud2,cloud);

sc.view(cloud2);


}

int main(int argc, char** argv) {
   	testRANSAC();
	return EXIT_SUCCESS;
}


