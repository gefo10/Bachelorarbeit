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
#include <cmath>
#include <thread>
#include <ScannerLib/Scanner.h>
#include <GLFW/glfw3.h>
//#include <CGAL/IO/read_ply_points.h>
//#include <CGAL/Polyhedron_3.h>
#include <ScannerLib/KdTree.h>
#include <ScannerLib/Point.h>
void testKdTree()
{

    std::vector<Point> cloud{

    Point {0.1,0.2,0.5},
    Point {0.25,0.2,0.5},
    Point {0.6,0.2,0.1},
    Point {0.51,0.142,0.1},
    Point {0.385,0.125,0.897},
    Point {0.635,0.654,0.5223},
    Point {0.7689,0.5654,0.5},
    Point {0.1,0.2,0.34432},
    Point {0.53253,0.889,0.123},
    Point {0.61,0.2101,0.09}
    };

    KdTree* tree = new KdTree(cloud);

    Point res;
    Point to_search{0.623,0.5,0.2};
    tree->search(to_search,res);

    std::cout << "x: " << res.x <<"  y:" << res.y << "  z:" << res.z << std::endl;
    delete tree;
}


void testMeshCreation()
{
 	// std::string model("../spartan_recap2.ply");
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

   Scanner sc("../config/defaultConfig.json");
   sc.load_ply<pcl::PointXYZRGB>("../elephant-model/elephant.ply",cloud);
   //sc.load_ply<pcl::PointXYZRGB>("../raptor-model/source/raptor.ply",cloud);


   sc.view(cloud);
   
   //auto poly1 = sc.gp3Normal_reconstruction(cloud);
   //sc.view(poly1);
   //auto poly2 = sc.gp3Mls_reconstruction(cloud);
   //sc.view(poly2);
   auto poly3 = sc.poissonNormal_reconstruction(cloud);
   sc.view(poly3);
   //auto poly4 = sc.poissonMls_reconstruction(cloud);
   //sc.view(poly4);
}

void testICP() 
{
 	// std::string model("../spartan_recap2.ply");
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);

   std::cout << std::fmod(-359.0,360.0) << std::endl;
   Scanner sc("../config/defaultConfig.json");
   sc.load_ply<pcl::PointXYZRGB>("../elephant-model/elephant.ply",cloud);

   sc.load_ply<pcl::PointXYZRGB>("../elephant-model/elephant.ply",cloud2);
   for(int i = 0; i < cloud2->points.size(); i++)
   {
       cloud2->points[i].r = 255;
       cloud2->points[i].g = 0;
       cloud2->points[i].b = 0;
   }
  for(int i = 0; i < cloud->points.size(); i++)
   {
       cloud->points[i].r = 0;
       cloud->points[i].g = 255;
       cloud->points[i].b = 0;
   }


   auto aligned_cloud = sc.align_ICP(cloud,cloud2);

   sc.view(aligned_cloud);

   // 
   // sc.load<pcl::PointXYZRGB>("rec2_plane.pcd",cloud);
   // sc.load<pcl::PointXYZRGB>("rec2_obj.pcd",cloud2);
 
}
void testRansac2() 
{
   //std::string model("../raptor-model/source/raptor.ply");


   //std::string model("../elephant-model/elephant-skeleton/source/elephant/elephant.ply");

   // std::string model("../spartan_recap2.ply");
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);

    Scanner sc("../config/defaultConfig.json");

    sc.load_pcd<pcl::PointXYZRGB>("../data/rec2_plane.pcd",cloud);
    //sc.load<pcl::PointXYZRGB>("../data/rec2_obj.pcd",cloud2);
     sc.view(cloud);
    //sc.view(cloud2);

//sc.view(cloud);

int t = std::log(1-0.99) / std::log(1-std::pow((1-0.3),3));
std::cout << "T = " << t << std::endl;
auto pair = pcl_helpers::RANSAC_SegmentPlane<PointXYZRGB>(cloud,10000,0.01f);
*cloud = *pair.second;

sc.view(cloud);

}


void testVectorResize() 
{
	std::vector<int> v {1,2,3,4,5,6,7,8,9,10};

	std::vector<int> indx {2,5,7,8}; //delete 3,6,8,9

	int count = 0;
	for(auto i : indx)
	{
		v.erase(v.begin() + i + count);
		count--;
	}

	for(auto val : v)
		std::cout << val << std::endl;
}
void testRansacPCL() 
{
   //std::string model("../raptor-model/source/raptor.ply");


   //std::string model("../elephant-model/elephant-skeleton/source/elephant/elephant.ply");

   // std::string model("../spartan_recap2.ply");
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);

    Scanner sc("../config/defaultConfig.json");

    sc.load_pcd<pcl::PointXYZRGB>("../data/rec2_plane.pcd",cloud);
    //sc.load<pcl::PointXYZRGB>("../data/rec2_obj.pcd",cloud2);
     sc.view(cloud);
    //sc.view(cloud2);

//sc.view(cloud);
pcl_helpers::ransac_pcl<pcl::PointXYZRGB>(cloud,0.001f);


sc.view(cloud);

}
void testPlaneDetection() 
{
   //std::string model("../raptor-model/source/raptor.ply");


   //std::string model("../elephant-model/elephant-skeleton/source/elephant/elephant.ply");

   // std::string model("../spartan_recap2.ply");
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);

    Scanner sc("../config/defaultConfig.json");

    sc.load_pcd<pcl::PointXYZRGB>("../data/plane_only0.pcd",cloud);
    sc.load_pcd<pcl::PointXYZRGB>("../data/plane_with_obj0.pcd",cloud2);
    sc.view(cloud2); 
    sc.view(cloud);
    //sc.view(cloud2);

//sc.view(cloud);
 sc.filterZ<PointXYZRGB>(cloud2,0.01f,1.0f);
 sc.filterZ<PointXYZRGB>(cloud,0.01f,1.0f);
 auto seg = pcl_helpers::plane_detection<pcl::PointXYZRGB>(cloud,0.1f,5000);

 auto plane = seg.first;
// sc.view(plane);
 //sc.filterX<PointXYZRGB>(cloud2,-0.5f,0.4f);
auto seg2 = pcl_helpers::plane_detection<pcl::PointXYZRGB>(cloud2,0.1f,5000);
sc.view(seg2.first);
auto result = seg2.first;
//sc.filterZ<PointXYZRGB>(result,0.94f,1.f);
 //sc.filterX<PointXYZRGB>(result,-0.15f,0.06f);
auto back = result;


seg2 = pcl_helpers::plane_detection<pcl::PointXYZRGB>(result,1e-2f,5000);
result = seg2.second;
sc.view(result);
pcl_helpers::statistical_removal(result,100,1e-4f);
sc.ransac_SVD<pcl::PointXYZRGB>(result,plane);
sc.view(result);
//auto aligned =sc.align_ICP(result,back);




//auto converted_result = sc.convert_pcl_points(result);
//KdTree* tree = new KdTree(converted_result);
//for(int i =0 ; i< cloud2->points.size(); i++)
//{
//
//    if (i % 1000 == 0 ) std::cout << i << "/" << cloud2->points.size() << std::endl;
//    Point r;
//    Point p {cloud2->points[i].x,cloud2->points[i].y,cloud2->points[i].z};
//
//
//    auto found = tree->search(p,r,0.0005f);
//    if(!found){
//        cloud2->points.erase(cloud2->points.begin() + i);
//        i--;
//    }
//}
//
//sc.view(aligned);
//delete tree;

}
void testRANSAC() 
{
    
    //std::string model("../raptor-model/source/raptor.ply");
    
    
   //std::string model("../elephant-model/elephant-skeleton/source/elephant/elephant.ply");

   // std::string model("../spartan_recap2.ply");
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);

    Scanner sc("../config/defaultConfig.json");
    
    sc.load_pcd<pcl::PointXYZRGB>("../data/rec2_plane.pcd",cloud);
    sc.load_pcd<pcl::PointXYZRGB>("../data/rec2_obj.pcd",cloud2);
     sc.view(cloud);
    sc.view(cloud2);
 
sc.view(cloud);
sc.ransac_SVD<pcl::PointXYZRGB>(cloud2,cloud);

sc.view(cloud2);


}


void showVisualizer()
{
       pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    Scanner sc("../config/defaultConfig.json");

   // sc.load_pcd<pcl::PointXYZRGB>("../data/test_ransac_obj2.pcd",cloud);
   sc.load_ply<pcl::PointXYZRGB>("../elephant-model/elephant.ply",cloud);
     sc.view(cloud);

}

void saveData()
{
    
    Scanner sc("../config/defaultConfig.json");

    std::string name{""};
    auto frame_0degree = sc.capture_FrameXYZRGB(true);
    sc.save_pcd<PointXYZRGB>(name + "_0degree.pcd",frame_0degree);
   
    auto frame_45degree = sc.capture_FrameXYZRGB(true);
    sc.save_pcd<PointXYZRGB>(name + "_45degree.pcd",frame_0degree);
    
    auto frame_90degree = sc.capture_FrameXYZRGB(true);
    sc.save_pcd<PointXYZRGB>(name + "_90degree.pcd",frame_0degree);
    
    auto frame_135degree = sc.capture_FrameXYZRGB(true);
    sc.save_pcd<PointXYZRGB>(name + "_135degree.pcd",frame_0degree);
    
    auto frame_180degree = sc.capture_FrameXYZRGB(true);
    sc.save_pcd<PointXYZRGB>(name + "_180degree.pcd",frame_0degree);

    auto frame_225degree = sc.capture_FrameXYZRGB(true);
    sc.save_pcd<PointXYZRGB>(name + "_225degree.pcd",frame_0degree);
    
    auto frame_270degree = sc.capture_FrameXYZRGB(true);
    sc.save_pcd<PointXYZRGB>(name + "_270degree.pcd",frame_0degree);
    
    auto frame_315degree = sc.capture_FrameXYZRGB(true);
    sc.save_pcd<PointXYZRGB>(name + "_315degree.pcd",frame_0degree);

    
    auto frame_360degree = sc.capture_FrameXYZRGB(true);
    sc.save_pcd<PointXYZRGB>(name + "_360degree.pcd",frame_0degree);





}
int main(int argc, char** argv) {
   	//testRANSAC();
    //testKdTree();
    //testRansac2();
    //testICP();
    showVisualizer();
    //testMeshCreation();
   // testRansacPCL();
	//testVectorResize();
    //testPlaneDetection();
	return EXIT_SUCCESS;
}


