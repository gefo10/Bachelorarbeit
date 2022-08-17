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

    //sc.load_pcd<pcl::PointXYZRGB>("../test_plane/plane.pcd",cloud);
    sc.load_pcd<pcl::PointXYZRGB>("../data/plane_only0.pcd",cloud);
    sc.load_pcd<pcl::PointXYZRGB>("../data/plane_with_obj0.pcd",cloud2);
    //sc.load_pcd<pcl::PointXYZRGB>("../test1/box_0degree.pcd",cloud2);

sc.filterZ<PointXYZRGB>(cloud2,0.01f,1.5f);
 sc.filterZ<PointXYZRGB>(cloud,0.01f,1.5f);


   sc.view(cloud);
  sc.view(cloud2); 
auto seg = pcl_helpers::plane_detection<pcl::PointXYZRGB>(cloud,0.1f,5000);
 auto plane = seg.first;
//pcl_helpers::statistical_removal(plane,100,0.1f);
sc.view(plane);

 //sc.filterX<PointXYZRGB>(cloud2,-0.5f,0.4f);
auto seg2 = pcl_helpers::plane_detection<pcl::PointXYZRGB>(cloud2,0.1f,5000);
auto result = seg2.first;
sc.view(result);
//sc.filterZ<PointXYZRGB>(result,0.94f,1.f);
 //sc.filterX<PointXYZRGB>(result,-0.15f,0.06f);
auto back = result;


//seg2 = pcl_helpers::plane_detection<pcl::PointXYZRGB>(result,0f,5000);
//result = seg2.first;
//sc.view(result);

sc.ransac_SVD<pcl::PointXYZRGB>(result,plane);
sc.view(result);
pcl_helpers::statistical_removal(result,100,0.01f);
sc.view(result);

//sc.save_pcd<PointXYZRGB>("chicken",result);

}

void testPlaneDetection2() 
{
   //std::string model("../raptor-model/source/raptor.ply");


   //std::string model("../elephant-model/elephant-skeleton/source/elephant/elephant.ply");

   // std::string model("../spartan_recap2.ply");
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);

    Scanner sc("../config/defaultConfig.json");

    sc.load_pcd<pcl::PointXYZRGB>("../data/plane_only0.pcd",cloud);
    sc.load_pcd<pcl::PointXYZRGB>("../data/plane_with_obj0.pcd",cloud2);
    //sc.load_pcd<pcl::PointXYZRGB>("../test_plane/plane.pcd",cloud);
    //sc.load_pcd<pcl::PointXYZRGB>("../test_plane/plane.pcd",cloud);
   //sc.load_pcd<pcl::PointXYZRGB>("../test2/chicken_0degree.pcd",cloud2);
    //sc.load_pcd<pcl::PointXYZRGB>("../test1/box_360degree.pcd",cloud2);

//sc.filterZ<PointXYZRGB>(cloud2,0.01f,2.f);
// sc.filterZ<PointXYZRGB>(cloud,0.01f,2.f);


  // sc.view(cloud);
  //sc.view(cloud2); 
//auto seg = pcl_helpers::plane_detection<pcl::PointXYZRGB>(cloud,0.1f,5000);
// auto plane = seg.first;
//pcl_helpers::statistical_removal(plane,100,0.1f);
//sc.view(plane);

 //sc.filterX<PointXYZRGB>(cloud2,-0.5f,0.4f);

//auto seg2 = pcl_helpers::plane_detection<pcl::PointXYZRGB>(cloud2,0.1f,5000);
//auto result = seg2.first;
//auto result = cloud2;
//sc.view(result);
//sc.filterZ<PointXYZRGB>(result,0.94f,1.f);
 //sc.filterX<PointXYZRGB>(result,-0.15f,0.06f);
//auto back = result;


//seg2 = pcl_helpers::plane_detection<pcl::PointXYZRGB>(result,0f,5000);
//result = seg2.first;
//sc.view(result);

sc.ransac_SVD<pcl::PointXYZRGB>(cloud2,cloud);
sc.view(cloud2);
//pcl_helpers::statistical_removal(result,100,0.01f);
//sc.view(result);

//sc.save_pcd<PointXYZRGB>("../extracted/box_360degree",result);

}

void testRecon()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);

    Scanner sc("../config/defaultConfig.json");

    sc.load_pcd<pcl::PointXYZRGB>("../extracted/chicken.pcd",cloud);
    sc.load_pcd<pcl::PointXYZRGB>("../extracted/chicken.pcd",cloud2);


sc.filterZ<PointXYZRGB>(cloud2,0.01f,0.8f);
 sc.filterZ<PointXYZRGB>(cloud,0.01f,0.8f);

 //auto obj1 = sc.convert_pcl_points(cloud2);
 //auto obj2 = sc.convert_pcl_points(cloud);
 RotateAndTranslateY(cloud,180, Eigen::Vector3f::Zero());
 RotateAndTranslateZ(cloud,0, Eigen::Vector3f{0.f,0.f,1.f});
auto result =  sc.align_ICP(cloud,cloud2);
sc.view(result);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZRGB>);
for(auto& p: result->points)
{
    PointXYZRGB new_p;
    new_p.x = p.x;
    new_p.y = p.y;
    new_p.z = p.z;
    new_p.r = static_cast<uint8_t>(p.r*255);
    new_p.g = static_cast<uint8_t>(p.g*255);
    new_p.b = static_cast<uint8_t>(p.b*255);

    cloud3->points.push_back(new_p);
}

pcl_helpers::statistical_removal(cloud3,100,0.01f);

sc.view(cloud3);
auto mesh = sc.gp3Mls_reconstruction(cloud3);
sc.view(mesh);
//sc.save_obj("test_chicken",mesh);
}

void testICP10()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud4 (new pcl::PointCloud<pcl::PointXYZRGB>);

   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud5 (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud6 (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud7 (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud8 (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud9 (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud10 (new pcl::PointCloud<pcl::PointXYZRGB>);
    Scanner sc("../config/defaultConfig.json");

   sc.load_pcd<pcl::PointXYZRGB>("../test_plane/plane.pcd",cloud);
    sc.load_pcd<pcl::PointXYZRGB>("../extracted/box_0degree.pcd",cloud2);
    sc.load_pcd<pcl::PointXYZRGB>("../extracted/box_45degree.pcd",cloud3);
    sc.load_pcd<pcl::PointXYZRGB>("../extracted/box_90degree.pcd",cloud4);
    sc.load_pcd<pcl::PointXYZRGB>("../extracted/box_135degree.pcd",cloud5);
    sc.load_pcd<pcl::PointXYZRGB>("../extracted/box_180degree.pcd",cloud6);
    sc.load_pcd<pcl::PointXYZRGB>("../extracted/box_225degree.pcd",cloud7);
    sc.load_pcd<pcl::PointXYZRGB>("../extracted/box_270degree.pcd",cloud8);
    sc.load_pcd<pcl::PointXYZRGB>("../extracted/box_315degree.pcd",cloud9);
    sc.load_pcd<pcl::PointXYZRGB>("../extracted/box_360degree.pcd",cloud10);
   

sc.filterX<PointXYZRGB>(cloud2,-0.3f,1.f);
sc.view(cloud2);
sc.filterX<PointXYZRGB>(cloud3,-0.3f,1.f);
sc.filterX<PointXYZRGB>(cloud4,-0.3f,1.f);
sc.filterX<PointXYZRGB>(cloud5,-0.3f,1.f);
sc.filterX<PointXYZRGB>(cloud6,-0.3f,1.f);
sc.filterX<PointXYZRGB>(cloud7,-0.3f,1.f);
sc.filterX<PointXYZRGB>(cloud8,-0.3f,1.f);
sc.filterX<PointXYZRGB>(cloud9,-0.3f,1.f);
sc.filterX<PointXYZRGB>(cloud10,0.3f,1.f);
//cloud2 = pcl_helpers::plane_detection<pcl::PointXYZRGB>(cloud2,0.1f,5000).first;
//cloud3 = pcl_helpers::plane_detection<pcl::PointXYZRGB>(cloud3,0.1f,5000).first;
//cloud4 = pcl_helpers::plane_detection<pcl::PointXYZRGB>(cloud4,0.1f,5000).first;
//cloud5 = pcl_helpers::plane_detection<pcl::PointXYZRGB>(cloud5,0.1f,5000).first;
//cloud6 = pcl_helpers::plane_detection<pcl::PointXYZRGB>(cloud6,0.1f,5000).first;
//cloud7 = pcl_helpers::plane_detection<pcl::PointXYZRGB>(cloud7,0.1f,5000).first;
//cloud8 = pcl_helpers::plane_detection<pcl::PointXYZRGB>(cloud8,0.1f,5000).first;
//cloud9 = pcl_helpers::plane_detection<pcl::PointXYZRGB>(cloud9,0.1f,5000).first;
//cloud10 = pcl_helpers::plane_detection<pcl::PointXYZRGB>(cloud10,0.1f,5000).first;

    RotateAndTranslateY(cloud3,-45,Eigen::Vector3f{0.f,0.f,0.f});
   auto result = sc.align_ICP(cloud2,cloud3);
   sc.view(result);
    RotateAndTranslateY(cloud4,-90,Eigen::Vector3f{0.f,0.f,0.0f});
   result = sc.align_ICP(result,cloud4);
   sc.view(result);
    RotateAndTranslateY(cloud5,-135,Eigen::Vector3f{0.f,0.f,0.0f});
    result = sc.align_ICP(result,cloud5);
   sc.view(result);
    RotateAndTranslateY(cloud6,-180,Eigen::Vector3f{0.f,0.f,0.0f});
    result = sc.align_ICP(result,cloud6);
    auto mesh = sc.gp3Mls_reconstruction(result);

    sc.view(mesh);
   sc.view(result);
    RotateAndTranslateY(cloud7,-225,Eigen::Vector3f{0.f,0.f,0.0f});
   result = sc.align_ICP(result,cloud7);
   sc.view(result);
    RotateAndTranslateY(cloud8,-270,Eigen::Vector3f{0.f,0.f,0.0f});
    result = sc.align_ICP(result,cloud8);
   sc.view(result);
    RotateAndTranslateY(cloud9,-315,Eigen::Vector3f{0.f,0.f,0.0f});
    result = sc.align_ICP(result,cloud9);
   sc.view(result);
    RotateAndTranslateY(cloud10,-360,Eigen::Vector3f{0.f,0.f,0.0f});
    result = sc.align_ICP(result,cloud10);
   sc.view(result);
   

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

void savePlane()
{
    
    Scanner sc("../config/defaultConfig.json");

    std::string name{"../test_plane/plane"};
    auto frame_0degree = sc.capture_FrameXYZRGB(true);
    sc.save_pcd<PointXYZRGB>(name + ".pcd",frame_0degree);
   


}


void saveData()
{
    
    Scanner sc("../config/defaultConfig.json");

    std::string name{"../test4/poor"};
    auto frame_0degree = sc.capture_FrameXYZRGB(true);
    sc.save_pcd<PointXYZRGB>(name + "_0degree.pcd",frame_0degree);
   
    auto frame_45degree = sc.capture_FrameXYZRGB(true);
    sc.save_pcd<PointXYZRGB>(name + "_45degree.pcd",frame_45degree);
    
    auto frame_90degree = sc.capture_FrameXYZRGB(true);
    sc.save_pcd<PointXYZRGB>(name + "_90degree.pcd",frame_90degree);
    
    auto frame_135degree = sc.capture_FrameXYZRGB(true);
    sc.save_pcd<PointXYZRGB>(name + "_135degree.pcd",frame_135degree);
    
    auto frame_180degree = sc.capture_FrameXYZRGB(true);
    sc.save_pcd<PointXYZRGB>(name + "_180degree.pcd",frame_180degree);

    auto frame_225degree = sc.capture_FrameXYZRGB(true);
    sc.save_pcd<PointXYZRGB>(name + "_225degree.pcd",frame_225degree);
    
    auto frame_270degree = sc.capture_FrameXYZRGB(true);
    sc.save_pcd<PointXYZRGB>(name + "_270degree.pcd",frame_270degree);
    
    auto frame_315degree = sc.capture_FrameXYZRGB(true);
    sc.save_pcd<PointXYZRGB>(name + "_315degree.pcd",frame_315degree);

    
    auto frame_360degree = sc.capture_FrameXYZRGB(true);
    sc.save_pcd<PointXYZRGB>(name + "_360degree.pcd",frame_360degree);





}


void loadSavedData()
{
    Scanner sc("../config/defaultConfig.json");
    PointCloudXYZRGBPtr cloud2 (new PointCloudXYZRGB);
    PointCloudXYZRGBPtr cloud1 (new PointCloudXYZRGB);
    PointCloudXYZRGBPtr cloud3 (new PointCloudXYZRGB);
    PointCloudXYZRGBPtr cloud4 (new PointCloudXYZRGB);
    PointCloudXYZRGBPtr cloud5 (new PointCloudXYZRGB);

    std::vector<PointCloudXYZRGBPtr> clouds {cloud1,cloud2,cloud3,cloud4,cloud5};
    std::vector<std::string> paths {
    {"../test1/"},
    {"../test2/"},
    {"../test3/"},
    {"../test4/"},
    {"../test5/"}
    };
    
    std::vector<std::string> degrees{
    {"_0degree.pcd"},
    {"_45degree.pcd"},
    {"_90degree.pcd"},
    {"_135degree.pcd"},
    {"_180degree.pcd"},
    {"_225degree.pcd"},
    {"_270degree.pcd"},
    {"_315degree.pcd"},
    {"_360degree.pcd"}
    };

    std::vector<std::string> objects{
        {"box"}, {"chicken"},{"snail"}, {"poor"},{"cow"}
    };


    for(int i = 0; i < 5; i++)
       for(auto& degree :degrees)
       {

           PointCloudXYZRGBPtr cloud (new PointCloudXYZRGB);
           sc.load_pcd<PointXYZRGB>(paths[i] +objects[i] + degree, cloud);
           sc.view(cloud);
       }

}

void testPipeline()
{
    // std::string model("../spartan_recap2.ply");
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);

    Scanner sc("../config/defaultConfig.json");

    sc.load_pcd<pcl::PointXYZRGB>("../test_plane/plane.pcd",cloud);
    sc.load_pcd<pcl::PointXYZRGB>("../test1/box_0degree.pcd",cloud2);
    sc.view(cloud);
    sc.view(cloud2); 
    
pcl_helpers::statistical_removal(cloud,100,1.0f);
pcl_helpers::statistical_removal(cloud2,100,1.0f);
    sc.view(cloud);
    sc.view(cloud2); 
 auto seg = pcl_helpers::plane_detection<pcl::PointXYZRGB>(cloud,0.1f,5000);

 auto plane = seg.first;
 sc.view(plane);
sc.filterX<PointXYZRGB>(cloud2,-0.5f,1.4f);
auto seg2 = pcl_helpers::plane_detection<pcl::PointXYZRGB>(cloud2,0.1f,5000);
auto result = seg2.second;
sc.view(result);
auto back = result;


seg2 = pcl_helpers::plane_detection<pcl::PointXYZRGB>(result,1e-3f,5000);
result = seg2.second;
sc.view(result);
pcl_helpers::statistical_removal(result,100,1e-4f);
sc.ransac_SVD<pcl::PointXYZRGB>(result,plane);
sc.view(result);
//auto aligned =sc.align_ICP(result,back);

   
}
int main(int argc, char** argv) {
   	//testRANSAC();
    //testKdTree();
 // testRansac2();
    //testICP();
    //showVisualizer();
    //saveData();
    //loadSavedData();
    //testMeshCreation();
    //
    testPlaneDetection2();
  //
   // testRansacPCL();
	//testVectorResize();
//    testICP10();

  //savePlane();
 // testPipeline();
	return EXIT_SUCCESS;
}


