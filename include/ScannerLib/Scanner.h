#pragma once
//#include <ScannerLib/pcl_helper.h>
#include <GL/glew.h>
#include <ScannerLib/openGLPointCloud.hpp>

#include <ScannerLib/JsonParser.h>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
#include <vector>
#include <iostream>
#include <fstream>
//#include <pcl/io/pcd_io.h>
#include <ScannerLib/ICP.h>
//#include <ScannerLib/Window.h>
#include <ScannerLib/Renderer.h>
//#include <pcl/PolygonMesh.h>

using PolygonMesh = pcl::PolygonMesh;
//void register_glfw_callbacks(window& app, glfw_state& app_state);

//Helper function for drawing pointcloud
//void draw_pointcloud(float width, float height, glfw_state& app_state, rs2::points& points);

// Helper function for dispaying time conveniently
//std::string pretty_time(std::chrono::nanoseconds duration);
// Helper function for rendering a seek bar
//void draw_seek_bar(rs2::playback& playback, int* seek_pos, float2& location, float width);


using PC_SIMPLE =pcl::PointCloud<pcl::PointXYZ>; 
using PC_RGB =  pcl::PointCloud<pcl::PointXYZRGB>;

using PointXYZ = pcl::PointXYZ;
using PointXYZRGB = pcl::PointXYZRGB;

using PointCloudXYZ = pcl::PointCloud<PointXYZ>;
using PointCloudXYZPtr = PointCloudXYZ::Ptr;

using PointCloudXYZRGB = pcl::PointCloud<PointXYZRGB>;
using PointCloudXYZRGBPtr = PointCloudXYZRGB::Ptr;


class Scanner {
  public:

    Scanner(std::string json_file)
       : config(json_file)
    {
        window_width = config.GetWindowWidth();
        window_height = config.GetWindowHeight();
        window_name = config.GetWindowName();
    };

    void pointcloud_activateCamera();

    //################################################
    //use Realsense Camera to capture exactly 1 frame 
    //#################################################
    PC_SIMPLE::Ptr capture_FrameXYZ(bool);
    PC_RGB::Ptr capture_FrameXYZRGB(bool);
    //######################################################
    

    //####################################################################################
    //Use Realsense Camera to capture multiple frames (capture until the window is closed)
    //########################################################################################
    std::vector<PC_SIMPLE> capture_FramesXYZ(bool,int count);
    std::vector<PC_RGB::Ptr> capture_FramesXYZRGB(bool,int count);
    //########################################################################################
    
     void load_obj(std::string path);
   

    float getWindowWidth() {return window_width;};
    float getWindowHeight() {return window_height;};
    



    //####################################################################################################################
    //Overloaded functions for the ICP algorithm
    //####################################################################################################################

    std::vector<Point> align_ICP(pcl::PointCloud<pcl::PointXYZ>::Ptr& sourceCloud,pcl::PointCloud<pcl::PointXYZ>::Ptr& targetCloud);
    std::vector<Point> align_ICP(pcl::PointCloud<pcl::PointXYZ>& sourceCloud,pcl::PointCloud<pcl::PointXYZ>& targetCloud);

    std::vector<Point> align_ICP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& sourceCloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& targetCloud);
    std::vector<Point> align_ICP(pcl::PointCloud<pcl::PointXYZRGB>& sourceCloud,pcl::PointCloud<pcl::PointXYZ>& targetCloud);
    
    std::vector<Point> align_ICP(std::vector<Point>& sourceCloud,std::vector<Point>& targetCloud);
    std::vector<Point>& align_ICP(std::vector<std::vector<Point>>& frames);

    //template<typename PointT>
    //std::vector<Point>& align_ICP(typename pcl::PointCloud<PointT>::Ptr)
  //  template<PointT>
  //  void align_ICP(typename pcl::PointCloud<PointT>::Ptr sourceCloud, typename pcl::PointCloud<PointT>::Ptr targetCloud)
  //  {

  //  } 
  //  //########################################################################################################################
    

    //PCL ICP DEMO TEST VERSION

    //#############################################################################
    //Overloaded functions to view a Pointcloud in 3D
    //#############################################################################
    void view(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void view(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
    void view(pcl::PointCloud<pcl::PointXYZ>& cloud);
    void view(pcl::PointCloud<pcl::PointXYZRGB>& cloud);
    void view(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& frames);
    void view(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& frames);
    void view(std::vector<pcl::PointCloud<pcl::PointXYZ>>& frames);
    void view(std::vector<pcl::PointCloud<pcl::PointXYZRGB>>& frames);
    void view(std::vector<Point>& frames);
    void view(std::vector<Point*>& frames);
    void view(PolygonMesh& mesh);
    //#############################################################################
    
   // void view_icp(std::vector<Point>& sourceCloud,std::vector<Point>& targetCloud);

    //Reconstruction Methods
    //Options: 
    //1.Poisson using normal estimation -> not satisfying result
    //2.Poisson using moving least squares algorithm -> bad result
    //3.Greedy triangulation using normal estimation -> not satisfying
    //4.Greedy triangulation using moving least squares algorithm -> best so far


    pcl::PolygonMesh gp3Normal_reconstruction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    pcl::PolygonMesh gp3Normal_reconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PolygonMesh gp3Normal_reconstruction(std::vector<Point> cloud);
    pcl::PolygonMesh gp3Mls_reconstruction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    pcl::PolygonMesh gp3Mls_reconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PolygonMesh gp3Mls_reconstruction(std::vector<Point> cloud);
    pcl::PolygonMesh poissonNormal_reconstruction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    pcl::PolygonMesh poissonNormal_reconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PolygonMesh poissonNormal_reconstruction(std::vector<Point> cloud);
    pcl::PolygonMesh poissonMls_reconstruction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    pcl::PolygonMesh poissonMls_reconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PolygonMesh poissonMls_reconstruction(std::vector<Point> cloud);



    //######################################################
    //Randsac using SVD 
    //######################################################
    template<typename PointT>
    void ransac_SVD(typename pcl::PointCloud<PointT>::Ptr cloud,typename pcl::PointCloud<PointT>::Ptr plane, int numPoints, float distanceThreshold)
    {
        pcl_helpers::RANSAC_SVD<PointT>(cloud,plane,numPoints,distanceThreshold);
    }

    template<typename PointT>
    void ransac_SVD(typename pcl::PointCloud<PointT>::Ptr cloud,typename pcl::PointCloud<PointT>::Ptr plane)
    {
        pcl_helpers::RANSAC_SVD<PointT>(cloud,plane,config.GetNumberOfPointsRansac(),config.GetDistanceThresholdRansac());
    }


   //########################################################
    //save a pointloud as PCD file
    //#####################################################
    template <typename PointT>
    void save_pcd(std::string file_name,typename pcl::PointCloud<PointT>::Ptr& frame)
    {
        if(file_name.find(".pcd") == std::string::npos)
            file_name += ".pcd";
        pcl::io::savePCDFileASCII(file_name.c_str(), frame);       
    }
    
    template <typename PointT>
    void save_pcd(const std::string file_name,std::vector<typename pcl::PointCloud<PointT>::Ptr>& frames)
    {
        for(int i =0 ; i < frames.size(); i++){
            std::string file_name_final = file_name + "_" + std::to_string(i);
            pcl::io::savePCDFileASCII(file_name_final.c_str(), frames[i]);       
        }
    }



    //#######################################################################
    
    //##########################################
    //Save mesh as obj/ply file format
    //###########################################
    void save_obj(const std::string file_name, PolygonMesh& mesh)
    {
          pcl::io::saveOBJFile(file_name.c_str(),mesh);
    }
    
    template<typename PointT>
    void save_ply(const std::string file_name,typename pcl::PointCloud<PointT>& frame)
    {
         pcl::io::savePLYFile(file_name.c_str(),frame);
    }
    
    void save_ply(const std::string file_name,PolygonMesh& mesh)
    {
         pcl::io::savePLYFile(file_name.c_str(),mesh);
    }
    
    


    //##################################################################################
    //load a pointcloud from a PCD file
    //###################################################################################
    template <typename PointT>
    void load_pcd(const std::string filename,typename pcl::PointCloud<PointT>::Ptr& cloud)
    {
        if (pcl::io::loadPCDFile<PointT> (filename, *cloud) == -1) //* load the file
        {
            throw "Couldn't read file test_pcd.pcd \n";
        }
    }
    //########################################################################################
    
    //Load PLY file 
    template<typename PointT>
    void load_ply(const std::string& filename, typename pcl::PointCloud<PointT>::Ptr cloud)
    {
          if(pcl::io::loadPLYFile<PointT> (filename, *cloud) == -1)
          {
              throw "Could't read file\n";
          }
    }
    

    //Estimate normals of pointcloud
    template<typename PointT>
    pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(typename pcl::PointCloud<PointT>::Ptr cloud)
    {
        return pcl_helpers::estimateNormals(cloud);
    }
    

    //###############################################################################
    //Filter points only in range (min, max) along X/Y/Z
    //###############################################################################
    template<typename PointT>
    void filterZ(typename pcl::PointCloud<PointT>::Ptr cloud,float min, float max)
    {
        pcl_helpers::filterZ(cloud,min,max);
    }

    template<typename PointT>
    void filterY(typename pcl::PointCloud<PointT>::Ptr cloud,float min, float max)
    {
        pcl_helpers::filterY(cloud,min,max);
    }

    template<typename PointT>
    void filterX(typename pcl::PointCloud<PointT>::Ptr cloud,float min, float max)
    {
        pcl_helpers::filterX(cloud,min,max);
    }
    //#################################################################################

     //###################################################################################
    //helpers to convert pointclouds from pcl to std::vector<Point> (locally defined)
    //######################################################################################
    std::vector<Point> convert_pcl_points(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    std::vector<Point> convert_pcl_points(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
    std::vector<Point> convert_pcl_points(pcl::PointCloud<pcl::PointXYZ>& cloud);
    std::vector<Point> convert_pcl_points(pcl::PointCloud<pcl::PointXYZRGB>& cloud);
    //####################################################################################

    //########################################################################################################
    //helpers to convert multiple frames of captured pointclouds to std::vector<Point> (contained in a second std::vector)
    //#################################################################################################################
    std::vector<std::vector<Point>> convert_pcl_points(std::vector<pcl::PointCloud<pcl::PointXYZ>>& cloud);
    std::vector<std::vector<Point>> convert_pcl_points(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cloud);
    std::vector<std::vector<Point>> convert_pcl_points(std::vector<pcl::PointCloud<pcl::PointXYZRGB>>& cloud);
    std::vector<std::vector<Point>> convert_pcl_points(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cloud);
    //###################################################################################################################

     //####################################################################################
    //helpers to convert pointclouds from pcl to std::vector<Point*> (locally defined)
    //######################################################################################
    std::vector<Point*> convert_pcl_points_ptr(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    std::vector<Point*> convert_pcl_points_ptr(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
    std::vector<Point*> convert_pcl_points_ptr(pcl::PointCloud<pcl::PointXYZ>& cloud);
    std::vector<Point*> convert_pcl_points_ptr(pcl::PointCloud<pcl::PointXYZRGB>& cloud);
    //####################################################################################

    //########################################################################################################
    //helpers to convert multiple frames of captured pointclouds to std::vector<Point> (contained in a second std::vector)
    //#################################################################################################################
    std::vector<std::vector<Point*>> convert_pcl_points_ptr(std::vector<pcl::PointCloud<pcl::PointXYZ>>& cloud);
    std::vector<std::vector<Point*>> convert_pcl_points_ptr(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cloud);
    std::vector<std::vector<Point*>> convert_pcl_points_ptr(std::vector<pcl::PointCloud<pcl::PointXYZRGB>>& cloud);
    std::vector<std::vector<Point*>> convert_pcl_points_ptr(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cloud);
    //###################################################################################################################

   
   

   void runPipeline(); 
   private:
    float window_width;
    float window_height;
    std::string window_name;
    JsonParser config;
};
