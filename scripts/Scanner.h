#include <GL/glew.h>
#include "openGLPointCloud.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include "openGLPointCloudSystem/Window.h"
#include "openGLPointCloudSystem/Renderer.h"


void register_glfw_callbacks(window& app, glfw_state& app_state);

//Helper function for drawing pointcloud
void draw_pointcloud(float width, float height, glfw_state& app_state, rs2::points& points);

// Helper function for dispaying time conveniently
std::string pretty_time(std::chrono::nanoseconds duration);
// Helper function for rendering a seek bar
//void draw_seek_bar(rs2::playback& playback, int* seek_pos, float2& location, float width);


using PC_SIMPLE =pcl::PointCloud<pcl::PointXYZ>; 
using PC_RGB =  pcl::PointCloud<pcl::PointXYZRGB>;


class Scanner {
  public:
    Scanner(): 
        window_width(1280), window_height(720),window_name("Pointcloud window") {};

    Scanner(float width,float height,const std::string& name): 
        window_width(width), window_height(height), window_name(name){};

    void pointcloud_activateCamera();

    //################################################
    //use Realsense Camera to capture exactly 1 frame 
    //#################################################
    PC_SIMPLE::Ptr captureOnce_PcData(bool);
    PC_RGB::Ptr captureOnce_PcRGBData(bool);
    //######################################################
    

    //####################################################################################
    //Use Realsense Camera to capture multiple frames (capture until the window is closed)
    //########################################################################################
    std::vector<PC_SIMPLE> capture_PcData(bool);
    std::vector<PC_RGB::Ptr> capture_PcRGBData(bool);
    //########################################################################################
    
    //########################################################
    //save a pointloud as PCD file
    //#####################################################
    template <typename PointT>
    void save(typename pcl::PointCloud<PointT>& frame,std::string file_name)
    {
        pcl::io::savePCDFileASCII(file_name.c_str(), frame);       
    }
    //#######################################################################
    

    //##################################################################################
    //load a pointcloud from a PCD file
    //###################################################################################
    template <typename PointT>
    void load(std::string filename,typename pcl::PointCloud<PointT>::Ptr& cloud)
    {
        if (pcl::io::loadPCDFile<PointT> (filename, *cloud) == -1) //* load the file
        {
            throw "Couldn't read file test_pcd.pcd \n";
        }
    }
    //########################################################################################
    
    void load_obj(std::string path);
   

    float getWindowWidth() {return window_width;};
    float getWindowHeight() {return window_height;};
    

    //####################################################################################################################
    //Overloaded functions for the ICP algorithm
    //####################################################################################################################

    std::vector<Point*> allign_ICP(pcl::PointCloud<pcl::PointXYZ>::Ptr& sourceCloud,pcl::PointCloud<pcl::PointXYZ>::Ptr& targetCloud);
    std::vector<Point*> allign_ICP(pcl::PointCloud<pcl::PointXYZ>& sourceCloud,pcl::PointCloud<pcl::PointXYZ>& targetCloud);

    std::vector<Point> allign_ICP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& sourceCloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& targetCloud);
    std::vector<Point*> allign_ICP(pcl::PointCloud<pcl::PointXYZRGB>& sourceCloud,pcl::PointCloud<pcl::PointXYZ>& targetCloud);
    
    std::vector<Point*> allign_ICP(std::vector<Point>& sourceCloud,std::vector<Point>& targetCloud);
    std::vector<Point*> allign_ICP(std::vector<Point*>& sourceCloud,std::vector<Point*>& targetCloud);
    //########################################################################################################################
    

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
    //#############################################################################
    
   // void view_icp(std::vector<Point>& sourceCloud,std::vector<Point>& targetCloud);

  private:
    //####################################################################################
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

    float window_width;
    float window_height;
    std::string window_name;
};
