#include <string>
#include <cstdlib>
#include <sstream>
#include <iostream>
#include <iomanip>

#include <cstring>

#include <ScannerLib/Scanner.h>
//#include "ICP.h"

PC_SIMPLE::Ptr Scanner::capture_FrameXYZ(bool show) try
{
    window app(window_width,window_height, window_name.c_str());            
    glfw_state app_state;
    register_glfw_callbacks(app,app_state);
  
    //Declare pointcloud object for calculating pointclouds and texture mappings 
    rs2::pointcloud pc;
    rs2::points points;

    //Declare realsense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    //Start streaming with default recommended configuration
    pipe.start();

        // Wait for frames from the camera to settle
    for (int i = 0; i < 50; i++) {
        auto frames = pipe.wait_for_frames(); //Drop several frames for auto-exposure
    }
 
    
    auto frames = pipe.wait_for_frames(); //Wait for the next set of frames from the camera
    auto depth = frames.get_depth_frame();
    auto color = frames.get_color_frame();
    //Generate the pointcloud and texture mappings
    points = pc.calculate(depth);

    PC_SIMPLE::Ptr pcl_points = pcl_helpers::pc_toPCL(points);
    
    while(app && show){
        draw_pointcloud(app.width(),app.height(),app_state,points);
    }

    return pcl_points;
} catch(const rs2::error& e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return nullptr;

} catch(const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return nullptr;
}



std::vector<PC_RGB::Ptr> Scanner::capture_FramesXYZRGB(bool show,int count) try
{
    window app(window_width,window_height, window_name.c_str());
    glfw_state app_state;
    register_glfw_callbacks(app,app_state);

    //Declare pointcloud object for calculating pointclouds and texture mappings 
    rs2::pointcloud pc;
    rs2::points points;

    //Declare realsense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;   

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //==================
    //Stream configuration
    //
    //cfg.enable_stream(RS2_STREAM_COLOR,window_width,window_height,RS2_FORMAT_BGR8,30);
    //cfg.enable_stream(RS2_STREAM_INFRARED,window_width,window_height,RS2_FORMAT_Y8,30);
    //cfg.enable_stream(RS2_STREAM_DEPTH,window_width,window_height,RS2_FORMAT_Z16,30);



    //Start streaming with (default) recommended configuration
    //pipe.start();
    //rs2::pipeline_profile selection = pipe.start(cfg);
    //rs2::device selected_device = selection.get_device();
    //auto depth_sensor = selected_device.first<rs2::depth_sensor>();
    
  // if(depth_sensor.supports(RS2_OPTION_LASER_POWER))
  //  {
  //      depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED,1.f); //Enable emmiter
  //      depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED,0.f); //Disable emitter

  //  }
  //  if(depth_sensor.supports(RS2_OPTION_LASER_POWER))
  //  {
  //      //Query min and max values
  //      auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
  //      depth_sensor.set_option(RS2_OPTION_LASER_POWER,range.max); //Set max power
  //      depth_sensor.set_option(RS2_OPTION_LASER_POWER,0.f); //Disable laser

  //  }
    pipe.start();



  //  // Wait for frames from the camera to settle
    for (int i = 0; i < 50; i++) {
        auto frames = pipe.wait_for_frames(); //Drop several frames for auto-exposure
    }
    
    std::vector<PC_RGB::Ptr> pointcloud_frames;

    while(app && count){

        auto frames = pipe.wait_for_frames(); //Wait for the next set of frames from the camera
        auto depth = frames.get_depth_frame();
        auto color = frames.get_color_frame();
        //Generate the pointcloud and texture mappings
        pc.map_to(color);
        points = pc.calculate(depth);
    
        //Convert generated Point Cloud to PCL Formatting
        PC_RGB::Ptr pcl_points = pcl_helpers::pcRGB_toPCL(points,color);

        if(show)
            draw_pointcloud_pcl(app.width(),app.height(),app_state,pcl_points);

        pointcloud_frames.push_back(pcl_points);
        count--;
    }
    
    return pointcloud_frames;
} catch(const rs2::error& e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return std::vector<PC_RGB::Ptr>();

} catch(const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return std::vector<PC_RGB::Ptr>();
}





std::vector<PC_SIMPLE> Scanner::capture_FramesXYZ(bool show,int count) try
{
    window app(window_width,window_height, window_name.c_str());            
    glfw_state app_state;
    register_glfw_callbacks(app,app_state);
  
    //Declare pointcloud object for calculating pointclouds and texture mappings 
    rs2::pointcloud pc;
    rs2::points points;

    //Declare realsense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    //Start streaming with default recommended configuration
    pipe.start();

        // Wait for frames from the camera to settle
    for (int i = 0; i < 50; i++) {
        auto frames = pipe.wait_for_frames(); //Drop several frames for auto-exposure
    }
    
    std::vector<PC_SIMPLE> pointcloud_frames;
    while(app && count) {

        auto frames = pipe.wait_for_frames(); //Wait for the next set of frames from the camera
        auto depth = frames.get_depth_frame();
        auto color = frames.get_color_frame();
        //Generate the pointcloud and texture mappings

        points = pc.calculate(depth);

        PC_SIMPLE::Ptr pcl_points = pcl_helpers::pc_toPCL(points);
    
        if(show)
            draw_pointcloud(app.width(),app.height(),app_state,points);
        pointcloud_frames.push_back(*pcl_points);
        count--;

    }

    return pointcloud_frames;
} catch(const rs2::error& e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return {};

} catch(const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return {};
}


PC_RGB::Ptr Scanner::capture_FrameXYZRGB(bool show) try
{
    window app(window_width,window_height, window_name.c_str());
    glfw_state app_state;
    register_glfw_callbacks(app,app_state);

    //Declare pointcloud object for calculating pointclouds and texture mappings 
    rs2::pointcloud pc;
    rs2::points points;

    //Declare realsense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;   

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //==================
    //Stream configuration
    //
    cfg.enable_stream(RS2_STREAM_COLOR,window_width,window_height,RS2_FORMAT_BGR8,30);
    cfg.enable_stream(RS2_STREAM_INFRARED,window_width,window_height,RS2_FORMAT_Y8,30);
    cfg.enable_stream(RS2_STREAM_DEPTH,window_width,window_height,RS2_FORMAT_Z16,30);



    //Start streaming with (default) recommended configuration
    //pipe.start();
    rs2::pipeline_profile selection = pipe.start();
    rs2::device selected_device = selection.get_device();
    auto depth_sensor = selected_device.first<rs2::depth_sensor>();
    
    if(depth_sensor.supports(RS2_OPTION_LASER_POWER))
    {
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED,1.f); //Enable emmiter
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED,0.f); //Disable emitter

    }
    if(depth_sensor.supports(RS2_OPTION_LASER_POWER))
    {
        //Query min and max values
        auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
        depth_sensor.set_option(RS2_OPTION_LASER_POWER,range.max); //Set max power
        depth_sensor.set_option(RS2_OPTION_LASER_POWER,0.f); //Disable laser

    }

    // Wait for frames from the camera to settle
    for (int i = 0; i < 30; i++) {
        auto frames = pipe.wait_for_frames(); //Drop several frames for auto-exposure
    }
    

    auto frames = pipe.wait_for_frames(); //Wait for the next set of frames from the camera
    auto depth = frames.get_depth_frame();
    auto color = frames.get_color_frame();
    //Generate the pointcloud and texture mappings
    pc.map_to(color);
    points = pc.calculate(depth);
    
    //Convert generated Point Cloud to PCL Formatting
    PC_RGB::Ptr pcl_points = pcl_helpers::pcRGB_toPCL(points,color);

    while(app && show){
        draw_pointcloud_pcl(app.width(),app.height(),app_state,pcl_points);
    }
    
    return pcl_points;
} catch(const rs2::error& e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return nullptr;

} catch(const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return nullptr;
}




void Scanner::pointcloud_activateCamera() try
{
    window app(window_width,window_height, window_name.c_str());            
    glfw_state app_state;
    register_glfw_callbacks(app,app_state);
  
    //Declare pointcloud object for calculating pointclouds and texture mappings 
    rs2::pointcloud pc;
    rs2::points points;

    //Declare realsense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    //Start streaming with default recommended configuration
    pipe.start();
    while(app){

        auto frames = pipe.wait_for_frames(); //Wait for the next set of frames from the camera
        auto depth = frames.get_depth_frame();
        auto color = frames.get_color_frame();

        //for cameras that dont have RGB sensor -> use infrared
        if(!color)
            color = frames.get_infrared_frame(); 

        //Generate the pointcloud and texture mappings
        points = pc.calculate(depth);
        
        //Tell pointcloud object to map to this color frame
        pc.map_to(color);

        //Upload the color frame to OpenGL
        app_state.tex.upload(color);

        PC_RGB::Ptr points_pcl = pcl_helpers::pcRGB_toPCL(points,color);
        draw_pointcloud_pcl(app.width(),app.height(),app_state,points_pcl);
    }

}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
}

//std::string pretty_time(std::chrono::nanoseconds duration)
//{
//    using namespace std::chrono;
//    auto hhh = duration_cast<hours>(duration);
//    duration -= hhh;
//    auto mm = duration_cast<minutes>(duration);
//    duration -= mm;
//    auto ss = duration_cast<seconds>(duration);
//    duration -= ss;
//    auto ms = duration_cast<milliseconds>(duration);
//
//    std::ostringstream stream;
//    stream << std::setfill('0') << std::setw(hhh.count() >= 10 ? 2 : 1) << hhh.count() << ':' <<
//        std::setfill('0') << std::setw(2) << mm.count() << ':' <<
//        std::setfill('0') << std::setw(2) << ss.count();
//    return stream.str();
//}


std::vector<Point> Scanner::convert_pcl_points(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
    std::vector<Point> points;
     for(unsigned int i = 0; i < cloud->points.size(); i++) {
            Point p;
            p.x = cloud->points[i].x;
            p.y = cloud->points[i].y;
            p.z = cloud->points[i].z;
            p.r = static_cast<float>(cloud->points[i].r) / 255;
            p.g = static_cast<float>(cloud->points[i].g) / 255;
            p.b = static_cast<float>(cloud->points[i].b) / 255;
            points.push_back(p);
   }

     return points;
}
std::vector<Point> Scanner::convert_pcl_points(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
    std::vector<Point> points;
     for(unsigned int i = 0; i < cloud.points.size(); i++) {
            Point p;
            p.x = cloud.points[i].x;
            p.y = cloud.points[i].y;
            p.z = cloud.points[i].z;
            p.r = static_cast<float>(cloud.points[i].r) / 255;
            p.g = static_cast<float>(cloud.points[i].g) / 255;
            p.b = static_cast<float>(cloud.points[i].b) / 255;
            points.push_back(p);
   }

     return points;
}
std::vector<Point> Scanner::convert_pcl_points(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    std::vector<Point> points;
     for(unsigned int i = 0; i < cloud.points.size(); i++) {
            Point p;
            p.x = cloud.points[i].x;
            p.y = cloud.points[i].y;
            p.z = cloud.points[i].z;
            points.push_back(p);
   }

     return points;
}
std::vector<Point> Scanner::convert_pcl_points(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    std::vector<Point> points;
     for(unsigned int i = 0; i < cloud->points.size(); i++) {
            Point p;
            p.x = cloud->points[i].x;
            p.y = cloud->points[i].y;
            p.z = cloud->points[i].z;
            points.push_back(p);
   }

     return points;
}

std::vector<std::vector<Point>> Scanner::convert_pcl_points(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& frames)
{
     std::vector<std::vector<Point>> frames_new;
     for(unsigned int i = 0; i < frames.size(); i++) {
         std::vector<Point> frame;
         for(unsigned int j = 0 ; j < frames[i]->size(); j++) {
            Point p;
            p.x = frames[i]->points[j].x;
            p.y = frames[i]->points[j].y;
            p.z = frames[i]->points[j].z;
            frame.push_back(p);
         }
         frames_new.push_back(frame);
   }

     return frames_new;
}
std::vector<std::vector<Point>> Scanner::convert_pcl_points(std::vector<pcl::PointCloud<pcl::PointXYZ>>& frames)
{
     std::vector<std::vector<Point>> frames_new;
     for(unsigned int i = 0; i < frames.size(); i++) {
         std::vector<Point> frame;
         for(unsigned int j = 0 ; j < frames[i].size(); j++) {
            Point p;
            p.x = frames[i].points[j].x;
            p.y = frames[i].points[j].y;
            p.z = frames[i].points[j].z;
            frame.push_back(p);
         }
         frames_new.push_back(frame);
   }

     return frames_new;
}
std::vector<std::vector<Point>> Scanner::convert_pcl_points(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& frames)
{
     std::vector<std::vector<Point>> frames_new;
     for(unsigned int i = 0; i < frames.size(); i++) {
         std::vector<Point> frame;
         for(unsigned int j = 0 ; j < frames[i]->size(); j++) {
            Point p;
            p.x = frames[i]->points[j].x;
            p.y = frames[i]->points[j].y;
            p.z = frames[i]->points[j].z;
            p.r = static_cast<float>(frames[i]->points[j].r) / 255;
            p.g = static_cast<float>(frames[i]->points[j].g) / 255;
            p.b = static_cast<float>(frames[i]->points[j].b) / 255;
            frame.push_back(p);
         }
         frames_new.push_back(frame);
   }

     return frames_new;
}
std::vector<std::vector<Point>> Scanner::convert_pcl_points(std::vector<pcl::PointCloud<pcl::PointXYZRGB>>& frames)
{
     std::vector<std::vector<Point>> frames_new;
     for(unsigned int i = 0; i < frames.size(); i++) {
         std::vector<Point> frame;
         for(unsigned int j = 0 ; j < frames[i].size(); j++) {
            Point p;
            p.x = frames[i].points[j].x;
            p.y = frames[i].points[j].y;
            p.z = frames[i].points[j].z;
            p.r = static_cast<float>(frames[i].points[j].r) / 255;
            p.g = static_cast<float>(frames[i].points[j].g) / 255;
            p.b = static_cast<float>(frames[i].points[j].b) / 255;
            frame.push_back(p);
         }
         frames_new.push_back(frame);
   }

     return frames_new;
}
void Scanner::load_obj(std::string path) 
{
        Window app(window_width,window_height,window_name.c_str());
        app.clearScreen();
        Renderer re(app);
        
        //glfw_state app_state;
       // register_glfw_callbacks(app,app_state);
        re.Draw(path);
}
void Scanner::view(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) 
{
     auto points = convert_pcl_points(cloud);

        Window w(window_width,window_height,window_name.c_str());
        w.clearScreen();

        Renderer re(w);
        re.DrawPoints(points);

}

void Scanner::view(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
        auto points = convert_pcl_points(cloud);
        Window w(window_width,window_height,window_name.c_str());
        w.clearScreen();

        Renderer re(w);
        re.DrawPoints(points);

}

void Scanner::view(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cloud)
{
     auto frames = convert_pcl_points(cloud);
     Window w(window_width,window_height,window_name.c_str());
     w.clearScreen();

     Renderer re(w);
     re.DrawPoints(frames);

}

void Scanner::view(std::vector<Point>& cloud)
{
    Window w(window_width,window_height,window_name.c_str());
    w.clearScreen();

    Renderer re(w);
    re.DrawPoints(cloud);
}

void Scanner::view(std::vector<pcl::PointCloud<pcl::PointXYZ>>& cloud)
{
     auto frames = convert_pcl_points(cloud);
     Window w(window_width,window_height,window_name.c_str());
     w.clearScreen();

     Renderer re(w);
     re.DrawPoints(frames);
}   
void Scanner::view(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cloud)
{
     auto frames = convert_pcl_points(cloud);
     Window w(window_width,window_height,window_name.c_str());
     w.clearScreen();

     Renderer re(w);
     re.DrawPoints(frames);
}
void Scanner::view(std::vector<pcl::PointCloud<pcl::PointXYZRGB>>& cloud)
{
    std::vector<std::vector<Point>> frames = convert_pcl_points(cloud);
     Window w(window_width,window_height,window_name.c_str());
     w.clearScreen();

     Renderer re(w);
     re.DrawPoints(frames);
}


void Scanner::view(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
        auto points = convert_pcl_points(cloud);
        Window w(window_width,window_height,window_name.c_str());
        w.clearScreen();

        Renderer re(w);
        re.DrawPoints(points);

}
void Scanner::view(std::vector<Point*>& frames)
{
        Window w(window_width,window_height,window_name.c_str());
        w.clearScreen();

        Renderer re(w);
        re.DrawPoints(frames);

}


void Scanner::view(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
        auto points = convert_pcl_points(cloud);
        Window w(window_width,window_height,window_name.c_str());
        w.clearScreen();

        Renderer re(w);
        re.DrawPoints(points);

}

void Scanner::view(PolygonMesh& mesh)
{
    pcl_helpers::view(mesh);
}

std::vector<Point> Scanner::align_ICP(pcl::PointCloud<pcl::PointXYZ>::Ptr& sourceCloud,pcl::PointCloud<pcl::PointXYZ>::Ptr& targetCloud)
{
    pcl_helpers::statistical_removal(sourceCloud,100,0.8f);
    pcl_helpers::statistical_removal(targetCloud,100,0.8f);

    auto dynamicCloud = convert_pcl_points(sourceCloud);
    auto staticCloud = convert_pcl_points(targetCloud);
   
    icp(dynamicCloud,staticCloud,config.GetMaxIterationsICP());
    for(int i = 0; i < staticCloud.size(); i++)
        dynamicCloud.push_back(staticCloud[i]);

    return dynamicCloud;
}

std::vector<Point> Scanner::align_ICP(pcl::PointCloud<pcl::PointXYZ>& sourceCloud,pcl::PointCloud<pcl::PointXYZ>& targetCloud)
{
    //pcl_helpers::statistical_removal(sourceCloud);
    //pcl_helpers::statistical_removal(targetCloud);

    auto dynamicCloud = convert_pcl_points(sourceCloud);
    auto staticCloud = convert_pcl_points(targetCloud);

    icp(dynamicCloud,staticCloud,config.GetMaxIterationsICP());
    for(int i = 0; i < staticCloud.size(); i++)
        dynamicCloud.push_back(staticCloud[i]);

    return dynamicCloud;
}

std::vector<Point> Scanner::align_ICP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& sourceCloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& targetCloud)
{
    pcl_helpers::statistical_removal(sourceCloud,100,0.8f);
    pcl_helpers::statistical_removal(targetCloud,100,0.8f);

    

    auto dynamicCloud = convert_pcl_points(sourceCloud);
    auto staticCloud = convert_pcl_points(targetCloud);
    test_cloud_random_shift(dynamicCloud);

    //std::cout << "x:" << dynamicCloud[0]->x << " y:" << dynamicCloud[0]->y << " z:" << dynamicCloud[0]->z <<std::endl;

    icp(dynamicCloud,staticCloud,config.GetMaxIterationsICP());
    for(int i = 0; i < staticCloud.size(); i++) {
        dynamicCloud.push_back(staticCloud[i]);
    }

    return dynamicCloud;
}
std::vector<Point> Scanner::align_ICP(pcl::PointCloud<pcl::PointXYZRGB>& sourceCloud,pcl::PointCloud<pcl::PointXYZ>& targetCloud)
{
    //pcl_helpers::statistical_removal(sourceCloud);
    //pcl_helpers::statistical_removal(targetCloud);

    auto dynamicCloud = convert_pcl_points(sourceCloud);
    auto staticCloud = convert_pcl_points(targetCloud);

    icp(dynamicCloud,staticCloud,config.GetMaxIterationsICP());
    for(int i = 0; i < staticCloud.size(); i++)
        dynamicCloud.push_back(staticCloud[i]);

    return dynamicCloud;  
}

std::vector<Point> Scanner::align_ICP(std::vector<Point>& sourceCloud,std::vector<Point>& targetCloud)
{

    icp(sourceCloud,targetCloud,config.GetMaxIterationsICP());
    for(int i = 0; i < targetCloud.size(); i++)
        sourceCloud.push_back(targetCloud[i]);

    return sourceCloud;

}


std::vector<Point>& Scanner::align_ICP(std::vector<std::vector<Point>>& frames)
{
    return icp(frames, this->config.GetMaxIterationsICP());
}
std::vector<Point*> Scanner::convert_pcl_points_ptr(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
    std::vector<Point*> points;
     for(unsigned int i = 0; i < cloud->points.size(); i++) {
            Point p;
            p.x = cloud->points[i].x;
            p.y = cloud->points[i].y;
            p.z = cloud->points[i].z;
            p.r = static_cast<float>(cloud->points[i].r) / 255;
            p.g = static_cast<float>(cloud->points[i].g) / 255;
            p.b = static_cast<float>(cloud->points[i].b) / 255;
            points.push_back(&p);
   }

     return points;
}
std::vector<Point*> Scanner::convert_pcl_points_ptr(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
    std::vector<Point*> points;
     for(unsigned int i = 0; i < cloud.points.size(); i++) {
            Point p;
            p.x = cloud.points[i].x;
            p.y = cloud.points[i].y;
            p.z = cloud.points[i].z;
            p.r = static_cast<float>(cloud.points[i].r) / 255;
            p.g = static_cast<float>(cloud.points[i].g) / 255;
            p.b = static_cast<float>(cloud.points[i].b) / 255;
            points.push_back(&p);
   }

     return points;
}
std::vector<Point*> Scanner::convert_pcl_points_ptr(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    std::vector<Point*> points;
     for(unsigned int i = 0; i < cloud.points.size(); i++) {
            Point p;
            p.x = cloud.points[i].x;
            p.y = cloud.points[i].y;
            p.z = cloud.points[i].z;
            points.push_back(&p);
   }

     return points;
}
std::vector<Point*> Scanner::convert_pcl_points_ptr(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    std::vector<Point*> points;
     for(unsigned int i = 0; i < cloud->points.size(); i++) {
        //if(cloud->points[i].z && cloud->points[i].z < 15.0f){
            Point p;
            p.x = cloud->points[i].x;
            p.y = cloud->points[i].y;
            p.z = cloud->points[i].z;
            //p.r = static_cast<float>(cloud->points[i].r) / 255;
            //p.g = static_cast<float>(cloud->points[i].g) / 255;
            //p.b = static_cast<float>(cloud->points[i].b) / 255;
            points.push_back(&p);
        //}
   }

     return points;
}

std::vector<std::vector<Point*>> Scanner::convert_pcl_points_ptr(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& frames)
{
     std::vector<std::vector<Point*>> frames_new;
     for(unsigned int i = 0; i < frames.size(); i++) {
         std::vector<Point*> frame;
         for(unsigned int j = 0 ; j < frames[i]->size(); j++) {
            Point p;
            p.x = frames[i]->points[j].x;
            p.y = frames[i]->points[j].y;
            p.z = frames[i]->points[j].z;
            //p.r = static_cast<float>(cloud->points[i].r) / 255;
            //p.g = static_cast<float>(cloud->points[i].g) / 255;
            //p.b = static_cast<float>(cloud->points[i].b) / 255;
            frame.push_back(&p);
         }
         frames_new.push_back(frame);
   }

     return frames_new;
}
std::vector<std::vector<Point*>> Scanner::convert_pcl_points_ptr(std::vector<pcl::PointCloud<pcl::PointXYZ>>& frames)
{
     std::vector<std::vector<Point*>> frames_new;
     for(unsigned int i = 0; i < frames.size(); i++) {
         std::vector<Point*> frame;
         for(unsigned int j = 0 ; j < frames[i].size(); j++) {
            Point p;
            p.x = frames[i].points[j].x;
            p.y = frames[i].points[j].y;
            p.z = frames[i].points[j].z;
            //p.r = static_cast<float>(cloud->points[i].r) / 255;
            //p.g = static_cast<float>(cloud->points[i].g) / 255;
            //p.b = static_cast<float>(cloud->points[i].b) / 255;
            frame.push_back(&p);
         }
         frames_new.push_back(frame);
   }

     return frames_new;
}
std::vector<std::vector<Point*>> Scanner::convert_pcl_points_ptr(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& frames)
{
     std::vector<std::vector<Point*>> frames_new;
     for(unsigned int i = 0; i < frames.size(); i++) {
         std::vector<Point*> frame;
         for(unsigned int j = 0 ; j < frames[i]->size(); j++) {
            Point p;
            p.x = frames[i]->points[j].x;
            p.y = frames[i]->points[j].y;
            p.z = frames[i]->points[j].z;
            p.r = static_cast<float>(frames[i]->points[j].r) / 255;
            p.g = static_cast<float>(frames[i]->points[j].g) / 255;
            p.b = static_cast<float>(frames[i]->points[j].b) / 255;
            frame.push_back(&p);
         }
         frames_new.push_back(frame);
   }

     return frames_new;
}
std::vector<std::vector<Point*>> Scanner::convert_pcl_points_ptr(std::vector<pcl::PointCloud<pcl::PointXYZRGB>>& frames)
{
     std::vector<std::vector<Point*>> frames_new;
     for(unsigned int i = 0; i < frames.size(); i++) {
         std::vector<Point*> frame;
         for(unsigned int j = 0 ; j < frames[i].size(); j++) {
            Point p;
            p.x = frames[i].points[j].x;
            p.y = frames[i].points[j].y;
            p.z = frames[i].points[j].z;
            p.r = static_cast<float>(frames[i].points[j].r) / 255;
            p.g = static_cast<float>(frames[i].points[j].g) / 255;
            p.b = static_cast<float>(frames[i].points[j].b) / 255;
            frame.push_back(&p);
         }
         frames_new.push_back(frame);
   }

     return frames_new;
}




pcl::PolygonMesh Scanner::gp3Normal_reconstruction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{   
    using namespace pcl;
    using namespace pcl_helpers;
    PolygonMesh mesh;
    poisson_gp3_reconstruction<PointXYZRGB,PointXYZRGBNormal>(cloud,2,1,mesh);
    return mesh;
}
pcl::PolygonMesh Scanner::gp3Normal_reconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{   
    using namespace pcl;
    using namespace pcl_helpers;
    PolygonMesh mesh;
    poisson_gp3_reconstruction<PointXYZ,PointNormal>(cloud,2,1,mesh);
    return mesh;

}
pcl::PolygonMesh Scanner::gp3Normal_reconstruction(std::vector<Point> cloud)
{   
    using namespace pcl;
    using namespace pcl_helpers;
    PolygonMesh mesh;
    poisson_gp3_reconstruction(cloud,2,1,mesh);
    return mesh;

}
pcl::PolygonMesh Scanner::gp3Mls_reconstruction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    using namespace pcl;
    using namespace pcl_helpers;
    PolygonMesh mesh;
    poisson_gp3_reconstruction<PointXYZRGB,PointXYZRGBNormal>(cloud,2,2,mesh);
    return mesh;

}
pcl::PolygonMesh Scanner::gp3Mls_reconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    using namespace pcl;
    using namespace pcl_helpers;
    PolygonMesh mesh;
    poisson_gp3_reconstruction<PointXYZ,PointNormal>(cloud,2,2,mesh);
    return mesh;

}
pcl::PolygonMesh Scanner::gp3Mls_reconstruction(std::vector<Point> cloud)
{
    using namespace pcl;
    using namespace pcl_helpers;
    PolygonMesh mesh;
    poisson_gp3_reconstruction(cloud,2,2,mesh);
    return mesh;

}
pcl::PolygonMesh Scanner::poissonNormal_reconstruction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    using namespace pcl;
    using namespace pcl_helpers;
    PolygonMesh mesh;
    poisson_gp3_reconstruction<PointXYZRGB,PointXYZRGBNormal>(cloud,1,1,mesh);
    return mesh;

}
pcl::PolygonMesh Scanner::poissonNormal_reconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    using namespace pcl;
    using namespace pcl_helpers;
    PolygonMesh mesh;
    poisson_gp3_reconstruction<PointXYZ,PointNormal>(cloud,1,1,mesh); 
    return mesh;

}
pcl::PolygonMesh Scanner::poissonNormal_reconstruction(std::vector<Point> cloud)
{
    using namespace pcl;
    using namespace pcl_helpers;
    PolygonMesh mesh;
    poisson_gp3_reconstruction(cloud,1,1,mesh);
    return mesh;

}
pcl::PolygonMesh Scanner::poissonMls_reconstruction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    using namespace pcl;
    using namespace pcl_helpers;
    PolygonMesh mesh;
    poisson_gp3_reconstruction<PointXYZRGB,PointXYZRGBNormal>(cloud,1,2,mesh);
    return mesh;

}

pcl::PolygonMesh Scanner::poissonMls_reconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    using namespace pcl;
    using namespace pcl_helpers;
    PolygonMesh mesh;
    poisson_gp3_reconstruction<PointXYZ,PointNormal>(cloud,1,2,mesh);
    return mesh;

}

pcl::PolygonMesh Scanner::poissonMls_reconstruction(std::vector<Point> cloud)
{
    using namespace pcl;
    using namespace pcl_helpers;
    PolygonMesh mesh;
    poisson_gp3_reconstruction(cloud,1,2,mesh);
    return mesh;

}

void Scanner::runPipeline()
{
    //capture plane data
    auto frames_plane = this->capture_FramesXYZRGB(true,2);

    auto frames_object = this->capture_FramesXYZRGB(true,10);


    //perform Ransac cloud segmentation

    for(int i = 0; i< frames_plane.size(); i++)
    {
        for(int j = 0; j < frames_object.size(); j++)
            this->ransac_SVD<pcl::PointXYZRGB>(frames_object[j],frames_plane[i]);

    }


    //convert to vector<Points>
   auto points_converted = convert_pcl_points(frames_object);

   //ICP
   auto& aligned_cloud = this->align_ICP(points_converted);

   PolygonMesh mesh = this->gp3Mls_reconstruction(aligned_cloud);

   this->save_obj(config.GetSavePath(),mesh);

   this->view(mesh);

}
