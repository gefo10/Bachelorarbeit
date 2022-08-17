#include <string>
#include <cstdlib>
#include <sstream>
#include <iostream>
#include <iomanip>

#include <cstring>

#include <ScannerLib/Scanner.h>
//#include "ICP.h"

PointCloudXYZPtr Scanner::capture_FrameXYZ(bool show) try
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

    PointCloudXYZPtr pcl_points = pcl_helpers::pc_toPCL(points);
    
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



std::vector<PointCloudXYZRGBPtr> Scanner::capture_FramesXYZRGB(bool show,int count) try
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
    
    std::vector<PointCloudXYZRGBPtr> pointcloud_frames;

    while(app && count){

        auto frames = pipe.wait_for_frames(); //Wait for the next set of frames from the camera
        auto depth = frames.get_depth_frame();
        auto color = frames.get_color_frame();
        //Generate the pointcloud and texture mappings
        pc.map_to(color);
        points = pc.calculate(depth);
    
        //Convert generated Point Cloud to PCL Formatting
        PointCloudXYZRGBPtr pcl_points = pcl_helpers::pcRGB_toPCL(points,color);

        if(show)
            draw_pointcloud_pcl(app.width(),app.height(),app_state,pcl_points);

        pointcloud_frames.push_back(pcl_points);
        count--;
    }
    
    return pointcloud_frames;
} catch(const rs2::error& e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return std::vector<PointCloudXYZRGBPtr>();

} catch(const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return std::vector<PointCloudXYZRGBPtr>();
}





std::vector<PointCloudXYZPtr> Scanner::capture_FramesXYZ(bool show,int count) try
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
    
    std::vector<PointCloudXYZPtr> pointcloud_frames;
    while(app && count) {

        auto frames = pipe.wait_for_frames(); //Wait for the next set of frames from the camera
        auto depth = frames.get_depth_frame();
        auto color = frames.get_color_frame();
        //Generate the pointcloud and texture mappings

        points = pc.calculate(depth);

        PointCloudXYZPtr pcl_points = pcl_helpers::pc_toPCL(points);
    
        if(show)
            draw_pointcloud(app.width(),app.height(),app_state,points);
        pointcloud_frames.push_back(pcl_points);
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


PointCloudXYZRGBPtr Scanner::capture_FrameXYZRGB(bool show) try
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
    
//    if(depth_sensor.supports(RS2_OPTION_LASER_POWER))
//    {
//        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED,1.f); //Enable emmiter
//        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED,0.f); //Disable emitter
//
//    }
//    if(depth_sensor.supports(RS2_OPTION_LASER_POWER))
//    {
//        //Query min and max values
//        auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
//        depth_sensor.set_option(RS2_OPTION_LASER_POWER,range.max); //Set max power
//        depth_sensor.set_option(RS2_OPTION_LASER_POWER,0.f); //Disable laser
//
//    }
//
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
    PointCloudXYZRGBPtr pcl_points = pcl_helpers::pcRGB_toPCL(points,color);

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

        PointCloudXYZRGBPtr points_pcl = pcl_helpers::pcRGB_toPCL(points,color);
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
        app.clearScreen(config.GetRedColor(),config.GetGreenColor(),config.GetBlueColor());
        Renderer re(app);
        
        //glfw_state app_state;
       // register_glfw_callbacks(app,app_state);
        re.Draw(path);
}
void Scanner::view(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) 
{
     auto points = convert_pcl_points(cloud);

        Window w(window_width,window_height,window_name.c_str());
        w.clearScreen(config.GetRedColor(),config.GetGreenColor(),config.GetBlueColor());

        Renderer re(w);
        re.DrawLine(Point{-0.75f,0.f,0.f}, Point{0.77f,0.f,0.f});
        re.DrawPoints(points);

}

void Scanner::view(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
        auto points = convert_pcl_points(cloud);
        Window w(window_width,window_height,window_name.c_str());
        w.clearScreen(config.GetRedColor(),config.GetGreenColor(),config.GetBlueColor());

        Renderer re(w);
        re.DrawPoints(points);

}

void Scanner::view(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cloud)
{
     auto frames = convert_pcl_points(cloud);
     Window w(window_width,window_height,window_name.c_str());
     w.clearScreen(config.GetRedColor(),config.GetGreenColor(),config.GetBlueColor());

     Renderer re(w);
     re.DrawPoints(frames);

}

void Scanner::view(std::vector<Point>& cloud)
{
    Window w(window_width,window_height,window_name.c_str());
    w.clearScreen(config.GetRedColor(),config.GetGreenColor(),config.GetBlueColor());

    Renderer re(w);
    
    
    re.DrawPoints(cloud);
}

void Scanner::view(std::vector<pcl::PointCloud<pcl::PointXYZ>>& cloud)
{
     auto frames = convert_pcl_points(cloud);
     Window w(window_width,window_height,window_name.c_str());
     w.clearScreen(config.GetRedColor(),config.GetGreenColor(),config.GetBlueColor());

     Renderer re(w);
     re.DrawPoints(frames);
}   
void Scanner::view(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cloud)
{
     auto frames = convert_pcl_points(cloud);
     Window w(window_width,window_height,window_name.c_str());
     w.clearScreen(config.GetRedColor(),config.GetGreenColor(),config.GetBlueColor());

     Renderer re(w);
     re.DrawPoints(frames);
}
void Scanner::view(std::vector<pcl::PointCloud<pcl::PointXYZRGB>>& cloud)
{
    std::vector<std::vector<Point>> frames = convert_pcl_points(cloud);
     Window w(window_width,window_height,window_name.c_str());
     w.clearScreen(config.GetRedColor(),config.GetGreenColor(),config.GetBlueColor());

     Renderer re(w);
     re.DrawPoints(frames);
}


void Scanner::view(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
        auto points = convert_pcl_points(cloud);
        Window w(window_width,window_height,window_name.c_str());
        w.clearScreen(config.GetRedColor(),config.GetGreenColor(),config.GetBlueColor());

        Renderer re(w);
        re.DrawPoints(points);

}
void Scanner::view(std::vector<Point*>& frames)
{
        Window w(window_width,window_height,window_name.c_str());
        w.clearScreen(config.GetRedColor(),config.GetGreenColor(),config.GetBlueColor());

        Renderer re(w);
        re.DrawPoints(frames);

}


void Scanner::view(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
        auto points = convert_pcl_points(cloud);
        Window w(window_width,window_height,window_name.c_str());
        w.clearScreen(config.GetRedColor(),config.GetGreenColor(),config.GetBlueColor());

        Renderer re(w);
        re.DrawPoints(points);

}

void Scanner::view(PolygonMesh& mesh)
{
    pcl_helpers::view(mesh,1.0f,1.0f,1.0f);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Scanner::align_ICP(pcl::PointCloud<pcl::PointXYZ>::Ptr& sourceCloud,pcl::PointCloud<pcl::PointXYZ>::Ptr& targetCloud)
{
    pcl_helpers::statistical_removal(sourceCloud,100,0.8f);
    pcl_helpers::statistical_removal(targetCloud,100,0.8f);

    auto dynamicCloud = convert_pcl_points(sourceCloud);
    auto staticCloud = convert_pcl_points(targetCloud);
   
    icp(dynamicCloud,staticCloud,config.GetMaxIterationsICP());
    for(int i = 0; i < staticCloud.size(); i++)
        dynamicCloud.push_back(staticCloud[i]);
    
    
    pcl::PointCloud<PointXYZ>::Ptr result (new pcl::PointCloud<PointXYZ>);
    for(auto& p : dynamicCloud)
    {
        PointXYZ new_p;
        new_p.x = p.x;
        new_p.y = p.y;
        new_p.z = p.z;
        //new_p.r = static_cast<uint8_t>(p.r * 255);
        //new_p.g = static_cast<uint8_t>(p.g * 255);
        //new_p.b = static_cast<uint8_t>(p.b * 255);
        result->points.push_back(new_p);

    }

    //return dynamicCloud;
    return result;
}

pcl::PointCloud<pcl::PointXYZ> Scanner::align_ICP(pcl::PointCloud<pcl::PointXYZ>& sourceCloud,pcl::PointCloud<pcl::PointXYZ>& targetCloud)
{
    //pcl_helpers::statistical_removal(sourceCloud);
    //pcl_helpers::statistical_removal(targetCloud);

    auto dynamicCloud = convert_pcl_points(sourceCloud);
    auto staticCloud = convert_pcl_points(targetCloud);

    icp(dynamicCloud,staticCloud,config.GetMaxIterationsICP());
    for(int i = 0; i < staticCloud.size(); i++)
        dynamicCloud.push_back(staticCloud[i]);
    
    pcl::PointCloud<PointXYZ>::Ptr result (new pcl::PointCloud<PointXYZ>);
    for(auto& p : dynamicCloud)
    {
        PointXYZ new_p;
        new_p.x = p.x;
        new_p.y = p.y;
        new_p.z = p.z;
        //new_p.r = static_cast<uint8_t>(p.r * 255);
        //new_p.g = static_cast<uint8_t>(p.g * 255);
        //new_p.b = static_cast<uint8_t>(p.b * 255);
        result->points.push_back(new_p);

    }

    
    //return dynamicCloud;
    return *result;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Scanner::align_ICP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& sourceCloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& targetCloud)
{
    //pcl_helpers::statistical_removal(sourceCloud,100,0.8f);
    //pcl_helpers::statistical_removal(targetCloud,100,0.8f);

    

    auto dynamicCloud = convert_pcl_points(sourceCloud);
    auto staticCloud = convert_pcl_points(targetCloud);
    

    //RotateAndTranslateZ(dynamicCloud,60,Eigen::Vector3f{0.f,1.5f,0.4f});
    auto quick_look = dynamicCloud;
    for(int i = 0; i < staticCloud.size(); i++) {
        quick_look.push_back(staticCloud[i]);
    }
    this->view(quick_look);


    //std::cout << "x:" << dynamicCloud[0]->x << " y:" << dynamicCloud[0]->y << " z:" << dynamicCloud[0]->z <<std::endl;


    icp(dynamicCloud,staticCloud,config.GetMaxIterationsICP());
    targetCloud->points.clear();
    sourceCloud->points.clear();
   
    for(int i = 0; i < staticCloud.size(); i++) {
        dynamicCloud.push_back(staticCloud[i]);
    }
    pcl::PointCloud<PointXYZRGB>::Ptr result (new pcl::PointCloud<PointXYZRGB>);
    for(auto& p : dynamicCloud)
    {
        PointXYZRGB new_p;
        new_p.x = p.x; 
        new_p.y = p.y; 
        new_p.z = p.z; 
        new_p.r = static_cast<uint8_t>(p.r * 255); 
        new_p.g = static_cast<uint8_t>(p.g * 255); 
        new_p.b = static_cast<uint8_t>(p.b * 255);
        result->points.push_back(new_p);

    }



    return result;
}

//std::vector<Point> Scanner::align_ICP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& sourceCloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& targetCloud)
//{
//    //pcl_helpers::statistical_removal(sourceCloud,100,0.8f);
//    //pcl_helpers::statistical_removal(targetCloud,100,0.8f);
//
//    
//
//    auto dynamicCloud = convert_pcl_points(sourceCloud);
//    auto staticCloud = convert_pcl_points(targetCloud);
//    
//
//    //RotateAndTranslateZ(dynamicCloud,60,Eigen::Vector3f{0.f,1.5f,0.4f});
//    auto quick_look = dynamicCloud;
//    for(int i = 0; i < staticCloud.size(); i++) {
//        quick_look.push_back(staticCloud[i]);
//    }
//    this->view(quick_look);
//
//
//    //std::cout << "x:" << dynamicCloud[0]->x << " y:" << dynamicCloud[0]->y << " z:" << dynamicCloud[0]->z <<std::endl;
//
//
//    icp(dynamicCloud,staticCloud,config.GetMaxIterationsICP());
//    targetCloud->points.clear();
//    sourceCloud->points.clear();
//    for(auto& p : staticCloud)
//    {
//        PointXYZRGB new_p;
//        new_p.x = p.x; 
//        new_p.y = p.y; 
//        new_p.z = p.z; 
//        new_p.r = static_cast<uint8_t>(p.r * 255); 
//        new_p.g = static_cast<uint8_t>(p.g * 255); 
//        new_p.b = static_cast<uint8_t>(p.b * 255);
//        sourceCloud->points.push_back(new_p);
//
//    } for(auto& p : dynamicCloud)
//    {
//        PointXYZRGB new_p;
//        new_p.x = p.x; 
//        new_p.y = p.y; 
//        new_p.z = p.z; 
//        new_p.r = static_cast<uint8_t>(p.r * 255); 
//        new_p.g = static_cast<uint8_t>(p.g * 255); 
//        new_p.b = static_cast<uint8_t>(p.b * 255);
//        targetCloud->points.push_back(new_p);
//
//    }
//    for(int i = 0; i < staticCloud.size(); i++) {
//        dynamicCloud.push_back(staticCloud[i]);
//    }
//
//
//
//    return dynamicCloud;
//}
pcl::PointCloud<pcl::PointXYZRGB> Scanner::align_ICP(pcl::PointCloud<pcl::PointXYZRGB>& sourceCloud,pcl::PointCloud<pcl::PointXYZ>& targetCloud)
{
    //pcl_helpers::statistical_removal(sourceCloud);
    //pcl_helpers::statistical_removal(targetCloud);

    auto dynamicCloud = convert_pcl_points(sourceCloud);
    auto staticCloud = convert_pcl_points(targetCloud);

    icp(dynamicCloud,staticCloud,config.GetMaxIterationsICP());
    for(int i = 0; i < staticCloud.size(); i++)
        dynamicCloud.push_back(staticCloud[i]);
   
    pcl::PointCloud<PointXYZRGB>::Ptr result (new pcl::PointCloud<PointXYZRGB>);

    for(auto& p : dynamicCloud)
    {
        PointXYZRGB new_p;
        new_p.x = p.x;
        new_p.y = p.y;
        new_p.z = p.z;
        new_p.r = static_cast<uint8_t>(p.r * 255);
        new_p.g = static_cast<uint8_t>(p.g * 255);
        new_p.b = static_cast<uint8_t>(p.b * 255);
        result->points.push_back(new_p);

    }

    //return dynamicCloud;  
    return *result;
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

    return poisson_gp3_reconstruction<PointXYZRGB,PointXYZRGBNormal>(cloud,2,1,
                                       config.GetMLSRadiusSearch(),
                                       config.GetNEKSearch(),
                                       config.GetGP3KSearch(),
                                       config.GetGP3MU(),
                                       config.GetGP3SearchRadius(),
                                       config.GetGP3MaxNearestNeighbors(),
                                       config.GetGP3NormalConsistency(),
                                       config.GetPoissonNThreads(),
                                       config.GetPoissonKSearch(),
                                       config.GetPoissonDepth(), 
                                       config.GetPoissonPointWeight(),
                                       config.GetPoissonSamplePNode(),
                                       config.GetPoissonScale(), 
                                       config.GetPoissonIsoDivide(),
                                       config.GetPoissonConfidence(),
                                       config.GetPoissonOutputPolygons(),
                                       config.GetPoissonManifold(),
                                       config.GetPoissonSolverDivide(),
                                       config.GetPoissonDegree());
;
}
pcl::PolygonMesh Scanner::gp3Normal_reconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{   
    using namespace pcl;
    using namespace pcl_helpers;
    return poisson_gp3_reconstruction<PointXYZ,PointNormal>(cloud,2,1,
                                       config.GetMLSRadiusSearch(),
                                       config.GetNEKSearch(),
                                       config.GetGP3KSearch(),
                                       config.GetGP3MU(),
                                       config.GetGP3SearchRadius(),
                                       config.GetGP3MaxNearestNeighbors(),
                                       config.GetGP3NormalConsistency(),
                                       config.GetPoissonNThreads(),
                                       config.GetPoissonKSearch(),
                                       config.GetPoissonDepth(),
                                       config.GetPoissonPointWeight(),
                                       config.GetPoissonSamplePNode(),
                                       config.GetPoissonScale(),
                                       config.GetPoissonIsoDivide(),
                                       config.GetPoissonConfidence(),
                                       config.GetPoissonOutputPolygons(),
                                       config.GetPoissonManifold(),
                                       config.GetPoissonSolverDivide(),
                                       config.GetPoissonDegree());


}
pcl::PolygonMesh Scanner::gp3Normal_reconstruction(std::vector<Point> cloud)
{   
    using namespace pcl;
    using namespace pcl_helpers;
    return poisson_gp3_reconstruction(cloud,2,1,
                                       config.GetMLSRadiusSearch(),
                                       config.GetNEKSearch(),
                                       config.GetGP3KSearch(),
                                       config.GetGP3MU(),
                                       config.GetGP3SearchRadius(),
                                       config.GetGP3MaxNearestNeighbors(),
                                       config.GetGP3NormalConsistency(),
                                       config.GetPoissonNThreads(),
                                       config.GetPoissonKSearch(),
                                       config.GetPoissonDepth(),
                                       config.GetPoissonPointWeight(),
                                       config.GetPoissonSamplePNode(),
                                       config.GetPoissonScale(),
                                       config.GetPoissonIsoDivide(),
                                       config.GetPoissonConfidence(),
                                       config.GetPoissonOutputPolygons(),
                                       config.GetPoissonManifold(),
                                       config.GetPoissonSolverDivide(),
                                       config.GetPoissonDegree());


}
pcl::PolygonMesh Scanner::gp3Mls_reconstruction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    using namespace pcl;
    using namespace pcl_helpers;
    return poisson_gp3_reconstruction<PointXYZRGB,PointXYZRGBNormal>(cloud,2,2,
                                       config.GetMLSRadiusSearch(),
                                       config.GetNEKSearch(),
                                       config.GetGP3KSearch(),
                                       config.GetGP3MU(),
                                       config.GetGP3SearchRadius(),
                                       config.GetGP3MaxNearestNeighbors(),
                                       config.GetGP3NormalConsistency(),
                                       config.GetPoissonNThreads(),
                                       config.GetPoissonKSearch(),
                                       config.GetPoissonDepth(),
                                       config.GetPoissonPointWeight(),
                                       config.GetPoissonSamplePNode(),
                                       config.GetPoissonScale(),
                                       config.GetPoissonIsoDivide(),
                                       config.GetPoissonConfidence(),
                                       config.GetPoissonOutputPolygons(),
                                       config.GetPoissonManifold(),
                                       config.GetPoissonSolverDivide(),
                                       config.GetPoissonDegree());


}
pcl::PolygonMesh Scanner::gp3Mls_reconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    using namespace pcl;
    using namespace pcl_helpers;
    return poisson_gp3_reconstruction<PointXYZ,PointNormal>(cloud,2,2,
                                       config.GetMLSRadiusSearch(),
                                       config.GetNEKSearch(),
                                       config.GetGP3KSearch(),
                                       config.GetGP3MU(),
                                       config.GetGP3SearchRadius(),
                                       config.GetGP3MaxNearestNeighbors(),
                                       config.GetGP3NormalConsistency(),
                                       config.GetPoissonNThreads(),
                                       config.GetPoissonKSearch(),
                                       config.GetPoissonDepth(),
                                       config.GetPoissonPointWeight(),
                                       config.GetPoissonSamplePNode(),
                                       config.GetPoissonScale(),
                                       config.GetPoissonIsoDivide(),
                                       config.GetPoissonConfidence(),
                                       config.GetPoissonOutputPolygons(),
                                       config.GetPoissonManifold(),
                                       config.GetPoissonSolverDivide(),
                                       config.GetPoissonDegree());


}
pcl::PolygonMesh Scanner::gp3Mls_reconstruction(std::vector<Point> cloud)
{
    using namespace pcl;
    using namespace pcl_helpers;
    return poisson_gp3_reconstruction(cloud,2,2,
                                       config.GetMLSRadiusSearch(),
                                       config.GetNEKSearch(),
                                       config.GetGP3KSearch(),
                                       config.GetGP3MU(),
                                       config.GetGP3SearchRadius(),
                                       config.GetGP3MaxNearestNeighbors(),
                                       config.GetGP3NormalConsistency(),
                                       config.GetPoissonNThreads(),
                                       config.GetPoissonKSearch(),
                                       config.GetPoissonDepth(),
                                       config.GetPoissonPointWeight(),
                                       config.GetPoissonSamplePNode(),
                                       config.GetPoissonScale(),
                                       config.GetPoissonIsoDivide(),
                                       config.GetPoissonConfidence(),
                                       config.GetPoissonOutputPolygons(),
                                       config.GetPoissonManifold(),
                                       config.GetPoissonSolverDivide(),
                                       config.GetPoissonDegree());


}
pcl::PolygonMesh Scanner::poissonNormal_reconstruction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    using namespace pcl;
    using namespace pcl_helpers;
    return poisson_gp3_reconstruction<PointXYZRGB,PointXYZRGBNormal>(cloud,1,1,
                                       config.GetMLSRadiusSearch(),
                                       config.GetNEKSearch(),
                                       config.GetGP3KSearch(),
                                       config.GetGP3MU(),
                                       config.GetGP3SearchRadius(),
                                       config.GetGP3MaxNearestNeighbors(),
                                       config.GetGP3NormalConsistency(),
                                       config.GetPoissonNThreads(),
                                       config.GetPoissonKSearch(),
                                       config.GetPoissonDepth(),
                                       config.GetPoissonPointWeight(),
                                       config.GetPoissonSamplePNode(),
                                       config.GetPoissonScale(),
                                       config.GetPoissonIsoDivide(),
                                       config.GetPoissonConfidence(),
                                       config.GetPoissonOutputPolygons(),
                                       config.GetPoissonManifold(),
                                       config.GetPoissonSolverDivide(),
                                       config.GetPoissonDegree());


}
pcl::PolygonMesh Scanner::poissonNormal_reconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    using namespace pcl;
    using namespace pcl_helpers;
    return poisson_gp3_reconstruction<PointXYZ,PointNormal>(cloud,1,1,
                                       config.GetMLSRadiusSearch(),
                                       config.GetNEKSearch(),
                                       config.GetGP3KSearch(),
                                       config.GetGP3MU(),
                                       config.GetGP3SearchRadius(),
                                       config.GetGP3MaxNearestNeighbors(),
                                       config.GetGP3NormalConsistency(),
                                       config.GetPoissonNThreads(),
                                       config.GetPoissonKSearch(),
                                       config.GetPoissonDepth(),
                                       config.GetPoissonPointWeight(),
                                       config.GetPoissonSamplePNode(),
                                       config.GetPoissonScale(),
                                       config.GetPoissonIsoDivide(),
                                       config.GetPoissonConfidence(),
                                       config.GetPoissonOutputPolygons(),
                                       config.GetPoissonManifold(),
                                       config.GetPoissonSolverDivide(),
                                       config.GetPoissonDegree());


}
pcl::PolygonMesh Scanner::poissonNormal_reconstruction(std::vector<Point> cloud)
{
    using namespace pcl;
    using namespace pcl_helpers;
    return poisson_gp3_reconstruction(cloud,1,1,
                                       config.GetMLSRadiusSearch(),
                                       config.GetNEKSearch(),
                                       config.GetGP3KSearch(),
                                       config.GetGP3MU(),
                                       config.GetGP3SearchRadius(),
                                       config.GetGP3MaxNearestNeighbors(),
                                       config.GetGP3NormalConsistency(),
                                       config.GetPoissonNThreads(),
                                       config.GetPoissonKSearch(),
                                       config.GetPoissonDepth(),
                                       config.GetPoissonPointWeight(),
                                       config.GetPoissonSamplePNode(),
                                       config.GetPoissonScale(),
                                       config.GetPoissonIsoDivide(),
                                       config.GetPoissonConfidence(),
                                       config.GetPoissonOutputPolygons(),
                                       config.GetPoissonManifold(),
                                       config.GetPoissonSolverDivide(),
                                       config.GetPoissonDegree());


}
pcl::PolygonMesh Scanner::poissonMls_reconstruction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    using namespace pcl;
    using namespace pcl_helpers;
    return poisson_gp3_reconstruction<PointXYZRGB,PointXYZRGBNormal>(cloud,1,2,
                                       config.GetMLSRadiusSearch(),
                                       config.GetNEKSearch(),
                                       config.GetGP3KSearch(),
                                       config.GetGP3MU(),
                                       config.GetGP3SearchRadius(),
                                       config.GetGP3MaxNearestNeighbors(),
                                       config.GetGP3NormalConsistency(),
                                       config.GetPoissonNThreads(),
                                       config.GetPoissonKSearch(),
                                       config.GetPoissonDepth(),
                                       config.GetPoissonPointWeight(),
                                       config.GetPoissonSamplePNode(),
                                       config.GetPoissonScale(),
                                       config.GetPoissonIsoDivide(),
                                       config.GetPoissonConfidence(),
                                       config.GetPoissonOutputPolygons(),
                                       config.GetPoissonManifold(),
                                       config.GetPoissonSolverDivide(),
                                       config.GetPoissonDegree());


}

pcl::PolygonMesh Scanner::poissonMls_reconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    using namespace pcl;
    using namespace pcl_helpers;
    return poisson_gp3_reconstruction<PointXYZ,PointNormal>(cloud,1,2,
                                       config.GetMLSRadiusSearch(),
                                       config.GetNEKSearch(),
                                       config.GetGP3KSearch(),
                                       config.GetGP3MU(),
                                       config.GetGP3SearchRadius(),
                                       config.GetGP3MaxNearestNeighbors(),
                                       config.GetGP3NormalConsistency(),
                                       config.GetPoissonNThreads(),
                                       config.GetPoissonKSearch(),
                                       config.GetPoissonDepth(),
                                       config.GetPoissonPointWeight(),
                                       config.GetPoissonSamplePNode(),
                                       config.GetPoissonScale(),
                                       config.GetPoissonIsoDivide(),
                                       config.GetPoissonConfidence(),
                                       config.GetPoissonOutputPolygons(),
                                       config.GetPoissonManifold(),
                                       config.GetPoissonSolverDivide(),
                                       config.GetPoissonDegree());


}

pcl::PolygonMesh Scanner::poissonMls_reconstruction(std::vector<Point> cloud)
{
    using namespace pcl;
    using namespace pcl_helpers;
    return poisson_gp3_reconstruction(cloud,1,2,
                                       config.GetMLSRadiusSearch(),
                                       config.GetNEKSearch(),
                                       config.GetGP3KSearch(),
                                       config.GetGP3MU(),
                                       config.GetGP3SearchRadius(),
                                       config.GetGP3MaxNearestNeighbors(),
                                       config.GetGP3NormalConsistency(),
                                       config.GetPoissonNThreads(),
                                       config.GetPoissonKSearch(),
                                       config.GetPoissonDepth(),
                                       config.GetPoissonPointWeight(),
                                       config.GetPoissonSamplePNode(),
                                       config.GetPoissonScale(),
                                       config.GetPoissonIsoDivide(),
                                       config.GetPoissonConfidence(),
                                       config.GetPoissonOutputPolygons(),
                                       config.GetPoissonManifold(),
                                       config.GetPoissonSolverDivide(),
                                       config.GetPoissonDegree());


}

void Scanner::runPipeline()
{
    //capture plane data
    auto plane = this->capture_FrameXYZRGB(true);

    auto frames_object_0degree   = this->capture_FrameXYZRGB(true);
    auto frames_object_45degree  = this->capture_FrameXYZRGB(true);
    auto frames_object_90degree  = this->capture_FrameXYZRGB(true);
    auto frames_object_135degree = this->capture_FrameXYZRGB(true);
    auto frames_object_180degree = this->capture_FrameXYZRGB(true);
    auto frames_object_225degree = this->capture_FrameXYZRGB(true);
    auto frames_object_270degree = this->capture_FrameXYZRGB(true);
    auto frames_object_315degree = this->capture_FrameXYZRGB(true);
    auto frames_object_360degree = this->capture_FrameXYZRGB(true);
    

    std::vector<PointCloudXYZRGBPtr> object_scans {
        frames_object_0degree,
        frames_object_45degree,  
        frames_object_90degree, 
        frames_object_135degree, 
        frames_object_180degree,
        frames_object_225degree,
        frames_object_270degree,
        frames_object_315degree,
        frames_object_360degree
    };

    //perform Ransac cloud segmentation

    for(int i = 0; i< object_scans.size(); i++)
    {
            this->ransac_SVD<pcl::PointXYZRGB>(object_scans[i],plane);

    }


    //convert to vector<Points>

   if(object_scans.size() < 1) 
       throw std::logic_error("no scans available\n");
   auto aligned_pc = object_scans[0];

   //ICP
   for(int i = 1; i < object_scans.size(); i++)
    aligned_pc = this->align_ICP(object_scans[i],aligned_pc);


   PolygonMesh mesh = this->gp3Mls_reconstruction(aligned_pc);

   this->save_obj(config.GetSavePath(),mesh);

   this->view(mesh);

}
