#include <string>
#include <cstdlib>
#include <imgui.h>
#include "imgui_impl_glfw.h"
#include <sstream>
#include <iostream>
#include <iomanip>

#include <cstring>

#include "Scanner.h"
#include "pcl_helper.h"
#include "ICP.h"

PC_SIMPLE::Ptr Scanner::captureOnce_PcData(bool show) try
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

} catch(const std::exception& e)
{
    std::cerr << e.what() << std::endl;
}


std::vector<PC_RGB::Ptr> Scanner::capture_PcRGBData(bool show) try
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

    while(app && show){

        auto frames = pipe.wait_for_frames(); //Wait for the next set of frames from the camera
        auto depth = frames.get_depth_frame();
        auto color = frames.get_color_frame();
        //Generate the pointcloud and texture mappings
        pc.map_to(color);
        points = pc.calculate(depth);
    
        //Convert generated Point Cloud to PCL Formatting
        PC_RGB::Ptr pcl_points = pcl_helpers::pcRGB_toPCL(points,color);

        draw_pointcloud_pcl(app.width(),app.height(),app_state,pcl_points);

        pointcloud_frames.push_back(pcl_points);
    }
    
    return pointcloud_frames;
} catch(const rs2::error& e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;

} catch(const std::exception& e)
{
    std::cerr << e.what() << std::endl;
}





std::vector<PC_SIMPLE> Scanner::capture_PcData(bool show) try
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
    while(app && show) {

        auto frames = pipe.wait_for_frames(); //Wait for the next set of frames from the camera
        auto depth = frames.get_depth_frame();
        auto color = frames.get_color_frame();
        //Generate the pointcloud and texture mappings

        points = pc.calculate(depth);

        PC_SIMPLE::Ptr pcl_points = pcl_helpers::pc_toPCL(points);
    
        draw_pointcloud(app.width(),app.height(),app_state,points);
        pointcloud_frames.push_back(*pcl_points);

    }

    return pointcloud_frames;
} catch(const rs2::error& e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;

} catch(const std::exception& e)
{
    std::cerr << e.what() << std::endl;
}


PC_RGB::Ptr Scanner::captureOnce_PcRGBData(bool show) try
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

} catch(const std::exception& e)
{
    std::cerr << e.what() << std::endl;
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


//void Scanner::record_pointcloudFrames(const std::string& outputfile) try
//{
//    window app(window_width,window_height,window_name.c_str());
//
//    //control GUI (recorded - allow play button, recording - show 'recording to file' text)
//    bool recorded = false;
//    bool recording = false;    
//    
//    //declare a texture for the depth image on the GPU
//    texture depth_image;
//
//    //declage frameset and frames that will hold the data from the camera 
//    rs2::frameset frames;
//    rs2::frame depth;
//
//    //declare depth colorizer for prettier visualisation of depth data
//    rs2::colorizer color_map;
//
//    //Create a shared pointer to a pipeline
//    auto pipe = std::make_shared<rs2::pipeline>();
//
//    //start streaming with default config 
//    pipe->start();
//
//    //Initalize a shared ptr to a device with the current device on the pipeline
//    rs2::device device = pipe->get_active_profile().get_device();
//
//    int seek_pos; //for controlling the seek bar
//
//    while(app) {
//        // Flags for displaying ImGui window
//        static const int flags = ImGuiWindowFlags_NoCollapse
//            | ImGuiWindowFlags_NoScrollbar
//            | ImGuiWindowFlags_NoSavedSettings
//            | ImGuiWindowFlags_NoTitleBar
//            | ImGuiWindowFlags_NoResize
//            | ImGuiWindowFlags_NoMove;
//
//        ImGui_ImplGlfw_NewFrame();
//        ImGui::SetNextWindowSize({ app.width(), app.height() });
//        ImGui::Begin("app", nullptr, flags);
//
//        // If the device is sreaming live and not from a file
//        if (!device.as<rs2::playback>())
//        {
//            frames = pipe->wait_for_frames(); // wait for next set of frames from the camera
//            depth = color_map.process(frames.get_depth_frame()); // Find and colorize the depth data
//        }
//
//        // Set options for the ImGui buttons
//        ImGui::PushStyleColor(ImGuiCol_TextSelectedBg, { 1, 1, 1, 1 });
//        ImGui::PushStyleColor(ImGuiCol_Button, { 36 / 255.f, 44 / 255.f, 51 / 255.f, 1 });
//        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, { 40 / 255.f, 170 / 255.f, 90 / 255.f, 1 });
//        ImGui::PushStyleColor(ImGuiCol_ButtonActive, { 36 / 255.f, 44 / 255.f, 51 / 255.f, 1 });
//        ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 12);
//
//        if (!device.as<rs2::playback>()) // Disable recording while device is playing
//        {
//            ImGui::SetCursorPos({ app.width() / 2 - 100, 3 * app.height() / 5 + 90});
//            ImGui::Text("Click 'record' to start recording");
//            ImGui::SetCursorPos({ app.width() / 2 - 100, 3 * app.height() / 5 + 110 });
//            if (ImGui::Button("record", { 50, 50 }))
//            {
//                // If it is the start of a new recording (device is not a recorder yet)
//                if (!device.as<rs2::recorder>())
//                {
//                    pipe->stop(); // Stop the pipeline with the default configuration
//                    pipe = std::make_shared<rs2::pipeline>();
//                    rs2::config cfg; // Declare a new configuration
//                    char* file = new char[strlen(outputfile.c_str()) + 4];
//                    strcpy(file,outputfile.c_str());
//                    strcat(file,".bat");
//
//                    cfg.enable_record_to_file(file);
//                    pipe->start(cfg); //File will be opened at this point
//                    device = pipe->get_active_profile().get_device();
//                }
//                else
//                { // If the recording is resumed after a pause, there's no need to reset the shared pointer
//                    device.as<rs2::recorder>().resume(); // rs2::recorder allows access to 'resume' function
//                }
//                recording = true;
//            }
//
//            /*
//            When pausing, device still holds the file.
//            */
//            if (device.as<rs2::recorder>())
//            {
//                if (recording)
//                {
//                    ImGui::SetCursorPos({ app.width() / 2 - 100, 3 * app.height() / 5 + 60 });
//                    char* text = new char[strlen( "Recording to file '") + strlen( outputfile.c_str())+  5];
//                    strcpy(text,"Recording to file '");
//                    strcat(text,outputfile.c_str());
//                    strcat(text,".bat'");
//                    ImGui::TextColored({ 255 / 255.f, 64 / 255.f, 54 / 255.f, 1 }, text);
//                }
//
//                // Pause the playback if button is clicked
//                ImGui::SetCursorPos({ app.width() / 2, 3 * app.height() / 5 + 110 });
//                if (ImGui::Button("pause\nrecord", { 50, 50 }))
//                {
//                    device.as<rs2::recorder>().pause();
//                    recording = false;
//                }
//
//                ImGui::SetCursorPos({ app.width() / 2 + 100, 3 * app.height() / 5 + 110 });
//                if (ImGui::Button(" stop\nrecord", { 50, 50 }))
//                {
//                    pipe->stop(); // Stop the pipeline that holds the file and the recorder
//                    pipe = std::make_shared<rs2::pipeline>(); //Reset the shared pointer with a new pipeline
//                    pipe->start(); // Resume streaming with default configuration
//                    device = pipe->get_active_profile().get_device();
//                    recorded = true; // Now we can run the file
//                    recording = false;
//                }
//            }
//        }
//
//        // After a recording is done, we can play it
//        if (recorded) {
//            ImGui::SetCursorPos({ app.width() / 2 - 100, 4 * app.height() / 5 + 30 });
//            ImGui::Text("Click 'play' to start playing");
//            ImGui::SetCursorPos({ app.width() / 2 - 100, 4 * app.height() / 5 + 50});
//            if (ImGui::Button("play", { 50, 50 }))
//            {
//                if (!device.as<rs2::playback>())
//                {
//                    pipe->stop(); // Stop streaming with default configuration
//                    pipe = std::make_shared<rs2::pipeline>();
//                    rs2::config cfg;
//                    cfg.enable_device_from_file("a.bag");
//                    pipe->start(cfg); //File will be opened in read mode at this point
//                    device = pipe->get_active_profile().get_device();
//                }
//                else
//                {
//                    device.as<rs2::playback>().resume();
//                }
//            }
//        }
//
//        // If device is playing a recording, we allow pause and stop
//        if (device.as<rs2::playback>())
//        {
//            rs2::playback playback = device.as<rs2::playback>();
//            if (pipe->poll_for_frames(&frames)) // Check if new frames are ready
//            {
//                depth = color_map.process(frames.get_depth_frame()); // Find and colorize the depth data for rendering
//            }
//
//            // Render a seek bar for the player
//            float2 location = { app.width() / 4, 4 * app.height() / 5 + 110 };
//            draw_seek_bar(playback , &seek_pos, location, app.width() / 2);
//
//            ImGui::SetCursorPos({ app.width() / 2, 4 * app.height() / 5 + 50 });
//            if (ImGui::Button(" pause\nplaying", { 50, 50 }))
//            {
//                playback.pause();
//            }
//
//            ImGui::SetCursorPos({ app.width() / 2 + 100, 4 * app.height() / 5 + 50 });
//            if (ImGui::Button("  stop\nplaying", { 50, 50 }))
//            {
//                pipe->stop();
//                pipe = std::make_shared<rs2::pipeline>();
//                pipe->start();
//                device = pipe->get_active_profile().get_device();
//            }
//        }
//
//        ImGui::PopStyleColor(4);
//        ImGui::PopStyleVar();
//
//        ImGui::End();
//        ImGui::Render();
//
//        // Render depth frames from the default configuration, the recorder or the playback
//        depth_image.render(depth, { app.width() * 0.25f, app.height() * 0.25f, app.width() * 0.5f, app.height() * 0.75f  });
//    }
//}
//catch (const rs2::error & e)
//{
//    std::cout << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
//}
//catch (const std::exception& e)
//{
//    std::cerr << e.what() << std::endl;
//}
//


std::string pretty_time(std::chrono::nanoseconds duration)
{
    using namespace std::chrono;
    auto hhh = duration_cast<hours>(duration);
    duration -= hhh;
    auto mm = duration_cast<minutes>(duration);
    duration -= mm;
    auto ss = duration_cast<seconds>(duration);
    duration -= ss;
    auto ms = duration_cast<milliseconds>(duration);

    std::ostringstream stream;
    stream << std::setfill('0') << std::setw(hhh.count() >= 10 ? 2 : 1) << hhh.count() << ':' <<
        std::setfill('0') << std::setw(2) << mm.count() << ':' <<
        std::setfill('0') << std::setw(2) << ss.count();
    return stream.str();
}


//void draw_seek_bar(rs2::playback& playback, int* seek_pos, float2& location, float width)
//{
//    int64_t playback_total_duration = playback.get_duration().count();
//    auto progress = playback.get_position();
//    double part = (1.0 * progress) / playback_total_duration;
//    *seek_pos = static_cast<int>(std::max(0.0, std::min(part, 1.0)) * 100);
//    auto playback_status = playback.current_status();
//    ImGui::PushItemWidth(width);
//    ImGui::SetCursorPos({ location.x, location.y });
//    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 12);
//    if (ImGui::SliderInt("##seek bar", seek_pos, 0, 100, "", true))
//    {
//        //Seek was dragged
//        if (playback_status != RS2_PLAYBACK_STATUS_STOPPED) //Ignore seek when playback is stopped
//        {
//            auto duration_db = std::chrono::duration_cast<std::chrono::duration<double, std::nano>>(playback.get_duration());
//            auto single_percent = duration_db.count() / 100;
//            auto seek_time = std::chrono::duration<double, std::nano>((*seek_pos) * single_percent);
//            playback.seek(std::chrono::duration_cast<std::chrono::nanoseconds>(seek_time));
//        }
//    }
//    std::string time_elapsed = pretty_time(std::chrono::nanoseconds(progress));
//    ImGui::SetCursorPos({ location.x + width + 10, location.y });
//    ImGui::Text("%s", time_elapsed.c_str());
//    ImGui::PopStyleVar();
//    ImGui::PopItemWidth();
//}


std::vector<Point> Scanner::convert_pcl_points(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
    std::vector<Point> points;
     for(unsigned int i = 0; i < cloud->points.size(); i++) {
        //if(cloud->points[i].z && cloud->points[i].z < 15.0f){
            Point p;
            p.x = cloud->points[i].x;
            p.y = cloud->points[i].y;
            p.z = cloud->points[i].z;
            p.r = static_cast<float>(cloud->points[i].r) / 255;
            p.g = static_cast<float>(cloud->points[i].g) / 255;
            p.b = static_cast<float>(cloud->points[i].b) / 255;
            points.push_back(p);
        //}
   }

     return points;
}
std::vector<Point> Scanner::convert_pcl_points(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
    std::vector<Point> points;
     for(unsigned int i = 0; i < cloud.points.size(); i++) {
        //if(cloud->points[i].z && cloud->points[i].z < 15.0f){
            Point p;
            p.x = cloud.points[i].x;
            p.y = cloud.points[i].y;
            p.z = cloud.points[i].z;
            p.r = static_cast<float>(cloud.points[i].r) / 255;
            p.g = static_cast<float>(cloud.points[i].g) / 255;
            p.b = static_cast<float>(cloud.points[i].b) / 255;
            points.push_back(p);
        //}
   }

     return points;
}
std::vector<Point> Scanner::convert_pcl_points(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    std::vector<Point> points;
     for(unsigned int i = 0; i < cloud.points.size(); i++) {
        //if(cloud->points[i].z && cloud->points[i].z < 15.0f){
            Point p;
            p.x = cloud.points[i].x;
            p.y = cloud.points[i].y;
            p.z = cloud.points[i].z;
            //p.r = static_cast<float>(cloud->points[i].r) / 255;
            //p.g = static_cast<float>(cloud->points[i].g) / 255;
            //p.b = static_cast<float>(cloud->points[i].b) / 255;
            points.push_back(p);
        //}
   }

     return points;
}
std::vector<Point> Scanner::convert_pcl_points(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    std::vector<Point> points;
     for(unsigned int i = 0; i < cloud->points.size(); i++) {
        //if(cloud->points[i].z && cloud->points[i].z < 15.0f){
            Point p;
            p.x = cloud->points[i].x;
            p.y = cloud->points[i].y;
            p.z = cloud->points[i].z;
            //p.r = static_cast<float>(cloud->points[i].r) / 255;
            //p.g = static_cast<float>(cloud->points[i].g) / 255;
            //p.b = static_cast<float>(cloud->points[i].b) / 255;
            points.push_back(p);
        //}
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
            //p.r = static_cast<float>(cloud->points[i].r) / 255;
            //p.g = static_cast<float>(cloud->points[i].g) / 255;
            //p.b = static_cast<float>(cloud->points[i].b) / 255;
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
            //p.r = static_cast<float>(cloud->points[i].r) / 255;
            //p.g = static_cast<float>(cloud->points[i].g) / 255;
            //p.b = static_cast<float>(cloud->points[i].b) / 255;
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
void Scanner::view(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
        auto points = convert_pcl_points(cloud);
        Window w(window_width,window_height,window_name.c_str());
        w.clearScreen();

        Renderer re(w);
        re.DrawPoints(points);

}



std::vector<Point> Scanner::allign_ICP(pcl::PointCloud<pcl::PointXYZ>::Ptr& sourceCloud,pcl::PointCloud<pcl::PointXYZ>::Ptr& targetCloud)
{
    auto dynamicCloud = convert_pcl_points(sourceCloud);
    auto staticCloud = convert_pcl_points(targetCloud);

    icp(dynamicCloud,staticCloud);
    for(int i = 0; i < staticCloud.size(); i++)
        dynamicCloud.push_back(staticCloud[i]);

    return dynamicCloud;
}

std::vector<Point> Scanner::allign_ICP(pcl::PointCloud<pcl::PointXYZ>& sourceCloud,pcl::PointCloud<pcl::PointXYZ>& targetCloud)
{
    auto dynamicCloud = convert_pcl_points(sourceCloud);
    auto staticCloud = convert_pcl_points(targetCloud);

    icp(dynamicCloud,staticCloud);
    for(int i = 0; i < staticCloud.size(); i++)
        dynamicCloud.push_back(staticCloud[i]);

    return dynamicCloud;
}

std::vector<Point> Scanner::allign_ICP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& sourceCloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& targetCloud)
{
    auto dynamicCloud = convert_pcl_points(sourceCloud);
    auto staticCloud = convert_pcl_points(targetCloud);

    

    icp(dynamicCloud,staticCloud);
    for(int i = 0; i < staticCloud.size(); i++)
        dynamicCloud.push_back(staticCloud[i]);

    return dynamicCloud;
}
std::vector<Point> Scanner::allign_ICP(pcl::PointCloud<pcl::PointXYZRGB>& sourceCloud,pcl::PointCloud<pcl::PointXYZ>& targetCloud)
{
    auto dynamicCloud = convert_pcl_points(sourceCloud);
    auto staticCloud = convert_pcl_points(targetCloud);

    icp(dynamicCloud,staticCloud);
    for(int i = 0; i < staticCloud.size(); i++)
        dynamicCloud.push_back(staticCloud[i]);

    return dynamicCloud;  
}

void Scanner::allign_ICP(std::vector<Point>& sourceCloud,std::vector<Point>& targetCloud)
{

    icp(sourceCloud,targetCloud);
    for(int i = 0; i < targetCloud.size(); i++)
        sourceCloud.push_back(targetCloud[i]);

}

//void Scanner::view_icp(std::vector<Point>& sourceCloud,std::vector<Point>& targetCloud)
//{
//
//    Window w(window_width,window_height,window_name.c_str());
//
//    Renderer re(w);
//    double lastTime;
//    for(int i = 0; i < 500; i ++){
//        icp(sourceCloud,targetCloud,1);
//        auto source_copy = sourceCloud;
//        for(int i = 0; i < targetCloud.size(); i++)
//            source_copy.push_back(targetCloud[i]);
//
//        
//        lastTime = glfwGetTime();
//        re.Draw_short(source_copy,lastTime);
//
//    }
//
//
//}
//
