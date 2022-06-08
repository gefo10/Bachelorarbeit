#include <librealsense2/rs.hpp>
#include <iostream>
#include <AndreiUtils/utilsOpenCVRealsense.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <GLFW/glfw3.h>
#include <thread>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include "pcl_helper.h"
#include <string>

using namespace cv;
using namespace Eigen;
using namespace rs2;
using namespace std;
using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
using window = GLFWwindow;
//void register_glfw_callbacks(window& app, state& app_state);

struct state {
    state() : yaw(0.0), pitch(0.0), last_x(0.0), last_y(0.0),
        ml(false), offset_x(0.0f), offset_y(0.0f) {}
    double yaw, pitch, last_x, last_y; bool ml; float offset_x, offset_y;
};




int main(int argc, char** argv) {

//    window app= GLFWCreateWindow(1280, 720, "RealSense PCL Pointcloud Example");
//    // Construct an object to manage view state
//    state app_state;
//    // register callbacks to allow manipulation of the pointcloud
//    register_glfw_callbacks(app, app_state);
//
    //pcl_helpers::testPrint();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr = 
        pcl_helpers::load_PLY("../raptor-model/source/raptor.ply", cloud);
    
    std::cout << cloud->size() << std::endl;
    pcl_helpers::estimateNormals(cloud);
    std::cout << cloud->size() << std::endl; 
   
    pcl_helpers::view(cloud_ptr);
    


}
// Registers the state variable and callbacks to allow mouse control of the pointcloud
//void register_glfw_callbacks(window& app, state& app_state)
//{
//    app.on_left_mouse = [&](bool pressed)
//    {
//        app_state.ml = pressed;
//    };
//
//    app.on_mouse_scroll = [&](double xoffset, double yoffset)
//    {
//        app_state.offset_x += static_cast<float>(xoffset);
//        app_state.offset_y += static_cast<float>(yoffset);
//    };
//
//    app.on_mouse_move = [&](double x, double y)
//    {
//        if (app_state.ml)
//        {
//            app_state.yaw -= (x - app_state.last_x);
//            app_state.yaw = std::max(app_state.yaw, -120.0);
//            app_state.yaw = std::min(app_state.yaw, +120.0);
//            app_state.pitch += (y - app_state.last_y);
//            app_state.pitch = std::max(app_state.pitch, -80.0);
//            app_state.pitch = std::min(app_state.pitch, +80.0);
//        }
//        app_state.last_x = x;
//        app_state.last_y = y;
//    };
//
//    app.on_key_release = [&](int key)
//    {
//        if (key == 32) // Escape
//        {
//            app_state.yaw = app_state.pitch = 0; app_state.offset_x = app_state.offset_y = 0.0;
//        }
//    };
//}
//// Registers the state variable and callbacks to allow mouse control of the pointcloud
//void register_glfw_callbacks(window& app, state& app_state)
//{
//    app.on_left_mouse = [&](bool pressed)
//    {
//        app_state.ml = pressed;
//    };
//
//    app.on_mouse_scroll = [&](double xoffset, double yoffset)
//    {
//        app_state.offset_x += static_cast<float>(xoffset);
//        app_state.offset_y += static_cast<float>(yoffset);
//    };
//
//    app.on_mouse_move = [&](double x, double y)
//    {
//        if (app_state.ml)
//        {
//            app_state.yaw -= (x - app_state.last_x);
//            app_state.yaw = std::max(app_state.yaw, -120.0);
//            app_state.yaw = std::min(app_state.yaw, +120.0);
//            app_state.pitch += (y - app_state.last_y);
//            app_state.pitch = std::max(app_state.pitch, -80.0);
//            app_state.pitch = std::min(app_state.pitch, +80.0);
//        }
//        app_state.last_x = x;
//        app_state.last_y = y;
//    };
//
//    app.on_key_release = [&](int key)
//    {
//        if (key == 32) // Escape
//        {
//            app_state.yaw = app_state.pitch = 0; app_state.offset_x = app_state.offset_y = 0.0;
//        }
//    };
//}
//// Handles all the OpenGL calls needed to display the point cloud
//void draw_pointcloud(window& app, state& app_state, const std::vector<pcl_ptr>& points)
//{
//    // OpenGL commands that prep screen for the pointcloud
//    glPopMatrix();
//    glPushAttrib(GL_ALL_ATTRIB_BITS);
//
//    float width = app.width(), height = app.height();
//
//    glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
//    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//
//    glMatrixMode(GL_PROJECTION);
//    glPushMatrix();
//    gluPerspective(60, width / height, 0.01f, 10.0f);
//
//    glMatrixMode(GL_MODELVIEW);
//    glPushMatrix();
//    gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);
//
//    glTranslatef(0, 0, +0.5f + app_state.offset_y*0.05f);
//    glRotated(app_state.pitch, 1, 0, 0);
//    glRotated(app_state.yaw, 0, 1, 0);
//    glTranslatef(0, 0, -0.5f);
//
//    glPointSize(width / 640);
//    glEnable(GL_TEXTURE_2D);
//
//    int color = 0;
//
//    for (auto&& pc : points)
//    {
//        auto c = colors[(color++) % (sizeof(colors) / sizeof(float3))];
//
//        glBegin(GL_POINTS);
//        glColor3f(c.x, c.y, c.z);
//
//        /* this segment actually prints the pointcloud */
//        for (int i = 0; i < pc->points.size(); i++)
//        {
//            auto&& p = pc->points[i];
//            if (p.z)
//            {
//                // upload the point and texture coordinates only for points we have depth data for
//                glVertex3f(p.x, p.y, p.z);
//            }
//        }
//
//        glEnd();
//    }
//
//    // OpenGL cleanup
//    glPopMatrix();
//    glMatrixMode(GL_PROJECTION);
//    glPopMatrix();
//    glPopAttrib();
//    glPushMatrix();
//}
// int main(int argc, char** argv) {
//     cout << "Hello World!" << endl;

//     rs2::pipeline_profile config;
//     rs2::pipeline pipeline;
//     rs2::align alignTo(RS2_STREAM_COLOR);

//     config = pipeline.start();

//     int frameCounter = 0, key;
//     while (true) {
//         rs2::frameset currentFrame = pipeline.wait_for_frames();
//         currentFrame = alignTo.process(currentFrame);  // Make sure the frames are spatially aligned
//         auto rs2Image = currentFrame.get_color_frame();
//         auto rs2Depth = currentFrame.get_depth_frame();
//         if (rs2Image.get_frame_number() != frameCounter) {
//             frameCounter = rs2Image.get_frame_number() - 1;  // -1 because frameCounter is incremented outside
    
//             Mat depth = AndreiUtils::depth_frame_to_meters(rs2Depth), image = AndreiUtils::frame_to_mat(rs2Image);
//             // cv::medianBlur(depth, depth, 3);

//             imshow("Depth image", depth);
//             imshow("Image image", image);
//             key = cv::waitKey(1);
//             if (key == 27 || key == 'q') {
//                 break;
//             }
//         }
//     }

//     pipeline.stop();

//     return 0;
// }

