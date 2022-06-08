#pragma once 

//#include <GL/glew.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "Window.h"
#include "Mesh.h"
#include "Point.h"
#include <string>


class Renderer
{
public:
    Renderer(Window& w);
    void DrawPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
    void DrawPoints(std::vector<Point>& points);
    void DrawPoints(std::vector<std::vector<Point>>& points);
    void Draw();
    void Draw(std::string filename);
    void Draw(Mesh mesh);

private:
    Window* _window;
    glm::mat4 model;
};
