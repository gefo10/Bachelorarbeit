#pragma once 

//#include <GL/glew.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <ScannerLib/Window.h>
#include <ScannerLib/Mesh.h>
#include <ScannerLib/Point.h>
#include <string>


class Renderer
{
public:
    Renderer(Window& w);
    void DrawPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
    void DrawPoints(std::vector<Point>& points);
    void DrawPoints(std::vector<Point*>& points);
    void DrawPoints(std::vector<std::vector<Point>>& points);
    void Draw();
    void Draw(std::string filename);
    void Draw(Mesh mesh);
    void DrawLine(const Point& p1, const Point& p2);

private:
    Window* _window;
    glm::mat4 model;
    std::vector<std::pair<VertexArray,unsigned int>> lines_vaos;
};

